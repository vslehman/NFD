/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2016,  Regents of the University of California,
 *                           Arizona Board of Regents,
 *                           Colorado State University,
 *                           University Pierre & Marie Curie, Sorbonne University,
 *                           Washington University in St. Louis,
 *                           Beijing Institute of Technology,
 *                           The University of Memphis.
 *
 * This file is part of NFD (Named Data Networking Forwarding Daemon).
 * See AUTHORS.md for complete list of NFD authors and contributors.
 *
 * NFD is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NFD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NFD, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "asf-strategy.hpp"

#include "measurement-helper.hpp"
#include "random.hpp"
#include "core/logger.hpp"

#include <boost/random/uniform_real_distribution.hpp>

namespace nfd {
namespace fw {

double
AsfProbing::getRandomNumber(double start, double end)
{
  boost::random::uniform_real_distribution<double> distribution(start, end);

  return distribution(getGlobalRng());
}

//==============================================================================
// Probability Function
//------------------------------------------------------------------------------
// p = n + 1 - j ; n: # faces
//     ---------
//     sum(ranks)
double
getProbabilityFunction1(uint64_t rank, uint64_t rankSum, uint64_t nFaces)
{
  return ((double)(nFaces + 1 - rank))/rankSum;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const time::seconds AsfProbing::DEFAULT_PROBING_INTERVAL = time::seconds(60);

AsfProbing::AsfProbing(MeasurementsAccessor& measurements)
  : m_probabilityFunction(&getProbabilityFunction1)
  , m_isProbingNeeded(true)
  , m_probingInterval(DEFAULT_PROBING_INTERVAL)
  , m_measurements(measurements)
{
  BOOST_ASSERT(m_probabilityFunction != nullptr);
}

void
AsfProbing::scheduleProbe(shared_ptr<fib::Entry> fibEntry, const time::milliseconds& interval)
{
  ndn::Name prefix = fibEntry->getPrefix();

  //NFD_LOG_DEBUG("Scheduling probe for " << prefix);

  // Set the probing flag for the namespace to true after PROBING_INTERVAL
  // period of time
  scheduler::schedule(interval, [this, prefix] () {
    shared_ptr<NamespaceInfo> info = MeasurementHelper::getNamespaceInfo(m_measurements, prefix);

    if (info == nullptr) {
      //NFD_LOG_DEBUG("FibEntry for " << prefix << " has been removed");
      return;
    }
    else {
      //NFD_LOG_TRACE("Probing is due for " << prefix);
      info->isProbingNeeded = true;
    }
  });
}

shared_ptr<Face>
AsfProbing::getFaceToProbe(const Face& inFace,
                                 const Interest& interest,
                                 shared_ptr<fib::Entry> fibEntry,
                                 const Face& faceUsed)
{
  //NFD_LOG_TRACE("Looking for face to probe " << fibEntry->getPrefix());

  FaceInfoFacePairSet rankedFaces(
    [] (FaceInfoFacePair lhs, FaceInfoFacePair rhs) -> bool {
      // Sort by RTT
      // If a face has timed-out, rank it behind non-timed-out faces
      if (lhs.first->rtt != RttStat::RTT_TIMEOUT && rhs.first->rtt == RttStat::RTT_TIMEOUT) {
        return true;
      }
      else if (lhs.first->rtt == RttStat::RTT_TIMEOUT && rhs.first->rtt != RttStat::RTT_TIMEOUT) {
        return false;
      }
      else {
        return lhs.first->srtt < rhs.first->srtt;
      }
  });

  // Put eligible faces into rankedFaces. If a face does not have an RTT measurement,
  // pick the face for probing immediately
  for (const fib::NextHop& hop : fibEntry->getNextHops()) {

    // Don't send probe Interest back to the incoming face or use the same face
    // as the forwarded Interest
    if (hop.getFace()->getId() == inFace.getId() ||
        hop.getFace()->getId() == faceUsed.getId())
    {
      continue;
    }

    FaceInfo* info = MeasurementHelper::getFaceInfo(m_measurements, *fibEntry, *hop.getFace());

    // If no RTT has been recorded, probe this face
    if (info == nullptr || info->srtt == RttStat::RTT_NO_MEASUREMENT) {
      //NFD_LOG_DEBUG("Found face to probe with no RTT measurement");
      return hop.getFace();
    }

    // Add FaceInfo to container sorted by RTT
    rankedFaces.insert(std::make_pair(make_shared<FaceInfo>(*info), hop.getFace()));
  }

  if (rankedFaces.empty()) {
    //NFD_LOG_DEBUG("Unable to find a face to probe");
    return nullptr;
  }

  return getFaceBasedOnProbability(rankedFaces);
}

bool
AsfProbing::isProbingNeeded(shared_ptr<fib::Entry> fibEntry)
{
  // Return the probing flag status for a namespace
  NamespaceInfo& info = MeasurementHelper::getOrCreateNamespaceInfo(m_measurements, *fibEntry);

  // If a first probe has not been scheduled for a namespace
  if (!info.hasFirstProbeBeenScheduled) {
    // Schedule first probe at random interval
    uint64_t interval = getRandomNumber(0, 5000);

    //NFD_LOG_DEBUG("Scheduling first probe for " << fibEntry->getPrefix() <<
    //              " in " << interval << " ms");

    // If the probe is scheduled for now, probe immediately
    if (interval == 0) {
      info.isProbingNeeded = true;
    }
    else {
      scheduleProbe(fibEntry, time::milliseconds(interval));
    }

    info.hasFirstProbeBeenScheduled = true;
  }

  return info.isProbingNeeded;
}

void
AsfProbing::afterForwardingProbe(shared_ptr<fib::Entry> fibEntry)
{
  // After probing is done, need to set probing flag to false and
  // schedule another future probe
  NamespaceInfo& info = MeasurementHelper::getOrCreateNamespaceInfo(m_measurements, *fibEntry);
  info.isProbingNeeded = false;

  scheduleProbe(fibEntry, getProbingInterval());
}

shared_ptr<Face>
AsfProbing::getFaceBasedOnProbability(const FaceInfoFacePairSet& rankedFaces)
{
  double randomNumber = getRandomNumber(0, 1);
  //NFD_LOG_TRACE("randomNumber: " << randomNumber);

  uint64_t rankSum = ((rankedFaces.size() + 1)*(rankedFaces.size()))/2;
  //NFD_LOG_TRACE("rankSum: " << rankSum);

  uint64_t rank = 1;

  double offset = 0;

  for (const FaceInfoFacePair pair : rankedFaces) {
    double probability = m_probabilityFunction(rank++, rankSum, rankedFaces.size());
    //NFD_LOG_TRACE("probability: " << probability << " for FaceId: " << pair.second->getId());

    // Is the random number within the bounds of this face's probability + the previous faces'
    // probability?
    //
    // e.g. (FaceId: 1, p=0.5), (FaceId: 2, p=0.33), (FaceId: 3, p=0.17)
    //      randomNumber = 0.92
    //
    //      The face with FaceId: 3 should be picked
    //      (0.68 < 0.5 + 0.33 + 0.17) == true
    //
    if (randomNumber < offset + probability) {
      //NFD_LOG_DEBUG("Found face to probe");
      return pair.second;
    }

    offset += probability;
    //NFD_LOG_TRACE("offset: " << offset);
  }

  //NFD_LOG_FATAL("Unable to find face to probe using probability!");
  throw std::runtime_error("Unable to find face to probe using probability!");
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

NFD_LOG_INIT("AsfStrategy");

const Name AsfStrategy::STRATEGY_NAME("ndn:/localhost/nfd/strategy/asf/%FD%01");
const time::seconds AsfStrategy::SUPPRESSION_TIME = time::seconds(2);

NFD_REGISTER_STRATEGY(AsfStrategy);

AsfStrategy::AsfStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder, name)
  , m_retxSuppression(SUPPRESSION_TIME)
  , m_probing(getMeasurements())
{
}

AsfStrategy::~AsfStrategy()
{
}

void
AsfStrategy::afterReceiveInterest(const Face& inFace,
                                  const Interest& interest,
                                  shared_ptr<fib::Entry> fibEntry,
                                  shared_ptr<pit::Entry> pitEntry)
{
  NFD_LOG_TRACE("Received Interest " << interest.getName() << " nonce=" << interest.getNonce());

  // Should the Interest be suppressed?
  RetxSuppression::Result suppressResult = m_retxSuppression.decide(inFace, interest, *pitEntry);

  switch (suppressResult) {
  case RetxSuppression::NEW:
  case RetxSuppression::FORWARD:
    break;
  case RetxSuppression::SUPPRESS:
    NFD_LOG_DEBUG(interest << " interestFrom " << inFace.getId() << " retx-suppress");
    return;
  }

  const fib::NextHopList& nexthops = fibEntry->getNextHops();

  if (nexthops.size() == 0) {
    NFD_LOG_TRACE("Rejecting Interest: No next hops for " << fibEntry->getPrefix());
    this->rejectPendingInterest(pitEntry);
    return;
  }
  else if (nexthops.size() == 1 && nexthops.front().getFace()->getId() == inFace.getId()) {
    // If the only nexthop that exists is the one the Interest was received on, reject
    NFD_LOG_TRACE("Rejecting Interest: inFace is the only nexthop for " << fibEntry->getPrefix());
    this->rejectPendingInterest(pitEntry);
    return;
  }

  const shared_ptr<Face> faceToUse = getBestFaceForForwarding(*fibEntry, inFace);

  if (faceToUse == nullptr) {
    NFD_LOG_TRACE("Rejecting Interest: No best face");
    this->rejectPendingInterest(pitEntry);
    return;
  }

  forwardInterest(interest, *fibEntry, pitEntry, faceToUse);

  // If necessary, send probe
  if (m_probing.isProbingNeeded(fibEntry)) {
    shared_ptr<Face> faceToProbe = m_probing.getFaceToProbe(inFace, interest, fibEntry, *faceToUse);

    if (faceToProbe != nullptr) {
      NFD_LOG_DEBUG("Sending probe for " << fibEntry->getPrefix()
                                         << " to FaceId: " << faceToProbe->getId());

      bool wantNewNonce = true;
      forwardInterest(interest, *fibEntry, pitEntry, faceToProbe, wantNewNonce);

      m_probing.afterForwardingProbe(fibEntry);
    }
  }
}

void
AsfStrategy::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                   const Face& inFace,
                                   const Data& data)
{
  NFD_LOG_TRACE("AsfStrategy::beforeSatisfyInterest");

  // Get measurements::Entry associated with the namespace
  shared_ptr<measurements::Entry> me = this->getMeasurements().findLongestPrefixMatch(*pitEntry);

  if (me == nullptr) {
    NFD_LOG_WARN("Could not find measurements entry for " << pitEntry->getName());
    return;
  }

  // Set or update entry lifetime
  this->getMeasurements().extendLifetime(*me, FaceInfo::MEASUREMENT_LIFETIME);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  // Get info associated with the face
  FaceInfo& face = info->faceInfoMap.at(inFace.getId());

  m_rttRecorder.record(face, pitEntry, me->getName(), inFace);

  // Extend lifetime for measurements associated with face
  info->extendFaceInfoLifetime(face, inFace);

  if (face.isTimeoutScheduled() && face.doesNameMatchLastInterest(data.getName())) {
    // Cancel timeout
    NFD_LOG_DEBUG("Canceling timeout event for " << pitEntry->getName());
    face.cancelTimeoutEvent();
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void
AsfStrategy::forwardInterest(const Interest& interest,
                             const fib::Entry& fibEntry,
                             shared_ptr<pit::Entry> pitEntry,
                             shared_ptr<Face> outFace,
                             bool wantNewNonce)
{
  NFD_LOG_DEBUG("Forwarding Interest using FaceId: " << outFace->getId());
  this->sendInterest(pitEntry, outFace, wantNewNonce);

  FaceInfo& info = MeasurementHelper::getOrCreateFaceInfo(getMeasurements(), fibEntry, *outFace);

  // Refresh measurements since Face is being used for forwarding
  NamespaceInfo& namespaceInfo = MeasurementHelper::getOrCreateNamespaceInfo(getMeasurements(), fibEntry);
  namespaceInfo.extendFaceInfoLifetime(info, *outFace);

  if (!info.isTimeoutScheduled()) {
    // Estimate and schedule timeout
    RttEstimator::Duration timeout = info.rttEstimator.computeRto();

    NFD_LOG_DEBUG("Scheduling timeout for " << fibEntry.getPrefix() << " FaceId: " << outFace->getId() <<
                  " in " << time::duration_cast<time::milliseconds>(timeout) << " ms");

    scheduler::EventId id = scheduler::schedule(timeout,
        bind(&AsfStrategy::onTimeout, this, interest.getName(), outFace->getId()));

    info.setTimeoutEvent(id, interest.getName());
  }
}

// These values allow faces with no measurements to be ranked better than timeouts
// srtt < RTT_NO_MEASUREMENT < RTT_TIMEOUT
static const Rtt SORTING_RTT_TIMEOUT = time::microseconds::max().count();
static const Rtt SORTING_RTT_NO_MEASUREMENT = SORTING_RTT_TIMEOUT/2;

struct FaceStats
{
public:
  shared_ptr<Face> face;
  Rtt rtt;
  Rtt srtt;
  uint64_t cost;
};

double
getValueForSorting(const FaceStats& stats)
{
  if (stats.rtt == RttStat::RTT_TIMEOUT) {
    return SORTING_RTT_TIMEOUT;
  }
  else if (stats.rtt == RttStat::RTT_NO_MEASUREMENT) {
    return SORTING_RTT_NO_MEASUREMENT;
  }
  else {
    return stats.srtt;
  }
}

const shared_ptr<Face>
AsfStrategy::getBestFaceForForwarding(const fib::Entry& fibEntry, const Face& inFace)
{
  NFD_LOG_INFO("Looking for best face for " << fibEntry.getPrefix());

  typedef std::function<bool(const FaceStats&, const FaceStats&)> FaceStatsPredicate;
  typedef std::set<FaceStats, FaceStatsPredicate> FaceStatsSet;

  FaceStatsSet rankedFaces(
    [] (const FaceStats& lhs, const FaceStats& rhs) -> bool {
      // Sort by RTT and then by cost
      double lhsValue = getValueForSorting(lhs);
      double rhsValue = getValueForSorting(rhs);

      if (lhsValue < rhsValue) {
        return true;
      }
      else if (lhsValue == rhsValue) {
        return lhs.cost < rhs.cost;
      }
      else {
        return false;
      }
  });

  for (const fib::NextHop& hop : fibEntry.getNextHops()) {

    if (hop.getFace()->getId() == inFace.getId()) {
      continue;
    }

    FaceInfo* info = MeasurementHelper::getFaceInfo(getMeasurements(), fibEntry, *hop.getFace());

    if (info == nullptr) {
      FaceStats stats = {hop.getFace(),
                         RttStat::RTT_NO_MEASUREMENT,
                         RttStat::RTT_NO_MEASUREMENT,
                         hop.getCost()};

      rankedFaces.insert(stats);
    }
    else {
      FaceStats stats = {hop.getFace(), info->rtt, info->srtt, hop.getCost()};
      rankedFaces.insert(stats);
    }
  }

  FaceStatsSet::iterator it = rankedFaces.begin();

  if (it != rankedFaces.end()) {
    return it->face;
  }
  else {
    NFD_LOG_WARN("Could not find best face to forward " << fibEntry.getPrefix());
    throw std::runtime_error("Could not find best face to forward " + fibEntry.getPrefix().toUri());
  }
}

void
AsfStrategy::onTimeout(const ndn::Name& interestName, FaceId faceId)
{
  NFD_LOG_INFO("FaceId: " << faceId << " for " << interestName << " has timed-out");

  shared_ptr<NamespaceInfo> info = MeasurementHelper::getNamespaceInfo(getMeasurements(), interestName);

  if (info == nullptr) {
    NFD_LOG_DEBUG("FibEntry for " << interestName << " was removed");
    return;
  }

  FaceInfoMap::iterator it = info->faceInfoMap.find(faceId);

  FaceInfo* record;

  if (it == info->faceInfoMap.end()) {
    const auto& pair = info->faceInfoMap.insert(std::make_pair(faceId, FaceInfo()));
    record = &pair.first->second;
  }
  else {
    record = &it->second;
  }

  if (record != nullptr) {
    record->rtt = RttStat::RTT_TIMEOUT;

    // There should never be a timeout for an Interest that does not match
    // FaceInfo.m_lastInterestName
    if (record->isTimeoutScheduled() && record->doesNameMatchLastInterest(interestName)) {
      record->cancelTimeoutEvent();
    }
    else {
      throw std::runtime_error("Timeout for " + interestName.toUri() +
        " does not match FaceInfo.m_lastInterest: " + record->getLastInterestName().toUri());
    }
  }
  else {
    throw std::runtime_error("Could not find or create FaceInfo for " + interestName.toUri() +
                             " FaceId: " + std::to_string(faceId));
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace fw
} // namespace nfd
