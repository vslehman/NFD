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

#include "asf-measurements.hpp"
#include "random.hpp"
#include "core/logger.hpp"

#include <boost/random/uniform_real_distribution.hpp>

namespace nfd {
namespace fw {

const time::seconds AsfStrategy::ProbingModule::DEFAULT_PROBING_INTERVAL = time::seconds(60);

AsfStrategy::ProbingModule::ProbingModule(AsfMeasurements& measurements)
  : m_probingInterval(DEFAULT_PROBING_INTERVAL)
  , m_measurements(measurements)
{
}

void
AsfStrategy::ProbingModule::scheduleProbe(shared_ptr<fib::Entry> fibEntry,
                                          const time::milliseconds& interval)
{
  ndn::Name prefix = fibEntry->getPrefix();

  // Set the probing flag for the namespace to true after PROBING_INTERVAL
  // period of time
  scheduler::schedule(interval, [this, prefix] () {
    shared_ptr<NamespaceInfo> info = m_measurements.getNamespaceInfo(prefix);

    if (info == nullptr) {
      // FibEntry with the passed prefix has been removed
      return;
    }
    else {
      info->isProbingNeeded = true;
    }
  });
}

shared_ptr<Face>
AsfStrategy::ProbingModule::getFaceToProbe(const Face& inFace,
                                           const Interest& interest,
                                           shared_ptr<fib::Entry> fibEntry,
                                           const Face& faceUsed)
{
  FaceInfoFacePairSet rankedFaces(
    [] (FaceInfoFacePair lhs, FaceInfoFacePair rhs) -> bool {
      // Sort by RTT
      // If a face has timed-out, rank it behind non-timed-out faces
      if (lhs.first->getRtt() != RttStats::RTT_TIMEOUT && rhs.first->getRtt() == RttStats::RTT_TIMEOUT) {
        return true;
      }
      else if (lhs.first->getRtt() == RttStats::RTT_TIMEOUT && rhs.first->getRtt() != RttStats::RTT_TIMEOUT) {
        return false;
      }
      else {
        return lhs.first->getSrtt() < rhs.first->getSrtt();
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

    FaceInfo* info = m_measurements.getFaceInfo(*fibEntry, *hop.getFace());

    // If no RTT has been recorded, probe this face
    if (info == nullptr || info->getSrtt() == RttStats::RTT_NO_MEASUREMENT) {
      return hop.getFace();
    }

    // Add FaceInfo to container sorted by RTT
    rankedFaces.insert(std::make_pair(make_shared<FaceInfo>(*info), hop.getFace()));
  }

  if (rankedFaces.empty()) {
    // No Face to probe
    return nullptr;
  }

  return getFaceBasedOnProbability(rankedFaces);
}

bool
AsfStrategy::ProbingModule::isProbingNeeded(shared_ptr<fib::Entry> fibEntry)
{
  // Return the probing status flag for a namespace
  NamespaceInfo& info = m_measurements.getOrCreateNamespaceInfo(*fibEntry);

  // If a first probe has not been scheduled for a namespace
  if (!info.hasFirstProbeBeenScheduled) {
    // Schedule first probe between 0 and 5 seconds
    uint64_t interval = getRandomNumber(0, 5000);
    scheduleProbe(fibEntry, time::milliseconds(interval));

    info.hasFirstProbeBeenScheduled = true;
  }

  return info.isProbingNeeded;
}

void
AsfStrategy::ProbingModule::afterForwardingProbe(shared_ptr<fib::Entry> fibEntry)
{
  // After probing is done, need to set probing flag to false and
  // schedule another future probe
  NamespaceInfo& info = m_measurements.getOrCreateNamespaceInfo(*fibEntry);
  info.isProbingNeeded = false;

  scheduleProbe(fibEntry, getProbingInterval());
}

shared_ptr<Face>
AsfStrategy::ProbingModule::getFaceBasedOnProbability(const FaceInfoFacePairSet& rankedFaces)
{
  double randomNumber = getRandomNumber(0, 1);
  uint64_t rankSum = ((rankedFaces.size() + 1) * (rankedFaces.size())) / 2;

  uint64_t rank = 1;
  double offset = 0;

  for (const FaceInfoFacePair pair : rankedFaces) {
    double probability = getProbingProbability(rank++, rankSum, rankedFaces.size());
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
      // Found face to probe
      return pair.second;
    }

    offset += probability;
  }

  throw std::runtime_error("Unable to find face to probe using probability!");
}

double
AsfStrategy::ProbingModule::getProbingProbability(uint64_t rank, uint64_t rankSum, uint64_t nFaces)
{
  // p = n + 1 - j ; n: # faces
  //     ---------
  //     sum(ranks)
  return ((double)(nFaces + 1 - rank)) / rankSum;
}

double
AsfStrategy::ProbingModule::getRandomNumber(double start, double end)
{
  boost::random::uniform_real_distribution<double> distribution(start, end);
  return distribution(getGlobalRng());
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

NFD_LOG_INIT("AsfStrategy");

const Name AsfStrategy::STRATEGY_NAME("ndn:/localhost/nfd/strategy/asf/%FD%01");
const time::milliseconds AsfStrategy::RETX_SUPPRESSION_INITIAL(10);
const time::milliseconds AsfStrategy::RETX_SUPPRESSION_MAX(250);

NFD_REGISTER_STRATEGY(AsfStrategy);

AsfStrategy::AsfStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder, name)
  , m_retxSuppression(RETX_SUPPRESSION_INITIAL,
                      RetxSuppressionExponential::DEFAULT_MULTIPLIER,
                      RETX_SUPPRESSION_MAX)
  , m_measurements(getMeasurements())
  , m_probing(m_measurements)
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
  // Should the Interest be suppressed?
  RetxSuppression::Result suppressResult = m_retxSuppression.decide(inFace, interest, *pitEntry);

  switch (suppressResult) {
  case RetxSuppression::NEW:
  case RetxSuppression::FORWARD:
    break;
  case RetxSuppression::SUPPRESS:
    NFD_LOG_DEBUG(interest << " from=" << inFace.getId() << " suppressed");
    return;
  }

  const fib::NextHopList& nexthops = fibEntry->getNextHops();

  if (nexthops.size() == 0) {
    NFD_LOG_TRACE("Reject pending Interest: No next hops for " << fibEntry->getPrefix());
    this->rejectPendingInterest(pitEntry);
    return;
  }
  else if (nexthops.size() == 1 && nexthops.front().getFace()->getId() == inFace.getId()) {
    // If the only nexthop that exists is the one the Interest was received on, reject
    NFD_LOG_TRACE("Reject pending Interest: inFace is the only nexthop for " << fibEntry->getPrefix());
    this->rejectPendingInterest(pitEntry);
    return;
  }

  const shared_ptr<Face> faceToUse = getBestFaceForForwarding(*fibEntry, inFace);

  if (faceToUse == nullptr) {
    NFD_LOG_TRACE("Reject pending Interest: No face to use for forwarding");
    this->rejectPendingInterest(pitEntry);
    return;
  }

  forwardInterest(interest, *fibEntry, pitEntry, faceToUse);

  // If necessary, send probe
  if (m_probing.isProbingNeeded(fibEntry)) {
    shared_ptr<Face> faceToProbe = m_probing.getFaceToProbe(inFace, interest, fibEntry, *faceToUse);

    if (faceToProbe != nullptr) {
      NFD_LOG_TRACE("Sending probe for " << fibEntry->getPrefix()
                                         << " to FaceId: " << faceToProbe->getId());

      forwardInterest(interest, *fibEntry, pitEntry, faceToProbe, true);
      m_probing.afterForwardingProbe(fibEntry);
    }
  }
}

void
AsfStrategy::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                   const Face& inFace,
                                   const Data& data)
{
  shared_ptr<NamespaceInfo> namespaceInfo = m_measurements.getNamespaceInfo(pitEntry->getName());
  
  if (namespaceInfo == nullptr) {
    NFD_LOG_TRACE("Could not find measurements entry for " << pitEntry->getName());
    return;
  }

  // Get info associated with the face
  FaceInfo& faceInfo = namespaceInfo->get(inFace.getId());

  faceInfo.recordRtt(pitEntry, inFace);

  // Extend lifetime for measurements associated with face
  namespaceInfo->extendFaceInfoLifetime(faceInfo, inFace);

  faceInfo.cancelTimeoutEvent(data.getName());
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
  this->sendInterest(pitEntry, outFace, wantNewNonce);

  FaceInfo& faceInfo = m_measurements.getOrCreateFaceInfo(fibEntry, *outFace);

  // Refresh measurements since Face is being used for forwarding
  NamespaceInfo& namespaceInfo = m_measurements.getOrCreateNamespaceInfo(fibEntry);
  namespaceInfo.extendFaceInfoLifetime(faceInfo, *outFace);

  if (!faceInfo.isTimeoutScheduled()) {
    // Estimate and schedule timeout
    RttEstimator::Duration timeout = faceInfo.computeRto();

    NFD_LOG_DEBUG("Scheduling timeout for " << fibEntry.getPrefix() << " FaceId: " << outFace->getId() <<
                  " in " << time::duration_cast<time::milliseconds>(timeout) << " ms");

    scheduler::EventId id = scheduler::schedule(timeout,
        bind(&AsfStrategy::onTimeout, this, interest.getName(), outFace->getId()));

    faceInfo.setTimeoutEvent(id, interest.getName());
  }
}

struct FaceStats
{
public:
  shared_ptr<Face> face;
  RttStats::Rtt rtt;
  RttStats::Rtt srtt;
  uint64_t cost;
};

double
getValueForSorting(const FaceStats& stats)
{
  // These values allow faces with no measurements to be ranked better than timeouts
  // srtt < RTT_NO_MEASUREMENT < RTT_TIMEOUT
  static const RttStats::Rtt SORTING_RTT_TIMEOUT = time::microseconds::max().count();
  static const RttStats::Rtt SORTING_RTT_NO_MEASUREMENT = SORTING_RTT_TIMEOUT/2;

  if (stats.rtt == RttStats::RTT_TIMEOUT) {
    return SORTING_RTT_TIMEOUT;
  }
  else if (stats.rtt == RttStats::RTT_NO_MEASUREMENT) {
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

    FaceInfo* info = m_measurements.getFaceInfo(fibEntry, *hop.getFace());

    if (info == nullptr) {
      FaceStats stats = {hop.getFace(),
                         RttStats::RTT_NO_MEASUREMENT,
                         RttStats::RTT_NO_MEASUREMENT,
                         hop.getCost()};

      rankedFaces.insert(stats);
    }
    else {
      FaceStats stats = {hop.getFace(), info->getRtt(), info->getSrtt(), hop.getCost()};
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
AsfStrategy::onTimeout(const ndn::Name& interestName, nfd::face::FaceId faceId)
{
  NFD_LOG_INFO("FaceId: " << faceId << " for " << interestName << " has timed-out");

  shared_ptr<NamespaceInfo> namespaceInfo = m_measurements.getNamespaceInfo(interestName);

  if (namespaceInfo == nullptr) {
    NFD_LOG_DEBUG("FibEntry for " << interestName << " was removed");
    return;
  }

  FaceInfo::Table::iterator it = namespaceInfo->find(faceId);

  if (it == namespaceInfo->end()) {
    it = namespaceInfo->insert(faceId);
  }

  FaceInfo& faceInfo = it->second;
  faceInfo.recordTimeout(interestName);
}

} // namespace fw
} // namespace nfd
