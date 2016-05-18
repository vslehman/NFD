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

#include "asf-probing.hpp"
#include "measurement-helper.hpp"
#include "core/logger.hpp"

namespace nfd {
namespace fw {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

NFD_LOG_INIT("AsfStrategy");

const Name AsfStrategy::STRATEGY_NAME("ndn:/localhost/nfd/strategy/asf/%FD%01");
const time::seconds AsfStrategy::SUPPRESSION_TIME = time::seconds(2);

NFD_REGISTER_STRATEGY(AsfStrategy);

AsfStrategy::AsfStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder, name)
  , m_retxSuppression(SUPPRESSION_TIME)
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
  if (m_probe.isProbingNeeded(fibEntry)) {
    shared_ptr<Face> faceToProbe = m_probe.getFaceToProbe(inFace, interest, fibEntry, *faceToUse);

    if (faceToProbe != nullptr) {
      NFD_LOG_DEBUG("Sending probe for " << fibEntry->getPrefix()
                                         << " to FaceId: " << faceToProbe->getId());

      bool wantNewNonce = true;
      forwardInterest(interest, *fibEntry, pitEntry, faceToProbe, wantNewNonce);

      m_probe.afterProbe(fibEntry);
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
