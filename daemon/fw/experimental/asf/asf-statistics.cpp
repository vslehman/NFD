/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2015,  Regents of the University of California,
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

#include "asf-statistics.hpp"

#include "../rtt-recorder.hpp"
#include "core/scheduler.hpp"
#include "table/fib-entry.hpp"
#include "table/measurements-accessor.hpp"

namespace nfd {
namespace fw {
namespace experimental {

NFD_LOG_INIT("AsfStatistics");

const time::microseconds AsfStatistics::MEASUREMENTS_LIFETIME = time::seconds(30);
const time::seconds AsfStatistics::LEARNING_PERIOD = time::seconds(10);

struct FaceStats
{
public:
  shared_ptr<Face> face;
  Rtt rtt;
  Rtt srtt;
  uint64_t cost;
};

// These values allow faces with no measurements to be ranked better than timeouts
// srtt < RTT_NO_MEASUREMENT < RTT_TIMEOUT
static const Rtt SORTING_RTT_TIMEOUT = time::microseconds::max().count();
static const Rtt SORTING_RTT_NO_MEASUREMENT = SORTING_RTT_TIMEOUT/2;

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
AsfStatistics::getBestFace(const fib::Entry& fibEntry, const Face& inFace)
{
  NFD_LOG_INFO("Looking for best face for " << fibEntry.getPrefix());

  if (!m_hasReceivedFirstInterest) {
    startLearningPeriod();
    m_hasReceivedFirstInterest = true;
  }

  // If it is still the learning period, use the learning period rules
  if (m_isLearningPeriod) {
    return getBestFaceDuringLearningPeriod(fibEntry, inFace);
  }

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

    FaceInfo* info = getFaceInfo(fibEntry, *hop.getFace());

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
AsfStatistics::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                     const Face& inFace,
                                     const Data& data)
{
  NFD_LOG_TRACE("AsfStatistics::beforeSatisfyInterest");

  // Get measurements::Entry associated with the namespace
  shared_ptr<measurements::Entry> me = m_measurements.findLongestPrefixMatch(*pitEntry);

  if (me == nullptr) {
    NFD_LOG_WARN("Could not find measurements entry for " << pitEntry->getName());
    return;
  }

  // Set or update entry lifetime
  m_measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

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

void
AsfStatistics::afterForwardInterest(const Interest& interest,
                                    const fib::Entry& fibEntry,
                                    const Face& face)
{
  FaceInfo& info = getOrCreateFaceInfo(fibEntry, face);

  // Refresh measurements since Face is being used for forwarding
  NamespaceInfo& namespaceInfo = getOrCreateNamespaceInfo(fibEntry);
  namespaceInfo.extendFaceInfoLifetime(info, face);

  if (!info.isTimeoutScheduled()) {
    // Estimate and schedule timeout
    RttEstimator::Duration timeout = info.rttEstimator.computeRto();

    NFD_LOG_DEBUG("Scheduling timeout for " << fibEntry.getPrefix() << " FaceId: " << face.getId() <<
                  " in " << time::duration_cast<time::milliseconds>(timeout) << " ms");

    scheduler::EventId id = scheduler::schedule(timeout,
        bind(&AsfStatistics::onTimeout, this, interest.getName(), face.getId()));

    info.setTimeoutEvent(id, interest.getName());
  }
}

void
AsfStatistics::onTimeout(const ndn::Name& interestName, FaceId faceId)
{
  NFD_LOG_INFO("FaceId: " << faceId << " for " << interestName << " has timed-out");

  shared_ptr<NamespaceInfo> info = getNamespaceInfo(interestName);

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

FaceInfo&
AsfStatistics::getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  NamespaceInfo& info = getOrCreateNamespaceInfo(fibEntry);

  return info.getOrCreateFaceInfo(fibEntry, face);
}

FaceInfo*
AsfStatistics::getFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  NamespaceInfo& info = getOrCreateNamespaceInfo(fibEntry);

  return info.getFaceInfo(fibEntry, face);
}

NamespaceInfo&
AsfStatistics::getOrCreateNamespaceInfo(const fib::Entry& fibEntry)
{
  shared_ptr<measurements::Entry> me = m_measurements.get(fibEntry);

  // Set or update entry lifetime
  m_measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  return *info;
}

shared_ptr<NamespaceInfo>
AsfStatistics::getNamespaceInfo(const ndn::Name& prefix)
{
  shared_ptr<measurements::Entry> me = m_measurements.findLongestPrefixMatch(prefix);

  if (me == nullptr) {
    NFD_LOG_WARN("Could not find measurements entry for " << prefix);
    return nullptr;
  }

  // Set or update entry lifetime
  m_measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  return info;
}

void
AsfStatistics::startLearningPeriod()
{
  if (!m_isLearningPeriod) {
    m_isLearningPeriod = true;

    scheduler::schedule(LEARNING_PERIOD, [this] {
      this->endLearningPeriod();
    });
  }
  else {
    throw std::runtime_error("Tried to start learning period while one is already in progress");
  }
}

void
AsfStatistics::endLearningPeriod()
{
  m_isLearningPeriod = false;
}

const shared_ptr<Face>
AsfStatistics::getBestFaceDuringLearningPeriod(const fib::Entry& fibEntry, const Face& inFace)
{
  shared_ptr<Face> backupFace;

  for (const fib::NextHop& hop : fibEntry.getNextHops()) {

    // Don't foward back to the inFace
    if (hop.getFace()->getId() == inFace.getId()) {
      continue;
    }

    // Use the lowest cost face if all others also timed out
    if (backupFace == nullptr) {
      return hop.getFace();
    }

    FaceInfo* info = getFaceInfo(fibEntry, *hop.getFace());

    if (info == nullptr) {
      // If the face has not collected measurements, use it
      return hop.getFace();
    }
    else if (info->rtt == RttStat::RTT_TIMEOUT) {
      // Do not use this Face if the last Interest timed out
      continue;
    }
    else {
      // The last interest did not timeout so continue using face
      return hop.getFace();
    }
  }

  return backupFace;
}

} // namespace experimental
} // namespace fw
} // namespace nfd
