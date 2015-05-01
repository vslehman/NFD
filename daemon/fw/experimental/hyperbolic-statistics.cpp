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

#include "hyperbolic-statistics.hpp"

#include "rtt-recorder.hpp"
#include "table/fib-entry.hpp"
#include "table/measurements-accessor.hpp"

namespace nfd {
namespace fw {
namespace experimental {

NFD_LOG_INIT("HyperbolicStatistics");

typedef HyperbolicStatistics::FaceInfo FaceInfo;
typedef HyperbolicStatistics::NamespaceInfo NamespaceInfo;

const time::microseconds HyperbolicStatistics::MEASUREMENTS_LIFETIME = time::seconds(30);

const fib::NextHop*
HyperbolicStatistics::getBestNextHop(const fib::Entry& fibEntry, const Face& inFace)
{
  NFD_LOG_INFO("Looking for best face for " << fibEntry.getPrefix());

  typedef std::pair<const fib::NextHop*, time::microseconds> NextHopRttPair;

  NextHopRttPair candidate = std::make_pair(nullptr, RttStat::RTT_NO_MEASUREMENT);

  for (const fib::NextHop& hop : fibEntry.getNextHops()) {

    NFD_LOG_DEBUG("Checking FaceId: " << hop.getFace()->getId() << " for " << fibEntry.getPrefix());

    // Don't forward Interest back to the incoming face
    if (hop.getFace()->getId() == inFace.getId()) {
      NFD_LOG_DEBUG("Not considering FaceId: " << inFace.getId() << " face == inFace");
      continue;
    }

    FaceInfo& info = getOrCreateFaceInfo(fibEntry, *hop.getFace());

    if (info.hasTimedOut) {
      NFD_LOG_DEBUG("Interest sent to " << hop.getFace()->getId() << " for " << fibEntry.getPrefix() <<
                    " previously timed-out");
    }

    if (candidate.first == nullptr) {
      if (info.rtt == RttStat::RTT_NO_MEASUREMENT) {
        NFD_LOG_DEBUG("Considering face " << hop.getFace()->getId() << " with no previous srtt measurement");
      }
      else {
        NFD_LOG_DEBUG("Considering face " << hop.getFace()->getId() << " with srtt measurement " << info.rtt);
      }

      candidate = std::make_pair(&hop, info.rtt);
    }
    else if (!info.hasTimedOut && info.rtt < candidate.second) {
      NFD_LOG_DEBUG("Considering face " << hop.getFace()->getId() << " with srtt measurement " << info.rtt << " < " << candidate.second);
      candidate = std::make_pair(&hop, info.rtt);
    }
  }

  return candidate.first;
}

void
HyperbolicStatistics::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                            const Face& inFace,
                                            const Data& data)
{
  NFD_LOG_TRACE("HyperbolicStatistics::beforeSatisfyInterest");

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
  FaceInfo& face = info->faceInfo.at(inFace.getId());

  m_rttRecorder.record(face, pitEntry, me->getName(), inFace);

  // Cancel timeout
  scheduler::cancel(face.timeoutEventId);
}

void
HyperbolicStatistics::afterForwardInterest(const fib::Entry& fibEntry, const Face& face)
{
  FaceInfo& info = getOrCreateFaceInfo(fibEntry, face);

  ndn::time::milliseconds timeout = time::seconds(1);

  info.timeoutEventId = scheduler::schedule(timeout,
      bind(&HyperbolicStatistics::onTimeout, this, fibEntry.getPrefix(), face.getId()));
}

void
HyperbolicStatistics::onTimeout(const ndn::Name& prefix, FaceId faceId)
{
  shared_ptr<NamespaceInfo> info = getNamespaceInfo(prefix);

  if (info == nullptr) {
    NFD_LOG_WARN("FibEntry for " << prefix << " was removed");
    return;
  }

  FaceInfoMap::iterator it = info->faceInfo.find(faceId);

  FaceInfo* record;

  if (it == info->faceInfo.end()) {
    const auto& pair = info->faceInfo.insert(std::make_pair(faceId, FaceInfo()));
    record = &pair.first->second;
  }
  else {
    record = &it->second;
  }

  if (record != nullptr) {
    NFD_LOG_INFO("FaceId: " << faceId << " for " << prefix << " has timed-out");
    record->hasTimedOut = true;
  }
}

FaceInfo&
HyperbolicStatistics::getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  NamespaceInfo& info = getOrCreateNamespaceInfo(fibEntry);

  FaceInfoMap::iterator it = info.faceInfo.find(face.getId());

  if (it == info.faceInfo.end()) {
    FaceInfo record;

    const auto& pair = info.faceInfo.insert(std::make_pair(face.getId(), std::move(record)));
    return pair.first->second;
  }
  else {
    return it->second;
  }
}

NamespaceInfo&
HyperbolicStatistics::getOrCreateNamespaceInfo(const fib::Entry& fibEntry)
{
  shared_ptr<measurements::Entry> me = m_measurements.get(fibEntry);

  // Set or update entry lifetime
  m_measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  return *info;
}

shared_ptr<NamespaceInfo>
HyperbolicStatistics::getNamespaceInfo(const ndn::Name& prefix)
{
  shared_ptr<measurements::Entry> me = m_measurements.get(prefix);

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

FaceInfo::FaceInfo()
{
}

} // namespace experimental
} // namespace fw
} // namespace nfd
