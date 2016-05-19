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

#include "asf-measurements.hpp"

namespace nfd {
namespace fw {

NFD_LOG_INIT("StrategyMeasurements");

const time::seconds FaceInfo::MEASUREMENT_LIFETIME = time::seconds(300);

FaceInfo::FaceInfo()
  : m_isTimeoutScheduled(false)
{
}

FaceInfo::~FaceInfo()
{
  cancelTimeoutEvent();
  scheduler::cancel(this->measurementExpirationId);
}

void
FaceInfo::setTimeoutEvent(const scheduler::EventId& id, const ndn::Name& interestName)
{
  if (!m_isTimeoutScheduled) {
    m_timeoutEventId = id;
    m_isTimeoutScheduled = true;
    m_lastInterestName = interestName;
  }
  else {
    throw std::runtime_error("Tried to schedule a timeout for a face that already has a timeout scheduled.");
  }
}

void
FaceInfo::cancelTimeoutEvent()
{
  scheduler::cancel(m_timeoutEventId);
  m_isTimeoutScheduled = false;
}

bool
FaceInfo::doesNameMatchLastInterest(const ndn::Name& name)
{
  return m_lastInterestName.isPrefixOf(name);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

FaceInfo*
NamespaceInfo::getFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  FaceInfoMap::iterator it = faceInfoMap.find(face.getId());

  if (it != faceInfoMap.end()) {
    return &it->second;
  }
  else {
    return nullptr;
  }
}

FaceInfo&
NamespaceInfo::getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  FaceInfoMap::iterator it = faceInfoMap.find(face.getId());

  FaceInfo* info;

  if (it == faceInfoMap.end()) {
    const auto& pair = faceInfoMap.insert(std::make_pair(face.getId(), FaceInfo()));
    info = &pair.first->second;

    extendFaceInfoLifetime(*info, face);
  }
  else {
    info = &it->second;
  }

  return *info;
}

void
NamespaceInfo::expireFaceInfo(FaceId faceId)
{
  NFD_LOG_DEBUG("Measurements for FaceId: " << faceId << " have expired");
  faceInfoMap.erase(faceId);
}

void
NamespaceInfo::extendFaceInfoLifetime(FaceInfo& info, const Face& face)
{
  NFD_LOG_DEBUG("Extending FaceInfo lifetime for Face ID: " << face.getId() <<
                " with last Interest name: " << info.getLastInterestName());

  // Cancel previous expiration
  scheduler::cancel(info.measurementExpirationId);

  // Refresh measurement
  scheduler::EventId id = scheduler::schedule(FaceInfo::MEASUREMENT_LIFETIME,
    bind(&NamespaceInfo::expireFaceInfo, this, face.getId()));

  info.measurementExpirationId = id;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const time::microseconds AsfMeasurements::MEASUREMENTS_LIFETIME = time::seconds(300);

FaceInfo*
AsfMeasurements::getFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  NamespaceInfo& info = getOrCreateNamespaceInfo(fibEntry);

  return info.getFaceInfo(fibEntry, face);
}

FaceInfo&
AsfMeasurements::getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  NamespaceInfo& info = getOrCreateNamespaceInfo(fibEntry);

  return info.getOrCreateFaceInfo(fibEntry, face);
}

shared_ptr<NamespaceInfo>
AsfMeasurements::getNamespaceInfo(const ndn::Name& prefix)
{
  shared_ptr<measurements::Entry> me = m_measurements.findLongestPrefixMatch(prefix);

  if (me == nullptr) {
    return nullptr;
  }

  // Set or update entry lifetime
  m_measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  return info;
}

NamespaceInfo&
AsfMeasurements::getOrCreateNamespaceInfo(const fib::Entry& fibEntry)
{
  shared_ptr<measurements::Entry> me = m_measurements.get(fibEntry);

  // Set or update entry lifetime
  m_measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  return *info;
}

} // namespace fw
} // namespace nfd
