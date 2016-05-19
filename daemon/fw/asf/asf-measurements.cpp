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

const RttStat::Rtt RttStat::RTT_TIMEOUT = -1;
const RttStat::Rtt RttStat::RTT_NO_MEASUREMENT = 0;
const double RttStat::ALPHA = 0.125;

double
RttStat::computeSrtt(Rtt previousSrtt, Rtt currentRtt)
{
  if (previousSrtt == RTT_NO_MEASUREMENT) {
    return currentRtt;
  }

  Rtt srtt = ALPHA * currentRtt + (1 - ALPHA) * previousSrtt;

  return srtt;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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

void
FaceInfo::recordRtt(const shared_ptr<pit::Entry> pitEntry, const Face& inFace)
{
  // Calculate RTT
  pit::OutRecordCollection::const_iterator outRecord = pitEntry->getOutRecord(inFace);
  time::steady_clock::Duration steadyRtt = time::steady_clock::now() - outRecord->getLastRenewed();
  RttStat::Duration durationRtt = time::duration_cast<RttStat::Duration>(steadyRtt);

  this->rtt = static_cast<RttStat::Rtt>(durationRtt.count());

  this->rttEstimator.addMeasurement(time::duration_cast<RttEstimator::Duration>(durationRtt));

  // Assign ewma of RTT to face
  this->srtt = computeSrtt(this->srtt, this->rtt);

  NFD_LOG_TRACE("Recording RTT for FaceId: " << inFace.getId()
                                              << " RTT: "    << this->rtt
                                              << " SRTT: "   << this->srtt);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

FaceInfo*
NamespaceInfo::getFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  FaceInfo::Table::iterator it = m_fit.find(face.getId());

  if (it != m_fit.end()) {
    return &it->second;
  }
  else {
    return nullptr;
  }
}

FaceInfo&
NamespaceInfo::getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face)
{
  FaceInfo::Table::iterator it = m_fit.find(face.getId());

  FaceInfo* info;

  if (it == m_fit.end()) {
    const auto& pair = m_fit.insert(std::make_pair(face.getId(), FaceInfo()));
    info = &pair.first->second;

    extendFaceInfoLifetime(*info, face);
  }
  else {
    info = &it->second;
  }

  return *info;
}

void
NamespaceInfo::expireFaceInfo(nfd::face::FaceId faceId)
{
  NFD_LOG_DEBUG("Measurements for FaceId: " << faceId << " have expired");
  m_fit.erase(faceId);
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
  extendLifetime(me);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  return info;
}

NamespaceInfo&
AsfMeasurements::getOrCreateNamespaceInfo(const fib::Entry& fibEntry)
{
  shared_ptr<measurements::Entry> me = m_measurements.get(fibEntry);

  // Set or update entry lifetime
  extendLifetime(me);

  shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
  BOOST_ASSERT(info != nullptr);

  return *info;
}

void
AsfMeasurements::extendLifetime(shared_ptr<measurements::Entry> me)
{
  if (me != nullptr) {
    m_measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);
  }
}

} // namespace fw
} // namespace nfd
