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

#ifndef NFD_DAEMON_FW_ASF_MEASUREMENTS_HPP
#define NFD_DAEMON_FW_ASF_MEASUREMENTS_HPP

#include "fw/rtt-estimator.hpp"

#include "fw/strategy-info.hpp"
#include "table/measurements-accessor.hpp"

namespace nfd {
namespace fw {

class RttStats
{
public:
  typedef double Rtt;

  RttStats()
    : m_srtt(RTT_NO_MEASUREMENT)
    , m_rtt(RTT_NO_MEASUREMENT)
  {
  }

  void
  addRttMeasurement(RttEstimator::Duration& durationRtt);

  void
  recordTimeout()
  {
    m_rtt = RTT_TIMEOUT;
  }

  Rtt
  getRtt() const
  {
    return m_rtt;
  }

  Rtt
  getSrtt() const
  {
    return m_srtt;
  }

  RttEstimator::Duration
  computeRto() const
  {
    return m_rttEstimator.computeRto();
  }

private:
  static double
  computeSrtt(Rtt previousSrtt, Rtt currentRtt);

public:
  static const Rtt RTT_TIMEOUT;
  static const Rtt RTT_NO_MEASUREMENT;

private:
  Rtt m_srtt;
  Rtt m_rtt;
  RttEstimator m_rttEstimator;

  static const double ALPHA;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/** \brief Strategy information for each face in a namespace
*/
class FaceInfo
{
public:
  FaceInfo();

  ~FaceInfo();

  void
  setTimeoutEvent(const scheduler::EventId& id, const ndn::Name& interestName);

  bool
  isTimeoutScheduled() const
  {
    return m_isTimeoutScheduled;
  }

  void
  cancelTimeoutEvent();

  const ndn::Name&
  getLastInterestName() const
  {
    return m_lastInterestName;
  }

  bool
  doesNameMatchLastInterest(const ndn::Name& name);

  void
  recordRtt(const shared_ptr<pit::Entry> pitEntry, const Face& inFace);

  void
  recordTimeout(const ndn::Name& interestName);

  RttEstimator::Duration
  computeRto() const
  {
    return m_rttStats.computeRto();
  }

  RttStats::Rtt
  getRtt() const
  {
    return m_rttStats.getRtt();
  }

  RttStats::Rtt
  getSrtt() const
  {
    return m_rttStats.getSrtt();
  }

  typedef std::unordered_map<nfd::face::FaceId, FaceInfo> Table;

public:
  // Timeout associated with measurement
  scheduler::EventId measurementExpirationId;

  static const time::seconds MEASUREMENT_LIFETIME;

private:
  RttStats m_rttStats;
  ndn::Name m_lastInterestName;

  // Timeout associated with Interest
  scheduler::EventId m_timeoutEventId;
  bool m_isTimeoutScheduled;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/** \brief stores stategy information about each face in this namespace
 */
class NamespaceInfo : public StrategyInfo
{
public:
  NamespaceInfo()
    : isProbingNeeded(false)
    , hasFirstProbeBeenScheduled(false)
  {
  }

  static constexpr int
  getTypeId()
  {
    return 9898;
  }

  FaceInfo&
  getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face);

  FaceInfo*
  getFaceInfo(const fib::Entry& fibEntry, const Face& face);

  void
  expireFaceInfo(nfd::face::FaceId faceId);

  void
  extendFaceInfoLifetime(FaceInfo& info, const Face& face);

  FaceInfo&
  get(nfd::face::FaceId faceId)
  {
    return m_fit.at(faceId);
  }

  FaceInfo::Table::iterator
  find(nfd::face::FaceId faceId)
  {
    return m_fit.find(faceId);
  }

  FaceInfo::Table::iterator
  end()
  {
    return m_fit.end();
  }

  const FaceInfo::Table::iterator
  insert(nfd::face::FaceId faceId)
  {
    const auto& pair = m_fit.insert(std::make_pair(faceId, FaceInfo()));
    return pair.first;
  }

public:
  bool isProbingNeeded;
  bool hasFirstProbeBeenScheduled;

private:
  FaceInfo::Table m_fit;
};

/** \brief Helper class to retrieve and create strategy measurements
 */
class AsfMeasurements
{
public:
  AsfMeasurements(MeasurementsAccessor& measurements)
    : m_measurements(measurements)
  {
  }

  FaceInfo*
  getFaceInfo(const fib::Entry& fibEntry, const Face& face);

  FaceInfo&
  getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face);

  shared_ptr<NamespaceInfo>
  getNamespaceInfo(const ndn::Name& prefix);

  NamespaceInfo&
  getOrCreateNamespaceInfo(const fib::Entry& fibEntry);

  void
  extendLifetime(shared_ptr<measurements::Entry> me);

private:
  MeasurementsAccessor& m_measurements;

  static const time::microseconds MEASUREMENTS_LIFETIME;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_ASF_MEASUREMENTS_HPP
