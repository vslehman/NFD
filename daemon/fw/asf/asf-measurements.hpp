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

class RttStat
{
public:
  RttStat()
    : srtt(RTT_NO_MEASUREMENT)
    , rtt(RTT_NO_MEASUREMENT)
  {
  }

  typedef double Rtt;

  static double
  computeSrtt(Rtt previousSrtt, Rtt currentRtt);

public:
  typedef time::microseconds Duration;

  Rtt srtt;
  Rtt rtt;
  RttEstimator rttEstimator;

  static const Rtt RTT_TIMEOUT;
  static const Rtt RTT_NO_MEASUREMENT;

private:
  static const double ALPHA;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/** \brief Strategy information for each face in a namespace
*/
class FaceInfo : public RttStat
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

public:
  // Timeout associated with measurement
  scheduler::EventId measurementExpirationId;

  static const time::seconds MEASUREMENT_LIFETIME;

private:
  ndn::Name m_lastInterestName;

  // Timeout associated with Interest
  scheduler::EventId m_timeoutEventId;
  bool m_isTimeoutScheduled;
};

typedef std::unordered_map<nfd::face::FaceId, FaceInfo> FaceInfoMap;

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

public:
  FaceInfoMap faceInfoMap;
  bool isProbingNeeded;
  bool hasFirstProbeBeenScheduled;
};

/** \brief Helper class to retrieve and create measurements
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
