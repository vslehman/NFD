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

#include "rtt-recorder.hpp"
#include "fw/strategy-info.hpp"

namespace nfd {
namespace fw {

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

typedef uint64_t FaceId;
typedef std::unordered_map<FaceId, FaceInfo> FaceInfoMap;

/** \brief stores stategy information about each face in this namespace
 */
class NamespaceInfo : public StrategyInfo
{
public:
  NamespaceInfo()
    : isProbingNeeded(false)
    , hasFirstProbeBeenScheduled(false)
    , isLearningPeriod(false)
  {
    startLearningPeriod();
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
  expireFaceInfo(FaceId faceId);

  void
  extendFaceInfoLifetime(FaceInfo& info, const Face& face);

private:
  void
  startLearningPeriod();

  void
  endLearningPeriod();

public:
  FaceInfoMap faceInfoMap;
  bool isProbingNeeded;
  bool hasFirstProbeBeenScheduled;

  shared_ptr<Face> lastUsedFace;
  bool isLearningPeriod;

  static const time::seconds LEARNING_PERIOD;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_ASF_MEASUREMENTS_HPP
