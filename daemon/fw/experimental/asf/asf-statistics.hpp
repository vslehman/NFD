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

#ifndef NFD_DAEMON_FW_EXPERIMENTAL_ASF_STATISTICS_HPP
#define NFD_DAEMON_FW_EXPERIMENTAL_ASF_STATISTICS_HPP

#include "../rtt-recorder.hpp"
#include "../statistics-module.hpp"
#include "strategy-measurements.hpp"
#include "fw/strategy-info.hpp"
#include "table/pit.hpp"

namespace nfd {

class MeasurementsAccessor;

namespace fib {
class Entry;
class NextHop;
}

namespace fw {
namespace experimental {

class AsfStatistics : public StatisticsModule
{
public:
  AsfStatistics(MeasurementsAccessor& measurements)
    : m_measurements(measurements)
    , m_isLearningPeriodEnabled(false)
  {
  }

  void
  beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry, const Face& inFace, const Data& data) DECL_OVERRIDE;

  const shared_ptr<Face>
  getBestFace(const fib::Entry& fibEntry, const Face& inFace) DECL_OVERRIDE;

  virtual void
  afterForwardInterest(const Interest& interest,
                       const fib::Entry& fibEntry,
                       const Face& face) DECL_OVERRIDE;

  void
  enableLearningPeriod()
  {
    m_isLearningPeriodEnabled = true;
  }

public:
  FaceInfo*
  getFaceInfo(const fib::Entry& fibEntry, const Face& face);

  NamespaceInfo&
  getOrCreateNamespaceInfo(const fib::Entry& fibEntry);

  shared_ptr<NamespaceInfo>
  getNamespaceInfo(const ndn::Name& prefix);

private:
  void
  onTimeout(const ndn::Name& interestName, FaceId faceId);

  FaceInfo&
  getOrCreateFaceInfo(const fib::Entry& fibEntry, const Face& face);

  const shared_ptr<Face>
  getBestFaceDuringLearningPeriod(const fib::Entry& fibEntry, const Face& inFace);

private:
  RttRecorder m_rttRecorder;
  MeasurementsAccessor& m_measurements;

  bool m_isLearningPeriodEnabled;

  static const time::microseconds MEASUREMENTS_LIFETIME;
};

} // namespace experimental
} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_EXPERIMENTAL_ASF_STATISTICS_HPP
