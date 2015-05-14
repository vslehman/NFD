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

#include "rtt-recorder.hpp"

namespace nfd {
namespace fw {
namespace experimental {

NFD_LOG_INIT("RttRecorder");

const Rtt RttStat::RTT_NO_MEASUREMENT = time::microseconds::max().count();

const double RttRecorder::ALPHA = 0.125;

void
RttRecorder::record(RttStat& stat,
                    const shared_ptr<pit::Entry> pitEntry,
                    const ndn::Name& prefix,
                    const Face& inFace)
{
  NFD_LOG_TRACE("Recording RTT");

  // Calculate RTT
  pit::OutRecordCollection::const_iterator outRecord = pitEntry->getOutRecord(inFace);
  time::steady_clock::Duration steadyRtt = time::steady_clock::now() - outRecord->getLastRenewed();
  RttRecorder::Duration durationRtt = time::duration_cast<RttRecorder::Duration>(steadyRtt);

  stat.rtt = static_cast<Rtt>(durationRtt.count());

  // Assign ewma of RTT to face
  stat.srtt = computeSrtt(stat.srtt, stat.rtt);

  stat.hasTimedOut = false;

  NFD_LOG_INFO("Recording RTT for " << prefix << " FaceId: " << inFace.getId()
                                              << " RTT: "    << stat.rtt
                                              << " SRTT: "   << stat.srtt);
}

Rtt
RttRecorder::computeSrtt(Rtt previousSrtt, Rtt currentRtt)
{
  if (previousSrtt == RttStat::RTT_NO_MEASUREMENT) {
    return currentRtt;
  }

  Rtt srtt = ALPHA*currentRtt + (1 - ALPHA)*previousSrtt;

  return srtt;
}

} // namespace experimental
} // namespace fw
} // namespace nfd
