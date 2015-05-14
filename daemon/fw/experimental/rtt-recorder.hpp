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

#ifndef NFD_DAEMON_FW_EXPERIMENTAL_RTT_RECORDER_HPP
#define NFD_DAEMON_FW_EXPERIMENTAL_RTT_RECORDER_HPP

#include "table/pit.hpp"

namespace nfd {
namespace fw {
namespace experimental {

typedef double Rtt;

class RttStat
{
public:
  RttStat()
    : srtt(RTT_NO_MEASUREMENT)
    , rtt(RTT_NO_MEASUREMENT)
    , hasTimedOut(false)
  {
  }

public:
  Rtt srtt;
  Rtt rtt;
  bool hasTimedOut;

  static const Rtt RTT_TIMEOUT;
  static const Rtt RTT_NO_MEASUREMENT;
};

class RttRecorder
{
public:
  void
  record(RttStat& stat,
         const shared_ptr<pit::Entry> pitEntry,
         const ndn::Name& prefix,
         const Face& inFace);

public:
  typedef time::microseconds Duration;

private:
  double
  computeSrtt(double previousSrtt, double currentRtt);

  static const double ALPHA;

};

} // namespace experimental
} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_EXPERIMENTAL_RTT_RECORDER_HPP
