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

#ifndef NFD_DAEMON_FW_EXPERIMENTAL_STRATEGY_MEASUREMENTS_HPP
#define NFD_DAEMON_FW_EXPERIMENTAL_STRATEGY_MEASUREMENTS_HPP

#include "../rtt-recorder.hpp"
#include "fw/strategy-info.hpp"

namespace nfd {
namespace fw {
namespace experimental {

/** \brief Strategy information for each face in a namespace
*/
class FaceInfo : public RttStat
{
public:
  FaceInfo();

public:
  scheduler::EventId timeoutEventId;
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
  {
  }

  static constexpr int
  getTypeId()
  {
    return 9898;
  }

public:
  FaceInfoMap faceInfo;
  bool isProbingNeeded;
  bool hasFirstProbeBeenScheduled;
};

} // namespace experimental
} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_EXPERIMENTAL_STRATEGY_MEASUREMENTS_HPP
