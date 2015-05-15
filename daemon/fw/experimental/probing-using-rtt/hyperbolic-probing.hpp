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

#ifndef NFD_DAEMON_FW_EXPERIMENTAL_HYPERBOLIC_PROBING_HPP
#define NFD_DAEMON_FW_EXPERIMENTAL_HYPERBOLIC_PROBING_HPP

#include "common.hpp"
#include "hyperbolic-statistics.hpp"
#include "strategy-measurements.hpp"
#include "../probing-module.hpp"
#include "table/fib.hpp"
#include "table/pit.hpp"

namespace ndn {
class Face;
}

namespace nfd {
namespace fw {
namespace experimental {

/** \brief Hyperbolic Probing Module version 1
 */
class HyperbolicProbingModule : public ProbingModule
{
public:
  HyperbolicProbingModule(HyperbolicStatistics& stats);

  virtual void
  scheduleProbe(shared_ptr<fib::Entry> fibEntry,
                const time::milliseconds& interval = DEFAULT_PROBING_INTERVAL) DECL_OVERRIDE;

  virtual shared_ptr<Face>
  getFaceToProbe(const Face& inFace,
                 const Interest& interest,
                 shared_ptr<fib::Entry> fibEntry,
                 const Face& faceUsed) DECL_OVERRIDE;

  bool
  isProbingNeeded(shared_ptr<fib::Entry> fibEntry) DECL_OVERRIDE;

  void
  afterProbe(shared_ptr<fib::Entry> fibEntry) DECL_OVERRIDE;

private:
  // Used to associate FaceInfo with the face in a NextHop
  typedef std::pair<shared_ptr<FaceInfo>, const fib::NextHop&> FaceInfoNextHopPair;
  typedef std::function<bool(FaceInfoNextHopPair, FaceInfoNextHopPair)> FaceInfoPredicate;
  typedef std::set<FaceInfoNextHopPair, FaceInfoPredicate> FaceInfoNextHopPairSet;

  shared_ptr<Face>
  getFaceBasedOnProbability(const FaceInfoNextHopPairSet& rankedFaces);

  typedef std::function<double(uint64_t /*rank*/, uint64_t /*rankSum*/, uint64_t /*nFaces*/)> ProbabilityFunction;

  const ProbabilityFunction m_probabilityFunction;

private:
  HyperbolicStatistics& m_stats;

  static const time::seconds DEFAULT_PROBING_INTERVAL;
};

} // namespace experimental
} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_EXPERIMENTAL_HYPERBOLIC_PROBING_HPP
