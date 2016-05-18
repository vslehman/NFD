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

#ifndef NFD_DAEMON_FW_ASF_PROBING_HPP
#define NFD_DAEMON_FW_ASF_PROBING_HPP

#include "common.hpp"
#include "asf-statistics.hpp"
#include "asf-measurements.hpp"
#include "probing-module.hpp"
#include "table/fib.hpp"
#include "table/pit.hpp"

namespace ndn {
class Face;
}

namespace nfd {
namespace fw {

/** \brief ASF Probing Module
 */
class AsfProbingModule : public ProbingModule
{
public:
  AsfProbingModule(AsfStatistics& stats);

  virtual void
  scheduleProbe(shared_ptr<fib::Entry> fibEntry,
                const time::milliseconds& interval) override;

  virtual shared_ptr<Face>
  getFaceToProbe(const Face& inFace,
                 const Interest& interest,
                 shared_ptr<fib::Entry> fibEntry,
                 const Face& faceUsed) override;

  bool
  isProbingNeeded(shared_ptr<fib::Entry> fibEntry) override;

  void
  afterProbe(shared_ptr<fib::Entry> fibEntry) override;

private:
  // Used to associate FaceInfo with the face in a NextHop
  typedef std::pair<shared_ptr<FaceInfo>, shared_ptr<Face>> FaceInfoFacePair;
  typedef std::function<bool(FaceInfoFacePair, FaceInfoFacePair)> FaceInfoPredicate;
  typedef std::set<FaceInfoFacePair, FaceInfoPredicate> FaceInfoFacePairSet;

  shared_ptr<Face>
  getFaceBasedOnProbability(const FaceInfoFacePairSet& rankedFaces);

  typedef std::function<double(uint64_t /*rank*/, uint64_t /*rankSum*/, uint64_t /*nFaces*/)> ProbabilityFunction;

  const ProbabilityFunction m_probabilityFunction;

private:
  AsfStatistics& m_stats;

  static const time::seconds DEFAULT_PROBING_INTERVAL;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_EXPERIMENTAL_ASF_PROBING_HPP
