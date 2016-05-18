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
#include "table/fib.hpp"
#include "table/pit.hpp"

#include "random.hpp"

#include <functional>
#include <random>
#include <boost/random/uniform_real_distribution.hpp>

namespace ndn {
class Face;
}

namespace nfd {
namespace fw {

/** \brief ASF Probing Module
 */
class AsfProbingModule
{
public:
  AsfProbingModule();

  void
  setStatsModule(AsfStatistics& stats);

  virtual void
  scheduleProbe(shared_ptr<fib::Entry> fibEntry,
                const time::milliseconds& interval);

  virtual shared_ptr<Face>
  getFaceToProbe(const Face& inFace,
                 const Interest& interest,
                 shared_ptr<fib::Entry> fibEntry,
                 const Face& faceUsed);

  bool
  isProbingNeeded(shared_ptr<fib::Entry> fibEntry);

  void
  afterProbe(shared_ptr<fib::Entry> fibEntry);

private:
  // Used to associate FaceInfo with the face in a NextHop
  typedef std::pair<shared_ptr<FaceInfo>, shared_ptr<Face>> FaceInfoFacePair;
  typedef std::function<bool(FaceInfoFacePair, FaceInfoFacePair)> FaceInfoPredicate;
  typedef std::set<FaceInfoFacePair, FaceInfoPredicate> FaceInfoFacePairSet;

  shared_ptr<Face>
  getFaceBasedOnProbability(const FaceInfoFacePairSet& rankedFaces);

  typedef std::function<double(uint64_t /*rank*/, uint64_t /*rankSum*/, uint64_t /*nFaces*/)> ProbabilityFunction;

  const ProbabilityFunction m_probabilityFunction;

public:
  void
  setGlobalSeed(uint64_t seed);

  void
  setNodeUid(const std::string& uid);

  void
  setProbingInterval(uint32_t interval)
  {
    m_probingInterval = time::seconds(interval);
  }

  const time::seconds&
  getProbingInterval() const
  {
    return m_probingInterval;
  }

PUBLIC_WITH_TESTS_ELSE_PROTECTED:
  double
  getRandomNumber(double start, double end);

private:
  void
  updateOverallSeed();

protected:
  bool m_isProbingNeeded;

private:
  time::seconds m_probingInterval;

  // Random number generation
  uint64_t m_globalSeed;
  std::string m_nodeUid;

  bool m_isGlobalSeedInitialized;
  bool m_isNodeUidInitialized;

private:
  AsfStatistics* m_stats;

  static const time::seconds DEFAULT_PROBING_INTERVAL;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_EXPERIMENTAL_ASF_PROBING_HPP
