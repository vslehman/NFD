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

#include "hyperbolic-probing.hpp"

#include "hyperbolic-statistics.hpp"
#include "../rtt-recorder.hpp"
#include "core/scheduler.hpp"

#include <random>

namespace nfd {
namespace fw {
namespace experimental {

NFD_LOG_INIT("HyperbolicProbing");

//==============================================================================
// Random Number Generator
//------------------------------------------------------------------------------
double
getRandomNumber(double start, double end)
{
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<> distribution(start, end);

  return distribution(generator);
}


//==============================================================================
// Probability Function #1
//------------------------------------------------------------------------------
// p = n + 1 - j ; n: # faces
//     ---------
//     sum(ranks)
double
getProbabilityFunction1(uint64_t rank, uint64_t rankSum, uint64_t nFaces)
{
  return ((double)(nFaces + 1 - rank))/rankSum;
}

//==============================================================================
// Probability Function #2
//------------------------------------------------------------------------------
// p =  2^(n - j)  ; n: # faces
//     -----------
//      (2^n) - 1
double
getProbabilityFunction2(uint64_t rank, uint64_t rankSum, uint64_t nFaces)
{
  return std::pow(2, nFaces - rank)/(std::pow(2, nFaces) - 1);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const time::seconds HyperbolicProbingModule::DEFAULT_PROBING_INTERVAL = time::seconds(10);

HyperbolicProbingModule::HyperbolicProbingModule(HyperbolicStatistics& stats)
  : ProbingModule(DEFAULT_PROBING_INTERVAL)
  , m_probabilityFunction(&getProbabilityFunction1)
  , m_stats(stats)
{
  BOOST_ASSERT(m_probabilityFunction != nullptr);
}

void
HyperbolicProbingModule::scheduleProbe(shared_ptr<fib::Entry> fibEntry,
                                       const time::milliseconds& interval)
{
  ndn::Name prefix = fibEntry->getPrefix();

  NFD_LOG_DEBUG("Scheduling probe for " << prefix);

  // Set the probing flag for the namespace to true after PROBING_INTERVAL
  // period of time
  scheduler::schedule(interval, [this, prefix] () {
    shared_ptr<NamespaceInfo> info = this->m_stats.getNamespaceInfo(prefix);

    if (info == nullptr) {
      NFD_LOG_DEBUG("FibEntry for " << prefix << " has been removed");
      return;
    }
    else {
      NFD_LOG_TRACE("Probing is due for " << prefix);
      info->isProbingNeeded = true;
    }
  });
}

shared_ptr<Face>
HyperbolicProbingModule::getFaceToProbe(const Face& inFace,
                                        const Interest& interest,
                                        shared_ptr<fib::Entry> fibEntry,
                                        const Face& faceUsed)
{
  NFD_LOG_TRACE("Looking for face to probe " << fibEntry->getPrefix());

  FaceInfoFacePairSet rankedFaces(
    [] (FaceInfoFacePair lhs, FaceInfoFacePair rhs) -> bool {
      // Sort by RTT
      // If a face has timed-out, rank it behind non-timed-out faces
      if (lhs.first->rtt != RttStat::RTT_TIMEOUT && rhs.first->rtt == RttStat::RTT_TIMEOUT) {
        return true;
      }
      else if (lhs.first->rtt == RttStat::RTT_TIMEOUT && rhs.first->rtt != RttStat::RTT_TIMEOUT) {
        return false;
      }
      else {
        return lhs.first->srtt < rhs.first->srtt;
      }
  });

  // Put eligible faces into rankedFaces. If a face does not have an RTT measurement,
  // pick the face for probing immediately
  for (const fib::NextHop& hop : fibEntry->getNextHops()) {

    // Don't send probe Interest back to the incoming face or use the same face
    // as the forwarded Interest
    if (hop.getFace()->getId() == inFace.getId() ||
        hop.getFace()->getId() == faceUsed.getId())
    {
      continue;
    }

    FaceInfo& info = m_stats.getOrCreateFaceInfo(*fibEntry, *hop.getFace());

    // If no RTT has been recorded, probe this face
    if (info.srtt == RttStat::RTT_NO_MEASUREMENT) {
      NFD_LOG_DEBUG("Found face to probe with no RTT measurement");
      return hop.getFace();
    }

    // Add FaceInfo to container sorted by RTT
    rankedFaces.insert(std::make_pair(make_shared<FaceInfo>(info), hop.getFace()));
  }

  if (rankedFaces.empty()) {
    NFD_LOG_DEBUG("Unable to find a face to probe");
    return nullptr;
  }

  return getFaceBasedOnProbability(rankedFaces);
}

bool
HyperbolicProbingModule::isProbingNeeded(shared_ptr<fib::Entry> fibEntry)
{
  // Return the probing flag status for a namespace
  NamespaceInfo& info = m_stats.getOrCreateNamespaceInfo(*fibEntry);

  // If a first probe has not been scheduled for a namespace
  if (!info.hasFirstProbeBeenScheduled) {
    // Schedule first probe at random interval
    uint64_t interval = getRandomNumber(0, 5000);

    NFD_LOG_DEBUG("Scheduling first probe for " << fibEntry->getPrefix() <<
                  " in " << interval << " ms");

    // If the probe is scheduled for now, probe immediately
    if (interval == 0) {
      info.isProbingNeeded = true;
    }
    else {
      scheduleProbe(fibEntry, time::milliseconds(interval));
    }

    info.hasFirstProbeBeenScheduled = true;
  }

  return info.isProbingNeeded;
}

void
HyperbolicProbingModule::afterProbe(shared_ptr<fib::Entry> fibEntry)
{
  // After probing is done, need to set probing flag to false and
  // schedule another future probe
  NamespaceInfo& info = m_stats.getOrCreateNamespaceInfo(*fibEntry);
  info.isProbingNeeded = false;

  scheduleProbe(fibEntry);
}

shared_ptr<Face>
HyperbolicProbingModule::getFaceBasedOnProbability(const FaceInfoFacePairSet& rankedFaces)
{
  double randomNumber = getRandomNumber(0, 1);
  NFD_LOG_TRACE("randomNumber: " << randomNumber);

  uint64_t rankSum = ((rankedFaces.size() + 1)*(rankedFaces.size()))/2;
  NFD_LOG_TRACE("rankSum: " << rankSum);

  uint64_t rank = 1;

  double offset = 0;

  for (const FaceInfoFacePair pair : rankedFaces) {
    double probability = m_probabilityFunction(rank++, rankSum, rankedFaces.size());
    NFD_LOG_TRACE("probability: " << probability << " for FaceId: " << pair.second->getId());

    // Is the random number within the bounds of this face's probability + the previous faces'
    // probability?
    //
    // e.g. (FaceId: 1, p=0.5), (FaceId: 2, p=0.33), (FaceId: 3, p=0.17)
    //      randomNumber = 0.92
    //
    //      The face with FaceId: 3 should be picked
    //      (0.68 < 0.5 + 0.33 + 0.17) == true
    //
    if (randomNumber < offset + probability) {
      NFD_LOG_DEBUG("Found face to probe");
      return pair.second;
    }

    offset += probability;
    NFD_LOG_TRACE("offset: " << offset);
  }

  NFD_LOG_FATAL("Unable to find face to probe using probability!");
  throw std::runtime_error("Unable to find face to probe using probability!");
}

} // namespace experimental
} // namespace fw
} // namespace nfd
