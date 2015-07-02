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

#include "probing-module.hpp"
#include "random.hpp"

#include <functional>
#include <random>
#include <boost/random/uniform_real_distribution.hpp>

namespace nfd {
namespace fw {
namespace experimental {

NFD_LOG_INIT("ProbingModule");

double
ProbingModule::getRandomNumber(double start, double end)
{
  if (!m_isGlobalSeedInitialized || !m_isNodeUidInitialized) {
    NFD_LOG_WARN("Global seed or Node UID has not been initialized!");
    BOOST_ASSERT(false);
  }

  boost::random::uniform_real_distribution<double> distribution(start, end);

  return distribution(getGlobalRng());
}

void
ProbingModule::setGlobalSeed(uint64_t seed)
{
  m_globalSeed = seed;
  m_isGlobalSeedInitialized = true;

  updateOverallSeed();
}

void
ProbingModule::setNodeUid(const std::string& uid)
{
  m_nodeUid = uid;
  m_isNodeUidInitialized = true;

  updateOverallSeed();
}

void
ProbingModule::updateOverallSeed()
{
  std::hash<std::string> hashFunction;
  size_t hash = hashFunction(m_nodeUid);

  size_t overallSeed = hash * m_globalSeed;

  getGlobalRng().seed(overallSeed);
}

} // namespace nfd
} // namespace fw
} // namespace experimental
