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

#include "tests/test-common.hpp"

#include "fw/experimental/probing-module.hpp"

#include <random>

namespace nfd {
namespace fw {
namespace experimental {
namespace tests {

class TestProbingModule : public ProbingModule
{
public:
  TestProbingModule()
    : ProbingModule(time::seconds(10))
  {
  }

  shared_ptr<Face>
  getFaceToProbe(const Face& inFace,
                 const Interest& interest,
                 shared_ptr<fib::Entry> fibEntry,
                 const Face& faceUsed)
  {
    return nullptr;
  }

  void
  scheduleProbe(shared_ptr<fib::Entry> fibEntry,
                const time::milliseconds& interval)
  {
  }

  bool
  isProbingNeeded(shared_ptr<fib::Entry> fibEntry)
  {
    return false;
  }

  void
  afterProbe(shared_ptr<fib::Entry> fibEntry)
  {
  }
};

BOOST_AUTO_TEST_SUITE(AsfProbing)

BOOST_AUTO_TEST_CASE(RandomSeed)
{
  TestProbingModule probe;

  // Get initial number
  probe.setGlobalSeed(128);
  probe.setNodeUid("/ndn/site/routerA");

  double first = probe.getRandomNumber(0, 10);

  // Should a new number
  double second = probe.getRandomNumber(0, 10);

  BOOST_CHECK(second != first);

  // Reset environment but change Node UID
  probe.setGlobalSeed(128);
  probe.setNodeUid("/ndn/site/routerB");

  double third = probe.getRandomNumber(0, 10);

  BOOST_CHECK(third != first);

  // Reset environment to same as start of test; should get the same number
  probe.setGlobalSeed(128);
  probe.setNodeUid("/ndn/site/routerA");

  double fourth = probe.getRandomNumber(0, 10);

  BOOST_CHECK_EQUAL(first, fourth);
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace tests
} // namespace experimental
} // namespace fw
} // namespace nfd
