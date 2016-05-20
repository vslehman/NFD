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

#include "fw/asf-strategy.hpp"

#include "tests/test-common.hpp"
#include "topology-tester.hpp"

namespace nfd {
namespace fw {
namespace tests {

using namespace nfd::tests;

BOOST_AUTO_TEST_SUITE(Fw)
BOOST_FIXTURE_TEST_SUITE(TestAsfStrategy, UnitTestTimeFixture)

class TwoLaptopsFixture : public UnitTestTimeFixture
{
protected:
  TwoLaptopsFixture()
  {
    /*
     *                  +---------+
     *           +----->| router1 |<------+
     *           |      +---------+       |
     *      10ms |                        | 10ms
     *           v                        v
     *      +---------+              +---------+
     *      | laptopA |              | laptopB |
     *      +---------+              +---------+
     *           ^                        ^
     *     100ms |                        |
     *           |      +---------+       | 100ms
     *           +----->| router2 |<------+
     *                  +---------+
     */

    router1 = topo.addForwarder("R1");
    router2 = topo.addForwarder("R2");
    laptopA = topo.addForwarder("A");
    laptopB = topo.addForwarder("B");

    topo.setStrategy<fw::AsfStrategy>(router1);
    topo.setStrategy<fw::AsfStrategy>(router2);
    topo.setStrategy<fw::AsfStrategy>(laptopA);
    topo.setStrategy<fw::AsfStrategy>(laptopB);

    // Router 1 links
    linkA = topo.addLink("R1A", time::milliseconds(10), {router1, laptopA});
    linkB = topo.addLink("R1B", time::milliseconds(10), {router1, laptopB});

    // Router 2 links
    linkC = topo.addLink("R2A", time::milliseconds(100), {router2, laptopA});
    linkD = topo.addLink("R2B", time::milliseconds(100), {router2, laptopB});
  }

protected:
  TopologyTester topo;

  TopologyNode router1;
  TopologyNode router2;
  TopologyNode laptopA;
  TopologyNode laptopB;
  shared_ptr<TopologyLink> linkA;
  shared_ptr<TopologyLink> linkB;
  shared_ptr<TopologyLink> linkC;
  shared_ptr<TopologyLink> linkD;
};

BOOST_FIXTURE_TEST_CASE(Basic, TwoLaptopsFixture)
{
  topo.registerPrefix(laptopA, linkA->getFace(laptopA), "ndn:/hr/B", 10);
  topo.registerPrefix(laptopA, linkC->getFace(laptopA), "ndn:/hr/B", 5);

  topo.registerPrefix(router1, linkB->getFace(router1), "ndn:/hr/B");

  topo.registerPrefix(router2, linkD->getFace(router2), "ndn:/hr/B");

  shared_ptr<TopologyAppLink> consumer = topo.addAppFace("c", laptopA);
  topo.addIntervalConsumer(consumer->getClientFace(), "ndn:/hr/B", time::seconds(1), 30);

  shared_ptr<TopologyAppLink> producer = topo.addAppFace("p", laptopB, "ndn:/hr/B");
  topo.addEchoProducer(producer->getClientFace());

  this->advanceClocks(time::milliseconds(1), time::seconds(30));

  // ASF should use the Face to router2 because it has lower routing cost.
  // After 5 seconds, a probe Interest should be sent to the Face to router1,
  // and the probe should return Data quicker. ASF should then use the Face
  // to router1 to forward the remaining Interests.
  BOOST_CHECK_EQUAL(consumer->getForwarderFace().getCounters().nOutData, 30);
  BOOST_CHECK_GE(linkA->getFace(laptopA).getCounters().nOutInterests, 24);
  BOOST_CHECK_LE(linkC->getFace(laptopA).getCounters().nOutInterests, 6);

  // If the link from laptopA to router1 fails, ASF should start using the Face
  // to router2 again.
  linkA->fail();

  topo.addIntervalConsumer(consumer->getClientFace(), "ndn:/hr/B", time::seconds(1), 30);
  this->advanceClocks(time::milliseconds(1), time::seconds(30));

  // Only 59 Data because the first Interest to router1 after the failure should timeout
  BOOST_CHECK_EQUAL(consumer->getForwarderFace().getCounters().nOutData, 59);
  BOOST_CHECK_LE(linkA->getFace(laptopA).getCounters().nOutInterests, 30);
  BOOST_CHECK_GE(linkC->getFace(laptopA).getCounters().nOutInterests, 30);

  // If the link from laptopA to router1 recovers, ASF should probe the Face
  // to router1 and start using it again.
  linkA->recover();

  // Advance time to ensure probing is due
  this->advanceClocks(time::milliseconds(1), time::seconds(10));

  topo.addIntervalConsumer(consumer->getClientFace(), "ndn:/hr/B", time::seconds(1), 30);
  this->advanceClocks(time::milliseconds(1), time::seconds(30));

  BOOST_CHECK_EQUAL(consumer->getForwarderFace().getCounters().nOutData, 89);
  BOOST_CHECK_GE(linkA->getFace(laptopA).getCounters().nOutInterests, 50);
  BOOST_CHECK_LE(linkC->getFace(laptopA).getCounters().nOutInterests, 40);

  // If both links fail, laptopA should forward to the next hop with the lowest cost
  linkA->fail();
  linkC->fail();

  topo.addIntervalConsumer(consumer->getClientFace(), "ndn:/hr/B", time::seconds(1), 30);
  this->advanceClocks(time::milliseconds(1), time::seconds(30));

  BOOST_CHECK_EQUAL(consumer->getForwarderFace().getCounters().nOutData, 89);
  BOOST_CHECK_LE(linkA->getFace(laptopA).getCounters().nOutInterests, 60);
  BOOST_CHECK_GE(linkC->getFace(laptopA).getCounters().nOutInterests, 60);
}

BOOST_FIXTURE_TEST_CASE(Nack, TwoLaptopsFixture)
{
  topo.registerPrefix(laptopA, linkA->getFace(laptopA), "ndn:/hr/B", 10);
  topo.registerPrefix(laptopA, linkC->getFace(laptopA), "ndn:/hr/B", 5);

  topo.registerPrefix(router1, linkB->getFace(router1), "ndn:/hr/B");

  shared_ptr<TopologyAppLink> consumer = topo.addAppFace("c", laptopA);
  topo.addIntervalConsumer(consumer->getClientFace(), "ndn:/hr/B", time::seconds(1), 30);

  shared_ptr<TopologyAppLink> producer = topo.addAppFace("p", laptopB, "ndn:/hr/B");
  topo.addEchoProducer(producer->getClientFace());

  // The strategy should first try to send to Router2. But since Router1 does not have a route for
  // /hr/B, it should return a NO_ROUTE Nack. The strategy should then start using the Face to
  // router1.
  this->advanceClocks(time::milliseconds(10), time::seconds(30));

  BOOST_CHECK_EQUAL(consumer->getForwarderFace().getCounters().nOutData, 29);
  BOOST_CHECK_EQUAL(linkA->getFace(laptopA).getCounters().nOutInterests, 29);

  // Router2 should receive 2 Interests: one for the very first Interest and
  // another from a probe
  BOOST_CHECK_EQUAL(linkC->getFace(laptopA).getCounters().nOutInterests, 2);
}

BOOST_AUTO_TEST_SUITE_END() // TestAsfStrategy
BOOST_AUTO_TEST_SUITE_END() // Fw

} // namespace tests
} // namespace fw
} // namespace nfd
