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

#include "asf-strategy.hpp"

#include "asf-probing.hpp"
#include "core/logger.hpp"

namespace nfd {
namespace fw {

NFD_LOG_INIT("AsfStrategy");

const Name AsfStrategy::STRATEGY_NAME("ndn:/localhost/nfd/strategy/asf/%FD%01");
const time::seconds AsfStrategy::SUPPRESSION_TIME = time::seconds(2);

NFD_REGISTER_STRATEGY(AsfStrategy);

AsfStrategy::AsfStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder, name)
  , m_stats(this->getMeasurements())
  , m_probe(std::unique_ptr<ProbingModule>(new AsfProbingModule(m_stats)))
  , m_retxSuppression(SUPPRESSION_TIME)
{
}

AsfStrategy::~AsfStrategy()
{
}

void
AsfStrategy::afterReceiveInterest(const Face& inFace,
                                  const Interest& interest,
                                  shared_ptr<fib::Entry> fibEntry,
                                  shared_ptr<pit::Entry> pitEntry)
{
  NFD_LOG_TRACE("Received Interest " << interest.getName() << " nonce=" << interest.getNonce());

  // Should the Interest be suppressed?
  RetxSuppression::Result suppressResult = m_retxSuppression.decide(inFace, interest, *pitEntry);

  switch (suppressResult) {
  case RetxSuppression::NEW:
  case RetxSuppression::FORWARD:
    break;
  case RetxSuppression::SUPPRESS:
    NFD_LOG_DEBUG(interest << " interestFrom " << inFace.getId() << " retx-suppress");
    return;
  }

  const fib::NextHopList& nexthops = fibEntry->getNextHops();

  if (nexthops.size() == 0) {
    NFD_LOG_TRACE("Rejecting Interest: No next hops for " << fibEntry->getPrefix());
    this->rejectPendingInterest(pitEntry);
    return;
  }
  else if (nexthops.size() == 1 && nexthops.front().getFace()->getId() == inFace.getId()) {
    // If the only nexthop that exists is the one the Interest was received on, reject
    NFD_LOG_TRACE("Rejecting Interest: inFace is the only nexthop for " << fibEntry->getPrefix());
    this->rejectPendingInterest(pitEntry);
    return;
  }

  const shared_ptr<Face> faceToUse = m_stats.getBestFace(*fibEntry, inFace);

  if (faceToUse == nullptr) {
    NFD_LOG_TRACE("Rejecting Interest: No best face");
    this->rejectPendingInterest(pitEntry);
    return;
  }

  forwardInterest(interest, *fibEntry, pitEntry, faceToUse);

  // If necessary, send probe
  if (m_probe->isProbingNeeded(fibEntry)) {
    shared_ptr<Face> faceToProbe = m_probe->getFaceToProbe(inFace, interest, fibEntry, *faceToUse);

    if (faceToProbe != nullptr) {
      NFD_LOG_DEBUG("Sending probe for " << fibEntry->getPrefix()
                                         << " to FaceId: " << faceToProbe->getId());

      bool wantNewNonce = true;
      forwardInterest(interest, *fibEntry, pitEntry, faceToProbe, wantNewNonce);

      m_probe->afterProbe(fibEntry);
    }
  }
}

void
AsfStrategy::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                   const Face& inFace,
                                   const Data& data)
{
  m_stats.beforeSatisfyInterest(pitEntry, inFace, data);
}

void
AsfStrategy::onConfig(const ConfigSection& configSection)
{
  for (const auto& pair : configSection) {
    if (pair.first == "global-seed") {
      uint64_t seed = pair.second.get_value<uint64_t>();
      m_probe->setGlobalSeed(seed);
    }
    else if (pair.first == "node-uid") {
      std::string uid = pair.second.get_value<std::string>();
      m_probe->setNodeUid(uid);
    }
    else if (pair.first == "probing-interval") {
      uint32_t interval = pair.second.get_value<uint32_t>();
      m_probe->setProbingInterval(interval);
    }
    else if (pair.first == "enable-learning-period") {
      bool isLearningPeriodEnabled = pair.second.get_value<bool>();

      if (isLearningPeriodEnabled) {
        NFD_LOG_INFO("Enabling learning period...");
        m_stats.enableLearningPeriod();
      }
    }
  }
}

void
AsfStrategy::forwardInterest(const Interest& interest,
                             const fib::Entry& fibEntry,
                             shared_ptr<pit::Entry> pitEntry,
                             shared_ptr<Face> outFace,
                             bool wantNewNonce)
{
  NFD_LOG_DEBUG("Forwarding Interest using FaceId: " << outFace->getId());
  this->sendInterest(pitEntry, outFace, wantNewNonce);

  m_stats.afterForwardInterest(interest, fibEntry, *outFace);
}

} // namespace fw
} // namespace nfd
