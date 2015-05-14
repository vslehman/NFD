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

#include "hyperbolic-strategy.hpp"

#include "hyperbolic-probing.hpp"
#include "core/logger.hpp"

namespace nfd {
namespace fw {
namespace experimental {

NFD_LOG_INIT("HyperbolicStrategy");

const Name HyperbolicStrategy::STRATEGY_NAME("ndn:/localhost/nfd/strategy/hyperbolic/%FD%01");
const time::seconds HyperbolicStrategy::SUPPRESSION_TIME = time::seconds(2);

NFD_REGISTER_STRATEGY(HyperbolicStrategy);

HyperbolicStrategy::HyperbolicStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder, name)
  , m_stats(this->getMeasurements())
  , m_probe(std::unique_ptr<ProbingModule>(new HyperbolicProbingModule(m_stats)))
  , m_retxSuppression(SUPPRESSION_TIME)
{
}

HyperbolicStrategy::~HyperbolicStrategy()
{
}

void
HyperbolicStrategy::afterReceiveInterest(const Face& inFace,
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

  const fib::NextHop* hopToUse = m_stats.getBestNextHop(*fibEntry, inFace);

  if (hopToUse == nullptr) {
    NFD_LOG_TRACE("Rejecting Interest: No best face");
    this->rejectPendingInterest(pitEntry);
    return;
  }

  forwardInterest(pitEntry, hopToUse->getFace());

  // If necessary, send probe
  if (m_probe->isProbingNeeded(fibEntry)) {
    shared_ptr<Face> faceToProbe = m_probe->getFaceToProbe(inFace, interest, fibEntry, *hopToUse);

    if (faceToProbe != nullptr) {
      NFD_LOG_DEBUG("Sending probe for " << fibEntry->getPrefix()
                                         << " to FaceId: " << faceToProbe->getId());

      bool wantNewNonce = true;
      forwardInterest(pitEntry, faceToProbe, wantNewNonce);

      m_probe->afterProbe(fibEntry);
    }
  }
}

void
HyperbolicStrategy::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                          const Face& inFace,
                                          const Data& data)
{
  m_stats.beforeSatisfyInterest(pitEntry, inFace, data);
}

void
HyperbolicStrategy::forwardInterest(shared_ptr<pit::Entry> pitEntry,
                                    shared_ptr<Face> outFace,
                                    bool wantNewNonce)
{
  NFD_LOG_DEBUG("Forwarding Interest using FaceId: " << outFace->getId());
  this->sendInterest(pitEntry, outFace, wantNewNonce);
}

} // namespace experimental
} // namespace fw
} // namespace nfd
