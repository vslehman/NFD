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

#include "../probing-module.hpp"
#include "../statistics-module.hpp"
#include "../strategy-base.hpp"

namespace nfd {
namespace fw {
namespace experimental {
namespace smart_flooding {

NFD_LOG_INIT("SmartFloodingStrategy");

////////////////////////////////////////////////////////////////////////////////////////////////////
// STATISTICS
////////////////////////////////////////////////////////////////////////////////////////////////////

class SmartFloodingStatistics : public StatisticsModule
{
public:
  SmartFloodingStatistics(MeasurementsAccessor& measurements)
    : m_measurements(measurements)
  {
  }

public:
  /** \brief stores stategy information about each face in this namespace
   */
  class NamespaceInfo : public StrategyInfo
  {
  public:
    NamespaceInfo()
      : bestFace(INVALID_FACEID)
    {
    }

    static constexpr int
    getTypeId()
    {
      return 9800;
    }

  public:
    FaceId bestFace;
  };

public:
  virtual const shared_ptr<Face>
  getBestFace(const fib::Entry& fibEntry, const Face& inFace) DECL_OVERRIDE
  {
    shared_ptr<measurements::Entry> entry = m_measurements.get(fibEntry);
    shared_ptr<NamespaceInfo> info = entry->getOrCreateStrategyInfo<NamespaceInfo>();

    // If a best face has not yet been recorded or the best face is the incoming face
    if (info->bestFace == INVALID_FACEID) {
      NFD_LOG_DEBUG("No known best face for " << fibEntry.getPrefix());
      return nullptr;
    }
    else if (info->bestFace == inFace.getId()) {
      NFD_LOG_DEBUG("Best face is same as incoming face for " << fibEntry.getPrefix());

      info->bestFace = INVALID_FACEID;
      return nullptr;
    }

    // Look for the face in the next hops of the FibEntry
    for (const fib::NextHop& hop : fibEntry.getNextHops()) {
      if (hop.getFace()->getId() == info->bestFace) {
        NFD_LOG_DEBUG("Found next hop with best face for " << fibEntry.getPrefix());
        return hop.getFace();
      }
    }

    NFD_LOG_DEBUG("Could not find next hop with best face for " << fibEntry.getPrefix());

    // Couldn't find the best face in the FibEntry so set it as invalid
    info->bestFace = INVALID_FACEID;
    return nullptr;
  }

  void
  beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                        const Face& inFace,
                        const Data& data) DECL_OVERRIDE
  {
    shared_ptr<measurements::Entry> entry = m_measurements.findLongestPrefixMatch(*pitEntry);

    if (entry == nullptr) {
      NFD_LOG_WARN("Could not find measurements entry for " << pitEntry->getName());
      return;
    }

    m_measurements.extendLifetime(*entry, MEASUREMENTS_LIFETIME);

    shared_ptr<NamespaceInfo> info = entry->getOrCreateStrategyInfo<NamespaceInfo>();

    // If this is the first Data packet received
    if (info->bestFace == INVALID_FACEID) {
      NFD_LOG_DEBUG("New best face for " << entry->getName() << " is FaceId: " << inFace.getId());
      info->bestFace = inFace.getId();
    }
  }

public:
  static const time::seconds MEASUREMENTS_LIFETIME;

private:
  MeasurementsAccessor& m_measurements;
};

const time::seconds SmartFloodingStatistics::MEASUREMENTS_LIFETIME = time::seconds(30);

////////////////////////////////////////////////////////////////////////////////////////////////////
// STRATEGY DECLARATION
////////////////////////////////////////////////////////////////////////////////////////////////////

class SmartFloodingStrategy : public StrategyBase
{
public:
  SmartFloodingStrategy(Forwarder& forwarder, const Name& name);

  virtual void
  afterReceiveInterest(const Face& inFace,
                       const Interest& interest,
                       shared_ptr<fib::Entry> fibEntry,
                       shared_ptr<pit::Entry> pitEntry) DECL_OVERRIDE;

  virtual void
  beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                        const Face& inFace, const Data& data) DECL_OVERRIDE;

  virtual void
  beforeExpirePendingInterest(shared_ptr<pit::Entry> pitEntry) DECL_OVERRIDE;

public:
  static const ndn::Name STRATEGY_NAME;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// STRATEGY IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////

const ndn::Name SmartFloodingStrategy::STRATEGY_NAME = "ndn:/localhost/nfd/strategy/smart-flooding/%FD%01";

NFD_REGISTER_STRATEGY(SmartFloodingStrategy);

SmartFloodingStrategy::SmartFloodingStrategy(Forwarder& forwarder, const Name& name = STRATEGY_NAME)
  : StrategyBase(forwarder,
                 name,
                 new SmartFloodingStatistics(this->getMeasurements()),
                 nullptr)
{
}

void
SmartFloodingStrategy::afterReceiveInterest(const Face& inFace,
                                            const Interest& interest,
                                            shared_ptr<fib::Entry> fibEntry,
                                            shared_ptr<pit::Entry> pitEntry)
{
  const fib::NextHopList& nexthops = fibEntry->getNextHops();

  if (nexthops.size() == 0) {
    NFD_LOG_TRACE("Rejecting Interest: No next hops for " << fibEntry->getPrefix());
    this->rejectPendingInterest(pitEntry);
    return;
  }

  const shared_ptr<Face> faceToUse = m_stats->getBestFace(*fibEntry, inFace);

  if (faceToUse == nullptr) {
    NFD_LOG_TRACE("No known best face; Flooding");

    // Flood to all interfaces except incoming interface
    for (const fib::NextHop& hop : fibEntry->getNextHops()) {
      if (hop.getFace()->getId() != inFace.getId()) {
        this->sendInterest(pitEntry, hop.getFace());
      }
    }
  }
  else {
    NFD_LOG_DEBUG("Forwarding Interest using FaceId: " << faceToUse->getId());
    this->sendInterest(pitEntry, faceToUse);
  }

  // If necessary, send probe
  if (m_probe != nullptr && m_probe->isProbingNeeded(fibEntry)) {
    shared_ptr<Face> faceToProbe = m_probe->getFaceToProbe(inFace, interest, fibEntry, *faceToUse);

    if (faceToProbe != nullptr) {
      NFD_LOG_DEBUG("Sending probe for " << fibEntry->getPrefix()
                                         << " to FaceId: " << faceToProbe->getId());

      bool wantNewNonce = true;
      this->sendInterest(pitEntry, faceToProbe, wantNewNonce);

      m_probe->afterProbe(fibEntry);
    }
  }
}

void
SmartFloodingStrategy::beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                                             const Face& inFace,
                                             const Data& data)
{
  if (m_stats != nullptr) {
    m_stats->beforeSatisfyInterest(pitEntry, inFace, data);
  }
}

void
SmartFloodingStrategy::beforeExpirePendingInterest(shared_ptr<pit::Entry> pitEntry)
{
}

} // namespace smart_flooding
} // namespace experimental
} // namespace fw
} // namespace nfd
