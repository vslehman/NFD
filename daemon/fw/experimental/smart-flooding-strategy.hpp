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

#ifndef NFD_DAEMON_FW_EXPERIMENTAL_SMART_FLOODING_STRATEGY_HPP
#define NFD_DAEMON_FW_EXPERIMENTAL_SMART_FLOODING_STRATEGY_HPP

#include "../strategy.hpp"

namespace nfd {
namespace fw {
namespace experimental {

class ProbingModule;
class StatisticsModule;

/** \brief Smart Flooding Strategy
 */
class SmartFloodingStrategy : public Strategy
{
public:
  SmartFloodingStrategy(Forwarder& forwarder, const Name& name = STRATEGY_NAME);

  virtual
  ~SmartFloodingStrategy();

public: // triggers
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

private:
  std::unique_ptr<StatisticsModule> m_stats;
  std::unique_ptr<ProbingModule> m_probe;

public:
  static const Name STRATEGY_NAME;
};

} // namespace experimental
} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_EXPERIMENTAL_SMART_FLOODING_STRATEGY_HPP
