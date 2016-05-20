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

#ifndef NFD_DAEMON_FW_ASF_STRATEGY_HPP
#define NFD_DAEMON_FW_ASF_STRATEGY_HPP

#include "asf-measurements.hpp"
#include "fw/retx-suppression-exponential.hpp"
#include "fw/strategy.hpp"

namespace nfd {
namespace fw {

/** \brief Adaptive SRTT-based Forwarding Strategy
 */
class AsfStrategy : public Strategy
{
public:
  AsfStrategy(Forwarder& forwarder, const Name& name = STRATEGY_NAME);

  virtual
  ~AsfStrategy();

public: // triggers
  virtual void
  afterReceiveInterest(const Face& inFace,
                       const Interest& interest,
                       shared_ptr<fib::Entry> fibEntry,
                       shared_ptr<pit::Entry> pitEntry) override;

  virtual void
  beforeSatisfyInterest(shared_ptr<pit::Entry> pitEntry,
                        const Face& inFace, const Data& data) override;

  virtual void
  afterReceiveNack(const Face& inFace, const lp::Nack& nack,
                   shared_ptr<fib::Entry> fibEntry,
                   shared_ptr<pit::Entry> pitEntry) override;

private:
  void
  forwardInterest(const Interest& interest,
                  const fib::Entry& fibEntry,
                  shared_ptr<pit::Entry> pitEntry,
                  shared_ptr<Face> outFace,
                  bool wantNewNonce = false);

  const shared_ptr<Face>
  getBestFaceForForwarding(const fib::Entry& fibEntry, const Face& inFace);

  void
  onTimeout(const ndn::Name& interestName, nfd::face::FaceId faceId);

private:
  RetxSuppressionExponential m_retxSuppression;
  AsfMeasurements m_measurements;

private:
  /** \brief ASF Probing Module
   */
  class ProbingModule
  {
  public:
    ProbingModule(AsfMeasurements& measurements);

    void
    scheduleProbe(shared_ptr<fib::Entry> fibEntry, const time::milliseconds& interval);

    shared_ptr<Face>
    getFaceToProbe(const Face& inFace,
                   const Interest& interest,
                   shared_ptr<fib::Entry> fibEntry,
                   const Face& faceUsed);

    bool
    isProbingNeeded(shared_ptr<fib::Entry> fibEntry);

    void
    afterForwardingProbe(shared_ptr<fib::Entry> fibEntry);

  private:
    // Used to associate FaceInfo with the face in a NextHop
    typedef std::pair<FaceInfo*, shared_ptr<Face>> FaceInfoFacePair;
    typedef std::function<bool(FaceInfoFacePair, FaceInfoFacePair)> FaceInfoPredicate;
    typedef std::set<FaceInfoFacePair, FaceInfoPredicate> FaceInfoFacePairSet;

    shared_ptr<Face>
    getFaceBasedOnProbability(const FaceInfoFacePairSet& rankedFaces);

    double
    getProbingProbability(uint64_t rank, uint64_t rankSum, uint64_t nFaces);

  PUBLIC_WITH_TESTS_ELSE_PROTECTED:
    double
    getRandomNumber(double start, double end);

  private:
    time::seconds m_probingInterval;
    AsfMeasurements& m_measurements;

    static const time::seconds DEFAULT_PROBING_INTERVAL;
  };

  ProbingModule m_probing;

public:
  static const Name STRATEGY_NAME;
  static const time::milliseconds RETX_SUPPRESSION_INITIAL;
  static const time::milliseconds RETX_SUPPRESSION_MAX;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_ASF_STRATEGY_HPP
