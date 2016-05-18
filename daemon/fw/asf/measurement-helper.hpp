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

#ifndef NFD_DAEMON_FW_ASF_MEASUREMENT_HELPER_HPP
#define NFD_DAEMON_FW_ASF_MEASUREMENT_HELPER_HPP

#include "asf-measurements.hpp"

#include "table/measurements-accessor.hpp"

namespace nfd {
namespace fw {

class MeasurementHelper
{
public:
  static FaceInfo*
  getFaceInfo(MeasurementsAccessor& measurements, const fib::Entry& fibEntry, const Face& face)
  {
    NamespaceInfo& info = getOrCreateNamespaceInfo(measurements, fibEntry);

    return info.getFaceInfo(fibEntry, face);
  }

  static FaceInfo&
  getOrCreateFaceInfo(MeasurementsAccessor& measurements, const fib::Entry& fibEntry, const Face& face)
  {
    NamespaceInfo& info = getOrCreateNamespaceInfo(measurements, fibEntry);

    return info.getOrCreateFaceInfo(fibEntry, face);
  }

  static shared_ptr<NamespaceInfo>
  getNamespaceInfo(MeasurementsAccessor& measurements, const ndn::Name& prefix)
  {
    shared_ptr<measurements::Entry> me = measurements.findLongestPrefixMatch(prefix);

    if (me == nullptr) {
      //NFD_LOG_WARN("Could not find measurements entry for " << prefix);
      return nullptr;
    }

    // Set or update entry lifetime
    measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

    shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
    BOOST_ASSERT(info != nullptr);

    return info;
  }

  static NamespaceInfo&
  getOrCreateNamespaceInfo(MeasurementsAccessor& measurements, const fib::Entry& fibEntry)
  {
    shared_ptr<measurements::Entry> me = measurements.get(fibEntry);

    // Set or update entry lifetime
    measurements.extendLifetime(*me, MEASUREMENTS_LIFETIME);

    shared_ptr<NamespaceInfo> info = me->getOrCreateStrategyInfo<NamespaceInfo>();
    BOOST_ASSERT(info != nullptr);

    return *info;
  }

private:
  static const time::microseconds MEASUREMENTS_LIFETIME;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_ASF_MEASUREMENT_HELPER_HPP
