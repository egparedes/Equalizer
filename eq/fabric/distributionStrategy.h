
/* Copyright (c) 2014, David Steiner <steiner@ifi.uzh.ch>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef EQFABRIC_DISTRIBUTIONSTRATEGY_H
#define EQFABRIC_DISTRIBUTIONSTRATEGY_H

#include <eq/fabric/api.h>
#include <iostream>

namespace eq
{
namespace fabric
{
/**
 * Strategy for distributed queues.
 */
enum DistributionStrategy
{
    DISTRIBUTION_STRATEGY_NONE,
    DISTRIBUTION_STRATEGY_EQUAL,
    DISTRIBUTION_STRATEGY_RANDOM,
    DISTRIBUTION_STRATEGY_LOAD_AWARE,
    DISTRIBUTION_STRATEGY_LATENCY_AWARE,
    DISTRIBUTION_STRATEGY_CENT_LOAD_AWARE
};

// EQFABRIC_API std::ostream& operator << ( std::ostream& os, const DistributionStrategy& distributionStrategy );
}
}

namespace lunchbox
{
template<> inline void byteswap( eq::fabric::DistributionStrategy& value )
{
    byteswap( reinterpret_cast< uint32_t& >( value ));
}
}

#endif // EQFABRIC_DISTRIBUTIONSTRATEGY_H
