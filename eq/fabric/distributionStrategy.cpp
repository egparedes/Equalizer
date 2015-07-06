
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

#include "distributionStrategy.h"

#include <lunchbox/bitOperation.h>
#include <lunchbox/debug.h>   // for LBABORT

namespace eq
{
namespace fabric
{
    
// FIXME
// std::ostream& operator << ( std::ostream& os, const DistributionStrategy& distributionStrategy )
// {
//     switch( distributionStrategy )
//     {
//     case DISTRIBUTION_STRATEGY_NONE:
//         os << "no distribution";
//         break;
//     case DISTRIBUTION_STRATEGY_EQUAL:
//         os << "equal distribution";
//         break;
//     case DISTRIBUTION_STRATEGY_RANDOM:
//         os << "random distribution";
//         break;
//     case DISTRIBUTION_STRATEGY_LOAD_AWARE:
//         os << "load aware distribution";
//         break;
//     default:
//         LBABORT( "Invalid distribution strategy value" );
//     }
// 
//     return os;
// }

}
}

namespace lunchbox
{
template<> EQFABRIC_API
int32_t getIndexOfLastBit< eq::fabric::DistributionStrategy >( eq::fabric::DistributionStrategy distributionStrategy )
{
    return getIndexOfLastBit( uint32_t( distributionStrategy ));
}

}
