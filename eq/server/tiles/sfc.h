
/* Copyright (c) 2015, David Steiner <steiner@ifi.uzh.ch>
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

#ifndef EQSERVER_TILES_SFC_H
#define EQSERVER_TILES_SFC_H

#include "types.h"

namespace eq
{
namespace server
{
namespace tiles
{

void hilbertDecode2D16bit( const uint32_t hilbert, uint32_t& index1, uint32_t& index2 );

void mortonDecode2D16bit( const uint32_t morton, uint32_t& index1, uint32_t& index2 );

inline void hilbertDecode( const uint32_t index, uint32_t& x, uint32_t& y )
{
    hilbertDecode2D16bit( index, x, y );
}

inline void mortonDecode( const uint32_t index, uint32_t& x, uint32_t& y )
{
    mortonDecode2D16bit( index, x, y );
}

}
}
}

#endif // EQSERVER_TILES_SFC_H
