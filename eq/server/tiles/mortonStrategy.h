
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

#ifndef EQSERVER_TILES_MORTONSTRATEGY_H
#define EQSERVER_TILES_MORTONSTRATEGY_H

#include "sfc.h"

namespace eq
{
namespace server
{
namespace tiles
{

/** Generates tiles for a channel using a morton strategy */
inline void generateMorton( std::vector< Vector2i >& tiles,
                            const Vector2i& dim )
{
    int num = dim.x() * dim.y();
    for( int i = 0; i < num; ++i )
    {
        uint32_t x, y;
        mortonDecode(i, x, y);
        tiles.push_back( Vector2i( x, y ));
    }
}

}
}
}

#endif // EQSERVER_TILES_MORTONSTRATEGY_H
