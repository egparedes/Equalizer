
/* Copyright (c) 2012, Stefan Eilemann <eile@equalizergraphics.com>
 *               2013-2014, David Steiner <steiner@ifi.uzh.ch>
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

#ifndef EQFABRIC_TILE_H
#define EQFABRIC_TILE_H

#include <eq/fabric/pixelViewport.h>
#include <eq/fabric/stealable.h>        // base class
#include <eq/fabric/viewport.h>

namespace eq
{
namespace fabric
{
/** @internal */
#pragma pack(push, 1)
class Tile : public Stealable
{
public:
    Tile() {}
    Tile( const PixelViewport& pvp_, const Viewport& vp_ )
        : Stealable(), pvp( pvp_ ), vp( vp_ ) {}

    uint32_t checksum;  // TEST
    Frustumf frustum;
    Frustumf ortho;
    PixelViewport pvp;
    Viewport vp;
};
#pragma pack(pop)
/*
std::ostream& operator << ( std::ostream& os, const Tile& tile )
{
    os << "[ " << tile.frustum << " " << tile.ortho << " " << tile.pvp << " " << tile.vp <<" ]";
    return os;
}*/
}
}

namespace lunchbox
{
template<> inline void byteswap( eq::fabric::Tile& tile )
{
    byteswap( tile.checksum );
    byteswap( tile.frustum );
    byteswap( tile.ortho );
    byteswap( tile.pvp );
    byteswap( tile.vp );
    byteswap( tile.stolen );
}
}

#endif // EQFABRIC_TILE_H
