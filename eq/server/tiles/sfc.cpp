
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

#include "sfc.h"

namespace eq
{
namespace server
{
namespace tiles
{

// http://and-what-happened.blogspot.ch/2011/08/fast-2d-and-3d-hilbert-curves-and.html

uint32_t mortonToHilbert2D( const uint32_t morton, const uint32_t bits )
{
    uint32_t hilbert = 0;
    uint32_t remap = 0xb4;
    uint32_t block = ( bits << 1 );
    while( block )
    {
        block -= 2;
        uint32_t mcode = ( ( morton >> block ) & 3 );
        uint32_t hcode = ( ( remap >> ( mcode << 1 ) ) & 3 );
        remap ^= ( 0x82000028 >> ( hcode << 3 ) );
        hilbert = ( ( hilbert << 2 ) + hcode );
    }
    return( hilbert );
}

uint32_t hilbertToMorton2D( const uint32_t hilbert, const uint32_t bits )
{
    uint32_t morton = 0;
    uint32_t remap = 0xb4;
    uint32_t block = ( bits << 1 );
    while( block )
    {
        block -= 2;
        uint32_t hcode = ( ( hilbert >> block ) & 3 );
        uint32_t mcode = ( ( remap >> ( hcode << 1 ) ) & 3 );
        remap ^= ( 0x330000cc >> ( hcode << 3 ) );
        morton = ( ( morton << 2 ) + mcode );
    }
    return( morton );
}

uint32_t mortonEncode2D16bit( uint32_t index1, uint32_t index2 )
{ // pack 2 16-bit indices into a 32-bit Morton code
    index1 &= 0x0000ffff;
    index2 &= 0x0000ffff;
    index1 |= ( index1 << 8 );
    index2 |= ( index2 << 8 );
    index1 &= 0x00ff00ff;
    index2 &= 0x00ff00ff;
    index1 |= ( index1 << 4 );
    index2 |= ( index2 << 4 );
    index1 &= 0x0f0f0f0f;
    index2 &= 0x0f0f0f0f;
    index1 |= ( index1 << 2 );
    index2 |= ( index2 << 2 );
    index1 &= 0x33333333;
    index2 &= 0x33333333;
    index1 |= ( index1 << 1 );
    index2 |= ( index2 << 1 );
    index1 &= 0x55555555;
    index2 &= 0x55555555;
    return( index1 | ( index2 << 1 ) );
}

void mortonDecode2D16bit( const uint32_t morton, uint32_t& index1, uint32_t& index2 )
{ // unpack 2 16-bit indices from a 32-bit Morton code
    uint32_t value1 = morton;
    uint32_t value2 = ( value1 >> 1 );
    value1 &= 0x55555555;
    value2 &= 0x55555555;
    value1 |= ( value1 >> 1 );
    value2 |= ( value2 >> 1 );
    value1 &= 0x33333333;
    value2 &= 0x33333333;
    value1 |= ( value1 >> 2 );
    value2 |= ( value2 >> 2 );
    value1 &= 0x0f0f0f0f;
    value2 &= 0x0f0f0f0f;
    value1 |= ( value1 >> 4 );
    value2 |= ( value2 >> 4 );
    value1 &= 0x00ff00ff;
    value2 &= 0x00ff00ff;
    value1 |= ( value1 >> 8 );
    value2 |= ( value2 >> 8 );
    value1 &= 0x0000ffff;
    value2 &= 0x0000ffff;
    index1 = value1;
    index2 = value2;
}

void hilbertDecode2D16bit( const uint32_t hilbert, uint32_t& index1, uint32_t& index2 )
{
    const uint32_t morton = hilbertToMorton2D( hilbert, 16 );
    mortonDecode2D16bit( morton, index1, index2 );
}

}
}
}

