
/* Copyright (c) 2014-2015, David Steiner <steiner@ifi.uzh.ch> 
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

#ifndef EQFABRIC_CHUNK_H
#define EQFABRIC_CHUNK_H

#include <eq/fabric/stealable.h>        // base class
#include <eq/fabric/range.h>

namespace eq
{
namespace fabric
{
/** @internal */
class Chunk : public Stealable
{
public:
    Chunk() {}
    Chunk( const Range& range_ )
        : range( range_ ) {}

    Range range;
};
}
}

namespace lunchbox
{
template<> inline void byteswap( eq::fabric::Chunk& chunk )
{
    byteswap( chunk.range );
    byteswap( chunk.stolen );
}
}

#endif // EQFABRIC_CHUNK_H
