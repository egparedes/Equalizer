
/* Copyright (c) 2013-2015, David Steiner <steiner@ifi.uzh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2011, Carsten Rohn <carsten.rohn@rtt.ag>
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

#ifndef EQS_CHUNKEQUALIZER_H
#define EQS_CHUNKEQUALIZER_H

#include "packageEqualizer.h"

namespace eq
{
namespace server
{

std::ostream& operator << ( std::ostream& os, const ChunkEqualizer* );

class ChunkEqualizer : public PackageEqualizer<eq::fabric::Chunk, float>
{
public:
    EQSERVER_API ChunkEqualizer();
    ChunkEqualizer( const ChunkEqualizer& from );
    ~ChunkEqualizer() {}

    virtual void toStream( std::ostream& os ) const { os << this; }

    virtual uint32_t getType() const { return fabric::CHUNK_EQUALIZER; }
};

} //server
} //eq

#endif // EQS_CHUNKEQUALIZER_H
