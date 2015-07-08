
/* Copyright (c) 2013-2015, David Steiner <steiner@ifi.uzh.ch>
 *               2011-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *               2011, Carsten Rohn <carsten.rohn@rtt.ag>
 *               2011, Daniel Nachbaur <danielnachbaur@gmail.com>
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

#include "types.h"
#include "compound.h"
#include "config.h"
#include "chunkQueue.h"
#include "compoundVisitor.h"
#include "server.h"

#include <co/global.h>

#include "chunkEqualizer.h"

namespace eq
{
namespace server
{

ChunkEqualizer::ChunkEqualizer()
    : PackageEqualizer<eq::fabric::Chunk, float>()
{
    setName("ChunkEqualizer");
//     co::Global::setIAttribute( co::Global::IATTR_QUEUE_MIN_SIZE, 1 );
//     co::Global::setIAttribute( co::Global::IATTR_QUEUE_REFILL, 1 );
}

ChunkEqualizer::ChunkEqualizer( const ChunkEqualizer& from )
    : PackageEqualizer<eq::fabric::Chunk, float>( from )
{
}

std::ostream& operator << ( std::ostream& os, const ChunkEqualizer* lb )
{
    if( lb )
    {
        os << lunchbox::disableFlush
           << "chunk_equalizer" << std::endl
           << "{" << std::endl
           << "    name \"" << lb->getName() << "\"" << std::endl
           << "    size " << lb->getChunkSize() << std::endl
           << "}" << std::endl << lunchbox::enableFlush;
    }
    return os;
}

} //server
} //eq
