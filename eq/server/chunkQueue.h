
/* Copyright (c) 2011-2012, Stefan Eilemann <eile@eyescale.ch>
 *               2011-2012, Daniel Nachbaur <danielnachbaur@googlemail.com>
 *                    2014, David Steiner <steiner@ifi.uzh.ch>
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

#ifndef EQSERVER_CHUNKQUEUE_H
#define EQSERVER_CHUNKQUEUE_H

#include "packageQueue.h"
#include "types.h"

namespace eq
{
namespace server
{
    std::ostream& operator << ( std::ostream& os, const ChunkQueue* frame );
}
}

#endif // EQSERVER_CHUNKQUEUE_H
