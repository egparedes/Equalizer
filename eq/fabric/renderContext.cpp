
/* Copyright (c) 2006-2016, Stefan Eilemann <eile@equalizergraphics.com>
 *                          Enrique G. Paredes <egparedes@ifi.uzh.ch>
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

#include "renderContext.h"
#include "tile.h"
#include "task.h"

namespace eq
{
namespace fabric
{

// cppcheck-suppress uninitMemberVar
RenderContext::RenderContext()
    : frameID( 0 )
    , overdraw( Vector4i::ZERO )
    , offset( Vector2i::ZERO )
    , tasks( TASK_NONE )
    , buffer( 0x0405 ) // GL_BACK
    , taskID( 0 )
    , period( 1 )
    , phase( 0 )
    , eye( EYE_CYCLOP )
    , finishDraw( false )
    , isLocal( false )
{
}

void RenderContext::apply( const Tile& tile )
{
    frustum = tile.frustum;
    ortho = tile.ortho;
    pvp = tile.pvp;
    vp = tile.vp;
    if( !isLocal )
    {
        pvp.x = 0;
        pvp.y = 0;
    }
}

std::ostream& operator << ( std::ostream& os, const RenderContext& ctx )
{
    return os << "ID " << ctx.frameID << " pvp " << ctx.pvp << " vp " << ctx.vp
              << " " << ctx.tasks << " " << ctx.range << " " << ctx.eye << " "
              << ctx.zoom;
}

}
}
