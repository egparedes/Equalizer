
/* Copyright (c) 2008-2016, Stefan Eilemann <eile@equalizergraphics.com>
 *                          Cedric Stalder <cedric.stalder@gmail.com>
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

#ifndef EQFABRIC_TASK_H
#define EQFABRIC_TASK_H

#include <iostream>
#include <lunchbox/types.h>

namespace eq
{
namespace fabric
{
namespace taskEnums
{
/** Tasks define the actions executed by a channel during rendering. */
enum Task
{
    TASK_NONE     = LB_BIT_NONE,
    TASK_DEFAULT  = LB_BIT1,   //!< leaf: all, other ASSEMBLE|READBACK|VIEW
    TASK_VIEW     = LB_BIT2,   //!< View start/finish
    TASK_CLEAR    = LB_BIT3,   //!< Clear the framebuffer
    TASK_DRAW     = LB_BIT4,  //!< Draw data to the framebuffer
    TASK_ASSEMBLE = LB_BIT5,  //!< Combine input frames
    TASK_READBACK = LB_BIT6,  //!< Read results to output frames
    TASK_CHANNEL_DRAW_FINISH = LB_BIT7, //!< Last channel draw done
    TASK_WINDOW_DRAW_FINISH = LB_BIT8, //!< Last window draw done
    TASK_PIPE_DRAW_FINISH = LB_BIT9, //!< Last pipe draw done
    TASK_NODE_DRAW_FINISH = LB_BIT10, //!< Last node draw done
    TASK_ALL      = LB_BIT_ALL_32,

    /** all draw finish tasks */
    TASK_DRAW_FINISH = TASK_CHANNEL_DRAW_FINISH | TASK_WINDOW_DRAW_FINISH |
                       TASK_PIPE_DRAW_FINISH | TASK_NODE_DRAW_FINISH,
    /** all render tasks */
    TASK_RENDER = TASK_CLEAR | TASK_DRAW | TASK_ASSEMBLE | TASK_READBACK |
                  TASK_DRAW_FINISH
};
}
}
}
#endif // EQ_TASK_H
