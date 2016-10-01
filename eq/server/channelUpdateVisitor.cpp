
/* Copyright (c) 2007-2016, Stefan Eilemann <eile@equalizergraphics.com>
 *                          Daniel Nachbaur <danielnachbaur@gmail.com>
 *                          Cedric Stalder <cedric.stalder@gmail.com>
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

#include "channelUpdateVisitor.h"

#include "colorMask.h"
#include "compound.h"
#include "frame.h"
#include "log.h"
#include "node.h"
#include "observer.h"
#include "pipe.h"
#include "segment.h"
#include "view.h"
#include "window.h"
#include "tileQueue.h"

#include <eq/fabric/commands.h>
#include <eq/fabric/paths.h>

#include <set>

#ifndef GL_BACK_LEFT
#  define GL_FRONT_LEFT 0x0400
#  define GL_FRONT_RIGHT 0x0401
#  define GL_BACK_LEFT 0x0402
#  define GL_BACK_RIGHT 0x0403
#  define GL_FRONT 0x0404
#  define GL_BACK 0x0405
#endif

namespace eq
{
namespace server
{

using fabric::QUAD;

namespace
{
static bool _setDrawBuffers();
static uint32_t _drawBuffer[2][2][NUM_EYES];
static bool LB_UNUSED _drawBufferInit = _setDrawBuffers();
bool _setDrawBuffers()
{
    const int32_t cyclop = lunchbox::getIndexOfLastBit( EYE_CYCLOP );
    const int32_t left = lunchbox::getIndexOfLastBit( EYE_LEFT );
    const int32_t right = lunchbox::getIndexOfLastBit( EYE_RIGHT );

    // [stereo][doublebuffered][eye]
    _drawBuffer[0][0][ cyclop ] = GL_FRONT;
    _drawBuffer[0][0][ left ] = GL_FRONT;
    _drawBuffer[0][0][ right ] = GL_FRONT;

    _drawBuffer[0][1][ cyclop ] = GL_BACK;
    _drawBuffer[0][1][ left ] = GL_BACK;
    _drawBuffer[0][1][ right ] = GL_BACK;

    _drawBuffer[1][0][ cyclop ] = GL_FRONT;
    _drawBuffer[1][0][ left ] = GL_FRONT_LEFT;
    _drawBuffer[1][0][ right ] = GL_FRONT_RIGHT;

    _drawBuffer[1][1][ cyclop ] = GL_BACK;
    _drawBuffer[1][1][ left ] = GL_BACK_LEFT;
    _drawBuffer[1][1][ right ] = GL_BACK_RIGHT;

    return true;
}
}

ChannelUpdateVisitor::ChannelUpdateVisitor( Channel* channel,
                                            const uint128_t frameID,
                                            const uint32_t frameNumber )
        : _channel( channel )
        , _eye( EYE_CYCLOP )
        , _frameID( frameID )
        , _frameNumber( frameNumber )
        , _updated( false )
{}

bool ChannelUpdateVisitor::_skipCompound( const Compound* compound )
{
    return ( compound->getChannel() != _channel ||
             !compound->isInheritActive( _eye ) ||
             compound->getInheritTasks() == fabric::TASK_NONE );
}

VisitorResult ChannelUpdateVisitor::visitPre( const Compound* compound )
{
    if( !compound->isInheritActive( _eye ))
        return TRAVERSE_PRUNE;

    _updateDrawFinish( compound );

    if( _skipCompound( compound ))
        return TRAVERSE_CONTINUE;

    const RenderContext& context = _setupRenderContext( compound );
    _updateFrameRate( compound );
    _updateViewStart( compound, context );

    if( compound->testInheritTask( fabric::TASK_CLEAR ))
        _sendClear( context );
    return TRAVERSE_CONTINUE;
}

VisitorResult ChannelUpdateVisitor::visitLeaf( const Compound* compound )
{
    if( !compound->isInheritActive( _eye ))
        return TRAVERSE_CONTINUE;

    if( _skipCompound( compound ))
    {
        _updateDrawFinish( compound );
        return TRAVERSE_CONTINUE;
    }

    const RenderContext& context = _setupRenderContext( compound );
    _updateFrameRate( compound );
    _updateViewStart( compound, context );
    _updateRender( compound, context );
    _updateViewFinish( compound, context );
    return TRAVERSE_CONTINUE;
}

VisitorResult ChannelUpdateVisitor::visitPost( const Compound* compound )
{
    if( _skipCompound( compound ))
        return TRAVERSE_CONTINUE;

    const RenderContext& context = _setupRenderContext( compound );
    _updateAssemble( compound, context );
    _updateReadback( compound, context );
    _updateViewFinish( compound, context );

    return TRAVERSE_CONTINUE;
}

RenderContext ChannelUpdateVisitor::_setupRenderContext(
    const Compound* compound )
{
    const Channel* destChannel = compound->getInheritChannel();
    LBASSERT( destChannel );

    RenderContext context = compound->setupRenderContext( _eye );
    context.frameID       = _frameID;
    context.buffer        = _getDrawBuffer( compound );
    context.bufferMask    = _getDrawBufferMask( compound );
    context.view          = destChannel->getViewVersion();

    const View* view = destChannel->getView();
    LBASSERT( context.view == co::ObjectVersion( view ));

    if( view )
    {
        // compute inherit vp (part of view covered by segment/view channel)
        const Segment* segment = destChannel->getSegment();
        LBASSERT( segment );

        const PixelViewport& pvp = destChannel->getPixelViewport();
        if( pvp.hasArea( ))
            context.vp.applyView( segment->getViewport(), view->getViewport(),
                                  pvp, destChannel->getOverdraw( ));
    }

    if( _channel != destChannel )
    {
        const PixelViewport& nativePVP = _channel->getPixelViewport();
        context.pvp.x = nativePVP.x;
        context.pvp.y = nativePVP.y;
    }
    // TODO: pvp size overcommit check?

    _setupDrawFinishTasks( compound, context );
    return context;
}

co::ObjectVersions ChannelUpdateVisitor::_selectFrames( const Frames& frames )
    const
{
    co::ObjectVersions frameIDs;
    for( Frame* frame : frames )
        if( frame->hasData( _eye ))
            frameIDs.push_back( co::ObjectVersion( frame ));
    return frameIDs;
}

uint128_ts ChannelUpdateVisitor::_selectQueues(
    const Compound* compound, const RenderContext& context ) const
{
    uint128_ts queues;
    const TileQueues& inputQueues = compound->getInputTileQueues();
    for( const TileQueue* inputQueue : inputQueues )
    {
        const TileQueue* outputQueue = inputQueue->getOutputQueue( context.eye);
        queues.push_back( outputQueue->getQueueMasterID( context.eye ));
    }
    return queues;
}

void ChannelUpdateVisitor::_updateRender( const Compound* compound,
                                        const RenderContext& context )
{
    if( !(context.tasks & fabric::TASK_RENDER) )
        return;

    const co::ObjectVersions& inFrames =
        (context.tasks & fabric::TASK_ASSEMBLE) ?
            _selectFrames( compound->getInputFrames( )) : co::ObjectVersions();
    const co::ObjectVersions& outFrames =
        (context.tasks & fabric::TASK_READBACK) ?
            _selectFrames( compound->getOutputFrames( )) : co::ObjectVersions();
    const uint128_ts& queues = _selectQueues( compound, context );

    _channel->send( fabric::CMD_CHANNEL_FRAME_RENDER )
        << context << inFrames << outFrames << queues;
    _updated = true;
    LBLOG( LOG_TASKS ) << "TASK render " << _channel->getName() <<  " "
                       << std::endl;
}

void ChannelUpdateVisitor::_setupDrawFinishTasks( const Compound* compound,
                                                  RenderContext& context ) const
{
    const Compound* lastDrawCompound = _channel->getLastDrawCompound();
    if( lastDrawCompound && lastDrawCompound != compound )
        return;

    // Only pass if this is the last eye pass of this compound
    if( !compound->isLastInheritEye( _eye ))
        return;

    if( !lastDrawCompound )
        _channel->setLastDrawCompound( compound );

    // Channel::frameDrawFinish
    context.tasks |= fabric::TASK_CHANNEL_DRAW_FINISH;

    // Window::frameDrawFinish
    Window* window = _channel->getWindow();
    const Channel* lastDrawChannel = window->getLastDrawChannel();
    if( lastDrawChannel && lastDrawChannel != _channel )
        return;

    window->setLastDrawChannel( _channel ); // in case not set
    context.tasks |= fabric::TASK_WINDOW_DRAW_FINISH;

    // Pipe::frameDrawFinish
    Pipe* pipe = _channel->getPipe();
    const Window* lastDrawWindow = pipe->getLastDrawWindow();
    if( lastDrawWindow && lastDrawWindow != window )
        return;

    pipe->setLastDrawWindow( window ); // in case not set
    context.tasks |= fabric::TASK_PIPE_DRAW_FINISH;

    // Node::frameDrawFinish
    Node* node = _channel->getNode();
    const Pipe* lastDrawPipe = node->getLastDrawPipe();
    if( lastDrawPipe && lastDrawPipe != pipe )
        return;

    node->setLastDrawPipe( pipe ); // in case not set
    context.tasks |= fabric::TASK_NODE_DRAW_FINISH;
}

void ChannelUpdateVisitor::_updateDrawFinish( const Compound* compound ) const
{
    Node* node = _channel->getNode();
    RenderContext context;
    _setupDrawFinishTasks( compound, context );

    if( context.tasks & fabric::TASK_CHANNEL_DRAW_FINISH )
    {
        node->send( fabric::CMD_CHANNEL_FRAME_DRAW_FINISH, _channel->getID( ))
            << _frameID << _frameNumber;
        LBLOG( LOG_TASKS ) << "TASK channel draw finish " << _channel->getName()
                           << " frame " << _frameNumber
                           << " id " << _frameID << std::endl;
    }

    if( context.tasks & fabric::TASK_WINDOW_DRAW_FINISH )
    {
        const Window* window = _channel->getWindow();
        node->send( fabric::CMD_WINDOW_FRAME_DRAW_FINISH, window->getID( ))
            << _frameID << _frameNumber;
        LBLOG( LOG_TASKS ) << "TASK window draw finish "  << window->getName()
                           <<  " frame " << _frameNumber
                           << " id " << _frameID << std::endl;
    }

    if( context.tasks & fabric::TASK_PIPE_DRAW_FINISH )
    {
        const Pipe* pipe = _channel->getPipe();
        node->send( fabric::CMD_PIPE_FRAME_DRAW_FINISH, pipe->getID( ))
            << _frameID << _frameNumber;
        LBLOG( LOG_TASKS ) << "TASK pipe draw finish " << pipe->getName()
                           << " frame " << _frameNumber
                           << " id " << _frameID << std::endl;
    }

    if( context.tasks & fabric::TASK_NODE_DRAW_FINISH )
    {
        node->send( fabric::CMD_NODE_FRAME_DRAW_FINISH, node->getID( ))
            << _frameID << _frameNumber;
        LBLOG( LOG_TASKS ) << "TASK node draw finish " << node->getName()
                           <<  " " << std::endl;
    }
}

void ChannelUpdateVisitor::_sendClear( const RenderContext& context )
{
    _channel->send( fabric::CMD_CHANNEL_FRAME_CLEAR ) << context;
    _updated = true;
    LBLOG( LOG_TASKS ) << "TASK clear " << _channel->getName() <<  " "
                       << std::endl;
}

void ChannelUpdateVisitor::_updateFrameRate( const Compound* compound ) const
{
    const float maxFPS = compound->getInheritMaxFPS();
    Window*     window = _channel->getWindow();

    if( maxFPS < window->getMaxFPS())
        window->setMaxFPS( maxFPS );
}

uint32_t ChannelUpdateVisitor::_getDrawBuffer( const Compound* compound ) const
{
    const DrawableConfig& dc = _channel->getWindow()->getDrawableConfig();
    const int32_t eye = lunchbox::getIndexOfLastBit( _eye );

    if( compound->getInheritIAttribute(Compound::IATTR_STEREO_MODE) == QUAD )
        return _drawBuffer[ dc.stereo ][ dc.doublebuffered ][ eye ];
    return _drawBuffer[ 0 ][ dc.doublebuffered ][ eye ];
}

fabric::ColorMask
ChannelUpdateVisitor::_getDrawBufferMask(const Compound* compound) const
{
    if( compound->getInheritIAttribute( Compound::IATTR_STEREO_MODE ) !=
        fabric::ANAGLYPH )
    {
        return ColorMask::ALL;
    }

    switch( _eye )
    {
        case EYE_LEFT:
            return ColorMask(
                compound->getInheritIAttribute(
                    Compound::IATTR_STEREO_ANAGLYPH_LEFT_MASK ));
        case EYE_RIGHT:
            return ColorMask(
                compound->getInheritIAttribute(
                    Compound::IATTR_STEREO_ANAGLYPH_RIGHT_MASK ));
        default:
            return ColorMask::ALL;
    }
}

void ChannelUpdateVisitor::_updateAssemble( const Compound* compound,
                                            const RenderContext& context )
{
    if( !compound->testInheritTask( fabric::TASK_ASSEMBLE ))
        return;

    const Frames& inputFrames = compound->getInputFrames();
    const co::ObjectVersions& frames = _selectFrames( inputFrames );
    LBASSERT( !inputFrames.empty( ));
    if( frames.empty( ))
        return;

    // assemble task
    LBLOG( LOG_ASSEMBLY | LOG_TASKS )
        << "TASK assemble " << _channel->getName() << " nFrames "
        << frames.size() << std::endl;
    _channel->send( fabric::CMD_CHANNEL_FRAME_ASSEMBLE ) << context << frames;
    _updated = true;
}

void ChannelUpdateVisitor::_updateReadback( const Compound* compound,
                                            const RenderContext& context )
{
    if( !compound->testInheritTask( fabric::TASK_READBACK ))
        return;

    const std::vector< Frame* >& outputFrames = compound->getOutputFrames();
    const co::ObjectVersions& frames = _selectFrames( outputFrames );
    LBASSERT( !outputFrames.empty( ));
    if( frames.empty() )
        return;

    // readback task
    _channel->send( fabric::CMD_CHANNEL_FRAME_READBACK ) << context << frames;
    _updated = true;
    LBLOG( LOG_ASSEMBLY | LOG_TASKS )
        << "TASK readback " << _channel->getName() << " nFrames "
        << frames.size() << std::endl;
}

void ChannelUpdateVisitor::_updateViewStart( const Compound* compound,
                                             const RenderContext& context )
{
    LBASSERT( !_skipCompound( compound ));
    if( _skipCompound( compound ) || !(context.tasks & fabric::TASK_VIEW ))
        return;

    // view start task
    LBLOG( LOG_TASKS ) << "TASK view start " << _channel->getName()
                       << std::endl;
    _channel->send( fabric::CMD_CHANNEL_FRAME_VIEW_START ) << context;
}

void ChannelUpdateVisitor::_updateViewFinish( const Compound* compound,
                                              const RenderContext& context )
{
    LBASSERT( !_skipCompound( compound ));
    if( _skipCompound( compound ) || !(context.tasks & fabric::TASK_VIEW ))
        return;

    // view finish task
    LBLOG( LOG_TASKS ) << "TASK view finish " << _channel->getName() <<  " "
                       << std::endl;
    _channel->send( fabric::CMD_CHANNEL_FRAME_VIEW_FINISH ) << context;
}

}
}
