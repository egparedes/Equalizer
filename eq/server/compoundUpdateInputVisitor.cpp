
/* Copyright (c) 2007-2012, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2011, Daniel Nachbaur <danielnachbaur@gmail.com>
 *               2013-2015, David Steiner <steiner@ifi.uzh.ch>
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

#include "compoundUpdateInputVisitor.h"

#include "frame.h"
#include "frameData.h"
#include "log.h"
#include "tileQueue.h"
#include "chunkQueue.h"
#include "server.h"

#include <eq/fabric/iAttribute.h>

namespace eq
{
namespace server
{
CompoundUpdateInputVisitor::CompoundUpdateInputVisitor(
    const Compound::FrameMap& outputFrames,
    const Compound::TileQueueMap& outputTileQueues,
    const Compound::ChunkQueueMap& outputChunkQueues )
    : _outputFrames( outputFrames )
    , _outputTileQueues( outputTileQueues )
    , _outputChunkQueues( outputChunkQueues )
{}

VisitorResult CompoundUpdateInputVisitor::visit( Compound* compound )
{
    if( !compound->isActive( ))
        return TRAVERSE_PRUNE;

    _updateQueues( compound );
    _updateFrames( compound );
    return TRAVERSE_CONTINUE;
}

void CompoundUpdateInputVisitor::_updateQueues( const Compound* compound )
{
    const TileQueues* inputTileQueues;
    compound->getInputPackageQueues( &inputTileQueues );
    for( TileQueuesCIter i = inputTileQueues->begin(); i != inputTileQueues->end(); ++i )
    {
        //----- Find corresponding output queue
        TileQueue* queue  = *i;
        const std::string& name = queue->getName();

        Compound::TileQueueMap::const_iterator j = _outputTileQueues.find( name );

        if( j == _outputTileQueues.end( ))
        {
            LBVERB << "Can't find matching output tile queue, ignoring input tile queue "
                   << name << std::endl;
            queue->unsetData();
            continue;
        }

        LBASSERT( queue->isAttached( ));

        TileQueue* outputTileQueue = j->second;
        queue->setOutputQueue( outputTileQueue, compound );
    }

    const ChunkQueues* inputChunkQueues;
    compound->getInputPackageQueues( &inputChunkQueues );
    for( ChunkQueuesCIter i = inputChunkQueues->begin(); i != inputChunkQueues->end(); ++i )
    {
        //----- Find corresponding output queue
        ChunkQueue* queue  = *i;
        const std::string& name = queue->getName();

        Compound::ChunkQueueMap::const_iterator j = _outputChunkQueues.find( name );

        if( j == _outputChunkQueues.end( ))
        {
            LBVERB << "Can't find matching output chunk queue, ignoring input chunk queue "
                   << name << std::endl;
            queue->unsetData();
            continue;
        }

        LBASSERT( queue->isAttached( ));

        ChunkQueue* outputChunkQueue = j->second;
        queue->setOutputQueue( outputChunkQueue, compound );
    }
}

void CompoundUpdateInputVisitor::_updateFrames( Compound* compound )
{
    const Channel* channel = compound->getChannel();
    if( !compound->testInheritTask( fabric::TASK_ASSEMBLE ) || !channel )
        return;

    const Frames& inputFrames = compound->getInputFrames();
    if( inputFrames.empty( ))
    {
        compound->unsetInheritTask( fabric::TASK_ASSEMBLE );
        return;
    }

    for( FramesCIter i = inputFrames.begin(); i != inputFrames.end(); ++i )
    {
        //----- Find corresponding output frame
        Frame* frame = *i;
        const std::string& name = frame->getName();

        Compound::FrameMap::const_iterator j = _outputFrames.find( name );

        if( j == _outputFrames.end( ))
        {
            LBVERB << "Can't find matching output frame, ignoring input frame "
                   << name << std::endl;
            frame->unsetData();
            continue;
        }

        //----- Set frame parameters:
        // 1) Frame offset
        Frame* outputFrame = j->second;
        const Channel* iChannel = compound->getInheritChannel();
        Vector2i frameOffset = outputFrame->getMasterData()->getOffset() +
                               frame->getNativeOffset();

        if( outputFrame->getCompound()->getInheritChannel() != iChannel )
            frameOffset = frame->getNativeOffset();
        else if( channel != iChannel )
        {
            // compute delta offset between source and destination, since the
            // channel's native origin (as opposed to destination) is used.
            const Viewport& frameVP = frame->getViewport();
            const PixelViewport& inheritPVP=compound->getInheritPixelViewport();
            PixelViewport framePVP( inheritPVP );

            framePVP.apply( frameVP );
            frameOffset.x() -= framePVP.x;
            frameOffset.y() -= framePVP.y;

            const PixelViewport& iChannelPVP = iChannel->getPixelViewport();
            frameOffset.x() -= iChannelPVP.x;
            frameOffset.y() -= iChannelPVP.y;
        }
        frame->setOffset( frameOffset );

        // 2) zoom
        _updateZoom( compound, frame, outputFrame );

        // 3) TODO input frames are moved using the offset. The pvp signifies
        //    the pixels to be used from the frame data.
        //framePVP.x = static_cast< int32_t >( frameVP.x * inheritPVP.w );
        //framePVP.y = static_cast< int32_t >( frameVP.y * inheritPVP.h );
        //frame->setInheritPixelViewport( framePVP );
        //----- Link input frame to output frame (connects frame data)
        outputFrame->addInputFrame( frame, compound );

        for( unsigned k = 0; k < NUM_EYES; ++k )
        {
            const Eye eye = Eye( 1<<k );
            if( compound->isInheritActive( eye ) &&  // eye pass used
                outputFrame->hasData( eye ))         // output data for eye pass
            {
                frame->commit();
                LBLOG( LOG_ASSEMBLY )
                    << "Input frame  \"" << name << "\" on channel \""
                    << channel->getName() << "\" id " << frame->getID() << " v"
                    << frame->getVersion() << "\" tile pos "
                    << frame->getOffset() << ' ' << frame->getZoom()
                    << std::endl;
                break;
            }
        }
    }
}

void CompoundUpdateInputVisitor::_updateZoom( const Compound* compound,
                                              Frame* frame,
                                              const Frame* outputFrame )
{
    Zoom zoom = frame->getNativeZoom();
    if( !zoom.isValid( )) // if zoom is not set, inherit from parent
        zoom = compound->getInheritZoom();

    // Zoom difference between output and input
    const FrameData* frameData = outputFrame->getMasterData();
    zoom /= frameData->getZoom();

    frame->setZoom( zoom );
}

}
}
