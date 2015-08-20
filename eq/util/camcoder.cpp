
/* Copyright (c) 2009, Maxim Makhinya
 *               2015, David Steiner
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "camcoder.h"

#include <iostream>
#include <fstream>
#include <algorithm>

namespace eq
{
namespace util
{

static const double PiConstant = 4 * std::atan(1);

Camcoder::Step Camcoder::getNextStep()
{
    LBASSERT( _steps.size() > 0 );

    const uint32_t lastStep = _steps.size() - 1;
    if( _currentStep >= lastStep )
    {
        stopPlaying();
        _currentStep = lastStep;
        return _steps[ _currentStep ];
    }
    do
    {
        Step &step = _steps[ _currentStep ];
        uint32_t frame = _currentFrame - _firstFrame;
        ++_currentStep;
        if( step.frame >= frame )
            return step;
    }
    while( _currentStep < _steps.size( ));

    _currentStep = lastStep;
    return _steps[ _currentStep ];
}

void Camcoder::setNextStep( Camcoder::Step step )
{
    step.frame = _currentFrame;
    step.keyEvent = _keyEvent;
    _keyEvent.key = 0;
    _steps.push_back( step );
    ++_currentStep;
}

bool Camcoder::loadAnimation( const std::string& fileName )
{
    _steps.clear();

    if( fileName.empty( ))
        return false;

    std::ifstream file;
    file.open( fileName.c_str( ));
    if( !file )
    {
        LBERROR << "File '" << fileName << "' could not be opened for reading" << std::endl;
        return false;
    }

    while ( !file.eof( ))
    {
        int frame = 0;
        eq::Vector3f position;
        eq::Matrix4f rotation;
        eq::Matrix4f modelRotation;
        eq::KeyEvent keyEvent;

        file >> frame
             >> keyEvent.key;
        read_from_stream( position, file );
        read_from_stream( rotation, file );
        read_from_stream( modelRotation, file );

        _steps.push_back( Step( frame, keyEvent, position, rotation, modelRotation ));
    }
    file.close();

    return true;
}

bool Camcoder::saveAnimation( const std::string& fileName )
{
    if( fileName.empty( ))
        return false;

    std::ofstream file;
    file.open( fileName.c_str( ));
    if( !file )
    {
        LBERROR << "File '" << fileName << "' could not be opened for writing" << std::endl;
        return false;
    }

    uint32_t first = _steps.begin()->frame;
    for( Steps::const_iterator it = _steps.begin(); it != _steps.end(); ++it )
    {
        file << it->frame - first   << " "
             << it->keyEvent.key    << " ";
        write_to_stream( (it->position), file );
        write_to_stream( (it->rotation), file );
        write_to_stream( (it->modelRotation), file );
        file << std::endl;
    }
    file.close();

    return true;
}

bool Camcoder::createLongShowcasePath(eq::Vector3f dollyArgs, eq::Vector3f radiusArgs,
                                      eq::Vector3f angleArgs, const eq::AABBf &bbox)
{
    LBINFO << "camcorder long showcase path:" << std::endl;
    LBINFO << " minPoint = " << bbox.getMin() << std::endl;
    LBINFO << " maxPoint = " << bbox.getMax() << std::endl;

    // Limit to sensible values
    for (unsigned i=0; i < 2; ++i)
    {
        dollyArgs[i] = std::max(-2.0f, std::min( dollyArgs[i], 2.0f ) );
        radiusArgs[i] = std::max(100.0f, std::min( dollyArgs[i], 0.0f ) );
        angleArgs[i] = std::max(-2.0f, std::min( angleArgs[i], 2.0f ) );
    }

    eq::Vector3f minPosition = bbox.getMin();
    eq::Vector3f center = bbox.getCenter();
    eq::Vector3f dimension = bbox.getDimension();
    size_t dollyCoord = dimension.find_max_index();
    size_t dollyCoordN = ( dollyCoord + 1 ) % 3;
    size_t dollyCoordP = ( dollyCoord + 2 ) % 3;
    eq::Vector3f axis( 0.0f );
    axis[dollyCoord] = 1.0f;

    float radius = std::max( dimension[dollyCoordN], dimension[dollyCoordP] );
    float dollyStep = dollyArgs[1] * dimension[dollyCoord];
    float radiusStep = radiusArgs[1] * radius;
    float angleStep = angleArgs[1] * 2 * PiConstant;

    LBINFO << " axis = " << axis << std::endl;
    LBINFO << " center = " << center << std::endl;
    LBINFO << " dimension = " << dimension << std::endl;
    LBINFO << " radius = " << radius << std::endl;
    LBINFO << " dollyStep = " << dollyStep << std::endl;
    LBINFO << " radiusStep = " << radiusStep << std::endl;
    LBINFO << " angleStep = " << angleStep << std::endl;

    eq::Vector3f position;
    eq::Matrix4f rotation = eq::Matrix4f::IDENTITY;
    if ( dollyCoord == 0 )
        rotation.rotate( PiConstant / -2.0, eq::Vector3f::LEFT );
    else if ( dollyCoord == 2 )
        rotation.rotate( PiConstant / -2.0, eq::Vector3f::UP );

    eq::Matrix4f modelRotation = eq::Matrix4f::IDENTITY;
    Step step;
    step.frame = 0;
    step.keyEvent.key = 0;
    _steps.clear();

    bool dollyActive = true;
    bool radiusActive = true;
    bool angleActive = true;
    while( dollyActive || radiusActive || angleActive)
    {
        float currentDolly = ( dollyActive ) ?
            minPosition[dollyCoord] + ( dimension[dollyCoord] * dollyArgs[0] ) + ( dollyStep * step.frame) :
            minPosition[dollyCoord] + ( dimension[dollyCoord] * dollyArgs[2] );
        float currentRadius = ( radiusActive ) ?
                    ( radius * radiusArgs[0] ) + ( radiusStep * step.frame ) :
                    ( radius * radiusArgs[2] );
        float currentAngle = ( angleActive ) ?
                    2*PiConstant * angleArgs[0] + ( angleStep * step.frame ) :
                    2*PiConstant * angleArgs[2];
        position[dollyCoordP] = 0;
        position[dollyCoord] = currentDolly;
        position[dollyCoordN] = -1.0*currentRadius;
        step.position = position;

        step.rotation = rotation;

        modelRotation = eq::Matrix4f::IDENTITY;
        modelRotation.rotate( currentAngle, axis );
        step.modelRotation = modelRotation;

        _steps.push_back( step );
        step.frame++;

        dollyActive = ( dollyArgs[1] * step.frame < dollyArgs[2] - dollyArgs[0] );
        radiusActive = ( radiusArgs[1] * step.frame < radiusArgs[2] - radiusArgs[0] );
        angleActive = ( angleArgs[1] * step.frame < angleArgs[2] - angleArgs[0] );
    }

    LBINFO << " camcoder steps = " << _steps.size() << std::endl;

    return true;
}


eq::KeyEvent Camcoder::getKeyEvent()
{
    return _keyEvent;
}

void Camcoder::setKeyEvent( const eq::KeyEvent& event )
{
    _keyEvent = event;
}

void Camcoder::startRecording()
{
    stopPlaying();
    _recording = true;
    _steps.clear();
}

void Camcoder::stopRecording()
{
    _currentStep = 0;
    _recording = false;
}

void Camcoder::startPlaying()
{
    stopRecording();
    _playing = true;
//     _firstFrame = _currentFrame;
}

void Camcoder::stopPlaying()
{
    _currentStep = 0;
    _playing = false;
}

void Camcoder::read_from_stream( eq::Vector3f& v, std::istream& is )
{
    for( size_t i = 0; i < v.DIMENSION; ++i )
        is >> v.at( i );
}

void Camcoder::read_from_stream( eq::Matrix4f& m, std::istream& is )
{
    for( size_t row_index = 0; row_index < m.ROWS; ++row_index )
    {
        for( size_t col_index = 0; col_index < m.COLS; ++col_index )
        {
            is >> m.at( row_index, col_index );
        }
    }
}

void Camcoder::write_to_stream( const eq::Vector3f& v, std::ostream& os ) const
{
    for( size_t i = 0; i < v.DIMENSION; ++i )
        os << v.at( i ) << " ";
}

void Camcoder::write_to_stream( const eq::Matrix4f& m, std::ostream& os ) const
{
     for( size_t row_index = 0; row_index < m.ROWS; ++row_index )
     {
         for( size_t col_index = 0; col_index < m.COLS; ++col_index )
         {
             os << m.at( row_index, col_index ) << " ";
         }
     }
}

}

}
