
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

#ifndef EQUTIL_CAMCODER_H
#define EQUTIL_CAMCODER_H

#include <eq/eq.h>
#include <eq/fabric/vmmlib.h>

namespace eq
{
namespace util
{

/**
 * Loads sequence of camera positions and interpolates them on a per-frame
 * basis.
 */
class Camcoder
{
public:
    struct Step;

    EQ_API Camcoder()
        : _currentStep( 0 )
        , _currentFrame( 0 )
        , _firstFrame( 0 )
        , _recording( false )
        , _playing( false )
    {}

    EQ_API bool loadAnimation( const std::string& fileName );

    EQ_API bool saveAnimation( const std::string& fileName );

    EQ_API bool createLongShowcasePath(eq::Vector3f dollyArgs,
                                eq::Vector3f radiusArgs,
                                eq::Vector3f angleArgs,
                                const eq::AABBf& bbox);

    EQ_API bool isValid() const {
        return !_steps.empty();
    }

    EQ_API bool isRecording() const {
        return _recording;
    }

    EQ_API bool isPlaying() const {
        return _playing;
    }

    EQ_API bool atLastStep() const {
        return _currentFrame == _steps.size() - 1;
    }

    EQ_API Step getNextStep();

    EQ_API void setNextStep( Step step );

    EQ_API uint32_t getCurrentFrame() {
        return _currentFrame;
    }

    EQ_API void setCurrentFrame( uint32_t frame ) {
        _currentFrame = frame;
    }

    EQ_API eq::KeyEvent getKeyEvent();

    EQ_API void setKeyEvent( const eq::KeyEvent& event );

    EQ_API void startRecording();

    EQ_API void stopRecording();

    EQ_API void startPlaying();

    EQ_API void stopPlaying();

    struct Step
    {
        Step()
            : frame( 0 )
            , position( eq::Vector3f( .0f, .0f, -1.0f ))
            , rotation( eq::Matrix4f::ZERO )
            , modelRotation( eq::Matrix4f::ZERO )
        {
            keyEvent.key = 0;
        }

        Step( uint32_t frame_,
              const eq::KeyEvent& keyEvent_,
              const eq::Vector3f& position_,
              const eq::Matrix4f& rotation_,
              const eq::Matrix4f& modelRotation_ )
            : frame( frame_ )
            , keyEvent( keyEvent_ )
            , position( position_ )
            , rotation( rotation_ )
            , modelRotation( modelRotation_ )
        {}

        Step( const eq::KeyEvent& keyEvent_,
              const eq::Vector3f& position_,
              const eq::Matrix4f& rotation_,
              const eq::Matrix4f& modelRotation_ )
            : frame( 0 )
            , keyEvent( keyEvent_ )
            , position( position_ )
            , rotation( rotation_ )
            , modelRotation( modelRotation_ )
        {}

        Step( const eq::Vector3f& position_,
              const eq::Matrix4f& rotation_,
              const eq::Matrix4f& modelRotation_ )
            : frame( 0 )
            , position( position_ )
            , rotation( rotation_ )
            , modelRotation( modelRotation_ )
        {
            keyEvent.key = 0;
        }

        uint32_t        frame;
        eq::KeyEvent    keyEvent;
        eq::Vector3f    position;
        eq::Matrix4f    rotation;
        eq::Matrix4f    modelRotation;
    };

    typedef std::vector< Step > Steps;

private:
    void read_from_stream( eq::Vector3f& v, std::istream& is );
    void read_from_stream( eq::Matrix4f& m, std::istream& is );
    void write_to_stream( const eq::Vector3f& v, std::ostream& os ) const;
    void write_to_stream( const eq::Matrix4f& m, std::ostream& os ) const;

    Steps        _steps;
    uint32_t     _currentStep;
    uint32_t     _currentFrame;
    uint32_t     _firstFrame;
    bool         _recording;
    bool         _playing;
    eq::KeyEvent _keyEvent;
};

}
}

#endif // EQUTIL_CAMCODER_H

