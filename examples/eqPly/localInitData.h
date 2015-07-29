
/* Copyright (c) 2007-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *               2015, David Steiner <steiner@ifi.uzh.ch> 
 *               2015, Enrique G. Paredes <egparedes@ifi.uzh.ch> 
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
! *
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

#ifndef EQ_PLY_LOCALINITDATA_H
#define EQ_PLY_LOCALINITDATA_H

#include "initData.h"

class FrameData;

namespace eqPly
{
/**
 * Manages the argument parsing and non-distributed part of the
 * initialization data.
 */
class LocalInitData : public InitData
{
public:
    LocalInitData();

    void parseArguments( const int argc, char** argv );

    const std::string& getPathInFilename()  const {
        return _pathInFilename;
    }
    const std::string& getPathOutFilename() const {
        return _pathOutFilename;
    }
    uint32_t           getMaxFrames()       const {
        return _maxFrames;
    }
    bool               useColor()           const {
        return _color;
    }
    bool               isResident()         const {
        return _isResident;
    }
    bool               exitAfterPlayback()  const {
        return _exitAfterPlayback;
    }

    const std::vector< std::string >& getFilenames() const
        { return _filenames; }

    const std::string& getTreePartition() const {
        return _treePartition;
    }
    bool createLongShowcase() const {
        return _createLongShowcase;
    }
    const eq::Vector3f& getHelixDollyArgs() const
        { return _showcaseDollyArgs; }
    const eq::Vector3f& getHelixRadiusArgs() const
        { return _showcaseRadiusArgs; }

    const eq::Vector3f& getHelixAngleArgs() const
        { return _showcaseAngleArgs; }

    LocalInitData& operator = ( const LocalInitData& from );

private:
    eq::Strings _filenames;
    std::string _pathInFilename;
    std::string _pathOutFilename;
    uint32_t    _maxFrames;
    bool        _exitAfterPlayback;
    bool        _color;
    bool        _isResident;
    std::string _treePartition;
    bool        _createLongShowcase;
    eq::Vector3f _showcaseDollyArgs;
    eq::Vector3f _showcaseRadiusArgs;
    eq::Vector3f _showcaseAngleArgs;
};
}

#endif // EQ_PLY_LOCALINITDATA_H
