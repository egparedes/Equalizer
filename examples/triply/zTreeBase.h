
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
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


#ifndef PLYLIB_ZTREEBASE_H
#define PLYLIB_ZTREEBASE_H

#include "api.h"
#include "typedefs.h"
#include <fstream>

namespace eqPly
{
class ZTreeDist;
}

namespace triply {

/*  The abstract base class for all kinds of kd-tree nodes.  */
class ZTreeBase
{
public:
    static const size_t NumberOfLevels;
    static const size_t MaxLevel;
    static const ZKey MinZKey;
    static const ZKey MaxZKey;

    virtual ~ZTreeBase() {};

    PLYLIB_API virtual void draw( VertexBufferState& state ) const = 0;
    PLYLIB_API void drawBoundingSphere( VertexBufferState& state ) const;
    PLYLIB_API virtual Index getNumberOfVertices() const = 0;

    const BoundingSphere& getBoundingSphere() const
    { return _boundingSphere; }

    const float* getRange() const { return &_range[0]; }

    virtual unsigned getNumberofChildren( ) const { return 0; }
    virtual const ZTreeBase* getChild( unsigned char /*childId*/ ) const { return 0; }
    virtual ZTreeBase* getChild( unsigned char /*childId*/ ) { return 0; }

    PLYLIB_API virtual const BoundingSphere& updateBoundingSphere() = 0;

protected:
    ZTreeBase() : _boundingSphere( 0.0f )
    {
        _range[0] = 0.0f;
        _range[1] = 1.0f;
        _zRange[0] = MinZKey;
        _zRange[1] = MaxZKey;
    }

    virtual void toStream( std::ostream& os )
    {
        os.write( reinterpret_cast< char* >( &_boundingSphere ),
                  sizeof( BoundingSphere ) );
        os.write( reinterpret_cast< char* >( &_range ), sizeof( Range ) );
    }

    virtual void fromMemory( char** addr, VertexBufferData& /*globalData*/ )
    {
    PLYLIBWARN << "--ZTREEPLY-- ZTreeBase::fromMemory()" << std::endl;
        memRead( reinterpret_cast< char* >( &_boundingSphere ), addr,
                 sizeof( BoundingSphere ) );
        memRead( reinterpret_cast< char* >( &_range ), addr,
                 sizeof( Range ) );
    PLYLIBWARN << "--ZTREEPLY-- ZTreeBase::fromMemory() ---- END" << std::endl;
    }

    virtual void setupTree(VertexData& modelData,
                           std::vector< ZKeyIndexPair >& zKeys,
                           const ZKey beginKey,const ZKey endKey, const size_t depth,
                           Vertex center, VertexBufferData& globalData ) = 0;

    virtual void updateRange() = 0;

    friend class ZTreeDist;
    friend class ZTreeNode;
    BoundingSphere  _boundingSphere;
    Range           _range;
    ZRange          _zRange;
};

} // namespace

#endif // PLYLIB_ZTREEBASE_H
