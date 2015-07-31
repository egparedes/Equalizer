
/* Copyright (c)      2007, Tobias Wolf <twolf@access.unizh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
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


#ifndef PLYLIB_MODELTREEBASE_H
#define PLYLIB_MODELTREEBASE_H

#include "api.h"
#include "typedefs.h"
#include <fstream>

namespace eqPly
{
class ModelTreeDist;
}

namespace triply
{
/*  The abstract base class for all kinds of tree nodes.  */
class ModelTreeBase
{
public:
    static const unsigned LeftChildId;
    static const unsigned RightChildId;
    static const unsigned MaxZLevel;
    static const ZKey MinZKey;
    static const ZKey MaxZKey;

    virtual ~ModelTreeBase() { }

    PLYLIB_API virtual void draw( TreeRenderState& state ) const = 0;
    PLYLIB_API void drawBoundingSphere(TreeRenderState &state ) const;
    PLYLIB_API virtual Index getNumberOfVertices() const = 0;

    const BoundingSphere& getBoundingSphere() const { return _boundingSphere; }

    const float* getRange() const { return &_range[0]; }

    virtual const ModelTreeBase* getChild( unsigned char /*childId*/ ) const { return 0; }
    virtual ModelTreeBase* getChild( unsigned char /*childId*/ ) { return 0; }

    virtual unsigned getNumberOfChildren( ) const { return 0; }
    bool isLeaf( ) const { return getNumberOfChildren() == 0; }

protected:
    ModelTreeBase() : _boundingSphere( 0.0f )
    {
        _range[0] = 0.0f;
        _range[1] = 1.0f;
    }

    virtual void toStream( std::ostream& os )
    {
        os.write( reinterpret_cast< char* >( &_boundingSphere ),
                  sizeof( BoundingSphere ) );
        os.write( reinterpret_cast< char* >( &_range ), sizeof( Range ) );
    }

    virtual void fromMemory( char** addr, ModelTreeData& /*globalData*/ )
    {
        memRead( reinterpret_cast< char* >( &_boundingSphere ), addr,
                 sizeof( BoundingSphere ) );
        memRead( reinterpret_cast< char* >( &_range ), addr,
                 sizeof( Range ) );
    }

    virtual void setupKDTree( VertexData& modelData,
                              const Index start,
                              const Index length,
                              const Axis axis,
                              const size_t depth,
                              ModelTreeData& treeData ) = 0;

    virtual void setupMKDTree( VertexData& modelData,
                              const Index start,
                              const Index length,
                              const Axis axis,
                              const size_t depth,
                              ModelTreeData& treeData ) = 0;

    virtual void setupZOctree( VertexData& modelData,
                               const std::vector< ZKeyIndexPair >& zKeys,
                               const ZKey beginKey,
                               const ZKey endKey,
                               const Vertex center,
                               const size_t depth,
                               ModelTreeData& treeData ) = 0;

    virtual const BoundingSphere& updateBoundingSphere() = 0;
    virtual void updateRange() = 0;

    friend class ModelTreeDist;
    friend class ModelTreeNode;

    BoundingSphere  _boundingSphere;
    Range           _range;
};

} // namespace

#endif // PLYLIB_MODELTREEBASE_H
