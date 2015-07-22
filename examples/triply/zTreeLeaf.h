
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2007, Tobias Wolf <twolf@access.unizh.ch>
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

#ifndef PLYLIB_ZTREELEAF_H
#define PLYLIB_ZTREELEAF_H

#include "zTreeBase.h"
#include "virtualVertexBufferData.h"

namespace triply
{
/*  The class for kd-tree leaf nodes.  */
class ZTreeLeaf : public ZTreeBase
{
public:
    ZTreeLeaf( VertexBufferData& data );
    virtual ~ZTreeLeaf();

    virtual void draw( VertexBufferState& state ) const;
    virtual Index getNumberOfVertices() const { return _indexLength; }

protected:
    virtual void toStream( std::ostream& os );
    virtual void fromMemory( char** addr, VertexBufferData& globalData );

    virtual void setupTree(VertexData& modelData,
                           std::vector< ZKeyIndexPair >& zKeys,
                           const ZKey beginKey,const ZKey endKey, const size_t depth,
                           Vertex center, VertexBufferData& globalData ) override;

    virtual const BoundingSphere& updateBoundingSphere();
    virtual void updateRange();

private:
    void setupRendering( VertexBufferState& state, GLuint* data ) const;
    void renderImmediate( VertexBufferState& state ) const;
    void renderDisplayList( VertexBufferState& state ) const;
    void renderBufferObject( VertexBufferState& state ) const;

    void loadVirtualData(VirtualVertexBufferDataPtr virtualVBD, bool useColors) const;

    friend class ZTreeDist;

    VertexBufferData&   _globalData;
    BoundingBox         _boundingBox;
    Vertex              _center;
    Index               _vertexStart;
    Index               _indexStart;
    Index               _indexLength;
    ShortIndex          _vertexLength;

    // For out-of-core rendering
    mutable VirtualVertexBufferDataPtr  _virtualGlobalData;
    mutable VertexVB         _verticesVB;
    mutable ColorVB          _colorsVB;
    mutable NormalVB         _normalsVB;
    mutable ShortIndexVB     _indicesVB;
    mutable bool            _vDataLoaded;
};

} // namespace

#endif // PLYLIB_ZTREELEAF_H
