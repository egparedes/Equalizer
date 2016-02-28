
/* Copyright (c) 2007-2015, Tobias Wolf <twolf@access.unizh.ch>
 *                          Stefan Eilemann <eile@equalizergraphics.com>
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

#ifndef TRIPLY_MODELTREELEAF_H
#define TRIPLY_MODELTREELEAF_H

#include "modelTreeBase.h"
#include "segmentedBuffer.h"

namespace triply
{

/*  The class for kd-tree leaf nodes.  */
class ModelTreeLeaf : public ModelTreeBase
{
public:
    explicit ModelTreeLeaf( ModelTreeData& treeData );
    explicit ModelTreeLeaf( ModelTreeData& treeData,
                            Index indexStart, Index indexLength,
                            Index vertexStart, ShortIndex vertexLength );

    virtual void clear() override;

    virtual void draw( RenderState& state ) const override;
    virtual Index getNumberOfVertices() const override { return _indexLength; }

protected:
    virtual void toStream( std::ostream& os ) override;
    virtual void fromMemory( char** addr, ModelTreeData& treeData ) override;

    virtual const BoundingSphere& updateBoundingSphere() override;
    virtual void updateRange() override;

private:
    void setupRendering(RenderState& state, GLuint* glBuffers ) const;
    void loadLeafData( bool useColors,
                        TreeDataManager *dataManager=0 ) const;
    void discardLeafData() const;


    void renderImmediate( RenderState& state ) const;
    void renderDisplayList( RenderState& state ) const;
    void renderBufferObject( RenderState& state ) const;
    void renderVAObject( RenderState& state ) const;

    const Vertex& getVertex( const size_t i ) const
    {
        return _dataBuffers[VERTEX_BUFFER_TYPE].at< Vertex >( i );
    }

    const Color& getColor( const size_t i ) const
    {
        return _dataBuffers[COLOR_BUFFER_TYPE].at< Color >( i );
    }

    const Normal& getNormal( const size_t i ) const
    {
        return _dataBuffers[NORMAL_BUFFER_TYPE].at< Normal >( i );
    }

    const ShortIndex& getIndex( const size_t i ) const
    {
        return _dataBuffers[INDEX_BUFFER_TYPE].at< ShortIndex >( i );
    }

    friend class ModelTreeDist;

    ModelTreeData&              _treeData;
    BoundingBox                 _boundingBox;
    Index                       _indexStart;
    Index                       _indexLength;
    Index                       _vertexStart;
    ShortIndex                  _vertexLength;

    // Mutable for out-of-core rendering
    mutable bool                _dataLoaded;
    mutable TreeDataManager*    _dataManager;
    mutable SegmentedBuffer     _dataBuffers[4];

    enum DrawStatsFields
    {
        RENDERED=0, UPLOADED, DATA_READ, DATA_DISCARD, VSEGS_UPLOADED, ISEGS_UPLOADED,
        DRAW_STATS_FIELDS_ALL
    };
    mutable size_t _drawStats[DRAW_STATS_FIELDS_ALL];
};

} // namespace

#endif // TRIPLY_MODELTREELEAF_H
