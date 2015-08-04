
/* Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
 *          2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
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


#include "modelTreeLeaf.h"
#include "modelTreeData.h"
#include "treeRenderState.h"
#include "vertexData.h"
#include <map>

namespace triply
{

ModelTreeLeaf::ModelTreeLeaf( ModelTreeData &treeData )
    : _treeData( treeData ),
      _vertexStart( 0 ), _indexStart( 0 ), _indexLength( 0 ), _vertexLength( 0 )
{
}

ModelTreeLeaf::~ModelTreeLeaf()
{
}

/*  Finish partial multiway kd-tree setup - sort, reindex and merge into global data.  */
void ModelTreeLeaf::setupMKDTree( VertexData& modelData,
                                  const Index start, const Index length,
                                  const Axis axis, const size_t /*depth*/,
                                  ModelTreeData& treeData,
                                  boost::progress_display& /*progress*/ )
{
    modelData.sort( start, length, axis );
    _vertexStart = treeData.vertices.size();
    _vertexLength = 0;
    _indexStart = treeData.indices.size();
    _indexLength = 0;

    // stores the new indices (relative to _start)
    std::map< Index, ShortIndex > newIndex;

    for( Index t = 0; t < length; ++t )
    {
        for( Index v = 0; v < 3; ++v )
        {
            Index i = modelData.triangles[start + t][v];
            if( newIndex.find( i ) == newIndex.end() )
            {
                newIndex[i] = _vertexLength++;
                // assert number of vertices does not exceed SmallIndex range
                TRIPLYASSERT( _vertexLength );
                treeData.vertices.push_back( modelData.vertices[i] );
                if( treeData.hasColors )
                    treeData.colors.push_back( modelData.colors[i] );
                treeData.normals.push_back( modelData.normals[i] );
            }
            treeData.indices.push_back( newIndex[i] );
            ++_indexLength;
        }
    }

#ifndef NDEBUG
    TRIPLYINFO << "setupKDTree" << "( " << _indexStart << ", " << _indexLength
             << "; start " << _vertexStart << ", " << _vertexLength
             << " vertices)." << std::endl;
#endif
}

/*  Finish partial octree setup - sort, reindex and merge into global data.  */
void ModelTreeLeaf::setupZOctree( VertexData& modelData,
                                  const std::vector< ZKeyIndexPair >& zKeys,
                                  const ZKey beginKey, const ZKey endKey,
                                  const Vertex center, const size_t depth,
                                  ModelTreeData& treeData,
                                  boost::progress_display& /*progress*/ )
{
    BoundingBox bbox = modelData.getBoundingBox();
    Vertex halfCellSize = (bbox[1] - bbox[0]) / (2 << depth);
    _vertexStart = treeData.vertices.size();
    _vertexLength = 0;
    _indexStart = treeData.indices.size();
    _indexLength = 0;

    std::vector< ZKeyIndexPair >::const_iterator beginIt =
            std::lower_bound( zKeys.begin(), zKeys.end(), beginKey,
                              ZKeyIndexPairLessCmpFunctor());
    std::vector< ZKeyIndexPair >::const_iterator endIt =
            std::lower_bound( beginIt, zKeys.end(), endKey,
                              ZKeyIndexPairLessCmpFunctor());
    Index start = std::distance( zKeys.begin(), beginIt );
    Index length = std::distance( beginIt, endIt );

    // stores the new indices (relative to start)
    std::map< Index, ShortIndex > newIndex;

    for( Index t = 0; t < length; ++t )
    {
        for( Index v = 0; v < 3; ++v )
        {
            Index i = modelData.triangles[ zKeys[start + t].second ][v];
            if( newIndex.find( i ) == newIndex.end() )
            {
                newIndex[i] = _vertexLength++;
                // assert number of vertices does not exceed SmallIndex range
                TRIPLYASSERT( _vertexLength );
                treeData.vertices.push_back( modelData.vertices[i] );
                if( _treeData.hasColors )
                    treeData.colors.push_back( modelData.colors[i] );
                treeData.normals.push_back( modelData.normals[i] );
            }
            treeData.indices.push_back( newIndex[i] );
            ++_indexLength;
        }
    }

     // Initialize and compute leaf bounding box
     _boundingBox[0] = center - halfCellSize;
     _boundingBox[1] = center + halfCellSize;
     
#ifndef NDEBUG
    TRIPLYINFO << "setupZOctree" << "( " << _indexStart << ", " << _indexLength
             << " / start " << _vertexStart << ", " << _vertexLength
             << " vertices)." << std::endl;
#endif
}

/*  Compute the bounding sphere of the leaf's indexed vertices.  */
const BoundingSphere& ModelTreeLeaf::updateBoundingSphere()
{
    // Initialize leaf bounding box using the vertices inside the leaf node
    _boundingBox[0] = _treeData.vertices[ _vertexStart ];
    _boundingBox[1] = _treeData.vertices[ _vertexStart ];

    for( Index i = 1; i < _vertexLength; ++i )
    {
        const Vertex& vertex = _treeData.vertices[ _vertexStart + i ];
        _boundingBox[0][0] = LB_MIN( _boundingBox[0][0], vertex[0] );
        _boundingBox[1][0] = LB_MAX( _boundingBox[1][0], vertex[0] );
        _boundingBox[0][1] = LB_MIN( _boundingBox[0][1], vertex[1] );
        _boundingBox[1][1] = LB_MAX( _boundingBox[1][1], vertex[1] );
        _boundingBox[0][2] = LB_MIN( _boundingBox[0][2], vertex[2] );
        _boundingBox[1][2] = LB_MAX( _boundingBox[1][2], vertex[2] );
    }

    // We determine a bounding sphere by:
    // 1) Using the inner sphere of the leaf cell bounding box as an estimate
    // 2) Test all points to be in that sphere
    // 3) Expand the sphere to contain all points outside.

    // 1) get inner sphere of bounding box as an initial estimate      
    _boundingSphere.x() = ( _boundingBox[0].x() + _boundingBox[1].x() ) * 0.5f;
    _boundingSphere.y() = ( _boundingBox[0].y() + _boundingBox[1].y() ) * 0.5f;
    _boundingSphere.z() = ( _boundingBox[0].z() + _boundingBox[1].z() ) * 0.5f;

    _boundingSphere.w()  = LB_MAX( _boundingBox[1].x() - _boundingBox[0].x(),
                                   _boundingBox[1].y() - _boundingBox[0].y() );
    _boundingSphere.w()  = LB_MAX( _boundingBox[1].z() - _boundingBox[0].z(),
                                   _boundingSphere.w() );
    _boundingSphere.w() *= 0.5f;
    
    float  radius        = _boundingSphere.w();
    float  radiusSquared =  radius * radius;
    Vertex center( _boundingSphere.array );

    // 2) test all points to be in the estimated bounding sphere
    for( Index offset = 0; offset < _vertexLength; ++offset )
    {
        const Vertex& vertex = _treeData.vertices[ _vertexStart + offset ];
        const Vertex centerToPoint   = vertex - center;
        const float  distanceSquared = centerToPoint.squared_length();

        if( distanceSquared > radiusSquared ) 
        {
            // 3) expand sphere to contain 'outside' points
            const float distance = sqrtf( distanceSquared );
            const float delta    = distance - radius;

            radius        = ( radius + distance ) * .5f;
            radiusSquared = radius * radius;
            const Vertex normDelta = normalize( centerToPoint ) * ( 0.5f * delta );
            center       += normDelta;

            LBASSERTINFO( Vertex( vertex-center ).squared_length() <=
                    ( radiusSquared + 2.f * std::numeric_limits<float>::epsilon( )),
                        vertex << " c " << center << " r " << radius << " ("
                                << Vertex( vertex-center ).length() << ")" );
        }
    }

#ifndef NDEBUG
    // 2a) re-test all points to be in the estimated bounding sphere
    for( Index offset = 0; offset < _vertexLength; ++offset )
    {
        const Vertex& vertex = _treeData.vertices[ _vertexStart + offset];

        const Vertex centerToPoint   = vertex - center;
        const float  distanceSquared = centerToPoint.squared_length();
        LBASSERTINFO( distanceSquared <=
                ( radiusSquared + 2.f * std::numeric_limits<float>::epsilon( )),
                      vertex << " c " << center << " r " << radius << " ("
                             << Vertex( vertex-center ).length() << ")" );
    }
#endif

    // store optimal bounding sphere
    _boundingSphere.x() = center.x();
    _boundingSphere.y() = center.y();
    _boundingSphere.z() = center.z();
    _boundingSphere.w() = radius;

#ifndef NDEBUG
    TRIPLYINFO << "updateBoundingSphere" << "( " << _boundingSphere << " )."
             << std::endl;
#endif

    return _boundingSphere;
}


/*  Compute the range of this child.  */
void ModelTreeLeaf::updateRange()
{
    _range[0] = 1.0f * _indexStart / _treeData.indices.size();
    _range[1] = _range[0] + 1.0f * _indexLength / _treeData.indices.size();

#ifndef NDEBUG
    TRIPLYINFO << "updateRange" << "( " << _range[0] << ", " << _range[1]
             << " )." << std::endl;
#endif
}

#define glewGetContext state.glewGetContext

/*  Set up rendering of the leaf nodes.  */
void ModelTreeLeaf::setupRendering( RenderState& state,
                                       GLuint* data ) const
{
    switch( state.getRenderMode() )
    {
    case RENDER_MODE_IMMEDIATE:
        break;

    case RENDER_MODE_BUFFER_OBJECT:
    {
        const char* charThis = reinterpret_cast< const char* >( this );

        // If out-of-core rendering, the real data will be uploaded later
        Vertex* verticesPtr =
                ( state.useOutOfCore() ) ? NULL : &_treeData.vertices[_vertexStart];
        Color* colorsPtr =
                ( state.useOutOfCore() ) ? NULL : &_treeData.colors[_vertexStart];
        Normal* normalsPtr =
                ( state.useOutOfCore() ) ? NULL : &_treeData.normals[_vertexStart];
        ShortIndex* indicesPtr =
                ( state.useOutOfCore() ) ? NULL : &_treeData.indices[_indexStart];

        if( data[VERTEX_OBJECT] == state.INVALID )
            data[VERTEX_OBJECT] = state.newBufferObject( charThis + 0 );
        glBindBuffer( GL_ARRAY_BUFFER, data[VERTEX_OBJECT] );
        glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Vertex ),
                      verticesPtr, GL_STATIC_DRAW );

        if( data[NORMAL_OBJECT] == state.INVALID )
            data[NORMAL_OBJECT] = state.newBufferObject( charThis + 1 );
        glBindBuffer( GL_ARRAY_BUFFER, data[NORMAL_OBJECT] );
        glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Normal ),
                      normalsPtr, GL_STATIC_DRAW );

        if( data[COLOR_OBJECT] == state.INVALID )
            data[COLOR_OBJECT] = state.newBufferObject( charThis + 2 );
        if( state.useColors() )
        {
            glBindBuffer( GL_ARRAY_BUFFER, data[COLOR_OBJECT] );
            glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Color ),
                          colorsPtr, GL_STATIC_DRAW );
        }

        if( data[INDEX_OBJECT] == state.INVALID )
            data[INDEX_OBJECT] = state.newBufferObject( charThis + 3 );
        glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, data[INDEX_OBJECT] );
        glBufferData( GL_ELEMENT_ARRAY_BUFFER, _indexLength * sizeof( ShortIndex ),
                      indicesPtr, GL_STATIC_DRAW );

        // If using out-of-core rendering, upload real data to buffers in a page-by-page way
        if( state.useOutOfCore() )
        {
            std::size_t vbLength = 0;
            std::size_t verticesOffset = 0;
            std::size_t colorsOffset = 0;
            std::size_t normalsOffset = 0;

            TRIPLYASSERT ( _verticesVB.numBlocks() == _normalsVB.numBlocks() );
            TRIPLYASSERT ( !state.useColors() ||
                           _verticesVB.numBlocks() == _colorsVB.numBlocks() );

            for( unsigned int i=0; i < _verticesVB.numBlocks(); ++i )
            {
                _verticesVB.getMemBlock(i, verticesPtr, vbLength);
                glBindBuffer( GL_ARRAY_BUFFER, data[VERTEX_OBJECT] );
                glBufferSubData( GL_ARRAY_BUFFER, verticesOffset, vbLength * sizeof( Vertex ),
                                 verticesPtr );
                verticesOffset += vbLength * sizeof( Vertex ) ;

                _normalsVB.getMemBlock(i, normalsPtr, vbLength);
                glBindBuffer( GL_ARRAY_BUFFER, data[NORMAL_OBJECT] );
                glBufferSubData( GL_ARRAY_BUFFER, normalsOffset, vbLength * sizeof( Normal ),
                                 normalsPtr );
                normalsOffset += vbLength * sizeof( Normal ) ;

                if( state.useColors() )
                {
                    _colorsVB.getMemBlock(i, colorsPtr, vbLength);
                    glBindBuffer( GL_ARRAY_BUFFER, data[COLOR_OBJECT] );
                    glBufferSubData( GL_ARRAY_BUFFER, colorsOffset, vbLength * sizeof( Color ),
                                     colorsPtr );
                    colorsOffset += vbLength * sizeof( Color);
                }
            }

            std::size_t idxOffset = 0;
            for( unsigned int i=0; i < _indicesVB.numBlocks(); ++i )
            {
                _indicesVB.getMemBlock(i, indicesPtr, vbLength);
                glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, data[INDEX_OBJECT] );
                glBufferSubData( GL_ELEMENT_ARRAY_BUFFER, idxOffset, vbLength * sizeof( ShortIndex ),
                                 indicesPtr );
                idxOffset += vbLength * sizeof( ShortIndex ) ;
            }
        }

        break;
    }
    case RENDER_MODE_DISPLAY_LIST:
    default:
    {
        if( data[0] == state.INVALID )
        {
            char* key = (char*)( this );
            if( state.useColors( ))
                ++key;
            data[0] = state.newDisplayList( key );
        }
        glNewList( data[0], GL_COMPILE );
        renderImmediate( state );
        glEndList();
        break;
    }
    }
}

/*  Draw the leaf.  */
void ModelTreeLeaf::draw( RenderState& state ) const
{
    if( state.stopRendering( ) )
        return;

    if( state.useOutOfCore() && !isDataLoaded() )
        loadVirtualData( state.getPagedData(), state.useColors() );
    state.updateRegion( _boundingBox );
    switch( state.getRenderMode() )
    {
      case RENDER_MODE_IMMEDIATE:
          renderImmediate( state );
          return;
      case RENDER_MODE_BUFFER_OBJECT:
          renderBufferObject( state );
          if( state.useOutOfCore() )
              freeVirtualData();
          return;
      case RENDER_MODE_DISPLAY_LIST:
      default:
          renderDisplayList( state );
          return;
    }
}

/*  Render the leaf with buffer objects.  */
void ModelTreeLeaf::renderBufferObject( RenderState& state ) const
{
    GLuint buffers[4];
    for( int i = 0; i < 4; ++i )
    {
        buffers[i] =
            state.getBufferObject( reinterpret_cast< const char* >(this) + i );
    }

    if( buffers[VERTEX_OBJECT] == state.INVALID ||
        buffers[NORMAL_OBJECT] == state.INVALID ||
        buffers[COLOR_OBJECT] == state.INVALID ||
        buffers[INDEX_OBJECT] == state.INVALID )
    {
        setupRendering( state, buffers );
    }


    if( state.useColors() )
    {
        glBindBuffer( GL_ARRAY_BUFFER, buffers[COLOR_OBJECT] );
        glColorPointer( 3, GL_UNSIGNED_BYTE, 0, 0 );
    }
    glBindBuffer( GL_ARRAY_BUFFER, buffers[NORMAL_OBJECT] );
    glNormalPointer( GL_FLOAT, 0, 0 );
    glBindBuffer( GL_ARRAY_BUFFER, buffers[VERTEX_OBJECT] );
    glVertexPointer( 3, GL_FLOAT, 0, 0 );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, buffers[INDEX_OBJECT] );
    glDrawElements( GL_TRIANGLES, GLsizei(_indexLength), GL_UNSIGNED_SHORT, 0 );
}


/*  Render the leaf with a display list.  */
inline
void ModelTreeLeaf::renderDisplayList( RenderState& state ) const
{
    char* key = (char*)( this );
    if( state.useColors( ))
        ++key;

    GLuint displayList = state.getDisplayList( key );

    if( displayList == state.INVALID )
        setupRendering( state, &displayList );

    glCallList( displayList );
}


/*  Render the leaf with immediate mode primitives or vertex arrays.  */
inline
void ModelTreeLeaf::renderImmediate( RenderState& state ) const
{
    glBegin( GL_TRIANGLES );
    if( state.useOutOfCore() )
    {
        for( Index idx = 0; idx < _indexLength; ++idx )
        {
            const ShortIndex i = _indicesVB[idx];
            if( state.useColors() )
                glColor3ubv( &(_colorsVB[i][0]) );
            glNormal3fv( &(_normalsVB[i][0]) );
            glVertex3fv( &(_verticesVB[i][0]) );
        }
    }
    else
    {
        for( Index offset = 0; offset < _indexLength; ++offset )
        {
            const Index i =_vertexStart + _treeData.indices[_indexStart + offset];
            if( state.useColors() )
                glColor3ubv( &_treeData.colors[i][0] );
            glNormal3fv( &_treeData.normals[i][0] );
            glVertex3fv( &_treeData.vertices[i][0] );
        }
    }
    glEnd();
}


/*  Read leaf node from memory.  */
void ModelTreeLeaf::fromMemory( char** addr, ModelTreeData& treeData )
{
    size_t nodeType;
    memRead( reinterpret_cast< char* >( &nodeType ), addr, sizeof( size_t ) );
    if( nodeType != LEAF_TYPE )
        throw MeshException( "Error reading binary file. Expected a leaf "
                             "node, but found something else instead." );
    ModelTreeBase::fromMemory( addr, treeData );
    memRead( reinterpret_cast< char* >( &_boundingBox ), addr,
             sizeof( BoundingBox ) );
    memRead( reinterpret_cast< char* >( &_vertexStart ), addr,
             sizeof( Index ) );
    memRead( reinterpret_cast< char* >( &_vertexLength ), addr,
             sizeof( ShortIndex ) );
    memRead( reinterpret_cast< char* >( &_indexStart ), addr,
             sizeof( Index ) );
    memRead( reinterpret_cast< char* >( &_indexLength ), addr,
             sizeof( Index ) );
}


/*  Write leaf node to output stream.  */
void ModelTreeLeaf::toStream( std::ostream& os )
{
    size_t nodeType = LEAF_TYPE;
    os.write( reinterpret_cast< char* >( &nodeType ), sizeof( size_t ));
    ModelTreeBase::toStream( os );
    os.write( reinterpret_cast< char* >( &_boundingBox ), sizeof( BoundingBox));
    os.write( reinterpret_cast< char* >( &_vertexStart ), sizeof( Index ));
    os.write( reinterpret_cast< char* >( &_vertexLength ),sizeof( ShortIndex ));
    os.write( reinterpret_cast< char* >( &_indexStart ), sizeof( Index ));
    os.write( reinterpret_cast< char* >( &_indexLength ), sizeof( Index ));
}

bool ModelTreeLeaf::isDataLoaded( ) const
{
    return _verticesVB.isValid() && _colorsVB.isValid() && _normalsVB.isValid();
}

void ModelTreeLeaf::loadVirtualData(PagedTreeDataPtr pagedVBD, bool useColors) const
{
    if( isDataLoaded() )
        return;
    TRIPLYASSERT( pagedVBD != 0 );

//    pagedVBD->getVertexData( _vertexStart, _vertexLength, useColors,
//                             _verticesVB, _colorsVB, _normalsVB );
    
    if (!_verticesVB.isValid())
        pagedVBD->getVertices(_vertexStart, _vertexLength, _verticesVB );
    if (useColors && !_colorsVB.isValid())
        pagedVBD->getColors(_vertexStart, _vertexLength, _colorsVB );
    if (!_normalsVB.isValid())
        pagedVBD->getNormals(_vertexStart, _vertexLength, _normalsVB );
    if (!_indicesVB.isValid())
        pagedVBD->getIndices(_indexStart, _indexLength, _indicesVB );
}

void ModelTreeLeaf::freeVirtualData() const
{
    _verticesVB.discard();
    _colorsVB.discard();
    _normalsVB.discard();
    _indicesVB.discard();
}

}
