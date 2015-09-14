
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
#include "meshData.h"
#include "treeDataManager.h"
#include "renderState.h"

namespace triply
{

ModelTreeLeaf::ModelTreeLeaf( ModelTreeData &treeData )
    : _treeData( treeData ),
      _indexStart( 0 ), _indexLength( 0 ), _vertexStart( 0 ), _vertexLength( 0 ),
      _dataReady( false ), _dataManager( 0 )
{
    for( size_t i=0; i <= ISegsUploaded; ++i)
    {
        _stats[i] = 0;
    }
}

ModelTreeLeaf::ModelTreeLeaf( ModelTreeData& treeData,
                              Index indexStart, Index indexLength,
                              Index vertexStart, ShortIndex vertexLength )
    : _treeData( treeData ),
      _indexStart( indexStart ), _indexLength( indexLength ),
      _vertexStart( vertexStart ), _vertexLength( vertexLength ),
      _dataReady( false ), _dataManager( 0 )
{
    for( size_t i=0; i <= ISegsUploaded; ++i)
    {
        _stats[i] = 0;
    }
}

void ModelTreeLeaf::clear()
{
#if 0
ifndef NDEBUG
    TRIPLYINFO << "LeafStats [v-" << _vertexStart << "] 0-Rendered   : \t" << _stats[Rendered] << std::endl;
    TRIPLYINFO << "LeafStats [v-" << _vertexStart << "] 1-Uploaded   : \t" << _stats[Uploaded] << std::endl;
    TRIPLYINFO << "LeafStats [v-" << _vertexStart << "] 2-DataRead   : \t" << _stats[DataRead] << std::endl;
    TRIPLYINFO << "LeafStats [v-" << _vertexStart << "] 3-DataDiscard: \t" << _stats[DataDiscard] << std::endl;
    TRIPLYINFO << "LeafStats [v-" << _vertexStart << "] 4-VSegsUpload: \t" << _stats[VSegsUploaded] << std::endl;
    TRIPLYINFO << "LeafStats [v-" << _vertexStart << "] 5-ISegsUpload: \t" << _stats[ISegsUploaded] << std::endl;
    TRIPLYASSERT( _stats[DataRead] == _stats[DataDiscard] );
#endif

    discardLocalData();
    _boundingBox[0] = Vertex::ZERO;
    _boundingBox[1] = Vertex::ZERO;
    _indexStart = 0;
    _indexLength = 0;
    _vertexStart = 0;
    _vertexLength = 0;
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
        _boundingBox[0][0] = std::min( _boundingBox[0][0], vertex[0] );
        _boundingBox[1][0] = std::max( _boundingBox[1][0], vertex[0] );
        _boundingBox[0][1] = std::min( _boundingBox[0][1], vertex[1] );
        _boundingBox[1][1] = std::max( _boundingBox[1][1], vertex[1] );
        _boundingBox[0][2] = std::min( _boundingBox[0][2], vertex[2] );
        _boundingBox[1][2] = std::max( _boundingBox[1][2], vertex[2] );
    }

    // We determine a bounding sphere by:
    // 1) Using the inner sphere of the leaf cell bounding box as an estimate
    // 2) Test all points to be in that sphere
    // 3) Expand the sphere to contain all points outside.

    // 1) get inner sphere of bounding box as an initial estimate      
    _boundingSphere.x() = ( _boundingBox[0].x() + _boundingBox[1].x() ) * 0.5f;
    _boundingSphere.y() = ( _boundingBox[0].y() + _boundingBox[1].y() ) * 0.5f;
    _boundingSphere.z() = ( _boundingBox[0].z() + _boundingBox[1].z() ) * 0.5f;

    _boundingSphere.w()  = std::max( _boundingBox[1].x() - _boundingBox[0].x(),
                                   _boundingBox[1].y() - _boundingBox[0].y() );
    _boundingSphere.w()  = std::max( _boundingBox[1].z() - _boundingBox[0].z(),
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
    _stats[Uploaded]++;
    if( !_dataReady )
        setupLocalData( state.useColors(), state.getDataManager( ) );

    switch( state.getRenderMode() )
    {
    case RENDER_MODE_IMMEDIATE:
        break;

    case RENDER_MODE_BUFFER_OBJECT:
    {
        const char* charThis = reinterpret_cast< const char* >( this );

        // Allocate empty VBOs
        if( data[VERTEX_BUFFER_TYPE] == state.INVALID )
            data[VERTEX_BUFFER_TYPE] = state.newBufferObject( charThis + 0 );
        glBindBuffer( GL_ARRAY_BUFFER, data[VERTEX_BUFFER_TYPE] );
        glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Vertex ), 0,
                      GL_STATIC_DRAW );

        if( data[NORMAL_BUFFER_TYPE] == state.INVALID )
            data[NORMAL_BUFFER_TYPE] = state.newBufferObject( charThis + 1 );
        glBindBuffer( GL_ARRAY_BUFFER, data[NORMAL_BUFFER_TYPE] );
        glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Normal ), 0,
                      GL_STATIC_DRAW );

        if( data[COLOR_BUFFER_TYPE] == state.INVALID )
            data[COLOR_BUFFER_TYPE] = state.newBufferObject( charThis + 2 );
        if( state.useColors() )
        {
            glBindBuffer( GL_ARRAY_BUFFER, data[COLOR_BUFFER_TYPE] );
            glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Color ), 0,
                          GL_STATIC_DRAW );
        }

        if( data[INDEX_BUFFER_TYPE] == state.INVALID )
            data[INDEX_BUFFER_TYPE] = state.newBufferObject( charThis + 3 );
        glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, data[INDEX_BUFFER_TYPE] );
        glBufferData( GL_ELEMENT_ARRAY_BUFFER, _indexLength * sizeof( ShortIndex ), 0,
                      GL_STATIC_DRAW );

        // Upload data to VBOs
        TRIPLYASSERT( _buffers[VERTEX_BUFFER_TYPE].numSegments() ==
                            _buffers[NORMAL_BUFFER_TYPE].numSegments() );
        TRIPLYASSERT( _buffers[VERTEX_BUFFER_TYPE].numSegments() ==
                            _buffers[COLOR_BUFFER_TYPE].numSegments()
                       || ! state.useColors() );

        size_t offset[3];
        for( unsigned int i=0; i < 3; ++i)
            offset[i] = 0;
        for( unsigned int i=0; i < _buffers[VERTEX_BUFFER_TYPE].numSegments(); ++i )
        {            
            _stats[VSegsUploaded]++;
            for( int bufferType=VERTEX_BUFFER_TYPE; bufferType < INDEX_BUFFER_TYPE;
                 ++bufferType )
            {
                if( bufferType != COLOR_BUFFER_TYPE || state.useColors() )
                {
                    SegmentedBuffer::Segment segment = _buffers[bufferType].getSegment( i );
                    glBindBuffer( GL_ARRAY_BUFFER, data[bufferType] );
                    glBufferSubData( GL_ARRAY_BUFFER, offset[bufferType],
                                     segment.size, segment.ptr );
                    offset[bufferType] += segment.size;
                }
            }
        }

        size_t idxOffset = 0;
        for( unsigned int i=0; i < _buffers[INDEX_BUFFER_TYPE].numSegments(); ++i )
        {
            _stats[ISegsUploaded]++;
            SegmentedBuffer::Segment segment =
                    _buffers[INDEX_BUFFER_TYPE].getSegment( i );
            glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, data[INDEX_BUFFER_TYPE] );
            glBufferSubData( GL_ELEMENT_ARRAY_BUFFER, idxOffset, segment.size, segment.ptr );
            idxOffset += segment.size;
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
    if( state.stopRendering() )
        return;

    state.updateRegion( _boundingBox );
    switch( state.getRenderMode() )
    {
      case RENDER_MODE_IMMEDIATE:
          renderImmediate( state );
          return;
      case RENDER_MODE_BUFFER_OBJECT:
          renderBufferObject( state );
          discardLocalData();
          return;
      case RENDER_MODE_DISPLAY_LIST:
      default:
          renderDisplayList( state );
          discardLocalData();
          return;
    }
}

/*  Render the leaf with buffer objects.  */
void ModelTreeLeaf::renderBufferObject( RenderState& state ) const
{
    _stats[Rendered]++;
    GLuint buffers[4];
    for( int i = 0; i < 4; ++i )
    {
        buffers[i] =
            state.getBufferObject( reinterpret_cast< const char* >(this) + i );
    }

    if( buffers[VERTEX_BUFFER_TYPE] == state.INVALID ||
        buffers[NORMAL_BUFFER_TYPE] == state.INVALID ||
        buffers[COLOR_BUFFER_TYPE] == state.INVALID  ||
        buffers[INDEX_BUFFER_TYPE] == state.INVALID )
    {
        setupRendering( state, buffers );
    }


    if( state.useColors() )
    {
        glBindBuffer( GL_ARRAY_BUFFER, buffers[COLOR_BUFFER_TYPE] );
        glColorPointer( 3, GL_UNSIGNED_BYTE, 0, 0 );
    }
    glBindBuffer( GL_ARRAY_BUFFER, buffers[NORMAL_BUFFER_TYPE] );
    glNormalPointer( GL_FLOAT, 0, 0 );
    glBindBuffer( GL_ARRAY_BUFFER, buffers[VERTEX_BUFFER_TYPE] );
    glVertexPointer( 3, GL_FLOAT, 0, 0 );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, buffers[INDEX_BUFFER_TYPE] );
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
    setupRendering( state, NULL );
    glBegin( GL_TRIANGLES );
    for( Index i = 0; i < _indexLength; ++i )
    {
        const ShortIndex idx = getIndex( i );
        if( state.useColors() )
            glColor3ubv( &(getColor( idx )[0]) );
        glNormal3fv( &(getNormal( idx )[0]) );
        glVertex3fv( &(getVertex( idx )[0]) );
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

void ModelTreeLeaf::setupLocalData( bool useColors,
                                    TreeDataManager* dataManager ) const
{
    if( _dataReady )
        return;

    _stats[DataRead]++;
    if( dataManager == 0 )
    {
        // In-core rendering: use single segment buffers
        _buffers[VERTEX_BUFFER_TYPE].set( &_treeData.vertices[_vertexStart][0],
                                          _vertexLength * sizeof( Vertex ));
        _buffers[NORMAL_BUFFER_TYPE].set( &_treeData.normals[_vertexStart][0],
                                          _vertexLength * sizeof( Normal ));
        if( useColors )
        {
            _buffers[COLOR_BUFFER_TYPE].set( &_treeData.colors[_vertexStart][0],
                                             _vertexLength * sizeof( Color ));
        }
        _buffers[INDEX_BUFFER_TYPE].set( &_treeData.indices[_indexStart],
                                         _indexLength * sizeof( ShortIndex ) );
    }
    else
    {
        // Out-of-core rendering: get segmented buffers from shared DataManager instance
        if( _dataManager == 0)
        {
            _dataManager = dataManager;
        }
        else
        {
            TRIPLYASSERT( _dataManager == dataManager );
        }

        if( useColors )
        {
            _dataManager->getVertexData( _vertexStart, _vertexLength,
                                         _buffers[VERTEX_BUFFER_TYPE],
                                         _buffers[NORMAL_BUFFER_TYPE],
                                         _buffers[COLOR_BUFFER_TYPE] );
        }
        else
        {
            _dataManager->getVertexData( _vertexStart, _vertexLength,
                                         _buffers[VERTEX_BUFFER_TYPE],
                                         _buffers[NORMAL_BUFFER_TYPE] );
        }
        _dataManager->getIndexData( _indexStart, _indexLength,
                                    _buffers[INDEX_BUFFER_TYPE] );
    }

    _dataReady =  true;
}

void ModelTreeLeaf::discardLocalData() const
{
    if( !_dataReady )
        return;

    _stats[DataDiscard]++;
    if( _dataManager != 0 )
    {
        _dataManager->discardVertexData( _vertexStart, _vertexLength );
        _dataManager->discardIndexData( _indexStart, _indexLength );
    }

    for( unsigned i=0; i < 4; ++i)
        _buffers[i].reset();

    _dataReady = false;
}

} // namespace triply
