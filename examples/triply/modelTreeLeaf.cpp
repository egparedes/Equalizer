
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

#define glewGetContext state.glewGetContext
//#define LOGDRAW

namespace triply
{

namespace detail
{
struct DrawElementsIndirectCommand
{
    DrawElementsIndirectCommand( uint countArg=0, uint primCountArg=0, uint firstIndexArg=0,
                                 uint baseVertexArg=0, uint baseInstanceArg=0)
        : count( countArg ), primCount( primCountArg ), firstIndex( firstIndexArg ),
          baseVertex( baseVertexArg ), baseInstance( baseInstanceArg )
    { }

    uint  count;
    uint  primCount;
    uint  firstIndex;
    uint  baseVertex;
    uint  baseInstance;
};
} // namespace detail


ModelTreeLeaf::ModelTreeLeaf( ModelTreeData &treeData )
    : _treeData( treeData ),
      _indexStart( 0 ), _indexLength( 0 ), _vertexStart( 0 ), _vertexLength( 0 ),
      _dataLoaded( false ), _dataManager( 0 )
{
#ifdef LOGDRAW
    for( size_t i=0; i < DRAW_STATS_FIELDS_ALL; ++i)
    {
        _drawStats[i] = 0;
    }
#endif
}

ModelTreeLeaf::ModelTreeLeaf( ModelTreeData& treeData,
                              Index indexStart, Index indexLength,
                              Index vertexStart, ShortIndex vertexLength )
    : _treeData( treeData ),
      _indexStart( indexStart ), _indexLength( indexLength ),
      _vertexStart( vertexStart ), _vertexLength( vertexLength ),
      _dataLoaded( false ), _dataManager( 0 )
{
#ifdef LOGDRAW
    for( size_t i=0; i <= ISEGS_UPLOADED; ++i)
    {
        _drawStats[i] = 0;
    }
#endif
}

void ModelTreeLeaf::clear()
{
#ifdef LOGDRAW
    TRIPLYINFO << "Draw stats [LEAF v-" << _vertexStart << "] 0-Rendered   : \t"
               << _drawStats[RENDERED] << std::endl;
    TRIPLYINFO << "Draw stats [LEAF v-" << _vertexStart << "] 1-Uploaded   : \t"
               << _drawStats[UPLOADED] << std::endl;
    TRIPLYINFO << "Draw stats [LEAF v-" << _vertexStart << "] 2-DataRead   : \t"
               << _drawStats[DATA_READ] << std::endl;
    TRIPLYINFO << "Draw stats [LEAF v-" << _vertexStart << "] 3-DataDiscard: \t"
               << _drawStats[DATA_DISCARD] << std::endl;
    TRIPLYINFO << "Draw stats [LEAF v-" << _vertexStart << "] 4-VSegsUpload: \t"
               << _drawStats[VSEGS_UPLOADED] << std::endl;
    TRIPLYINFO << "Draw stats [LEAF v-" << _vertexStart << "] 5-ISegsUpload: \t"
               << _drawStats[ISEGS_UPLOADED] << std::endl;
    TRIPLYASSERT( _drawStats[DATA_READ] == _drawStats[DATA_DISCARD] );
#endif

    discardLeafData();
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

/*  Set up rendering of the leaf nodes.  */
void ModelTreeLeaf::setupRendering( RenderState& state,
                                    GLuint* data ) const
{
#ifdef LOGDRAW
    _drawStats[UPLOADED]++;
#endif
    if( !_dataLoaded )
        loadLeafData( state.useColors(), state.getDataManager( ) );

    switch( state.getRenderMode() )
    {
    case RENDER_MODE_IMMEDIATE:
        break;

    case RENDER_MODE_DISPLAY_LIST:
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

    case RENDER_MODE_BUFFER_OBJECT:
    case RENDER_MODE_VA_OBJECT:
    {
        // Allocate GL objects
        const char* charThis = reinterpret_cast< const char* >( this );
        if( state.getRenderMode() == RENDER_MODE_VA_OBJECT )
        {
            // Allocate empty VAOs
            static const int VAO_TYPE = BUFFER_TYPE_ALL;
            if( data[VAO_TYPE] == state.INVALID )
                data[VAO_TYPE] = state.newVertexArray( charThis );

            glBindVertexArray( data[VAO_TYPE] );  // VAO definition -- begin
        }

        if( data[VERTEX_BUFFER_TYPE] == state.INVALID )
            data[VERTEX_BUFFER_TYPE] = state.newBufferObject( charThis + VERTEX_BUFFER_TYPE );
        glBindBuffer( GL_ARRAY_BUFFER, data[VERTEX_BUFFER_TYPE] );
        glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Vertex ), 0,
                      GL_STATIC_DRAW );

        if( data[NORMAL_BUFFER_TYPE] == state.INVALID )
            data[NORMAL_BUFFER_TYPE] = state.newBufferObject( charThis + NORMAL_BUFFER_TYPE );
        glBindBuffer( GL_ARRAY_BUFFER, data[NORMAL_BUFFER_TYPE] );
        glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Normal ), 0,
                      GL_STATIC_DRAW );

        if( data[COLOR_BUFFER_TYPE] == state.INVALID )
            data[COLOR_BUFFER_TYPE] = state.newBufferObject( charThis + COLOR_BUFFER_TYPE );
        if( state.useColors() )
        {
            glBindBuffer( GL_ARRAY_BUFFER, data[COLOR_BUFFER_TYPE] );
            glBufferData( GL_ARRAY_BUFFER, _vertexLength * sizeof( Color ), 0,
                          GL_STATIC_DRAW );
        }

        if( data[INDEX_BUFFER_TYPE] == state.INVALID )
            data[INDEX_BUFFER_TYPE] = state.newBufferObject( charThis + INDEX_BUFFER_TYPE );
        glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, data[INDEX_BUFFER_TYPE] );
        glBufferData( GL_ELEMENT_ARRAY_BUFFER, _indexLength * sizeof( ShortIndex ), 0,
                      GL_STATIC_DRAW );

        // Upload data to VBOs
        TRIPLYASSERT( _dataBuffers[VERTEX_BUFFER_TYPE].numSegments() ==
                            _dataBuffers[NORMAL_BUFFER_TYPE].numSegments() );
        TRIPLYASSERT( _dataBuffers[VERTEX_BUFFER_TYPE].numSegments() ==
                            _dataBuffers[COLOR_BUFFER_TYPE].numSegments()
                       || ! state.useColors() );

        size_t offset[3];
        for( unsigned int i=0; i < 3; ++i)
            offset[i] = 0;
        for( unsigned int i=0; i < _dataBuffers[VERTEX_BUFFER_TYPE].numSegments(); ++i )
        {            
#ifdef LOGDRAW
            _drawStats[VSEGS_UPLOADED]++;
#endif
            for( int bufferType=VERTEX_BUFFER_TYPE; bufferType < INDEX_BUFFER_TYPE;
                 ++bufferType )
            {
                if( bufferType != COLOR_BUFFER_TYPE || state.useColors() )
                {
                    TRIPLYASSERT( _dataBuffers[bufferType].isValid( ));
                    SegmentedBuffer::Segment segment = _dataBuffers[bufferType].getSegment( i );
                    glBindBuffer( GL_ARRAY_BUFFER, data[bufferType] );
                    glBufferSubData( GL_ARRAY_BUFFER, offset[bufferType],
                                     segment.size, segment.ptr );
                    offset[bufferType] += segment.size;
                }
            }
        }

        size_t idxOffset = 0;
        for( unsigned int i=0; i < _dataBuffers[INDEX_BUFFER_TYPE].numSegments(); ++i )
        {
            TRIPLYASSERT( _dataBuffers[INDEX_BUFFER_TYPE].isValid( ));
#ifdef LOGDRAW
            _drawStats[ISEGS_UPLOADED]++;
#endif
            SegmentedBuffer::Segment segment =
                    _dataBuffers[INDEX_BUFFER_TYPE].getSegment( i );
            glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, data[INDEX_BUFFER_TYPE] );
            glBufferSubData( GL_ELEMENT_ARRAY_BUFFER, idxOffset, segment.size, segment.ptr );
            idxOffset += segment.size;
        }

        if( state.getRenderMode() == RENDER_MODE_VA_OBJECT )
        {
            // Define VAO elements
            if( state.useColors() )
            {
                glEnableClientState( GL_COLOR_ARRAY );
                glBindBuffer( GL_ARRAY_BUFFER, data[COLOR_BUFFER_TYPE] );
                glColorPointer( 3, GL_UNSIGNED_BYTE, 0, 0 );
            }

            glEnableClientState( GL_NORMAL_ARRAY );
            glBindBuffer( GL_ARRAY_BUFFER, data[NORMAL_BUFFER_TYPE] );
            glNormalPointer( GL_FLOAT, 0, 0 );

            glEnableClientState( GL_VERTEX_ARRAY );
            glBindBuffer( GL_ARRAY_BUFFER, data[VERTEX_BUFFER_TYPE] );
            glVertexPointer( 3, GL_FLOAT, 0, 0 );

            glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, data[INDEX_BUFFER_TYPE] );

            glBindVertexArray( 0 ); // VAO definition -- end
        }
        break;
    }
    default:
        TRIPLYASSERT( false );
    }
}

void ModelTreeLeaf::loadLeafData( bool useColors,
                                    TreeDataManager* dataManager ) const
{
    if( _dataLoaded )
        return;

#ifdef LOGDRAW
    _drawStats[DATA_READ]++;
#endif
    if( dataManager == 0 )
    {
        // In-core rendering: use single segment buffers
        _dataBuffers[VERTEX_BUFFER_TYPE].set( &_treeData.vertices[_vertexStart][0],
                                          _vertexLength * sizeof( Vertex ));
        _dataBuffers[NORMAL_BUFFER_TYPE].set( &_treeData.normals[_vertexStart][0],
                                          _vertexLength * sizeof( Normal ));
        if( useColors )
        {
            _dataBuffers[COLOR_BUFFER_TYPE].set( &_treeData.colors[_vertexStart][0],
                                             _vertexLength * sizeof( Color ));
        }
        _dataBuffers[INDEX_BUFFER_TYPE].set( &_treeData.indices[_indexStart],
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
                                         _dataBuffers[VERTEX_BUFFER_TYPE],
                                         _dataBuffers[NORMAL_BUFFER_TYPE],
                                         _dataBuffers[COLOR_BUFFER_TYPE] );
        }
        else
        {
            _dataManager->getVertexData( _vertexStart, _vertexLength,
                                         _dataBuffers[VERTEX_BUFFER_TYPE],
                                         _dataBuffers[NORMAL_BUFFER_TYPE] );
        }
        _dataManager->getIndexData( _indexStart, _indexLength,
                                    _dataBuffers[INDEX_BUFFER_TYPE] );
    }

    _dataLoaded =  true;
}

void ModelTreeLeaf::discardLeafData() const
{
    if( !_dataLoaded )
        return;

#ifdef LOGDRAW
    _drawStats[DATA_DISCARD]++;
#endif
    if( _dataManager != 0 )
    {
        _dataManager->discardVertexData( _vertexStart, _vertexLength );
        _dataManager->discardIndexData( _indexStart, _indexLength );
    }

    for( unsigned i=0; i < 4; ++i)
        _dataBuffers[i].reset();

    _dataLoaded = false;
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
          discardLeafData();
          return;
      case RENDER_MODE_DISPLAY_LIST:
          renderDisplayList( state );
          discardLeafData();
          return;
      case RENDER_MODE_VA_OBJECT:
          renderVAObject( state );
          discardLeafData();
          return;
      case RENDER_MODE_BUFFER_OBJECT:
      default:
          renderBufferObject( state );
          discardLeafData();
          return;
    }
}

/*  Render the leaf with immediate mode primitives or vertex arrays.  */
inline
void ModelTreeLeaf::renderImmediate( RenderState& state ) const
{
    setupRendering( state, NULL );
    glBegin( GL_TRIANGLES );

    TRIPLYASSERT( _indexLength <= _dataBuffers[INDEX_BUFFER_TYPE].size() );

    for( Index i = 0; i < _indexLength; ++i )
    {
        const ShortIndex idx = getIndex( i );

        TRIPLYASSERT( idx < _dataBuffers[VERTEX_BUFFER_TYPE].size() );

        if( state.useColors() )
            glColor3ubv( &(getColor( idx )[0]) );
        glNormal3fv( &(getNormal( idx )[0]) );
        glVertex3fv( &(getVertex( idx )[0]) );
    }
    glEnd();
}

/*  Render the leaf with buffer objects.  */
void ModelTreeLeaf::renderBufferObject( RenderState& state ) const
{
#ifdef LOGDRAW
    _drawStats[RENDERED]++;
#endif
    GLuint buffers[BUFFER_TYPE_ALL];
    for( int i = VERTEX_BUFFER_TYPE; i < BUFFER_TYPE_ALL; ++i )
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

/*  Render the leaf with a VAO.  */
inline
void ModelTreeLeaf::renderVAObject( RenderState& state ) const
{
#ifdef LOGDRAW
    _drawStats[RENDERED]++;
#endif
    static const int VAO_TYPE = BUFFER_TYPE_ALL;
    GLuint objects[BUFFER_TYPE_ALL + 1];
    for( int i = VERTEX_BUFFER_TYPE; i < BUFFER_TYPE_ALL; ++i )
    {
        objects[i] =
            state.getBufferObject( reinterpret_cast< const char* >(this) + i );
    }
    objects[VAO_TYPE] =
        state.getVertexArray( reinterpret_cast< const char* >(this));

    if( objects[VERTEX_BUFFER_TYPE] == state.INVALID ||
        objects[NORMAL_BUFFER_TYPE] == state.INVALID ||
        objects[COLOR_BUFFER_TYPE] == state.INVALID  ||
        objects[INDEX_BUFFER_TYPE] == state.INVALID  ||
        objects[VAO_TYPE] == state.INVALID )
    {
        setupRendering( state, objects );
    }

    glBindVertexArray( objects[VAO_TYPE] );
    glDrawElements( GL_TRIANGLES, GLsizei(_indexLength), GL_UNSIGNED_SHORT, 0 );
    glBindVertexArray( 0 );
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

} // namespace triply
