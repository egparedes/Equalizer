
/* Copyright (c) 2011-2012, Stefan Eilemann <eile@eyescale.ch>
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


#include "renderState.h"

namespace triply 
{
RenderState::RenderState( const GLEWContext* glewContext )
        : _pmvMatrix( Matrix4f::IDENTITY )
        , _glewContext( glewContext )
        , _renderMode( RENDER_MODE_BUFFER_OBJECT ) /* RENDER_MODE_DISPLAY_LIST  */
        , _useColors( false )
        , _useFrustumCulling( true )
        , _useBoundingSpheres( false )
        , _outOfCore( false )
        , _dataManager( 0 )
        , _maxKeys( 8192 )
{
    _range[0] = 0.f;
    _range[1] = 1.f;
    resetRegion();
    TRIPLYASSERT( glewContext );
} 

void RenderState::setRenderMode( const RenderMode mode )
{ 
    if( _renderMode == mode )
        return;

    _renderMode = mode;

    // Check if VBO funcs available, else fall back to display lists
    if( _renderMode == RENDER_MODE_BUFFER_OBJECT && !GLEW_VERSION_1_5 )
    {
        TRIPLYINFO << "VBO not available, using display lists" << std::endl;
        _renderMode = RENDER_MODE_DISPLAY_LIST;
    }
}

void RenderState::resetRegion()
{
    _region[0] = std::numeric_limits< float >::max();
    _region[1] = std::numeric_limits< float >::max();
    _region[2] = -std::numeric_limits< float >::max();
    _region[3] = -std::numeric_limits< float >::max();
}

void RenderState::updateRegion( const BoundingBox& box )
{
    const Vertex corners[8] = { Vertex( box[0][0], box[0][1], box[0][2] ),
                                Vertex( box[1][0], box[0][1], box[0][2] ),
                                Vertex( box[0][0], box[1][1], box[0][2] ),
                                Vertex( box[1][0], box[1][1], box[0][2] ),
                                Vertex( box[0][0], box[0][1], box[1][2] ),
                                Vertex( box[1][0], box[0][1], box[1][2] ),
                                Vertex( box[0][0], box[1][1], box[1][2] ),
                                Vertex( box[1][0], box[1][1], box[1][2] ) };

    Vector4f region(  std::numeric_limits< float >::max(),
                      std::numeric_limits< float >::max(),
                     -std::numeric_limits< float >::max(),
                     -std::numeric_limits< float >::max( ));

    for( size_t i = 0; i < 8; ++i )
    {
        const Vertex corner = _pmvMatrix * corners[i];
        region[0] = std::min( corner[0], region[0] );
        region[1] = std::min( corner[1], region[1] );
        region[2] = std::max( corner[0], region[2] );
        region[3] = std::max( corner[1], region[3] );
    }

    // transform region of interest from [ -1 -1 1 1 ] to normalized viewport
    const Vector4f normalized( region[0] * .5f + .5f,
                               region[1] * .5f + .5f,
                               ( region[2] - region[0] ) * .5f,
                               ( region[3] - region[1] ) * .5f );

    declareRegion( normalized );
    _region[0] = std::min( _region[0], normalized[0] );
    _region[1] = std::min( _region[1], normalized[1] );
    _region[2] = std::max( _region[2], normalized[2] );
    _region[3] = std::max( _region[3], normalized[3] );
}


Vector4f RenderState::getRegion() const
{
    if( _region[0] > _region[2] || _region[1] > _region[3] )
        return Vector4f::ZERO;

    return _region;
}

const void* RenderState::addGlObject( const void* key )
{
    const void* lruKey = 0;
    auto it = _keyMap.find( key );
    if( it != _keyMap.end( ))
    {
        _activeKeys.erase( it->second );
        _keyMap.erase( it );
    }

    _activeKeys.push_front( key );
    _keyMap[key] = _activeKeys.begin();

    if( _keyMap.size() > _maxKeys )
    {
        TRIPLYASSERT( _activeKeys.rbegin() != _activeKeys.rend() );
        lruKey = *( _activeKeys.rbegin() );
        _keyMap.erase( lruKey );
        _activeKeys.pop_back();
    }

    return lruKey; // Key of the object to be deleted
}

void RenderState::touchGlObject( const void* key )
{
    auto it = _keyMap.find( key );
    if( it != _keyMap.end( ))
        _activeKeys.splice( _activeKeys.begin(), _activeKeys, it->second );
}


// ---- SimpleRenderState
GLuint SimpleRenderState::getDisplayList( const void* key )
{
    if( _displayLists.find( key ) == _displayLists.end() )
        return INVALID;

    touchGlObject( key );
    return _displayLists[key];
}
        
GLuint SimpleRenderState::newDisplayList( const void* key )
{
    _displayLists[key] = glGenLists( 1 );
    const void* oldKey = addGlObject( key );
    if( oldKey )
        deleteDisplayList( oldKey );
    return _displayLists[key];
}

void SimpleRenderState::deleteDisplayList( const void* key )
{
    if( _displayLists.find( key ) != _displayLists.end() )
    {
        glDeleteLists( _displayLists[key], 2 );
        _displayLists.erase( key );
    }
}
        
GLuint SimpleRenderState::getBufferObject( const void* key )
{
    if( _bufferObjects.find( key ) == _bufferObjects.end() )
        return INVALID;
    touchGlObject( key );
    return _bufferObjects[key];
}
        
GLuint SimpleRenderState::newBufferObject( const void* key )
{
    if( !GLEW_VERSION_1_5 )
        return INVALID;
    glGenBuffers( 1, &_bufferObjects[key] );
    const void* oldKey = addGlObject( key );
    if( oldKey )
        deleteBufferObject( oldKey );
    return _bufferObjects[key];
}
        
void SimpleRenderState::deleteBufferObject( const void* key )
{
    if( _bufferObjects.find( key ) != _bufferObjects.end() )
    {
        glDeleteBuffers( 1, &_bufferObjects[key] );
        _bufferObjects.erase( key );
    }
}

GLuint SimpleRenderState::getVertexArray( const void* key )
{
    if( _vertexArrays.find( key ) == _vertexArrays.end() )
        return INVALID;
    touchGlObject( key );
    return _vertexArrays[key];
}

GLuint SimpleRenderState::newVertexArray( const void* key )
{
    glGenVertexArrays( 1, &_vertexArrays[key] );
    const void* oldKey = addGlObject( key );
    if( oldKey )
        deleteVertexArray( oldKey );
    return _vertexArrays[key];
}

void SimpleRenderState::deleteVertexArray( const void* key )
{
    if( _vertexArrays.find( key ) != _vertexArrays.end() )
    {
        glDeleteVertexArrays( 1, &_vertexArrays[key] );
        _vertexArrays.erase( key );
    }
}

void SimpleRenderState::deleteGlObjects()
{
    for( GLMapCIter i = _displayLists.begin(); i != _displayLists.end(); ++i )
        glDeleteLists( i->second, 1 );

    for( GLMapCIter i = _bufferObjects.begin(); i != _bufferObjects.end(); ++i )
        glDeleteBuffers( 1, &(i->second) );

    for( GLMapCIter i = _vertexArrays.begin(); i != _vertexArrays.end(); ++i )
        glDeleteVertexArrays( 1, &(i->second) );

    _displayLists.clear();
    _bufferObjects.clear();
    _vertexArrays.clear();
}

}
