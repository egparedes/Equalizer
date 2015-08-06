
/* Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
 *               2009-2012, Stefan Eilemann <eile@equalizergraphics.com>
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

#include "vertexData.h"
#include "ply.h"

#include <cstdlib>
#include <algorithm>

#if (( __GNUC__ > 4 ) || ((__GNUC__ == 4) && (__GNUC_MINOR__ >= 4)) )
#  include <parallel/algorithm>
using __gnu_parallel::sort;
#else
using std::sort;
#endif

namespace triply
{

/*  Contructor.  */
VertexData::VertexData()
    : _invertFaces( false )
{
    _boundingBox[0] = Vertex( 0.0f );
    _boundingBox[1] = Vertex( 0.0f );
}

/*  Read the vertex and (if available/wanted) color data from the open file.  */
void VertexData::readVertices( PlyFile* file, const int nVertices,
                               const bool readColors )
{
    // temporary vertex structure for ply loading
    struct _Vertex
    {
        float           x;
        float           y;
        float           z;
        unsigned char   r;
        unsigned char   g;
        unsigned char   b;
    } vertex;

    PlyProperty vertexProps[] =
    {
        { "x", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, x ), 0, 0, 0, 0 },
        { "y", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, y ), 0, 0, 0, 0 },
        { "z", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, z ), 0, 0, 0, 0 },
        { "red", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, r ), 0, 0, 0, 0 },
        { "green", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, g ), 0, 0, 0, 0 },
        { "blue", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, b ), 0, 0, 0, 0 }
    };

    // use all 6 properties when reading colors, only the first 3 otherwise
    int limit = readColors ? 6 : 3;
    for( int i = 0; i < limit; ++i )
        ply_get_property( file, "vertex", &vertexProps[i] );

    vertices.clear();
    vertices.reserve( nVertices );

    if( readColors )
    {
        colors.clear();
        colors.reserve( nVertices );
    }

    // read in the vertices
    for( int i = 0; i < nVertices; ++i )
    {
        ply_get_element( file, static_cast< void* >( &vertex ) );
        vertices.push_back( Vertex( vertex.x, vertex.y, vertex.z ) );
        if( readColors )
            colors.push_back( Color( vertex.r, vertex.g, vertex.b ) );
    }
}


/*  Read the index data from the open file.  */
void VertexData::readTriangles( PlyFile* file, const int nFaces )
{
    // temporary face structure for ply loading
    struct _Face
    {
        unsigned char   nVertices;
        int*            vertices;
    } face;

    PlyProperty faceProps[] =
    {
        { "vertex_indices", PLY_INT, PLY_INT, offsetof( _Face, vertices ),
          1, PLY_UCHAR, PLY_UCHAR, offsetof( _Face, nVertices ) }
    };

    ply_get_property( file, "face", &faceProps[0] );

    triangles.clear();
    triangles.reserve( nFaces );

    // read in the faces, asserting that they are only triangles
    uint8_t ind1 = _invertFaces ? 2 : 0;
    uint8_t ind3 = _invertFaces ? 0 : 2;
    for( int i = 0; i < nFaces; ++i )
    {
        ply_get_element( file, static_cast< void* >( &face ) );
        TRIPLYASSERT( face.vertices != 0 );
        if( face.nVertices != 3 )
        {
            free( face.vertices );
            throw MeshException( "Error reading PLY file. Encountered a "
                                 "face which does not have three vertices." );
        }
        triangles.push_back( Triangle( face.vertices[ind1],
                                       face.vertices[1],
                                       face.vertices[ind3] ) );

        // free the memory that was allocated by ply_get_element
        free( face.vertices );
    }
}


/*  Open a PLY file and read vertex, color and index data.  */
bool VertexData::readPlyFile( const std::string& filename )
{
    int     nPlyElems;
    char**  elemNames;
    int     fileType;
    float   version;
    bool    result = false;

    PlyFile* file = ply_open_for_reading( const_cast<char*>( filename.c_str( )),
                                          &nPlyElems, &elemNames,
                                          &fileType, &version );
    if( !file )
    {
        TRIPLYERROR << "Unable to open PLY file " << filename
                  << " for reading." << std::endl;
        return result;
    }
    TRIPLYASSERT( elemNames != 0 );

    for( int i = 0; i < nPlyElems; ++i )
    {
        int nElems;
        int nProps;

        PlyProperty** props = ply_get_element_description( file, elemNames[i],
                                                           &nElems, &nProps );
        TRIPLYASSERT( props != 0 );

        if( equal_strings( elemNames[i], "vertex" ) )
        {
            bool hasColors = false;
            // determine if the file stores vertex colors
            for( int j = 0; j < nProps; ++j )
                if( equal_strings( props[j]->name, "red" ) )
                    hasColors = true;

            readVertices( file, nElems, hasColors );
            TRIPLYASSERT( vertices.size() == static_cast< size_t >( nElems ) );
            if( hasColors )
            {
                TRIPLYASSERT( colors.size() == static_cast< size_t >( nElems ));
            }
        }
        else if( equal_strings( elemNames[i], "face" ) )
        try
        {
            readTriangles( file, nElems );
            TRIPLYASSERT( triangles.size() == static_cast< size_t >( nElems ) );
            result = true;
        }
        catch( const std::exception& e )
        {
            TRIPLYERROR << "Unable to read PLY file, an exception occured:  "
                      << e.what() << std::endl;
            // stop for loop by setting the loop variable to break condition
            // this way resources still get released even on error cases
            i = nPlyElems;
        }

        // free the memory that was allocated by ply_get_element_description
        for( int j = 0; j < nProps; ++j )
            free( props[j] );
        free( props );
    }

    ply_close( file );

    // free the memory that was allocated by ply_open_for_reading
    for( int i = 0; i < nPlyElems; ++i )
        free( elemNames[i] );
    free( elemNames );

    return result;
}

/*  Calculate the face or vertex normals of the current vertex data.  */
void VertexData::calculateNormals()
{
#ifndef NDEBUG
    int wrongNormals = 0;
#endif

    normals.clear();
    normals.reserve( vertices.size() );

    // initialize all normals to zero
    for( size_t i = 0; i < vertices.size(); ++i )
        normals.push_back( Normal( 0, 0, 0 ) );

    // iterate over all triangles and add their normals to adjacent vertices
#pragma omp parallel for
    for( ssize_t i = 0; i < ssize_t( triangles.size( )); ++i )
    {
        const Index i0 = triangles[i][0];
        const Index i1 = triangles[i][1];
        const Index i2 = triangles[i][2];
        const Normal normal = vertices[i0].compute_normal( vertices[i1],
                                                           vertices[i2] );
#ifndef NDEBUG
        // count emtpy normals in debug mode
        if( normal.length() == 0.0f )
            ++wrongNormals; // racy with OpenMP, but not critical
#endif

        normals[i0] += normal;
        normals[i1] += normal;
        normals[i2] += normal;
    }

    // normalize all the normals
#pragma omp parallel for
    for( ssize_t i = 0; i < ssize_t( vertices.size( )); ++i )
        normals[i].normalize();

#ifndef NDEBUG
    if( wrongNormals > 0 )
        TRIPLYINFO << wrongNormals << " faces have no valid normal." << std::endl;
#endif
}

/*  Calculate the bounding box of the current vertex data.  */
void VertexData::calculateBoundingBox()
{
    _boundingBox[0] = vertices[0];
    _boundingBox[1] = vertices[0];
    for( size_t v = 1; v < vertices.size(); ++v )
        for( size_t i = 0; i < 3; ++i )
        {
            _boundingBox[0][i] = std::min( _boundingBox[0][i], vertices[v][i] );
            _boundingBox[1][i] = std::max( _boundingBox[1][i], vertices[v][i] );
        }
}

/*  Scales the data to be within +- baseSize/2 (default 2.0) coordinates.  */
void VertexData::scale( const float baseSize )
{
    // calculate bounding box if not yet done
    if( _boundingBox[0].length() == 0.0f && _boundingBox[1].length() == 0.0f )
        calculateBoundingBox();

    // find largest dimension and determine scale factor
    float factor = 0.0f;
    for( size_t i = 0; i < 3; ++i )
        factor = std::max( factor, _boundingBox[1][i] - _boundingBox[0][i] );
    factor = baseSize / factor;

    // determine scale offset
    Vertex offset;
    for( size_t i = 0; i < 3; ++i )
        offset[i] = ( _boundingBox[0][i] + _boundingBox[1][i] ) * 0.5f;

    // scale the data
#pragma omp parallel for
    for( ssize_t v = 0; v < ssize_t( vertices.size( )); ++v )
        for( size_t i = 0; i < 3; ++i )
        {
            vertices[v][i] -= offset[i];
            vertices[v][i] *= factor;
        }

    // scale the bounding box
    for( size_t v = 0; v < 2; ++v )
        for( size_t i = 0; i < 3; ++i )
        {
            _boundingBox[v][i] -= offset[i];
            _boundingBox[v][i] *= factor;
        }
}

} // namespace triply
