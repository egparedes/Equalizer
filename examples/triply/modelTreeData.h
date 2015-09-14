
/* Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
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


#ifndef TRIPLY_MODELTREEDATA_H
#define TRIPLY_MODELTREEDATA_H


#include "typedefs.h"
#include <vector>
#include <fstream>


namespace triply 
{    
    /** Holds the final kd-tree data, sorted and reindexed.  */
    class ModelTreeData
    {
    public:
        ModelTreeData()
            : hasColors(false), _skippedVertices( 0 ), _skippedIndices( 0 )
        { }

        void clear()
        {
            vertices.clear();
            colors.clear();
            normals.clear();
            indices.clear();
        }
        
        /*  Write the vectors' sizes and contents to the given stream.  */
        void toStream( std::ostream& os )
        {
            calculateBoundingBox();
            writeBoundingBox( os );
            writeVector( os, vertices );
            writeVector( os, colors );
            writeVector( os, normals );
            writeVector( os, indices );
        }
        
        /*  Read the vectors' sizes and contents from the given MMF address.  */
        void fromMemory( char** addr )
        {
            clear();
            readBoundingBox( addr );
            readVector( addr, vertices );
            readVector( addr, colors );
            readVector( addr, normals );
            readVector( addr, indices );
            hasColors = !colors.empty();
        }

        /*  Read the vectors' sizes and skip the contents from the given MMF address.  */
        void fromMemorySkipData( char** addr )
        {
            clear();
            readBoundingBox( addr );
            _skippedVertices = skipVector< Vertex >( addr );
            hasColors = ( skipVector< Color >( addr ) > 0 );
            skipVector< Normal >( addr );
            _skippedIndices = skipVector< ShortIndex >( addr );
        }

        void calculateBoundingBox()
        {
            boundingBox[0] = vertices[0];
            boundingBox[1] = vertices[0];
            for( size_t v = 1; v < vertices.size(); ++v )
            {
                for( size_t i = 0; i < 3; ++i )
                {
                    boundingBox[0][i] = std::min( boundingBox[0][i], vertices[v][i] );
                    boundingBox[1][i] = std::max( boundingBox[1][i], vertices[v][i] );
                }
            }
        }

        size_t getNumVertices() const
        {
            return ( vertices.size() > 0 ) ? vertices.size() : _skippedVertices;
        }

        size_t getNumIndices() const
        {
            return ( indices.size() > 0 ) ? indices.size() : _skippedIndices;
        }

        size_t getTotalSize() const
        {
            return getNumVertices() * sizeof( Vertex )
                   + getNumVertices() * sizeof( Color ) * size_t( hasColors )
                   + getNumVertices() * sizeof( Normal )
                   + getNumIndices() * sizeof( ShortIndex );
        }

        BoundingBox getBoundingBox() const { return boundingBox; }

        std::vector< Vertex >       vertices;
        std::vector< Color >        colors;
        std::vector< Normal >       normals;
        std::vector< ShortIndex >   indices;
        BoundingBox                 boundingBox;
        bool                        hasColors;

    private:        
        void writeBoundingBox( std::ostream& os )
        {
            os.write( reinterpret_cast< char* >( &boundingBox[0] ), 2 * sizeof( Vertex ) );
        }

        void readBoundingBox( char** addr )
        {
            memRead( reinterpret_cast< char* >( &boundingBox[0] ), addr, 2*sizeof( Vertex ) );
        }


        /*  Helper function to write a vector to output stream.  */
        template< class T >
        void writeVector( std::ostream& os, std::vector< T >& v )
        {
            size_t length = v.size();
            os.write( reinterpret_cast< char* >( &length ), 
                      sizeof( size_t ) );
            if( length > 0 )
                os.write( reinterpret_cast< char* >( &v[0] ), 
                          length * sizeof( T ) );
        }
        
        /*  Helper function to read a vector from the MMF address.  */
        template< class T >
        void readVector( char** addr, std::vector< T >& v )
        {
            size_t length;
            memRead( reinterpret_cast< char* >( &length ), addr, 
                     sizeof( size_t ) );
            if( length > 0 )
            {
                v.resize( length );
                memRead( reinterpret_cast< char* >( &v[0] ), addr, 
                         length * sizeof( T ) );
            }
        }

        /*  Helper function to skip a vector from the MMF address.  */
        template< class T >
        size_t skipVector( char** addr)
        {
            size_t length;
            memRead( reinterpret_cast< char* >( &length ), addr,
                     sizeof( size_t ) );
            if( length > 0 )
            {
                *addr += length * sizeof( T );
            }
            return length;
        }

        size_t      _skippedVertices;
        size_t      _skippedIndices;
    };
}


#endif // TRIPLY_MODELTREEDATA_H
