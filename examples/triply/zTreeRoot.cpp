
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *               2009-2014, Stefan Eilemann <eile@equalizergraphics.com>
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


#include "zTreeRoot.h"
#include "vertexBufferState.h"
#include "vertexData.h"
#include <string>
#include <sstream>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifndef _WIN32
#   include <sys/mman.h>
#endif

namespace triply
{

namespace detail
{

static const std::string BinaryNameTag = "zoct";

static const vmml::Vector3f COLOR1(215, 159, 155);
static const vmml::Vector3f COLOR2(47, 89, 168);

ZKey genZCode( const Vertex& point, const BoundingBox& bbox )
{
    Vertex minPoint = bbox[0];
    Vertex maxPoint = bbox[1];
    ZKey key = 0;

    for( int i=ZTreeBase::MaxLevel; i >= 0; --i )
    {
      key <<= 3;
      if( point[0] + point[0] > minPoint[0] + maxPoint[0] )
      {
        key |= 0x1;
        minPoint[0] = (minPoint[0] + maxPoint[0]) * 0.5;
      }
      else
      {
        maxPoint[0] = (minPoint[0] + maxPoint[0]) * 0.5;
      }
      if( point[1] + point[1] > minPoint[1] + maxPoint[1] )
      {
        key |= 0x2;
        minPoint[1] = (minPoint[1] + maxPoint[1]) * 0.5;
      }
      else
      {
        maxPoint[1] = (minPoint[1] + maxPoint[1]) * 0.5;
      }
      if( point[2] + point[2] > minPoint[2] + maxPoint[2] )
      {
        key |= 0x4;
        minPoint[2] = (minPoint[2] + maxPoint[2]) * 0.5;
      }
      else
      {
        maxPoint[2] = (minPoint[2] + maxPoint[2]) * 0.5;
      }
    }

    return key;
}


// decode a given 64-bit morton code to an integer (x,y,z) coordinate
inline void mortonDecode(uint64_t morton, uint32_t& x, uint32_t& y, uint32_t& z){
    x = 0;
    y = 0;
    z = 0;
    for (uint64_t i = 0; i < (sizeof(uint64_t) * CHAR_BIT)/3; ++i) {
        x |= ((morton & (uint64_t( 1ull ) << uint64_t((3ull * i) + 0ull))) >> uint64_t(((3ull * i) + 0ull)-i));
        y |= ((morton & (uint64_t( 1ull ) << uint64_t((3ull * i) + 1ull))) >> uint64_t(((3ull * i) + 1ull)-i));
        z |= ((morton & (uint64_t( 1ull ) << uint64_t((3ull * i) + 2ull))) >> uint64_t(((3ull * i) + 2ull)-i));
    }
}

template <std::size_t RadixMinSize=32>
inline static void insertionSort(ZKeyIndexPair *pairs, std::size_t size)
{
    unsigned i, j;
    ZKeyIndexPair k;
    for( i=0; i != size; i++, pairs[j]=k)
    {
        for( j=i, k=pairs[j]; j && k.first < pairs[j-1].first; j--)
        {
            pairs[j] = pairs[j-1];
        }
    }
}

template <std::size_t RadixMinSize=32>
static void inplaceRadixSort(ZKeyIndexPair* pairs, std::size_t size, unsigned char byte)
{
    using std::swap;

    if( size < RadixMinSize )
    {
        insertionSort(pairs, size);
    }
    else
    {
        unsigned i, k, end;
        ZKeyIndexPair j;

        unsigned count[256] = {};
        for( i=0; i < size; ++i )
        {
            count[getByte(pairs[i].first, byte)]++;
        }

        unsigned bucket[256];
        bucket[0] = 0;
        for( i=1; i < 256; i++ )
        {
            bucket[i] = bucket[i-1] + count[i-1];
        }

        for( i=0; i < 256; i++ )
        {
            end = (i>0?bucket[i-1]:0) + count[i];
            for( ; bucket[i] < end; bucket[i]++ )
            {
                j = pairs[bucket[i]];
                while( (k=getByte(j.first, byte))!=i )
                {
                    unsigned xx = bucket[k];
                    bucket[k]++;
                    swap(j, pairs[xx]);
                }
                pairs[bucket[i]] = j;
            }
        }

        if( byte-- > 0 )
        {
            for( i=0; i<256; i++ )
            {
                if( count[i]>0 )
                {
                    inplaceRadixSort(pairs + bucket[i]-count[i], count[i], byte);
                }
            }
        }
    }
}

template <std::size_t RadixMinSize=32>
inline static void sortPairs( std::vector< ZKeyIndexPair >& pairs )
{
    inplaceRadixSort( pairs.data(), pairs.size(), SORTKEY_BIT_SIZE );
}

// Given H,S,L in range of 0-1
// Returns a Color (RGB struct) in range of 0-255
Color HSL2RGB(float h, float sl, float l)
{
    double v;
    double r,g,b;

    r = l;   // default to gray
    g = l;
    b = l;
    v = (l <= 0.5) ? (l * (1.0 + sl)) : (l + sl - l * sl);
    if (v > 0)
    {
          double m;
          double sv;
          int sextant;
          double fract, vsf, mid1, mid2;

          m = l + l - v;
          sv = (v - m ) / v;
          h *= 6.0;
          sextant = (int)h;
          fract = h - sextant;
          vsf = v * sv * fract;
          mid1 = m + vsf;
          mid2 = v - vsf;
          switch (sextant)
          {
                case 0:
                      r = v;
                      g = mid1;
                      b = m;
                      break;
                case 1:
                      r = mid2;
                      g = v;
                      b = m;
                      break;
                case 2:
                      r = m;
                      g = v;
                      b = mid1;
                      break;
                case 3:
                      r = m;
                      g = mid2;
                      b = v;
                      break;
                case 4:
                      r = mid1;
                      g = m;
                      b = v;
                      break;
                case 5:
                      r = v;
                      g = m;
                      b = mid2;
                      break;
          }
    }
    Color rgb;
    rgb[0] = static_cast<uint8_t>( std::floor(r * 255.0f) );
    rgb[1] = static_cast<uint8_t>( std::floor(g * 255.0f) );
    rgb[2] = static_cast<uint8_t>( std::floor(b * 255.0f) );

    return rgb;
}

/*  Write mesh file with colored meshes and vertices  (OFF format).  */
void exportSortedMesh( const std::string& name,
                       const VertexData& modelData,
                       const std::vector< ZKeyIndexPair >& zKeys)
{
    std::string filename = name + ".off";
    std::ofstream output( filename.c_str(), std::ios::out );
    if( output )
    {
        // enable exceptions on stream errors
        output.exceptions( std::ofstream::failbit | std::ofstream::badbit );
        try
        {
            output << "COFF" << std::endl;
            output << modelData.vertices.size() << " "
                   << modelData.triangles.size() << " "
                   << "0" << std::endl;

            Index EmptyIdx = modelData.vertices.size() + 10;
            std::vector< Index > newVIndices( modelData.vertices.size(), EmptyIdx );

            Index currentV = 0;
            for (std::size_t z=0; z < zKeys.size(); ++z)
            {
                for (std::size_t v=0; v < 3; ++v)
                {
                    Index vIndex = modelData.triangles[ zKeys[z].second ][ v ];
                    if ( newVIndices[ vIndex ] == EmptyIdx)
                    {
                        float f = (currentV*1.0f) / (1.0f*modelData.vertices.size());
//                         Color color = detail::HSL2RGB(f, 0.75, 0.5);
                        vmml::Vector3f color = COLOR1 * (1.f - f) + COLOR2 * f;
                        output << modelData.vertices[vIndex][0] << " "
                               << modelData.vertices[vIndex][1] << " "
                               << modelData.vertices[vIndex][2] << " "
                               << static_cast<unsigned>(color[0]) << " "
                               << static_cast<unsigned>(color[1]) << " "
                               << static_cast<unsigned>(color[2]) << " "
                               << std::endl;

                        newVIndices[ vIndex ] = currentV++;
                    }
                }
            }

            // non referenced vertices
            for (std::size_t v=0; v < modelData.vertices.size(); ++v)
            {
                if ( newVIndices[ v ] == EmptyIdx)
                {
                    output << modelData.vertices[v][0] << " "
                           << modelData.vertices[v][1] << " "
                           << modelData.vertices[v][2] << " "
                               << "0 "
                               << "0 "
                               << "0 "
                           << std::endl;
                }
            }

            for (std::size_t z=0; z < zKeys.size(); ++z)
            {
                float f = (z*1.0f) / (modelData.triangles.size()*1.0f);
                Color color = detail::HSL2RGB(f, 0.75, 0.5);
                output << "3 "
                       << newVIndices[ modelData.triangles[ zKeys[z].second ][0] ] << " "
                       << newVIndices[ modelData.triangles[ zKeys[z].second ][1] ] << " "
                       << newVIndices[ modelData.triangles[ zKeys[z].second ][2] ] << " "
                       << static_cast<unsigned>(color[0]) << " "
                       << static_cast<unsigned>(color[1]) << " "
                       << static_cast<unsigned>(color[2]) << " "
                       << std::endl;
            }
        }
        catch( const std::exception& e )
        {
            PLYLIBERROR << "Unable to write OFF mesh file, an exception "
                      << "occured:  " << e.what() << std::endl;
        }
        output.close();
    }
    else
    {
        PLYLIBERROR << "Unable to create mesh file." << std::endl;
    }
}


/*  Write Z-order curve as a bunch of colored cubes (OFF format).  */
void exportZCurve( const std::string& name, const BoundingBox& bbox,
                   unsigned numberOfLevels )
{
    PLYLIBASSERT( bbox[0] != bbox[1] );
    PLYLIBASSERT( numberOfLevels > 0 );

    std::vector< Vertex >   vertices;
    std::vector< Color >    colors;
    std::vector< Triangle > triangles;

//    Vertex cellSize = (bbox[1] - bbox[0]) / (1u << (numberOfLevels-1))*1.0f;
    size_t numberOfCells = (1 << (numberOfLevels-1));
    numberOfCells *= numberOfCells * numberOfCells;

    // cube = half of the cell size?
    Vertex cubeSize = (bbox[1] - bbox[0]) / (1u << numberOfLevels) * 2.f;

    for (uint64_t i=0; i < numberOfCells; i++)
    {
        uint32_t cellX, cellY, cellZ;
        detail::mortonDecode(i, cellX, cellY, cellZ);

        // generate cube in the center of the cell given by cellX, cellY, cellZ
       vertices.push_back( Vertex( cellX - .01, cellY - .01, cellZ - .01) * cubeSize - bbox[1] );
       vertices.push_back( Vertex( cellX + .01, cellY - .01, cellZ - .01) * cubeSize - bbox[1] );
       vertices.push_back( Vertex( cellX - .01, cellY + .01, cellZ - .01) * cubeSize - bbox[1] );
       vertices.push_back( Vertex( cellX - .01, cellY - .01, cellZ + .01) * cubeSize - bbox[1] );
       vertices.push_back( Vertex( cellX - .01, cellY + .01, cellZ + .01) * cubeSize - bbox[1] );
       vertices.push_back( Vertex( cellX + .01, cellY - .01, cellZ + .01) * cubeSize - bbox[1] );
       vertices.push_back( Vertex( cellX + .01, cellY + .01, cellZ - .01) * cubeSize - bbox[1] );
       vertices.push_back( Vertex( cellX + .01, cellY + .01, cellZ + .01) * cubeSize - bbox[1] );
       
       float f = (i*1.0f) / (numberOfCells*1.0f) * .5f + .5f;
       int v = vertices.size() - 8;
       if( v > 8 )
       {
           int p = v - 8;
           triangles.push_back( Triangle( p + 1, v + 1, v + 6 ));
           triangles.push_back( Triangle( v + 6, p + 6, p + 1 ));
           triangles.push_back( Triangle( p + 2, v + 2, v + 6 ));
           triangles.push_back( Triangle( v + 6, p + 6, p + 2 ));
           triangles.push_back( Triangle( p + 2, v + 2, v + 4 ));
           triangles.push_back( Triangle( v + 4, p + 4, p + 2 ));
           triangles.push_back( Triangle( p + 4, v + 4, v + 3 ));
           triangles.push_back( Triangle( v + 3, p + 3, p + 4 ));
           triangles.push_back( Triangle( p + 3, v + 3, v + 5 ));
           triangles.push_back( Triangle( v + 5, p + 5, p + 3 ));
           triangles.push_back( Triangle( p + 1, v + 1, v + 5 ));
           triangles.push_back( Triangle( v + 5, p + 5, p + 1 ));
           colors.push_back( Color( f * 255 ));
       }

    }

    std::string filename = name + ".zcurve.off";
    std::ofstream output( filename.c_str(), std::ios::out );
    if( output )
    {
        // enable exceptions on stream errors
        output.exceptions( std::ofstream::failbit | std::ofstream::badbit );
        try
        {
            output << "COFF" << std::endl;
            output << vertices.size() << " "
                   << triangles.size() << " "
                   << "0" << std::endl;

            for (std::size_t v=0; v < vertices.size(); ++v)
            {
                output << vertices[v][0] << " "
                       << vertices[v][1] << " "
                       << vertices[v][2] << " "
                       << 0 << " "
                       << 0 << " "
                       << 0 << " "
                       << std::endl;
            }
            for (std::size_t t=0; t < triangles.size(); ++t)
            {
                output << 3 << " " << triangles[t][0] << " "
                       << triangles[t][1] << " "
                       << triangles[t][2] << " "
                       << int( colors[t][0] )  << " "
                       << int( colors[t][1] ) << " "
                       << int( colors[t][2] )
                       << std::endl;
            }
        }
        catch( const std::exception& e )
        {
            PLYLIBERROR << "Unable to write OFF Z-curve file, an exception "
                      << "occured:  " << e.what() << std::endl;
        }
        output.close();
    }
    else
    {
        PLYLIBERROR << "Unable to create Z-curve file." << std::endl;
    }
}

} // namespace detail


typedef vmml::frustum_culler< float >  FrustumCuller;

//#define LOGCULL
void ZTreeRoot::cullDraw( VertexBufferState& state ) const
{
    _beginRendering( state );
    
#ifdef LOGCULL
    size_t verticesRendered = 0;
    size_t verticesOverlap  = 0;
#endif

    const Range& range = state.getRange();
    FrustumCuller culler;
    culler.setup( state.getProjectionModelViewMatrix( ));

    // start with root node
    std::vector< const triply::ZTreeBase* > candidates;
    candidates.push_back( this );

    while( !candidates.empty() )
    {
        if( state.stopRendering( ))
            return;

        const triply::ZTreeBase* treeNode = candidates.back();
        candidates.pop_back();
            
        // completely out of range check
        if( treeNode->getRange()[0] >= range[1] ||
            treeNode->getRange()[1] < range[0] )
        {
            continue;
        }

        // bounding sphere view frustum culling
        const vmml::Visibility visibility = state.useFrustumCulling() ?
                            culler.test_sphere( treeNode->getBoundingSphere( )) :
                            vmml::VISIBILITY_FULL;
//        const vmml::Visibility visibility = vmml::VISIBILITY_FULL;
        PLYLIBASSERT (state.useFrustumCulling());
        switch( visibility )
        {
            case vmml::VISIBILITY_FULL:
                // if fully visible and fully in range, render it
                if( treeNode->getRange()[0] >= range[0] && 
                    treeNode->getRange()[1] <  range[1] )
                {
                    treeNode->draw( state );
                    if( state.showBoundingSpheres() )
                        treeNode->drawBoundingSphere( state );
#ifdef LOGCULL
                    verticesRendered += treeNode->getNumberOfVertices();
#endif
                    break;
                }
                // partial range, fall through to partial visibility

            case vmml::VISIBILITY_PARTIAL:
            {

                if( treeNode->getNumberofChildren() == 0 )
                {
                    if( treeNode->getRange()[0] >= range[0] )
                    {
                        treeNode->draw( state );
                        if( state.showBoundingSpheres() )
                            treeNode->drawBoundingSphere( state );
#ifdef LOGCULL
                        verticesRendered += treeNode->getNumberOfVertices();
                        if( visibility == vmml::VISIBILITY_PARTIAL )
                            verticesOverlap  += treeNode->getNumberOfVertices();
#endif
                    }
                    // else drop, to be drawn by 'previous' channel
                }
                else
                {
                    for( unsigned i=0; i < 8; ++i)
                    {
                        const triply::ZTreeBase* child = treeNode->getChild( i );
                        if( child != 0 )
                            candidates.push_back( child );
                    }
                }
                break;
            }
            case vmml::VISIBILITY_NONE:
                // do nothing
                break;
        }
    }
    
    _endRendering( state );

#ifdef LOGCULL
    const size_t verticesTotal = _data.vertices.size(); //model->getNumberOfVertices();
    PLYLIBINFO
        << getName() << " rendered " << verticesRendered * 100 / verticesTotal
        << "% of model, overlap <= " << verticesOverlap * 100 / verticesTotal
        << "%" << std::endl;
#endif    
}

/*  Delegate rendering to node routine.  */
void ZTreeRoot::draw( VertexBufferState& state ) const
{
    ZTreeNode::draw( state );
}

/*  Begin octree setup, go through full range starting with x axis.  */
void ZTreeRoot::setupTree( VertexData& modelData )
{
    // modelData is VertexData, _data is VertexBufferData
    _data.clear();

    // Compute Z-codes for triangles and sort them
    const BoundingBox& bbox = modelData.getBoundingBox();
    if( bbox[0] == bbox[1] )
        modelData.calculateBoundingBox();
    PLYLIBASSERT(  bbox[0] == modelData.getBoundingBox()[0] );
    PLYLIBASSERT(  bbox[1] == modelData.getBoundingBox()[1] );

    std::vector< ZKeyIndexPair > zKeys;
    zKeys.resize( modelData.triangles.size() );

    PLYLIBINFO << "--ZTREEPLY-- Generating Z Keys..." << std::endl;
#pragma omp parallel for
    for( Index i=0; i < zKeys.size(); ++i )
    {
//        PLYLIBINFO << "--ZTREEPLY-- Vertex position [" << i << "][0] = " << modelData.vertices[ modelData.triangles[ i ][ 0 ] ] << std::endl;
//        PLYLIBINFO << "--ZTREEPLY-- Vertex position [" << i << "][1] = " << modelData.vertices[ modelData.triangles[ i ][ 1 ] ] << std::endl;
//        PLYLIBINFO << "--ZTREEPLY-- Vertex position [" << i << "][2] = " << modelData.vertices[ modelData.triangles[ i ][ 2 ] ] << std::endl;
//        PLYLIBINFO << "--ZTREEPLY-- Triangle position [" << i << "] = "
//                   << ( modelData.vertices[ modelData.triangles[ i ][ 0 ] ] +
//                        modelData.vertices[ modelData.triangles[ i ][ 1 ] ] +
//                        modelData.vertices[ modelData.triangles[ i ][ 2 ] ] ) / 3.0f
//                   << std::endl;

        zKeys[ i ].first = detail::genZCode(
                   ( modelData.vertices[ modelData.triangles[ i ][ 0 ] ] +
                     modelData.vertices[ modelData.triangles[ i ][ 1 ] ] +
                     modelData.vertices[ modelData.triangles[ i ][ 2 ] ] ) / 3.0f,
                   bbox );
        zKeys[ i ].second = i;

//        PLYLIBINFO << "--ZTREEPLY-- Z key[" << i << "]= " << zKeys[ i ].first << std::endl;
    }
    PLYLIBINFO << "--ZTREEPLY-- Sorting Z Keys..." << std::endl;
    detail::sortPairs( zKeys );
//    for( Index i=0; i < zKeys.size(); ++i )
//    {
//        PLYLIBINFO << "--ZTREEPLY-- Z key["<< i <<"]= " << zKeys[ i ].first << " / " << zKeys[ i ].second << std::endl;
//        PLYLIBINFO << "--ZTREEPLY-- Z ordered triangle position [" << zKeys[ i ].first << "] = "
//                   << ( modelData.vertices[ modelData.triangles[ zKeys[ i ].second ][ 0 ] ] +
//                        modelData.vertices[ modelData.triangles[ zKeys[ i ].second ][ 1 ] ] +
//                        modelData.vertices[ modelData.triangles[ zKeys[ i ].second ][ 2 ] ] ) / 3.0f
//                   << std::endl;
//    }

//    PLYLIBINFO << "--ZTREEPLY-- MaxZkey = " << std::hex << (~0ull) << std::dec << std::endl;
//    PLYLIBINFO << "--ZTREEPLY-- ( 8*sizeof( ZKey ) - SORTKEY_BIT_SIZE ) = " << ( 8*sizeof( ZKey ) - SORTKEY_BIT_SIZE ) << std::endl;

    detail::exportSortedMesh( _name, modelData, zKeys );
    detail::exportZCurve( _name, bbox, 4 );

    ZTreeNode::setupTree( modelData, zKeys, ZTreeBase::MinZKey, ZTreeBase::MaxZKey,
                          0, (bbox[0] + bbox[1]) / 2.0, _data );
    ZTreeNode::updateBoundingSphere();
    ZTreeNode::updateRange();
    _data.calculateBoundingBox();

#ifndef NDEBUG
//#if 0
    // re-test all points to be in the bounding sphere
    Vertex center( _boundingSphere.array );
    float  radius        = _boundingSphere.w();
    float  radiusSquared =  radius * radius;
    for( size_t offset = 0; offset < _data.vertices.size(); ++offset )
    {
        const Vertex& vertex = _data.vertices[ offset ];

        const Vertex centerToPoint   = vertex - center;
        const float  distanceSquared = centerToPoint.squared_length();
        PLYLIBASSERT( distanceSquared <= radiusSquared*1.5 );
    }
#endif
}

/*  Write binary representation of the octree to file.  */
bool ZTreeRoot::writeToFile( const std::string& filename )
{
    bool result = false;
    _name = filename;

    std::ofstream output( getBinaryName().c_str(),
                          std::ios::out | std::ios::binary );
    if( output )
    {
        // enable exceptions on stream errors
        output.exceptions( std::ofstream::failbit | std::ofstream::badbit );
        try
        {
            toStream( output );
            result = true;
        }
        catch( const std::exception& e )
        {
            PLYLIBERROR << "Unable to write binary file, an exception "
                      << "occured:  " << e.what() << std::endl;
        }
        output.close();
    }
    else
    {
        PLYLIBERROR << "Unable to create binary file." << std::endl;
    }

    return result;
}

/*  Read binary octree representation, construct from ply if unavailable.  */
bool ZTreeRoot::readFromFile(const std::string& filename, bool loadVertexData)
{
    _hasVertexData = loadVertexData;
    _name = filename;
    PLYLIBWARN << "--ZTREEPLY-- ZTreeRoot::readFromFile()" << std::endl;
    PLYLIBWARN << "_hasVertexData = " << _hasVertexData << std::endl;

    if( _readBinary( getBinaryName() ) ||
        _constructFromPly( filename ) )
    {
        PLYLIBWARN << "--ZTREEPLY-- ZTreeRoot::readFromFile() ---- END" << std::endl;
        return true;
    }

    _name = "";

    PLYLIBWARN << "--ZTREEPLY-- ZTreeRoot::readFromFile() ---- FALSE END" << std::endl;
    return false;
}


std::string ZTreeRoot::getBinaryName() const
{
    return getArchitectureFilename( _name, detail::BinaryNameTag );
}

/*  Write root node to output stream and continue with other nodes.  */
void ZTreeRoot::toStream( std:: ostream& os )
{
    size_t version = FILE_VERSION;
    os.write( reinterpret_cast< char* >( &version ), sizeof( size_t ) );
    size_t nodeType = ROOT_TYPE;
    os.write( reinterpret_cast< char* >( &nodeType ), sizeof( size_t ) );
    _data.toStream( os );
    ZTreeNode::toStream( os );
}


/*  Read root node from memory and continue with other nodes.  */
void ZTreeRoot::fromMemory(char* start )
{
    PLYLIBWARN << "--ZTREEPLY-- ZTreeRoot::fromMemory()" << std::endl;
    char** addr = &start;
    size_t version;
    memRead( reinterpret_cast< char* >( &version ), addr, sizeof( size_t ) );
    if( version != FILE_VERSION )
        throw MeshException( "Error reading binary file. Version in file "
                             "does not match the expected version." );
    size_t nodeType;
    memRead( reinterpret_cast< char* >( &nodeType ), addr, sizeof( size_t ) );
    if( nodeType != ROOT_TYPE )
        throw MeshException( "Error reading binary file. Expected the root "
                             "node, but found something else instead." );

    if(_hasVertexData)
        _data.fromMemory( addr );
    else
        _data.skipFromMemory( addr );
    ZTreeNode::fromMemory( addr, _data );
    PLYLIBWARN << "--ZTREEPLY-- ZTreeRoot::fromMemory() ---- END" << std::endl;
}

/*  Functions extracted out of readFromFile to enhance readability.  */
bool ZTreeRoot::_constructFromPly( const std::string& filename )
{
    PLYLIBINFO << "Constructing new from PLY file." << std::endl;

    VertexData data;
    if( _invertFaces )
        data.useInvertedFaces();
    if( !data.readPlyFile( filename ) )
    {
        PLYLIBERROR << "Unable to load PLY file." << std::endl;
        return false;
    }

    PLYLIBWARN << "BBox[0] + " << data.getBoundingBox()[0] << std::endl;
    PLYLIBWARN << "BBox[0] + " << data.getBoundingBox()[0] << std::endl;

    data.calculateNormals();
    data.scale( 2.0f );
    setupTree( data );
    if( !writeToFile( filename ))
    {
        PLYLIBWARN << "Unable to write binary representation." << std::endl;
        PLYLIBWARN << "Out-of-core will not work." << std::endl;
        _hasVertexData = false;
    }

    return true;
}

bool ZTreeRoot::_readBinary(std::string filename)
{
#ifdef WIN32

    // replace dir delimiters since '\' is often used as escape char
    for( size_t i=0; i<filename.length(); ++i )
        if( filename[i] == '\\' )
            filename[i] = '/';

    // try to open binary file
    HANDLE file = CreateFile( filename.c_str(), GENERIC_READ, FILE_SHARE_READ,
                              0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 );
    if( file == INVALID_HANDLE_VALUE )
        return false;

    PLYLIBINFO << "Reading cached binary representation." << std::endl;

    // create a file mapping
    HANDLE map = CreateFileMapping( file, 0, PAGE_READONLY, 0, 0,
                                    filename.c_str( ));
    CloseHandle( file );
    if( !map )
    {
        PLYLIBERROR << "Unable to read binary file, file mapping failed."
                  << std::endl;
        return false;
    }

    // get a view of the mapping
    char* addr   = static_cast< char* >( MapViewOfFile( map, FILE_MAP_READ, 0,
                                                        0, 0 ) );
    bool  result = false;

    if( addr )
    {
        try
        {
            fromMemory( addr  );
            result = true;
        }
        catch( const std::exception& e )
        {
            PLYLIBERROR << "Unable to read binary file, an exception occured:  "
                      << e.what() << std::endl;
        }
        UnmapViewOfFile( addr );
    }
    else
    {
        PLYLIBERROR << "Unable to read binary file, memory mapping failed."
                  << std::endl;
        return false;
    }

    CloseHandle( map );
    return result;

#else
    // try to open binary file
    int fd = open( filename.c_str(), O_RDONLY );
    if( fd < 0 )
        return false;

    // retrieving file information
    struct stat status;
    fstat( fd, &status );

    // create memory mapped file
    char* addr   = static_cast< char* >( mmap( 0, status.st_size, PROT_READ,
                                               MAP_SHARED, fd, 0 ) );
    bool  result = false;
    if( addr != MAP_FAILED )
    {
        try
        {
            fromMemory( addr );
            result = true;
        }
        catch( const std::exception& e )
        {
            PLYLIBERROR << "Unable to read binary file, an exception occured:  "
                      << e.what() << std::endl;
        }
        munmap( addr, status.st_size );
    }
    else
    {
        PLYLIBERROR << "Unable to read binary file, memory mapping failed."
                  << std::endl;
    }

    close( fd );
    return result;
#endif
}

/*  Set up the common OpenGL state for rendering of all nodes.  */
void ZTreeRoot::_beginRendering( VertexBufferState& state ) const
{
    state.resetRegion();
    switch( state.getRenderMode() )
    {
#ifdef GL_ARB_vertex_buffer_object
    case RENDER_MODE_BUFFER_OBJECT:
        glPushClientAttrib( GL_CLIENT_VERTEX_ARRAY_BIT );
        glEnableClientState( GL_VERTEX_ARRAY );
        glEnableClientState( GL_NORMAL_ARRAY );
        if( state.useColors() )
            glEnableClientState( GL_COLOR_ARRAY );
#endif
    case RENDER_MODE_DISPLAY_LIST:
    case RENDER_MODE_IMMEDIATE:
    default:
        ;
    }
}

/*  Tear down the common OpenGL state for rendering of all nodes.  */
void ZTreeRoot::_endRendering( VertexBufferState& state ) const
{
    switch( state.getRenderMode() )
    {
#ifdef GL_ARB_vertex_buffer_object
    case RENDER_MODE_BUFFER_OBJECT:
    {
        // deactivate VBO and EBO use
#define glewGetContext state.glewGetContext
        glBindBuffer( GL_ARRAY_BUFFER_ARB, 0);
        glBindBuffer( GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
        glPopClientAttrib();
    }
#endif
    case RENDER_MODE_DISPLAY_LIST:
    case RENDER_MODE_IMMEDIATE:
    default:
        ;
    }
}

}
