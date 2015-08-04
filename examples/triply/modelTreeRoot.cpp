
/* Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
 *          2009-2012, Stefan Eilemann <eile@equalizergraphics.com>
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


#include "modelTreeRoot.h"
#include "treeRenderState.h"
#include "vertexData.h"
#include "mmap.h"
#include <string>
#include <sstream>
#include <cstring>

#define NUM_ELEMS( a ) (sizeof( a ) / sizeof( a[ 0 ] ))

namespace triply
{

namespace detail
{
static const char* PartitionTags[] = { "kd", "z" };
static unsigned PartitionArities[] = { 2, 8 }; //{ 2, 8 };

bool isValidPartition( TreePartitionRule partition)
{
    return partition >= KDTREE_PARTITION && partition <= OCTREE_PARTITION;
}
} // detail


TreePartitionRule ModelTreeRoot::makeTreePartitionRule(const char* partitionTag)
{
    for( unsigned i=0; i < NUM_ELEMS( detail::PartitionTags ); ++i )
    {
        if( strcmp( partitionTag, detail::PartitionTags[i]) == 0 )
            return TreePartitionRule( i );
    }
    return TreePartitionRule(-1);
}

typedef vmml::frustum_culler< float >  FrustumCuller;

//#define LOGCULL
void ModelTreeRoot::cullDraw( RenderState& state ) const
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
    std::vector< const triply::ModelTreeBase* > candidates;
    candidates.push_back( this );

    while( !candidates.empty() )
    {
        if( state.stopRendering( ))
            return;

        const triply::ModelTreeBase* treeNode = candidates.back();
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

                if( treeNode->getNumberOfChildren() == 0 )
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
                    for( unsigned i=0; i < ModelTreeNode::_arity; ++i)
                    {
                        const triply::ModelTreeBase* child = treeNode->getChild( i );
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
    const size_t verticesTotal = _globalData.vertices.size(); //model->getNumberOfVertices();
    TRIPLYINFO
        << getName() << " rendered " << verticesRendered * 100 / verticesTotal
        << "% of model, overlap <= " << verticesOverlap * 100 / verticesTotal
        << "%" << std::endl;
#endif
}

/*  Delegate rendering to node routine.  */
void ModelTreeRoot::draw( RenderState& state ) const
{
    ModelTreeNode::draw( state );
}

bool ModelTreeRoot::setupTree( VertexData& modelData, TreePartitionRule partition,
                               boost::progress_display& progress )
{
    TRIPLYASSERT( detail::isValidPartition( partition ));

    ModelTreeNode::_arity = detail::PartitionArities[partition];
    allocateChildArray();

    const BoundingBox& bbox = modelData.getBoundingBox();
    _treeData.clear();
    _treeData.hasColors = !modelData.colors.empty();
    _treeData.boundingBox = bbox; // _treeData.calculateBoundingBox();

    {
        // For kd-tree
        const Axis axis = modelData.getLongestAxis( 0, modelData.triangles.size() );

        // For ZOctree
        std::vector< ZKeyIndexPair > zKeys;

        switch( partition )
        {
        case KDTREE_PARTITION:
            //  Begin kd-tree setup, go through full range starting with x axis.
            ModelTreeNode::setupMKDTree( modelData, 0, modelData.triangles.size(),
                                         axis, 0, _treeData, progress );
            break;

        case OCTREE_PARTITION:
            // Compute Z-codes for triangles and sort them.
            modelData.genZKeys( zKeys, ModelTreeBase::MaxZLevel );
            modelData.sortZKeys( zKeys );

            ModelTreeNode::setupZOctree( modelData, zKeys,
                                         ModelTreeBase::MinZKey, ModelTreeBase::MaxZKey + 1,
                                         (bbox[0] + bbox[1]) / 2.0, 0,
                                         _treeData, progress );
            break;

        default:
            TRIPLYASSERT( 0 );
            return false;
        }
    }
    
    ++progress;
    ModelTreeNode::updateBoundingSphere();
    ++progress;
    ModelTreeNode::updateRange();


#if 0
    TRIPLYINFO << _treeData.boundingBox[0] << std::endl;
    TRIPLYINFO << _treeData.boundingBox[1] << std::endl;
    TRIPLYINFO << ModelTreeBase::_boundingSphere << std::endl;
    
    // re-test all points to be in the bounding sphere
    Vertex center( _boundingSphere.array );
    float  radius        = _boundingSphere.w();
    float  radiusSquared =  radius * radius;
    for( size_t offset = 0; offset < _globalData.vertices.size(); ++offset )
    {
        const Vertex& vertex = _globalData.vertices[ offset ];

        const Vertex centerToPoint   = vertex - center;
        const float  distanceSquared = centerToPoint.squared_length();
        LBASSERTINFO( distanceSquared <= radiusSquared,
                      distanceSquared << " > " << radiusSquared );
    }
#endif

    return true;
}

/*  Write binary representation of the tree to file.  */
bool ModelTreeRoot::writeToFile( const std::string& filename )
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
            TRIPLYERROR << "Unable to write binary file, an exception "
                      << "occured:  " << e.what() << std::endl;
        }
        output.close();
    }
    else
    {
        TRIPLYERROR << "Unable to create binary file." << std::endl;
    }

    return result;
}

/*  Read binary octree representation, construct from ply if unavailable.  */
bool ModelTreeRoot::readFromFile( const std::string& filename,
                                  TreePartitionRule partition,
                                  bool inCoreData )
{
    if( !detail::isValidPartition( partition ))
        return false;
    ModelTreeNode::_arity = detail::PartitionArities[partition];

    _partition = partition;
    _inCoreData = inCoreData;
    _name = filename;

    if( _readBinary( getBinaryName( ) ) || _constructFromPly( filename ) )
    {
        return true;
    }

    _name = "";

    return false;
}


std::string ModelTreeRoot::getBinaryName( ) const
{
    std::ostringstream oss;
    oss << detail::PartitionTags[_partition];
    oss << ModelTreeNode::_arity;
    return getArchitectureFilename( _name, oss.str() );
}

/*  Write root node to output stream and continue with other nodes.  */
void ModelTreeRoot::toStream( std:: ostream& os )
{
    size_t version = FILE_VERSION;
    os.write( reinterpret_cast< char* >( &version ), sizeof( size_t ) );
    size_t partition = _partition;
    os.write( reinterpret_cast< char* >( &partition ), sizeof( size_t ) );
    size_t treeArity = ModelTreeNode::_arity;
    os.write( reinterpret_cast< char* >( &treeArity ), sizeof( size_t ) );
    size_t nodeType = ROOT_TYPE;
    os.write( reinterpret_cast< char* >( &nodeType ), sizeof( size_t ) );
    _treeData.toStream( os );
    ModelTreeNode::toStream( os );
}


/*  Read root node from memory and continue with other nodes.  */
void ModelTreeRoot::fromMemory( char* start )
{
    char** addr = &start;
    size_t version;
    memRead( reinterpret_cast< char* >( &version ), addr, sizeof( size_t ) );
    if( version != FILE_VERSION )
        throw MeshException( "Error reading binary file. Version in file "
                             "does not match the expected version." );

    size_t partition;
    memRead( reinterpret_cast< char* >( &partition ), addr, sizeof( size_t ) );
    size_t treeArity;
    memRead( reinterpret_cast< char* >( &treeArity ), addr, sizeof( size_t ) );
    if( _partition != partition || ModelTreeNode::_arity != treeArity)
        throw MeshException( "Error reading binary file. Invalid tree specification." );

    size_t nodeType;
    memRead( reinterpret_cast< char* >( &nodeType ), addr, sizeof( size_t ) );
    if( nodeType != ROOT_TYPE )
        throw MeshException( "Error reading binary file. Expected the root "
                             "node, but found something else instead." );

    if( _inCoreData )
        _treeData.fromMemory( addr );
    else
        _treeData.skipFromMemory( addr );

    allocateChildArray();
    ModelTreeNode::fromMemory( addr, _treeData );
}

/*  Functions extracted out of readFromFile to enhance readability.  */
bool ModelTreeRoot::_constructFromPly( const std::string& filename )
{
    TRIPLYINFO << "Reading PLY file." << std::endl;
    boost::progress_display progress( 25 );

    VertexData data;
    if( _invertFaces )
        data.useInvertedFaces();
    if( !data.readPlyFile( filename ) )
    {
        TRIPLYERROR << "Unable to load PLY file." << std::endl;
        return false;
    }
    ++progress;

    data.calculateNormals();
    data.scale( 2.0f );
    ++progress;

    setupTree( data, _partition, progress );
    ++progress;
    if( !writeToFile( filename ))
    {
        TRIPLYWARN << "Unable to write binary representation." << std::endl;
        TRIPLYWARN << "Out-of-core will not work." << std::endl;
        _inCoreData = false;
    }

    ++progress;
    return true;
}

bool ModelTreeRoot::_readBinary(std::string filename)
{
    // create memory mapped file
    char* addr;
    bool  result = false;

    if( openMMap( filename, &addr ))
    {
        try
        {
            fromMemory( addr );
            result = true;
        }
        catch( const std::exception& e )
        {
            TRIPLYERROR << "Unable to read binary file, an exception occured:  "
                      << e.what() << std::endl;
        }
    }
    else
    {
        TRIPLYWARN << "Unable to read binary file, memory mapping failed."
                  << std::endl;
    }

    closeMMap( &addr );

    return result;
}

/*  Set up the common OpenGL state for rendering of all nodes.  */
void ModelTreeRoot::_beginRendering( RenderState& state ) const
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
void ModelTreeRoot::_endRendering( RenderState& state ) const
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
