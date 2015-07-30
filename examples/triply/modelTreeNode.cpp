
/* Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
 *               2008-2011, Stefan Eilemann <eile@equalizergraphics.com>
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


#include "modelTreeNode.h"
#include "modelTreeLeaf.h"
#include "treeRenderState.h"
#include "vertexData.h"
#include <cmath>
#include <set>

namespace triply
{

inline static bool _subdivide( const Index length, const size_t depth )
{
    return ( length / 2 > LEAF_SIZE ) || ( depth < 3 && length > 1 );
}

ModelTreeNode::ModelTreeNode( unsigned arity )
    : _arity( arity ), _children( 0 )
{
    allocateChildren();
}

/*  Destructor, clears up children as well.  */
ModelTreeNode::~ModelTreeNode()
{
    deallocateChildren();
}

/*  Draw the node by rendering the children.  */
void ModelTreeNode::draw(TreeRenderState &state ) const
{
    if( state.stopRendering( ) )
        return;

    for( unsigned i=0; i < _arity; ++i )
    {
        if( _children[i] != 0 )
            _children[i]->draw( state );
    }
}

Index ModelTreeNode::getNumberOfVertices() const
{
    Index result = 0;
    for( unsigned i=0; i < _arity; ++i )
    {
        if( _children[i] != 0 )
            result += _children[i]->getNumberOfVertices();
    }
    return result;
}

unsigned ModelTreeNode::getNumberOfChildren() const
{
    unsigned result = 0;
    for( unsigned i=0; i < _arity; ++i )
    {
        if( _children[i] != 0 )
            result++;
    }
    return result;
}

/*  Write node to output stream and continue with remaining nodes.  */
void ModelTreeNode::toStream( std::ostream& os )
{
    size_t nodeType = NODE_TYPE;
    os.write( reinterpret_cast< char* >( &nodeType ), sizeof( size_t ) );
    ModelTreeBase::toStream( os );

    size_t numberOfChildren = getNumberOfChildren();
    os.write( reinterpret_cast< char* >( &numberOfChildren ), sizeof( size_t ) );
    for( size_t i=0; i < _arity; ++i)
    {
        if( _children[i] != 0 )
        {
            os.write( reinterpret_cast< char* >( &i), sizeof( size_t ) );
            static_cast< ModelTreeNode* >( _children[i] )->toStream( os );
        }
    }
}

/*  Read node from memory and continue with remaining nodes.  */
void ModelTreeNode::fromMemory( char** addr, ModelTreeData& treeData )
{
    // read node itself
    size_t nodeType;
    memRead( reinterpret_cast< char* >( &nodeType ), addr, sizeof( size_t ) );
    if( nodeType != NODE_TYPE )
        throw MeshException( "Error reading binary file. Expected a regular "
                             "node, but found something else instead." );
    ModelTreeBase::fromMemory( addr, treeData );

    size_t numberOfChildren;
    memRead( reinterpret_cast< char* >( &numberOfChildren ), addr, sizeof( size_t ) );
    for( size_t i=0; i < numberOfChildren; ++i)
    {
        // read child index
        size_t child;
        memRead( reinterpret_cast< char* >( &child ), addr, sizeof( size_t ) );

        // read child node type (peek ahead)
        memRead( reinterpret_cast< char* >( &nodeType ), addr, sizeof( size_t ) );
        if( nodeType != NODE_TYPE && nodeType != LEAF_TYPE )
            throw MeshException( "Error reading binary file. Expected either a "
                                 "regular or a leaf node, but found neither." );
        *addr -= sizeof( size_t );
        if( nodeType == NODE_TYPE )
        {
            _children[child] = new ModelTreeNode( _arity );
        }
        else
        {
            _children[child] = new ModelTreeLeaf( treeData );
        }
        _children[child]->fromMemory( addr, treeData );
    }
}

/*  Continue kd-tree setup, create intermediary or leaf nodes as required.  */
void ModelTreeNode::setupKDTree( VertexData& modelData,
                                 const Index start, const Index length,
                                 const Axis axis, const size_t depth,
                                 ModelTreeData& treeData )
{
#ifndef NDEBUG
    PLYLIBINFO << "setupKDTree"
             << "( " << start << ", " << length << ", " << axis << ", " 
             << depth << " )." << std::endl;
#endif

    modelData.sort( start, length, axis );
    const Index median = start + ( length / 2 );

    // left child will include elements smaller than the median
    const Index leftLength    = length / 2;
    const bool  subdivideLeft = _subdivide( leftLength, depth );

    if( subdivideLeft )
        _children[ ModelTreeBase::LeftChildId ] =  new ModelTreeNode( _arity );
    else
        _children[ ModelTreeBase::LeftChildId ] =  new ModelTreeLeaf( treeData );
    
    // right child will include elements equal to or greater than the median
    const Index rightLength    = ( length + 1 ) / 2;
    const bool  subdivideRight = _subdivide( rightLength, depth );

    if( subdivideRight )
        _children[ ModelTreeBase::RightChildId ] =  new ModelTreeNode( _arity );
    else
        _children[ ModelTreeBase::RightChildId ] =  new ModelTreeLeaf( treeData );
    
    // move to next axis and continue contruction in the child nodes
    const Axis newAxisLeft  = subdivideLeft ? 
                        modelData.getLongestAxis( start , leftLength ) : AXIS_X;

    const Axis newAxisRight = subdivideRight ? 
                        modelData.getLongestAxis( median, rightLength ) : AXIS_X;

    static_cast< ModelTreeNode* >
        ( _children[ ModelTreeBase::LeftChildId ] )->setupKDTree( modelData, start, leftLength, newAxisLeft, depth+1,
                                  treeData );
    static_cast< ModelTreeNode* >
        ( _children[ ModelTreeBase::RightChildId ] )->setupKDTree( modelData, median, rightLength, newAxisRight, depth+1,
                               treeData );

}

/*  Continue z-octree setup, create intermediary or leaf nodes as required.  */
void ModelTreeNode::setupZOctree( VertexData& modelData,
                                  const std::vector< ZKeyIndexPair >& zKeys,
                                  const ZKey beginKey, const ZKey endKey,
                                  const Vertex center, const size_t depth,
                                  ModelTreeData& treeData )
{
#ifndef NDEBUG
    PLYLIBINFO << "setupZOctree"
             << "( " << beginKey << ", " << endKey << ", " << depth
             << " )." << std::endl;
#endif
    ZKey currentKey = beginKey;

    // Compute level
    const BoundingBox& bbox = modelData.getBoundingBox();
    Vertex halfChildSize = ( bbox[1] - bbox[0] ) / ( 2 << (depth + 1) );

    ZKey keyStep = 1ull << 3 * ( ModelTreeBase::MaxZLevel - depth );
    std::vector< ZKeyIndexPair >::const_iterator childBeginIt =
            std::lower_bound( zKeys.begin(), zKeys.end(), currentKey,
                              ZKeyIndexPairLessCmpFunctor() );
    for( unsigned i=0; i < _arity; ++i )
    {
        std::vector< ZKeyIndexPair >::const_iterator childEndIt =
                std::lower_bound( childBeginIt, zKeys.end(), currentKey + keyStep,
                                  ZKeyIndexPairLessCmpFunctor() );
        Index length = std::distance( childBeginIt, childEndIt );

        if( length > 0 )
        {
            if( depth < ModelTreeBase::MaxZLevel - 1 && _subdivide( length, depth ) )
                _children[i] = new ModelTreeNode( _arity );
            else
                _children[i] = new ModelTreeLeaf( treeData );

            Vertex offset = halfChildSize;
            offset[0] *= ( i & 0x1 ) ? 1.0 : -1.0;
            offset[1] *= ( i & 0x2 ) ? 1.0 : -1.0;
            offset[2] *= ( i & 0x4 ) ? 1.0 : -1.0;
            static_cast< ModelTreeNode* >
                    ( _children[i] )->setupZOctree( modelData, zKeys,
                                                    currentKey, currentKey + keyStep,
                                                    center + offset, depth + 1,
                                                    treeData );
        }

        currentKey += keyStep;
        childBeginIt = childEndIt;
    }

    PLYLIBASSERT( beginKey == beginKey );
}

/*  Compute the bounding sphere from the children's bounding spheres.  */
const BoundingSphere& ModelTreeNode::updateBoundingSphere()
{
    _boundingSphere.w() = 0;

    // Merge the bounding spheres returned by the children
    for( unsigned i=0; i < _arity; ++i )
    {
        if( _children[i] != 0 )
        {
            if( _boundingSphere.w() == 0 )
            {
                _boundingSphere = _children[i]->updateBoundingSphere();
            }
            else
            {
                const BoundingSphere& sphere0 = _boundingSphere;
                const BoundingSphere& sphere1 = _children[i]->updateBoundingSphere();
                Vertex center0( sphere0.array );
                Vertex center1( sphere1.array );
                Vertex diff = center1 - center0;

                float diffLength = diff.length();
                if( std::fabs((sphere1.w() - sphere0.w()) >= diffLength ) )
                {
                    // One sphere is fully included in the other
                    //
                    // From "Real-Time Collision Detection" book
                    // (section "6.5.2: Merging Two Spheres"
                    if( sphere1.w() > sphere0.w() )
                        _boundingSphere = sphere1;
                    else
                        _boundingSphere = sphere0;
                }
                else
                {
                    // Not fully overlapping spheres: compute enclosing sphere
                    diff.normalize();
                    const Vertex outer0 = center0 - diff * sphere0.w();
                    const Vertex outer1 = center1 + diff * sphere1.w();
                    Vertex vertexBoundingSphere = Vertex( outer0 + outer1 ) * 0.5f;

                    _boundingSphere.x() = vertexBoundingSphere.x();
                    _boundingSphere.y() = vertexBoundingSphere.y();
                    _boundingSphere.z() = vertexBoundingSphere.z();
                    _boundingSphere.w() = Vertex( outer0 - outer1 ).length() * 0.5f;
                }
            }
        }
    }


#ifndef NDEBUG
    PLYLIBINFO << "updateBoundingSphere" << "( " << _boundingSphere << " )."
             << std::endl;
#endif

    return _boundingSphere;
}

/*  Compute the range from the children's ranges.  */
void ModelTreeNode::updateRange()
{
    _range[0] = FLT_MAX;
    _range[1] = FLT_MIN;
    // update the children's ranges
    for( unsigned i=0; i < _arity; ++i )
    {
        if( _children[i] != 0 )
        {
            _children[i]->updateRange();
            // set node range to min/max of the children's ranges
            _range[0] = LB_MIN( _range[0], ( _children[i] )->getRange()[0] );
            _range[1] = LB_MAX( _range[1], ( _children[i] )->getRange()[1] );
        }
    }

#ifndef NDEBUG
    PLYLIBINFO << "updateRange" << "( " << _range[0] << ", " << _range[1]
             << " )." << std::endl;
#endif
}

void ModelTreeNode::allocateChildren()
{
    LBASSERT( _arity > 0 );
    LBASSERT( !_children );

    _children = new ModelTreeBase*[_arity];
    for( unsigned i=0; i < _arity; ++i )
    {
        _children[i] = 0;
    }
}

void ModelTreeNode::deallocateChildren()
{
    if( _children != 0 )
    {
        for( unsigned i=0; i < _arity; ++i )
        {
            if( _children[i] != 0 )
            {
                delete _children[i];
            }
        }
        delete[] _children;
        _children = 0;
    }
}

}
