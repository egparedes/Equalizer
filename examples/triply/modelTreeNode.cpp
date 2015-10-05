
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
#include "renderState.h"
#include "meshData.h"
#include <cmath>
#include <set>
#include <utility>

namespace triply
{

ModelTreeNode::ModelTreeNode( unsigned arity, ModelTreeBasePtr* childrenPtr )
{
    TRIPLYASSERT( childrenPtr == 0 );
    if ( childrenPtr != 0 )
    {
        _children.init( childrenPtr, arity );
    }
    else
    {
        _children.resize( arity, 0 );
    }
}

/*  Destructor, clears up children as well.  */
ModelTreeNode::~ModelTreeNode()
{
    clear();
    _children.clear( true );
}

void ModelTreeNode::clear()
{
    for( unsigned i=0; i < _children.size(); ++i )
    {
        if( _children[i] != 0 )
        {
            _children[i]->clear();
            delete _children[i];
            _children[i] = 0;
        }
    }
}

/*  Draw the node by rendering the children.  */
void ModelTreeNode::draw(RenderState &state ) const
{
    if( state.stopRendering( ) || _children.size() == 0 )
        return;

    for( unsigned i=0; i < _children.size(); ++i )
    {
        if( _children[i] != 0 )
            _children[i]->draw( state );
    }
}

Index ModelTreeNode::getNumberOfVertices() const
{
    Index result = 0;
    for( unsigned i=0; i < _children.size(); ++i )
    {
        if( _children[i] != 0 )
            result += _children[i]->getNumberOfVertices();
    }
    return result;
}

unsigned ModelTreeNode::getNumberOfChildren() const
{
    unsigned result = 0;
    for( unsigned i=0; i < _children.size(); ++i )
    {
        if( _children[i] != 0 )
            result++;
    }

    return result;
}

// A pair of numbers per sublevel: (#totalnodes, #leaves)
std::vector< std::pair< unsigned, unsigned > > ModelTreeNode::getDescendantsPerLevel( ) const
{
    std::vector< std::pair< unsigned, unsigned > > nodesPerLevel;

    if ( _children.size() > 0 )
    {
        nodesPerLevel.push_back( std::make_pair< unsigned, unsigned >( 0, 0 ) );
        for( unsigned i=0; i < _children.size(); ++i )
        {
            if( _children[i] != 0 )
            {
                nodesPerLevel[0].first++;
                if( _children[i]->isLeaf() )
                    nodesPerLevel[0].second++;
                std::vector< std::pair< unsigned, unsigned > > childDescendants =
                        _children[i]->getDescendantsPerLevel();

                if( childDescendants.size() > 0 )
                {
                    if( nodesPerLevel.size() < childDescendants.size() + 1 )
                        nodesPerLevel.resize( childDescendants.size() + 1 );
                    for( unsigned l=0; l < childDescendants.size(); ++l )
                    {
                        nodesPerLevel[l + 1].first += childDescendants[l].first;
                        nodesPerLevel[l + 1].second += childDescendants[l].second;
                    }
                }
            }
        }
    }

    return nodesPerLevel;
}

/*  Write node to output stream and continue with remaining nodes.  */
void ModelTreeNode::toStream( std::ostream& os )
{
    size_t nodeType = NODE_TYPE;
    os.write( reinterpret_cast< char* >( &nodeType ), sizeof( size_t ) );
    ModelTreeBase::toStream( os );

    size_t numberOfChildren = getNumberOfChildren();
    os.write( reinterpret_cast< char* >( &numberOfChildren ), sizeof( size_t ) );
    if( numberOfChildren > 0 )
    {
        for( size_t i=0; i < _children.size(); ++i)
        {
            if( _children[i] != 0 )
            {
                os.write( reinterpret_cast< char* >( &i), sizeof( size_t ) );
                static_cast< ModelTreeNode* >( _children[i] )->toStream( os );
            }
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
            _children[child] = new ModelTreeNode( _children.size() );
        }
        else
        {
            _children[child] = new ModelTreeLeaf( treeData );
        }
        _children[child]->fromMemory( addr, treeData );
    }
}

/*  Compute the bounding sphere from the children's bounding spheres.  */
const BoundingSphere& ModelTreeNode::updateBoundingSphere()
{
    _boundingSphere.w() = 0;

    if ( _children.size() == 0 )
        return _boundingSphere;

    // Merge the bounding spheres returned by the children
    for( unsigned i=0; i < _children.size(); ++i )
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
                float radiusDiff = std::fabs(sphere1.w() - sphere0.w());
                if( radiusDiff >= diffLength )
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
    TRIPLYINFO << "updateBoundingSphere" << "( " << _boundingSphere << " )."
             << std::endl;
#endif

    return _boundingSphere;
}

/*  Compute the range from the children's ranges.  */
void ModelTreeNode::updateRange()
{
    if ( _children.size() == 0 )
        return;

    _range[0] = FLT_MAX;
    _range[1] = FLT_MIN;
    // update the children's ranges
    for( unsigned i=0; i < _children.size(); ++i )
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
    TRIPLYINFO << "updateRange" << "( " << _range[0] << ", " << _range[1]
             << " )." << std::endl;
#endif
}

}
