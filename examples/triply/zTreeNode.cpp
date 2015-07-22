
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
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


#include "zTreeNode.h"
#include "zTreeLeaf.h"
#include "vertexBufferState.h"
#include "vertexData.h"
#include <set>

namespace triply
{

inline static bool _subdivide( const Index length, const size_t depth )
{
    return ( length / 2 > LEAF_SIZE ) || ( depth < 1 && length > 1 );
}


ZTreeNode::ZTreeNode()
{
    for( unsigned i=0; i < 8; ++i)
    {
        _childNodes[i] = 0;
    }
}


/*  Destructor, clears up children as well.  */
ZTreeNode::~ZTreeNode()
{
    for( unsigned i=0; i < 8; ++i)
    {
        if( _childNodes[i] != 0 )
        {
            delete _childNodes[i];
            _childNodes[i] = 0;
        }
    }
}

/*  Draw the node by rendering the children.  */
void ZTreeNode::draw( VertexBufferState& state ) const
{
    if( state.stopRendering( ) )
        return;

    for( unsigned i=0; i < 8; ++i)
    {
        if( _childNodes[i] != 0 )
            _childNodes[i]->draw( state );
    }
}

Index ZTreeNode::getNumberOfVertices() const
{
    Index result = 0;
    for( unsigned i=0; i < 8; ++i)
    {
        if( _childNodes[i] != 0 )
            result += _childNodes[i]->getNumberOfVertices();
    }
    return result;
}

unsigned ZTreeNode::getNumberofChildren() const
{
    unsigned result = 0;
    for( unsigned i=0; i < 8; ++i)
    {
        if( _childNodes[i] != 0 )
            result++;
    }
    return result;
}

/*  Write node to output stream and continue with remaining nodes.  */
void ZTreeNode::toStream( std::ostream& os )
{
    size_t nodeType = NODE_TYPE;
    os.write( reinterpret_cast< char* >( &nodeType ), sizeof( size_t ) );
    ZTreeBase::toStream( os );

    size_t numberOfChildren = getNumberOfChildren();
    os.write( reinterpret_cast< char* >( &numberOfChildren ), sizeof( size_t ) );
    for( size_t child=0; child < 8; ++child)
    {
        if( _childNodes[child] != 0 )
        {
            os.write( reinterpret_cast< char* >( &child), sizeof( size_t ) );
            static_cast< ZTreeNode* >( _childNodes[child] )->toStream( os );
        }
    }
}

/*  Read node from memory and continue with remaining nodes.  */
void ZTreeNode::fromMemory( char** addr, VertexBufferData& globalData )
{
    PLYLIBWARN << "--ZTREEPLY-- ZTreeNode::fromMemory()" << std::endl;

    // read node itself
    size_t nodeType;
    memRead( reinterpret_cast< char* >( &nodeType ), addr, sizeof( size_t ) );
    PLYLIBWARN << "--ZTREEPLY-- ZTreeNode::memRead()" << std::endl;
    if( nodeType != NODE_TYPE )
        throw MeshException( "Error reading binary file. Expected a regular "
                             "node, but found something else instead." );
    ZTreeBase::fromMemory( addr, globalData );

    size_t numberOfChildren;
    memRead( reinterpret_cast< char* >( &numberOfChildren ), addr, sizeof( size_t ) );
    for( size_t i=0; i < numberOfChildren; ++i)
    {
        // read child index
        size_t child;
        memRead( reinterpret_cast< char* >( &child ), addr, sizeof( size_t ) );

        // read child node type (peek ahead)
        memRead( reinterpret_cast< char* >( &nodeType ), addr, sizeof( size_t ) );
        PLYLIBWARN << "--ZTREEPLY-- ZTreeNode::child " << child << std::endl;
        if( nodeType != NODE_TYPE && nodeType != LEAF_TYPE )
            throw MeshException( "Error reading binary file. Expected either a "
                                 "regular or a leaf node, but found neither." );
        *addr -= sizeof( size_t );
        if( nodeType == NODE_TYPE )
        {
            _childNodes[child] = new ZTreeNode;
        }
        else
        {
            _childNodes[child] = new ZTreeLeaf( globalData);
        }
        _childNodes[child]->fromMemory( addr, globalData );
    }

    PLYLIBWARN << "--ZTREEPLY-- ZTreeNode::fromMemory() ---- END" << std::endl;
}


/*  Continue octree setup, create intermediary or leaf nodes as required.  */
void ZTreeNode::setupTree(VertexData& modelData, std::vector< ZKeyIndexPair >& zKeys,
                          const ZKey beginKey, const ZKey endKey, const size_t depth,
                          Vertex center, VertexBufferData& globalData )
{
#ifndef NDEBUG
    PLYLIBINFO << "setupTree"
             << "( " << beginKey << ", " << endKey << ", " << depth
             << " )." << std::endl;
#endif
    _zRange[0] = beginKey;
    _zRange[1] = endKey;
    ZKey currentKey = beginKey;

    // Compute level
    const BoundingBox& bbox = modelData.getBoundingBox();
    Vertex halfChildSize = (bbox[1] - bbox[0]) / (2 << (depth + 1));

    ZKey keyStep = 1ull << 3*(ZTreeBase::MaxLevel - depth);
    PLYLIBINFO << "keyStep = 0x" << std::hex << keyStep << " | " << std::dec << keyStep<< std::endl;
    std::vector< ZKeyIndexPair >::iterator childBeginIt =
            std::lower_bound( zKeys.begin(), zKeys.end(), currentKey,
                              ZKeyIndexPairLessCmpFunctor());
    for( unsigned i=0; i < 8; ++i )
    {
        PLYLIBINFO << "----  Z range = 0x" << std::hex << std::setfill('0') << std::setw(16) << currentKey << " - 0x" << (currentKey + keyStep) << std::setfill(' ') << std::dec << std::endl;
        PLYLIBINFO << "DEPTH - " << depth << " - child /" << i << "/ ----" << std::endl;
        PLYLIBINFO << "*childBeginIt = " << childBeginIt->first << std::endl;
        PLYLIBINFO << "pos(childBeginIt) = " << std::distance(zKeys.begin(), childBeginIt) << std::endl;
        std::vector< ZKeyIndexPair >::iterator childEndIt =
                std::lower_bound( childBeginIt, zKeys.end(), currentKey + keyStep,
                                  ZKeyIndexPairLessCmpFunctor() );
        Index length = std::distance( childBeginIt, childEndIt );
        PLYLIBINFO << "*childEndIt = " << childEndIt->first << std::endl;
        PLYLIBINFO << "pos(childEndIt) = " << std::distance(zKeys.begin(), childEndIt) << std::endl;
        PLYLIBINFO << "length = " << length << std::endl;

        if( length > 0 )
        {
            if( depth < MaxLevel - 1 && _subdivide( length, depth ) )
                _childNodes[i] = new ZTreeNode;
            else
                _childNodes[i] = new ZTreeLeaf( globalData );

            Vertex offset = halfChildSize;
            offset[0] *= ( i & 0x1 ) ? 1.0 : -1.0;
            offset[1] *= ( i & 0x2 ) ? 1.0 : -1.0;
            offset[2] *= ( i & 0x4 ) ? 1.0 : -1.0;
            static_cast< ZTreeNode* >
                    ( _childNodes[i] )->setupTree(modelData, zKeys, currentKey, currentKey + keyStep,
                                                  depth + 1, center + offset, globalData );
        }

        currentKey += keyStep;
        childBeginIt = childEndIt;
    }

    PLYLIBASSERT( beginKey == beginKey );
}


/*  Compute the bounding sphere from the children's bounding spheres.  */
const BoundingSphere& ZTreeNode::updateBoundingSphere()
{
    unsigned firstI = 0;
    while(firstI < 8 && _childNodes[firstI] == 0)
         ++firstI;
    _boundingSphere = _childNodes[firstI]->updateBoundingSphere();

    // take the bounding spheres returned by the children
    for( unsigned i=firstI + 1; i < 8; ++i)
    {
        if( _childNodes[i] != 0 )
        {
            const BoundingSphere& sphere1 = _boundingSphere;
            const BoundingSphere& sphere2 = _childNodes[i]->updateBoundingSphere();

            // compute enclosing sphere
            const Vertex center1( sphere1.array );
            const Vertex center2( sphere2.array );
            Vertex c1ToC2     = center2 - center1;
            c1ToC2.normalize();

            const Vertex outer1 = center1 - c1ToC2 * sphere1.w();
            const Vertex outer2 = center2 + c1ToC2 * sphere2.w();

            Vertex vertexBoundingSphere = Vertex( outer1 + outer2 ) * 0.5f;
            _boundingSphere.x() = vertexBoundingSphere.x();
            _boundingSphere.y() = vertexBoundingSphere.y();
            _boundingSphere.z() = vertexBoundingSphere.z();
            _boundingSphere.w() = Vertex( outer1 - outer2 ).length() * 0.5f;
        }
    }

#ifndef NDEBUG
    PLYLIBINFO << "updateBoundingSphere" << "( " << _boundingSphere << " )."
             << std::endl;
#endif

    return _boundingSphere;
}


/*  Compute the range from the children's ranges.  */
void ZTreeNode::updateRange()
{
    _range[0] = FLT_MAX;
    _range[1] = FLT_MIN;
    // update the children's ranges
    for( unsigned i=0; i < 8; ++i)
    {
        if( _childNodes[i] != 0 )
        {
            _childNodes[i]->updateRange();
            // set node range to min/max of the children's ranges
            _range[0] = LB_MIN( _range[0], ( _childNodes[i] )->getRange()[0] );
            _range[1] = LB_MAX( _range[1], ( _childNodes[i] )->getRange()[1] );
        }
    }

#ifndef NDEBUG
    PLYLIBINFO << "updateRange" << "( " << _range[0] << ", " << _range[1]
             << " )." << std::endl;
#endif
}

unsigned char ZTreeNode::getNumberOfChildren() const
{
    unsigned char result = 0;
    for( unsigned i=0; i < 8; ++i)
    {
        if( _childNodes[i] != 0 )
            result++;
    }
    return result;
}


}
