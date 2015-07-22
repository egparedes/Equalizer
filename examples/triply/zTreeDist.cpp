
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2010, Cedric Stalder <cedric.stalder@gmail.com>
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

#include "zTreeDist.h"

#include "zTreeLeaf.h"
#include "zTreeRoot.h"

namespace triply
{

ZTreeDist::ZTreeDist()
    : _root( 0 )
    , _node( 0 )
    , _isRoot( false )
{
    for( unsigned i=0; i < 8; ++i)
    {
        _childrenNodes[i] = 0;
    }
}

ZTreeDist::ZTreeDist( ZTreeRoot* root )
    : _root( root )
    , _node( root )
    , _isRoot( true )
{
    for( unsigned i=0; i < 8; ++i)
    {
        _childrenNodes[i] = 0;
        if( root->getChild( i ) )
        {
            _childrenNodes[i] = new ZTreeDist( root, root->getChild( i ) );
        }
    }
}

ZTreeDist::ZTreeDist( ZTreeRoot* root, ZTreeBase* node )
        : _root( root )
        , _node( node )
        , _isRoot( false )
{
    for( unsigned i=0; i < 8; ++i)
    {
        _childrenNodes[i] = 0;
    }

    if( !node || node->getNumberofChildren() == 0)
        return;

    for( unsigned i=0; i < 8; ++i)
    {
        if( node->getChild( i ) )
        {
            _childrenNodes[i] = new ZTreeDist( root, node->getChild( i ) );
        }
    }
}

ZTreeDist::~ZTreeDist()
{
    for( unsigned i=0; i < 8; ++i)
    {
        if( _childrenNodes[i] != 0 )
        {
            delete _childrenNodes[i];
            _childrenNodes[i] = 0;
        }
    }
}

void ZTreeDist::registerTree( co::LocalNodePtr node )
{
    LBASSERT( !isAttached() );
    LBCHECK( node->registerObject( this ));

    for( unsigned i=0; i < 8; ++i)
    {
        if( _childrenNodes[i] != 0 )
        {
            _childrenNodes[i]->registerTree( node );
        }
    }
}

void ZTreeDist::deregisterTree()
{
    LBASSERT( isAttached() );
    LBASSERT( isMaster( ));

    getLocalNode()->deregisterObject( this );

    for( unsigned i=0; i < 8; ++i)
    {
        if( _childrenNodes[i] != 0 )
        {
            _childrenNodes[i]->deregisterTree();
        }
    }
}

ZTreeRoot* ZTreeDist::loadModel( co::NodePtr master,
                                 co::LocalNodePtr localNode,
                                 const eq::uint128_t& modelID )
{
    LBASSERT( !_root && !_node );

    if( !localNode->syncObject( this, master, modelID ))
    {
        LBWARN << "Mapping of model failed" << std::endl;
        return 0;
    }
    return _root;
}

unsigned ZTreeDist::getNumberOfChildren() const
{
    unsigned result = 0;
    for( unsigned i=0; i < 8; ++i)
    {
        if( _childrenNodes[i] != 0 )
            result++;
    }
    return result;
}

void ZTreeDist::getInstanceData( co::DataOStream& os )
{
    LBASSERT( _node );
    unsigned numChildren = getNumberOfChildren();
    os << _isRoot << numChildren;

    if( getNumberOfChildren() > 0 )
    {
        for( unsigned i=0; i < 8; ++i)
        {
            if( _childrenNodes[i] != 0 )
            {
                os << i << _childrenNodes[i]->getID();
            }
        }

        if( _isRoot )
        {
            LBASSERT( _root );

            const VertexBufferData& data = _root->_data;

            os << _root->_hasVertexData;
            if (_root->_hasVertexData)
                os << data.vertices << data.colors << data.normals << data.indices;
            os << _root->_name;
        }
    }
    else
    {
        LBASSERT( dynamic_cast< const ZTreeLeaf* >( _node ));
        const ZTreeLeaf* leaf =
            static_cast< const ZTreeLeaf* >( _node );

        os << leaf->_boundingBox[0] << leaf->_boundingBox[1]
           << uint64_t( leaf->_vertexStart ) << uint64_t( leaf->_indexStart )
           << uint64_t( leaf->_indexLength ) << leaf->_vertexLength;
    }

    os << _node->_boundingSphere << _node->_range;
}

void ZTreeDist::applyInstanceData( co::DataIStream& is )
{
    LBASSERT( !_node );

    ZTreeNode* node = 0;
    ZTreeBase* base = 0;
    unsigned numChildren;
    eq::uint128_t childIDs[8];
    for( unsigned i=0; i < 8; ++i)
    { childIDs[i] = 0; }

    is >> _isRoot >> numChildren ;

    if( numChildren > 0 )
    {
        for( unsigned i=0; i < numChildren; ++i)
        {
            unsigned childNumber;
            is >> childNumber;
            is >> childIDs[ childNumber ];
        }

        if( _isRoot )
        {
            ZTreeRoot* root = new ZTreeRoot;
            VertexBufferData& data = root->_data;

            is >> root->_hasVertexData;
            if (root->_hasVertexData)
                is >> data.vertices >> data.colors >> data.normals >> data.indices;
            is >> root->_name;

            node  = root;
            _root = root;
        }
        else
        {
            LBASSERT( _root );
            node = new ZTreeNode;
        }

        base   = node;
        co::NodePtr from = is.getRemoteNode();
        co::LocalNodePtr to = is.getLocalNode();
        for( unsigned i=0; i < 8; ++i)
        {
            if( childIDs[i] != 0 )
            {
                _childrenNodes[i] = new ZTreeDist( _root, 0 );
                co::f_bool_t childSync = to->syncObject( _childrenNodes[i], from, childIDs[i] );
                LBCHECK( childSync.wait( ) );
            }
        }

        for( unsigned i=0; i < 8; ++i)
        {
            if( childIDs[i] != 0 )
                node->_childNodes[i] = _childrenNodes[i]->_node;
        }
    }
    else
    {
        LBASSERT( !_isRoot );
        VertexBufferData& data = _root->_data;
        ZTreeLeaf* leaf = new ZTreeLeaf( data );

        uint64_t i1, i2, i3;
        is >> leaf->_boundingBox[0] >> leaf->_boundingBox[1]
           >> i1 >> i2 >> i3 >> leaf->_vertexLength;
        leaf->_vertexStart = size_t( i1 );
        leaf->_indexStart = size_t( i2 );
        leaf->_indexLength = size_t( i3 );

        base = leaf;
    }

    LBASSERT( base );
    is >> base->_boundingSphere >> base->_range;

    _node = base;
}

}
