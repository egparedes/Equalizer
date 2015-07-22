
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

#ifndef PLYLIB_ZTREENODE_H
#define PLYLIB_ZTREENODE_H

#include "api.h"
#include "zTreeBase.h"

namespace triply
{
/* The class for regular (non-leaf) octree nodes.  */
class ZTreeNode : public ZTreeBase
{
public:
    ZTreeNode();
    PLYLIB_API virtual ~ZTreeNode();

    PLYLIB_API void draw( VertexBufferState& state ) const override;
    PLYLIB_API Index getNumberOfVertices() const override;

    virtual unsigned getNumberofChildren( ) const override;
    virtual const ZTreeBase* getChild( unsigned char childId ) const override
        { return ( _childNodes[childId] != 0 )? _childNodes[childId]: 0; }

    virtual ZTreeBase* getChild( unsigned char childId ) override
        { return ( _childNodes[childId] != 0 )? _childNodes[childId]: 0; }

protected:
    PLYLIB_API virtual void toStream( std::ostream& os ) override;
    PLYLIB_API virtual void fromMemory( char** addr, VertexBufferData& globalData ) override;

    PLYLIB_API void setupTree(VertexData& modelData,
                              std::vector< ZKeyIndexPair >& zKeys,
                              const ZKey beginKey,const ZKey endKey, const size_t depth,
                              Vertex center, VertexBufferData& globalData ) override;
    PLYLIB_API const BoundingSphere& updateBoundingSphere() override;
    PLYLIB_API void updateRange() override;

    unsigned char getNumberOfChildren() const;

private:
    friend class ZTreeDist;
    ZTreeBase* _childNodes[8];
};
}
#endif // PLYLIB_ZTREENODE_H
