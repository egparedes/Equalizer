
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *
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


#ifndef TRIPLY_ZTREEGENERATOR_H
#define TRIPLY_ZTREEGENERATOR_H

#include "typedefs.h"
#include "treeGenerator.h"
#include "modelTreeBase.h"
#include "vertexData.h"
#include <cstdint>
#include <utility>

namespace triply
{

/*  Multiway Z-order generator (usually octree).  */
class ZTreeGenerator : public TreeGenerator
{
public:
    static const std::string Partition;

    typedef std::uint64_t               ZKey;
    typedef std::pair< ZKey, Index >    ZKeyIndexPair;

    static const unsigned MaxZLevel;
    static const ZKey MinZKey;
    static const ZKey MaxZKey;

    struct ZKeyIndexPairLessCmpFunctor
    {
        inline bool operator()( const ZKeyIndexPair& lhs, const ZKeyIndexPair& rhs ) const
        {
            return lhs.first < rhs.first;
        }

        inline bool operator()( const ZKeyIndexPair& lhs, const ZKey rhs ) const
        {
           return lhs.first < rhs;
        }
    };

    struct State
    {
        State( ZKey beginKeyArg, ZKey endKeyArg, size_t depthArg )
            : beginKey( beginKeyArg ), endKey( endKeyArg ), depth( depthArg )
        { }

        ZKey beginKey;
        ZKey endKey;
        size_t depth;
    };

    TRIPLY_API ZTreeGenerator();
    TRIPLY_API virtual ~ZTreeGenerator() { }

    TRIPLY_API virtual const std::string getPartition() const override;

    TRIPLY_API virtual bool generate( VertexData& modelData,
                                      ModelTreeRoot& treeRoot,
                                      ModelTreeData& treeData,
                                      boost::progress_display& progress ) override;

private:
    ModelTreeNode* generateNode( void* state );
    ModelTreeLeaf* generateLeaf( void* state );

    ZKey generateZCode( const Vertex& point, unsigned maxLevel=MaxZLevel );
    void initZKeys( unsigned maxLevel=MaxZLevel );
    bool subdivide( const Index length, const size_t depth );

    BoundingBox _bbox;
    std::vector< ZKeyIndexPair > _zKeys;
    VertexData* _modelData;
    ModelTreeRoot* _treeRoot;
    ModelTreeData* _treeData;
    boost::progress_display* _progress;
    unsigned _initialCount;
};

} // namespace

#endif    // TRIPLY_ZTREEGENERATOR_H
