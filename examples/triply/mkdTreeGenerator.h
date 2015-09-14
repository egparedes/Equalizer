
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *               2008-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2007, Tobias Wolf <twolf@access.unizh.ch>
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


#ifndef TRIPLY_MKDTREEGENERATOR_H
#define TRIPLY_MKDTREEGENERATOR_H

#include "typedefs.h"
#include "treeGenerator.h"
#include "modelTreeBase.h"
#include "meshData.h"

namespace triply
{

/*  Multiway kd-tree generator.  */
class MKDGenerator : public TreeGenerator
{
public:
    static const std::string Partition;

    // enumeration for the sort axis
    enum Axis
    {
        AXIS_X,
        AXIS_Y,
        AXIS_Z
    };

    struct TriangleLessCmpFunctor
    {
        TriangleLessCmpFunctor( const MeshData& dataArg,
                                const MKDGenerator::Axis axisArg );

        bool operator()( const Triangle& t1, const Triangle& t2 ) const;

        const MeshData& data;
        const MKDGenerator::Axis axis;
    };

    struct State
    {
        State( Axis axisArg, Index startArg, Index lengthArg, size_t depthArg )
            : axis( axisArg ), start( startArg ), length( lengthArg ),
              depth( depthArg )
        { }

        Axis axis;
        Index start;
        Index length;
        size_t depth;
    };

    TRIPLY_API MKDGenerator();
    TRIPLY_API virtual ~MKDGenerator() { }

    TRIPLY_API virtual const std::string getPartition() const override;

    TRIPLY_API virtual bool generate( MeshData& meshData,
                                      ModelTreeRoot& treeRoot,
                                      ModelTreeData& treeData,                                      
                                      boost::progress_display& progress ) override;

private:
    ModelTreeNode* generateNode( void* state );
    ModelTreeLeaf* generateLeaf( void* state );

    Axis getLongestAxis( const size_t start, const size_t elements );
    bool subdivide( const Index length, const size_t depth );
    void sortVertices( const Index start, const Index length,
                       const Axis axis );

    MeshData* _meshData;
    ModelTreeRoot* _treeRoot;
    ModelTreeData* _treeData;
    boost::progress_display* _progress;
    unsigned _initialCount;
};

std::ostream& operator<<( std::ostream& os, const MKDGenerator::Axis& axis );

} // namespace triply

#endif // TRIPLY_MKDTREEGENERATOR_H
