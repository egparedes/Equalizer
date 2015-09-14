
/* Copyright (c)      2007, Tobias Wolf <twolf@access.unizh.ch>
 *               2009-2014, Stefan Eilemann <eile@equalizergraphics.com>
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


#ifndef TRIPLY_MODELTREEROOT_H
#define TRIPLY_MODELTREEROOT_H

#include "typedefs.h"
#include "modelTreeData.h"
#include "modelTreeNode.h"
#include "treeDataManager.h"
#include <triply/api.h>
#include <vector>

namespace triply
{
/*  The class for tree root nodes.  */
class ModelTreeRoot : public ModelTreeNode
{
public:
    TRIPLY_API ModelTreeRoot( )
        : ModelTreeNode(), _invertFaces(false), _inCoreData(false)
    { }

    TRIPLY_API ModelTreeRoot( unsigned arity )
        : ModelTreeNode( arity, 0 ), _invertFaces(false), _inCoreData(false)
    { }

    TRIPLY_API virtual void clear();

    TRIPLY_API virtual void cullDraw( RenderState& state ) const;
    TRIPLY_API virtual void draw( RenderState& state ) const;

    TRIPLY_API bool setupTree( MeshData& modelData,
                               const TreeInfo& info,
                               boost::progress_display* progressPtr=0);

    TRIPLY_API bool writeToFile( const std::string& filename);
    TRIPLY_API bool readFromFile( const std::string& filename,
                                  const TreeInfo& info=TreeInfo("kd", 2),
                                  bool inCoreData=true );

    TRIPLY_API bool hasColors() const { return _treeData.hasColors; }
    TRIPLY_API BoundingBox getBoundingBox() const { return _treeData.getBoundingBox(); }
    TRIPLY_API size_t getTotalVertices() const { return _treeData.vertices.size(); }
    TRIPLY_API size_t getTotalIndices() const { return _treeData.indices.size(); }
    TRIPLY_API size_t getTotalMemory() const { return _treeData.getTotalSize(); }

    TRIPLY_API void useInvertedFaces() { _invertFaces = true; }

    TRIPLY_API const std::string& getName() const { return _name; }
    TRIPLY_API std::string getBinaryName() const;

protected:
    virtual void toStream( std::ostream& os );
    virtual void fromMemory( char* start );

private:
    bool constructFromPly( const std::string& filename, const TreeInfo& info );
    bool readBinary( std::string filename );
    void showBuildStats(const MeshData &modelData );

    void beginRendering( RenderState& state ) const;
    void endRendering( RenderState& state ) const;

    friend class ModelTreeDist;

    ModelTreeData       _treeData;
    bool                _invertFaces;
    bool                _inCoreData;
    std::string         _partition;
    std::string         _name;
};

}  // namespace


#endif // TRIPLY_MODELTREEROOT_H
