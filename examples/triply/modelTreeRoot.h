
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


#ifndef PLYLIB_MODELTREEROOT_H
#define PLYLIB_MODELTREEROOT_H

#include "api.h"
#include "modelTreeData.h"
#include "modelTreeNode.h"
#include "pagedTreeData.h"


namespace triply
{
/*  The class for tree root nodes.  */
class ModelTreeRoot : public ModelTreeNode
{
public:
    PLYLIB_API ModelTreeRoot( )
        : ModelTreeNode(), _partition( KDTREE_PARTITION ),
          _invertFaces(false), _inCoreData(false)
    { }

    PLYLIB_API ModelTreeRoot( unsigned arity )
        : ModelTreeNode( arity ), _partition( KDTREE_PARTITION ),
          _invertFaces(false), _inCoreData(false)
    { }

    static PLYLIB_API TreePartitionRule makeTreePartitionRule(const char* partitionTag);

    PLYLIB_API virtual void cullDraw( TreeRenderState& state ) const;
    PLYLIB_API virtual void draw( TreeRenderState& state ) const;

    PLYLIB_API bool setupTree( VertexData& modelData, TreePartitionRule partition );

    PLYLIB_API bool writeToFile( const std::string& filename);
    PLYLIB_API bool readFromFile( const std::string& filename,
                                  TreePartitionRule partition=KDTREE_PARTITION,
                                  bool inCoreData=true);
    PLYLIB_API bool hasColors() const { return _treeData.hasColors; }
    PLYLIB_API BoundingBox getBoundingBox() const { return _treeData.getBoundingBox(); }

    PLYLIB_API void useInvertedFaces() { _invertFaces = true; }

    PLYLIB_API const std::string& getName() const { return _name; }
    PLYLIB_API std::string getBinaryName() const;

protected:
    virtual void toStream( std::ostream& os );
    virtual void fromMemory( char* start );

private:
    bool _constructFromPly( const std::string& filename );
    bool _readBinary( std::string filename );

    void _beginRendering( TreeRenderState& state ) const;
    void _endRendering( TreeRenderState& state ) const;

    friend class ModelTreeDist;

    ModelTreeData      _treeData;
    TreePartitionRule  _partition;
    bool               _invertFaces;
    bool               _inCoreData;  // For out-of-core rendering
    std::string        _name;

};

}  // namespace


#endif // PLYLIB_MODELTREEROOT_H
