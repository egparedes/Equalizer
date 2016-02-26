
/* Copyright (c) 2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
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


#ifndef TRIPLY_TREEDATAMANAGER_H
#define TRIPLY_TREEDATAMANAGER_H

#include "typedefs.h"
#include <triply/api.h>
#include <lunchbox/refPtr.h>

namespace triply {

namespace detail { class TreeDataManager; }

class TreeDataManager
{
public:
    static const size_t DefaultMaxMemoryHint = 4*1024*1024*1024ull; // 4 GiB

    TRIPLY_API explicit TreeDataManager();
    TRIPLY_API ~TreeDataManager();

    TRIPLY_API bool init( const std::string& filename, bool useColors,
                          size_t maxMemoryHint=DefaultMaxMemoryHint );

    TRIPLY_API void getVertexData( Index start, Index length,
                                   SegmentedBuffer& vertices,
                                   SegmentedBuffer& normals );
    TRIPLY_API void getVertexData( Index start, Index length,
                                   SegmentedBuffer& vertices,
                                   SegmentedBuffer& normals,
                                   SegmentedBuffer& colors );
    TRIPLY_API void getIndexData( Index start, Index length,
                                  SegmentedBuffer& indices );

    TRIPLY_API void discardVertexData( Index start, Index length );
    TRIPLY_API void discardIndexData( Index start, Index length );

    TRIPLY_API size_t getFreeVertexBlocks();
    TRIPLY_API size_t getFreeIndexBlocks();
    TRIPLY_API size_t getUsedVertexBlocks();
    TRIPLY_API size_t getUsedIndexBlocks();

//    void toStream( std::ostream& os );
//    void fromMemory( char** addr );
//    void fromMemorySkipData( char** addr );
//    void calculateBoundingBox();
//    size_t getNumVertices() const;
//    size_t getNumIndices() const;
//    size_t getTotalSize() const;
//    BoundingBox getBoundingBox() const;

private:   
    typedef lunchbox::RefPtr< detail::TreeDataManager > SharedDataPtr;
    SharedDataPtr _impl;
};

} // namespace triply

#endif // TRIPLY_TREEDATAMANAGER_H
