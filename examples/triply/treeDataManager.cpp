
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


#include "treeDataManager.h"
#include "segmentedBuffer.h"
#include "mmap.h"
#include <lunchbox/referenced.h>
#include <lunchbox/scopedMutex.h>
#include <lunchbox/spinLock.h>
#include <lunchbox/stdExt.h>

#include <algorithm>
#include <fstream>
#include <list>
#include <unordered_map>
#include <utility>
#include <vector>

namespace triply {

namespace detail
{

class TreeDataManager : public lunchbox::Referenced
{
public:
    enum Constants
    {
        NUM_BUFFER_TYPES = 4ull,
        VERTEX_KEY_BITS = 16ull,
        NORMAL_KEY_BITS = VERTEX_KEY_BITS,
        COLOR_KEY_BITS = VERTEX_KEY_BITS,
        INDEX_KEY_BITS = VERTEX_KEY_BITS + 2,
        VERTEX_BLOCK_LENGTH = (1ull << VERTEX_KEY_BITS),
        NORMAL_BLOCK_LENGTH = (1ull << NORMAL_KEY_BITS),
        COLOR_BLOCK_LENGTH = (1ull << COLOR_KEY_BITS),
        INDEX_BLOCK_LENGTH = (1ull << INDEX_KEY_BITS),
    };

    typedef size_t BlockKey;
    typedef std::list< size_t > BlockIndexList;

    static const BlockKey InvalidBlockKey = -1;
    static const size_t InvalidBlockSize = 0;

    static const size_t BlockKeyBits[NUM_BUFFER_TYPES];
    static const size_t BlockElements[NUM_BUFFER_TYPES];
    static const size_t BlockSize[NUM_BUFFER_TYPES];

    template< typename T, const size_t NumElements >
    struct Block
    {
        Block()
            : key( InvalidBlockKey )
        { }

        size_t key;
        size_t size;
        T data[NumElements];
    };

    struct BlockInfo
    {
        inline BlockInfo()
            : id( 0 ), readersCount( 0 )
        { }

        inline BlockInfo( size_t blockId, BlockIndexList::iterator blockListIt,
                          size_t activeReadersCount )
            : id( blockId ), listIt( blockListIt), readersCount( activeReadersCount )
        { }

        size_t id;
        BlockIndexList::iterator listIt;
        size_t readersCount;
    };

    typedef stde::hash_map< BlockKey, BlockInfo > BlockMap;
    //typedef std::unordered_map< BlockKey, BlockInfo > BlockMap;

    struct BlockCache
    {
        inline BlockCache( size_t numBlocks=0 )
            : vertBlocks( numBlocks), normBlocks( numBlocks ),
              colBlocks( numBlocks ), idxBlocks( numBlocks )
        { }

        inline void resize( size_t numBlocks )
        {
            vertBlocks.resize( numBlocks );
            normBlocks.resize( numBlocks );
            colBlocks.resize( numBlocks );
            idxBlocks.resize( numBlocks );
        }

        inline BlockKey& getKey( BufferType type, size_t blockId )
        {
            switch( type )
            {
            case VERTEX_BUFFER_TYPE:
                TRIPLYASSERT( blockId < vertBlocks.size() );
                return vertBlocks[blockId].key;
                break;
            case NORMAL_BUFFER_TYPE:
                TRIPLYASSERT( blockId < normBlocks.size() );
                return normBlocks[blockId].key;
                break;
            case COLOR_BUFFER_TYPE:
                TRIPLYASSERT( blockId < colBlocks.size() );
                return colBlocks[blockId].key;
                break;
            case INDEX_BUFFER_TYPE:
                TRIPLYASSERT( blockId < idxBlocks.size() );
                return idxBlocks[blockId].key;
                break;
            default:
                TRIPLYASSERT( 0 );
                break;
            };
            return const_cast< BlockKey& >( InvalidBlockKey );
        }

        inline size_t& getSize( BufferType type, size_t blockId )
        {
            switch( type )
            {
            case VERTEX_BUFFER_TYPE:
                TRIPLYASSERT( blockId < vertBlocks.size() );
                return vertBlocks[blockId].size;
                break;
            case NORMAL_BUFFER_TYPE:
                TRIPLYASSERT( blockId < normBlocks.size() );
                return normBlocks[blockId].size;
                break;
            case COLOR_BUFFER_TYPE:
                TRIPLYASSERT( blockId < colBlocks.size() );
                return colBlocks[blockId].size;
                break;
            case INDEX_BUFFER_TYPE:
                TRIPLYASSERT( blockId < idxBlocks.size() );
                return idxBlocks[blockId].size;
                break;
            default:
                TRIPLYASSERT( 0 );
                break;
            };
            return const_cast< size_t& >( InvalidBlockSize );
        }

        inline char* getData( BufferType type, size_t blockId )
        {
            char* result = 0;
            switch( type )
            {
            case VERTEX_BUFFER_TYPE:
                TRIPLYASSERT( blockId < vertBlocks.size() );
                result = reinterpret_cast< char* >( &( vertBlocks[blockId].data[0] ));
                break;
            case NORMAL_BUFFER_TYPE:
                TRIPLYASSERT( blockId < normBlocks.size() );
                result = reinterpret_cast< char* >( &( normBlocks[blockId].data[0] ));
                break;
            case COLOR_BUFFER_TYPE:
                TRIPLYASSERT( blockId < colBlocks.size() );
                result = reinterpret_cast< char* >( &( colBlocks[blockId].data[0] ));
                break;
            case INDEX_BUFFER_TYPE:
                TRIPLYASSERT( blockId < idxBlocks.size() );
                result = reinterpret_cast< char* >( &( idxBlocks[blockId].data[0] ));
                break;
            default:
                TRIPLYASSERT( 0 );
                break;
            };
            return result;
        }

        std::vector< Block< Vertex, VERTEX_BLOCK_LENGTH > >     vertBlocks;
        std::vector< Block< Normal, NORMAL_BLOCK_LENGTH > >     normBlocks;
        std::vector< Block< Color, COLOR_BLOCK_LENGTH > >       colBlocks;
        std::vector< Block< ShortIndex, INDEX_BLOCK_LENGTH > >  idxBlocks;
    };

    TreeDataManager()
        : _cache( 0 ), _filename(""), _useColors( false )
    {
        _boundingBox[0] = Vertex( 0.0 );
        _boundingBox[1] = Vertex( 0.0 );
        for( size_t i=0; i < NUM_BUFFER_TYPES; ++i )
        {
            _totalElems[i] = 0;
            _mmapData[i] = 0;
        }
    }

    ~TreeDataManager()
    { }

    bool isValid()
    {
        return _mmap.isValid();
    }

    bool useColors()
    {
        return _useColors;
    }

    void init( const std::string& filename, bool withColors, size_t maxMemoryHint )
    {
        TRIPLYASSERT( _filename == "" && filename != "");

        for( size_t i=0; i < NUM_BUFFER_TYPES; ++i )
            _lock[i].set();

        _filename = filename;
        _useColors = withColors;
        size_t totalMemPerBlock =
                std::accumulate( BlockSize, BlockSize + NUM_BUFFER_TYPES, 0 );
        if( !_useColors )
            totalMemPerBlock -= BlockSize[COLOR_BUFFER_TYPE];
        size_t numBlocks = std::max( ( maxMemoryHint / totalMemPerBlock ), size_t(2) );

        _cache.resize( numBlocks );
        for( size_t type=VERTEX_BUFFER_TYPE; type <= INDEX_BUFFER_TYPE; ++type )
        {
            for( size_t idx=0; idx < numBlocks; ++idx )
            {
                _freeBlocks[type].push_back( idx );
            }
        }

        openBinary();

        for( size_t i=0; i < NUM_BUFFER_TYPES; ++i )
            _lock[i].unset();
    }

    SegmentedBuffer get( BufferType type, Index start, Index length )
    {
        TRIPLYASSERT( type >= VERTEX_BUFFER_TYPE && type <= INDEX_BUFFER_TYPE );
        TRIPLYASSERT( start + length <= _totalElems[type] );

        SegmentedBuffer result =
                SegmentedBuffer( BlockSize[type] ,
                                 ( start % BlockElements[type] ) * BufferElementSizes[type] );
        for( BlockKey key=computeKey( type, start );
             key * BlockElements[type] < start + length; ++key )
        {
            TRIPLYASSERT( key * BlockElements[type] < _totalElems[type] );

            _lock[type].set();
            if( _blockMap[type].count( key ) == 0 )
            {
                // Load data into cache from memory map
                TRIPLYASSERT( _freeBlocks[type].size() > 0 );
                size_t freeBlockId = *( _freeBlocks[type].begin() );
                BlockKey freeBlockKey = _cache.getKey( type, freeBlockId );
                if( _blockMap[type].count( freeBlockKey ) > 0 && freeBlockKey != InvalidBlockKey )
                    _blockMap[type].erase( freeBlockKey );

                loadBlock( freeBlockId, type, key );
                _blockMap[type][key] = BlockInfo( freeBlockId, _freeBlocks[type].begin(), 0 );
            }

            BlockInfo& blockInfo = _blockMap[type][key];
            if( blockInfo.readersCount == 0)
            {
                // Move to active list
#ifndef NDEBUG
                size_t oldBusySize = _busyBlocks[type].size();
                size_t oldFreeSize = _freeBlocks[type].size();
#endif
                BlockIndexList::iterator nextListIt = blockInfo.listIt;
                ++nextListIt;
                _busyBlocks[type].splice( _busyBlocks[type].end(), _freeBlocks[type],
                                          blockInfo.listIt, nextListIt );
                TRIPLYASSERT( _busyBlocks[type].size() == oldBusySize + 1);
                TRIPLYASSERT( _freeBlocks[type].size() == oldFreeSize - 1);
            }
            blockInfo.readersCount++;
            _lock[type].unset();

            if( start + length >= ((key + 1) * BlockElements[type]) )
            {
                // Normal block
                result.addSegment( _cache.getData( type, blockInfo.id )  );
            }
            else
            {
                // Truncated last block
                size_t numElements = std::min( length,
                                               ( start + length ) % BlockElements[type] );
                result.setEnd( _cache.getData( type, blockInfo.id ),
                                numElements * BufferElementSizes[type]);
            }
        }

        TRIPLYASSERT( result.size() == length * BufferElementSizes[type]);

        return result;
    }

    void discard( BufferType type, Index start, Index length )
    {
        TRIPLYASSERT( type >= VERTEX_BUFFER_TYPE && type <= INDEX_BUFFER_TYPE );
        TRIPLYASSERT( start + length <= _totalElems[type] );

        _lock[type].set();
        for( BlockKey key=computeKey( type, start );
             key * BlockElements[type] < start + length; ++key )
        {
            TRIPLYASSERT( _blockMap[type].count( key ) > 0 );

            BlockInfo& blockInfo = _blockMap[type][key];
            blockInfo.readersCount--;
            if( blockInfo.readersCount == 0)
            {
                // Move to free list
#ifndef NDEBUG
                size_t oldBusySize = _busyBlocks[type].size();
                size_t oldFreeSize = _freeBlocks[type].size();
#endif
                BlockIndexList::iterator nextListIt = blockInfo.listIt;
                ++nextListIt;
                _freeBlocks[type].splice( _freeBlocks[type].end(), _busyBlocks[type],
                                          blockInfo.listIt, nextListIt );
                TRIPLYASSERT( _busyBlocks[type].size() == oldBusySize - 1);
                TRIPLYASSERT( _freeBlocks[type].size() == oldFreeSize + 1);
            };
        }
        _lock[type].unset();
    }

private:
    inline BlockKey computeKey( BufferType type, Index index ) const
    {
        return index >> BlockKeyBits[type];
    }

    void loadBlock( size_t blockId, BufferType type, BlockKey key )
    {
        size_t numElems = BlockElements[type];
        if( (key + 1) * BlockElements[type] > _totalElems[type] )
        {
            numElems = _totalElems[type] - key * BlockElements[type];
        }

        _cache.getKey( type, blockId ) = key;
        _cache.getSize( type, blockId ) = numElems;
        memcpy( _cache.getData( type, blockId ),
                _mmapData[type] + key * BlockSize[type],
                numElems * BufferElementSizes[type] );
    }

    bool openBinary()
    {
        // create memory mapped file
        bool result = _mmap.open( _filename );
        if( result )
        {
            try
            {
                char* dataAddr = _mmap.getPtr();
                size_t version;
                memRead( reinterpret_cast< char* >( &version ), &dataAddr, sizeof( size_t ) );
                if( version != FILE_VERSION )
                    throw MeshException( "Error reading binary file. Version in file "
                                         "does not match the expected version." );

                size_t partitionLength;
                char partitionChars[256];
                memRead( reinterpret_cast< char* >( &partitionLength ), &dataAddr, sizeof( size_t ) );
                memRead( &(partitionChars[0]), &dataAddr, partitionLength * sizeof( char ) );
                size_t treeArity;
                memRead( reinterpret_cast< char* >( &treeArity ), &dataAddr, sizeof( size_t ) );
                if( partitionLength < 1 || treeArity < 2 )
                    throw MeshException( "Error reading binary file. Invalid tree specification." );

                size_t totalTreeNodes;
                memRead( reinterpret_cast< char* >( &totalTreeNodes ), &dataAddr, sizeof( size_t ) );
                size_t nodeType;
                memRead( reinterpret_cast< char* >( &nodeType ), &dataAddr, sizeof( size_t ) );
                if( nodeType != ROOT_TYPE )
                    throw MeshException( "Error reading binary file. Expected the root "
                                         "node, but found something else instead." );

                memRead( reinterpret_cast< char* >( &_boundingBox[0] ),
                         &dataAddr, 2*sizeof( Vertex ) );
                memRead( reinterpret_cast< char* >( &_totalElems[VERTEX_BUFFER_TYPE] ),
                         &dataAddr, sizeof( size_t ) );
                _mmapData[VERTEX_BUFFER_TYPE] = dataAddr;

                dataAddr += _totalElems[VERTEX_BUFFER_TYPE] * sizeof(Vertex);
                memRead( reinterpret_cast< char* >( &_totalElems[COLOR_BUFFER_TYPE] ),
                         &dataAddr, sizeof( size_t ) );
                _mmapData[COLOR_BUFFER_TYPE] = dataAddr;

                dataAddr += _totalElems[COLOR_BUFFER_TYPE] * sizeof(Color);
                memRead( reinterpret_cast< char* >( &_totalElems[NORMAL_BUFFER_TYPE] ),
                         &dataAddr, sizeof( size_t ) );
                _mmapData[NORMAL_BUFFER_TYPE] = dataAddr;

                dataAddr += _totalElems[NORMAL_BUFFER_TYPE] * sizeof(Normal);
                memRead( reinterpret_cast< char* >( &_totalElems[INDEX_BUFFER_TYPE] ),
                         &dataAddr, sizeof( size_t ) );
                _mmapData[INDEX_BUFFER_TYPE] = dataAddr;

                result = true;
            }
            catch( const std::exception& e )
            {
                TRIPLYERROR << "Unable to read binary file, an exception occured:  "
                          << e.what() << std::endl;
            }
        }
        else
        {
            TRIPLYERROR << "Unable to read binary file, memory mapping failed."
                      << std::endl;
        }

        return result;
    }

    void closeBinary()
    {
        _mmap.close();
        _mmapData[VERTEX_BUFFER_TYPE] = reinterpret_cast< char* >( 0 );
        _mmapData[COLOR_BUFFER_TYPE] = reinterpret_cast< char* >( 0 );
        _mmapData[NORMAL_BUFFER_TYPE] = reinterpret_cast< char* >( 0 );
        _mmapData[INDEX_BUFFER_TYPE] = reinterpret_cast< char* >( 0 );
    }

    BoundingBox         _boundingBox;
    std::size_t         _totalElems[NUM_BUFFER_TYPES];
    lunchbox::SpinLock  _lock[NUM_BUFFER_TYPES];
    BlockMap            _blockMap[NUM_BUFFER_TYPES];
    BlockIndexList      _freeBlocks[NUM_BUFFER_TYPES];
    BlockIndexList      _busyBlocks[NUM_BUFFER_TYPES];
    BlockCache          _cache;
    MMap                _mmap;
    char*               _mmapData[NUM_BUFFER_TYPES];
    std::string         _filename;
    bool                _useColors;
};

// "static const" member declarations to avoid linking errors
const size_t TreeDataManager::InvalidBlockKey;
const size_t TreeDataManager::InvalidBlockSize;

const size_t TreeDataManager::BlockKeyBits[TreeDataManager::NUM_BUFFER_TYPES] =
{
    TreeDataManager::VERTEX_KEY_BITS,
    TreeDataManager::NORMAL_KEY_BITS,
    TreeDataManager::COLOR_KEY_BITS,
    TreeDataManager::INDEX_KEY_BITS
};

const size_t TreeDataManager::BlockElements[TreeDataManager::NUM_BUFFER_TYPES] =
{
    TreeDataManager::VERTEX_BLOCK_LENGTH,
    TreeDataManager::NORMAL_BLOCK_LENGTH,
    TreeDataManager::COLOR_BLOCK_LENGTH,
    TreeDataManager::INDEX_BLOCK_LENGTH
};

const size_t TreeDataManager::BlockSize[TreeDataManager::NUM_BUFFER_TYPES] =
{
    TreeDataManager::VERTEX_BLOCK_LENGTH * VERTEX_BUFFER_ELEMENT_SIZE,
    TreeDataManager::NORMAL_BLOCK_LENGTH * NORMAL_BUFFER_ELEMENT_SIZE,
    TreeDataManager::COLOR_BLOCK_LENGTH * COLOR_BUFFER_ELEMENT_SIZE,
    TreeDataManager::INDEX_BLOCK_LENGTH * INDEX_BUFFER_ELEMENT_SIZE
};

} // namespace detail


// "static const" member declarations to avoid linking errors
const size_t TreeDataManager::DefaultMaxMemoryHint;

TreeDataManager::TreeDataManager( )
    : _impl( new detail::TreeDataManager())
{ }

TreeDataManager::~TreeDataManager( )
{ }

bool TreeDataManager::init( const std::string& filename, bool useColors,
                            size_t maxMemoryHint )
{
    _impl->init( filename, useColors, maxMemoryHint );
    return _impl->isValid();
}

void TreeDataManager::getVertexData( Index start, Index length,
                                     SegmentedBuffer& vertices,
                                     SegmentedBuffer& normals )
{
    vertices = _impl->get( VERTEX_BUFFER_TYPE, start, length );
    normals = _impl->get( NORMAL_BUFFER_TYPE, start, length );
}

void TreeDataManager::getVertexData( Index start, Index length,
                                     SegmentedBuffer& vertices,
                                     SegmentedBuffer& normals,
                                     SegmentedBuffer& colors )
{
    vertices = _impl->get( VERTEX_BUFFER_TYPE, start, length );
    normals = _impl->get( NORMAL_BUFFER_TYPE, start, length );
    if( _impl->useColors() )
        colors = _impl->get( COLOR_BUFFER_TYPE, start, length );
}

void TreeDataManager::getIndexData( Index start, Index length, SegmentedBuffer& indices )
{
    indices = _impl->get( INDEX_BUFFER_TYPE, start, length );
}

void TreeDataManager::discardVertexData( Index start, Index length )
{
    _impl->discard( VERTEX_BUFFER_TYPE, start, length );
    _impl->discard( NORMAL_BUFFER_TYPE, start, length );
    if( _impl->useColors() )
        _impl->discard( COLOR_BUFFER_TYPE, start, length );
}

void TreeDataManager::discardIndexData( Index start, Index length )
{
    _impl->discard( INDEX_BUFFER_TYPE, start, length );
}

} // namespace triply
