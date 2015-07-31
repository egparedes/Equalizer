
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


#ifndef PLYLIB_PAGEDTREEDATA_H
#define PLYLIB_PAGEDTREEDATA_H

#include "typedefs.h"
#include "mmap.h"

#include <lunchbox/lock.h>
#include <lunchbox/referenced.h>
#include <lunchbox/refPtr.h>

#include <fstream>
#include <list>
#include <map>
#include <utility>
#include <vector>

namespace triply {

template <typename T >
class PagedBuffer
{
public:
    typedef T DataType;
    typedef std::size_t iterator;

    inline PagedBuffer() : _offset(0), _pageSize(0), _size(0), _virtualTreeData(0),
                             _pType(INVALID_PAGE_TYPE), _valid(false) { }

    inline PagedBuffer(std::size_t offset, std::size_t pageSize,
                         std::size_t totalSize, PagedTreeData* bufferData,
                         PageType pType)
        : _offset(offset), _pageSize(pageSize), _size(totalSize),
          _virtualTreeData(bufferData), _pType(pType), _valid(false) { }

    inline bool isValid() const { return _valid; }
    inline std::size_t size() const { return _size; }

    inline iterator begin() const { return 0; }
    inline iterator end() const { return _size; }
    inline iterator rbegin() const { return _size - 1; }
    inline iterator rend() const { return -1; }

    inline std::size_t numBlocks() const { return _keyPairs.size(); }

    inline void getMemBlock(const std::size_t i, T*& address,
                            std::size_t& blockSize) const
    {
        if( i < numBlocks())
        {
            address = _keyPairs[i].second;
            blockSize = _pageSize;

            if( i == numBlocks()-1 )
            {
                blockSize = (_size + _offset) % _pageSize;
            }
            if( i == 0 )
            {
                address += _offset;
                blockSize -= _offset;
            }
        }
    }

    inline T* ptr(std::size_t i) const
    {
        PLYLIBASSERT( i < _size && _valid );
        i += _offset;
        return _keyPairs[i /_pageSize].second + (i % _pageSize);
    }

    inline T& operator[](std::size_t i) const
    {
        return *(ptr(i));
    }

    void discard(); // Defined after VirtualModelTreeData declaration

    friend class PagedTreeData;

private:
    typedef std::pair<PageKey, T*> KeyPair;

    std::size_t _offset;
    std::size_t _pageSize;
    std::size_t _size;
    std::vector< KeyPair > _keyPairs;
    PagedTreeData* _virtualTreeData;
    PageType _pType;
    bool _valid;
};


/** Holds the final tree data, sorted and reindexed.  */
class PagedTreeData : public lunchbox::Referenced
{
public:
    static const std::size_t DefaultPageSize = 131072; // 65536
    static const std::size_t DefaultMaxPages = 65536;

    template <typename T>
    struct PageData
    {
        PageData( std::size_t size=0 ) : data(size) {}
        inline T* ptr() { return &(data[0]); }
        std::vector< T > data;
    };

    PagedTreeData();

    ~PagedTreeData();

    void init( const std::string& fName,
               std::size_t pageSize=DefaultPageSize,
               std::size_t maxPages=DefaultMaxPages );
    void clear();

    void discard( PageKey key, PageType pType );
    bool verify( PageKey key, PageType pType );

    std::size_t getPageSize() const { return _pageSize; }
    std::size_t getMaxPages() const { return _maxPages; }

    void getVertices( std::size_t start, std::size_t count,
                      PagedBuffer< Vertex >& verticesVB );

    void getColors( std::size_t start, std::size_t count,
                    PagedBuffer< Color >& colorsVB );

    void getNormals( std::size_t start, std::size_t count,
                     PagedBuffer< Normal >& normalsVB );

    void getVertexData( std::size_t start, std::size_t count, bool useColors,
                        PagedBuffer< Vertex >& verticesVB,
                        PagedBuffer< Color >& colorsVB,
                        PagedBuffer< Normal >& normalsVB );

    void getIndices( std::size_t start, std::size_t count,
                     PagedBuffer< ShortIndex >& indicesVB );


private:
    bool mapHasKey(PageKey key, PageType pType);
    void mapEraseKey(PageKey key, PageType pType);
    std::size_t mapSize(PageType pType);

    char* getPageAddress(PageKey key, PageType pType);
    std::size_t getPageByteSize(PageKey key, PageType pType);

    template < typename T >
    PageData< T >& getPage(PageKey key, PageType pType);

    void freePage(PageKey key, PageType pType);

    //  Helper function to get real elements from the data pages
    template < typename T >
    void getElems(std::size_t start, std::size_t count,
                  PageType pType, PagedBuffer< T >& vBuffer);

    bool openBinary();
    void closeBinary();

    //  Helper function to read data from the MMF address
    template< typename T >
    void readData( char* addr, std::vector< T >& v, size_t length );

    // Model sizes
    BoundingBox _boundingBox;
    std::size_t _totalElems[TOTAL_PAGE_TYPES];

    // Data sizes
    std::size_t _pageSize;
    std::size_t _maxPages;
    static const std::size_t _sizeOfType[TOTAL_PAGE_TYPES];

    // Paging data
    std::map< PageKey, PageData< Vertex > > _vPages;
    std::map< PageKey, PageData< Color > > _cPages;
    std::map< PageKey, PageData< Normal > > _nPages;
    std::map< PageKey, PageData< ShortIndex > > _idxPages;
    std::map< PageKey, std::size_t > _activePages[TOTAL_PAGE_TYPES];
    std::list< PageKey > _disposablePages[TOTAL_PAGE_TYPES];

    // Memory map
    char* _dataAddr[TOTAL_PAGE_TYPES];
    char* _mmapAddr;
    std::string _fName;

    // Multithreading and locking
    lunchbox::Lock _lock[TOTAL_PAGE_TYPES];
};

template < typename T >
inline void PagedTreeData::getElems( std::size_t start, std::size_t count,
                                     PageType pType, PagedBuffer< T >& vBuffer )
{
    if( _mmapAddr == MMAP_BAD_ADDRESS )
        openBinary();
    PLYLIBASSERT( start + count <= _totalElems[pType] );
//    PLYLIBERROR << "start = " << start << std::endl;
//    PLYLIBERROR << "count = " << count << std::endl;
//    PLYLIBERROR << "_totalElems[pType] = " << _totalElems[pType] << std::endl;
//    PLYLIBERROR << "pType = " << int(pType) << std::endl;

    PageKey key = start / _pageSize;
    std::size_t offset = start % _pageSize;
    std::size_t pages = 1 + ((count + offset) / _pageSize);
    vBuffer.discard();

    vBuffer._offset = offset;
    vBuffer._pageSize = _pageSize;
    vBuffer._size = count;
    vBuffer._pType = pType;
    vBuffer._keyPairs.resize(pages);
    for( std::size_t i=0; i < pages; ++i, ++key )
    {
        vBuffer._keyPairs[i].first = key;
        vBuffer._keyPairs[i].second = getPage< T >(key, pType).ptr();
    }
    vBuffer._virtualTreeData = this;
    vBuffer._valid = true;
}

template< typename T >
inline void PagedTreeData::readData( char* addr, std::vector< T >& v, size_t length )
{
    if( length > 0 )
    {
        v.resize( length );
        memcpy( reinterpret_cast< char* >( &v[0] ), addr, length * sizeof( T ) );
    }
}

typedef PagedTreeData* PagedTreeDataPtr;
typedef lunchbox::RefPtr< PagedTreeData > SharedPagedTreeDataPtr;


// Remainingt VirtualBuffer functions
template< class T >
inline void PagedBuffer<T>::discard( )
{
    if( _valid )
    {
        PLYLIBASSERT( _virtualTreeData );
        _valid = false;
        for( std::size_t i=0; i < _keyPairs.size(); ++i )
        {
            _virtualTreeData->discard(_keyPairs[i].first, _pType);
        }
    }
}

typedef PagedBuffer< Vertex >         PagedVertexBuffer;
typedef PagedBuffer< Color >          PagedColorBuffer;
typedef PagedBuffer< Normal >         PagedNormalBuffer;
typedef PagedBuffer< ShortIndex >     PagedShortIndexBuffer;

typedef PagedVertexBuffer*        PagedVertexBufferPtr;
typedef PagedColorBuffer*         PagedColorBufferPtr;
typedef PagedNormalBuffer*        PagedNormalBufferPtr;
typedef PagedShortIndexBuffer*    PagedShortIndexBufferPtr;

} // namespace triply

#endif // PLYLIB_PAGEDTREEDATA_H
