
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


#include "virtualVertexBufferData.h"

#include <algorithm>
#include <fstream>
#include <vector>

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifndef _WIN32
#   include <sys/mman.h>
#endif

namespace triply {    

const std::size_t VirtualVertexBufferData::_sizeOfType[TOTAL_PAGE_TYPES] =
    { sizeof(Vertex), sizeof(Color), sizeof(Normal), sizeof(ShortIndex) };

VirtualVertexBufferData::VirtualVertexBufferData()
    : _pageSize(0), _maxPages(0),
      _mmapAddr(static_cast<char*>(MAP_FAILED)),
      _mmapSize(0),
      _fd(-1), _mmaped(false), _fName("")
{
    _boundingBox[0] = Vertex( 0.0 );
    _boundingBox[1] = Vertex( 0.0 );
    for (int t=POSITION_PAGE_TYPE; t < TOTAL_PAGE_TYPES; ++t)
    {
        _lock[t].set();
        _totalElems[t] = 0;
        _dataAddr[t] = NULL;
        _lock[t].unset();
    }
}

VirtualVertexBufferData::~VirtualVertexBufferData()
{
    clear();
}

void VirtualVertexBufferData::init(const std::string& fName,
                                   std::size_t pageSize, std::size_t maxPages)
{
    _fName = fName;
    _pageSize = pageSize;
    _maxPages = maxPages;
}

void VirtualVertexBufferData::clear()
{
    for (int t=POSITION_PAGE_TYPE; t < TOTAL_PAGE_TYPES; ++t)
    {
        _lock[t].set();
        _lock[t].set();
        _lock[t].set();
        _lock[t].set();
    }

    closeMMap();
    _vPages.clear();
    _cPages.clear();
    _nPages.clear();
    _idxPages.clear();
    for (int t=POSITION_PAGE_TYPE; t < TOTAL_PAGE_TYPES; ++t)
    {
        _activePages[t].clear();
        _disposablePages[t].clear();
        _totalElems[t] = 0;
        _lock[t].unset();
    }
}

void VirtualVertexBufferData::discard(PageKey key, PageType pType)
{
    PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );

    _lock[pType].set();
     PLYLIBASSERT( _activePages[pType][key] > 0 );
    _activePages[pType][key]--;
    if (_activePages[pType][key] <= 0)
        _disposablePages[pType].push_back(key);
    _lock[pType].unset();
}

bool VirtualVertexBufferData::verify(PageKey key, PageType pType)
{
    PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );

    _lock[pType].set();
    bool result = ( _activePages[pType][key] > 0 );
    _lock[pType].unset();
    return result;
}

bool VirtualVertexBufferData::getVertices(std::size_t start, std::size_t count,
                                          VirtualBuffer< Vertex >& verticesVB)
{
    return getElems< Vertex >(start, count, POSITION_PAGE_TYPE, verticesVB);
}

bool VirtualVertexBufferData::getColors(std::size_t start, std::size_t count,
                                        VirtualBuffer< Color >& colorsVB)
{
    return getElems< Color >(start, count, COLOR_PAGE_TYPE, colorsVB);
}

bool VirtualVertexBufferData::getNormals(std::size_t start, std::size_t count,
                                         VirtualBuffer< Normal >& normalsVB)
{
    return getElems< Normal >(start, count, NORMAL_PAGE_TYPE, normalsVB);
}

bool VirtualVertexBufferData::getVertexData(std::size_t start, std::size_t count,
                                            bool useColors,
                                            VirtualBuffer< Vertex >& verticesVB,
                                            VirtualBuffer< Color >& colorsVB,
                                            VirtualBuffer< Normal >& normalsVB)
{
    if (!getVertices(start, count, verticesVB) || !getNormals(start, count, normalsVB))
        return false;
    else if (useColors)
        return  getColors(start, count, colorsVB);
    else
        return true;
}

bool VirtualVertexBufferData::getIndices(std::size_t start, std::size_t count,
                                         VirtualBuffer< ShortIndex >& indicesVB)
{
    return getElems< ShortIndex >(start, count, SHORTINDEX_PAGE_TYPE, indicesVB);
}


inline bool VirtualVertexBufferData::mapHasKey(PageKey key, PageType pType)
{
    switch (pType)
    {
    case POSITION_PAGE_TYPE:
        return (_vPages.count(key) > 0);
        break;
    case COLOR_PAGE_TYPE:
        return (_cPages.count(key) > 0);
        break;
    case NORMAL_PAGE_TYPE:
        return (_nPages.count(key) > 0);
        break;
    case SHORTINDEX_PAGE_TYPE:
        return (_idxPages.count(key) > 0);
        break;
    default:
        PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );
        break;
    }
    return false;
}

inline void VirtualVertexBufferData::mapEraseKey(PageKey key, PageType pType)
{
    switch (pType)
    {
    case POSITION_PAGE_TYPE:
        _vPages.erase(key);
        break;
    case COLOR_PAGE_TYPE:
        _cPages.erase(key);
        break;
    case NORMAL_PAGE_TYPE:
        _nPages.erase(key);
        break;
    case SHORTINDEX_PAGE_TYPE:
        _idxPages.erase(key);
        break;
    default:
        PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );
        break;
    }
}

inline std::size_t VirtualVertexBufferData::mapSize(PageType pType)
{
    switch (pType)
    {
    case POSITION_PAGE_TYPE:
        return _vPages.size();
        break;
    case COLOR_PAGE_TYPE:
        return _cPages.size();
        break;
    case NORMAL_PAGE_TYPE:
        return _nPages.size();
        break;
    case SHORTINDEX_PAGE_TYPE:
        return _idxPages.size();
        break;
    default:
        PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );
        break;
    }
    return 0;
}

inline char* VirtualVertexBufferData::getPageAddress(PageKey key, PageType pType)
{
    PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );
    PLYLIBASSERT( key * _pageSize < _totalElems[pType] );
    return _dataAddr[pType] + key * _pageSize * _sizeOfType[pType];
}

inline std::size_t VirtualVertexBufferData::getPageByteSize(PageKey key, PageType pType)
{
    PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );
    PLYLIBASSERT( key * _pageSize < _totalElems[pType] );
    return std::min(_pageSize, _totalElems[pType] - key * _pageSize);
}

template < typename T >
VirtualVertexBufferData::PageData< T >&
VirtualVertexBufferData::getPage(PageKey key, PageType pType)
{
    PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );

    _lock[pType].set();
    if (!_mmaped)
        openMMap();
    bool loadFromDisk = !mapHasKey(key, pType);

    PageData< T >* pageData = NULL;
    switch (pType)
    {
    case POSITION_PAGE_TYPE:
        pageData = reinterpret_cast< PageData< T >* >( &(_vPages[key]) );
        break;
    case COLOR_PAGE_TYPE:
        pageData = reinterpret_cast< PageData< T >* >( &(_cPages[key]) );
        break;
    case NORMAL_PAGE_TYPE:
        pageData = reinterpret_cast< PageData< T >* >( &(_nPages[key]) );
        break;
    case SHORTINDEX_PAGE_TYPE:
        pageData = reinterpret_cast< PageData< T >* >( &(_idxPages[key]) );
        break;
    default:
        PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );
        break;
    }
    _disposablePages[pType].remove(key);
    _activePages[pType][key]++;

    // Load page from mmap when needed
    if (loadFromDisk)
    {
        // Remove some disposable pages from the map when needed
        while (mapSize(pType) >= _maxPages)
        {
            std::list< PageKey >::iterator it = _disposablePages[pType].begin();
            while (_activePages[pType][*it] > 0 && it != _disposablePages[pType].end())
                ++it;
            PLYLIBASSERT( it != _disposablePages[pType].end() );
            if (it != _disposablePages[pType].end())
                freePage(*it, pType);
        }

        PLYLIBASSERT( _mmaped );
        readData( getPageAddress(key, pType), pageData->data, getPageByteSize(key, pType) );
    }
    _lock[pType].unset();

    return *pageData;
}

void VirtualVertexBufferData::freePage(PageKey key, PageType pType)
{
    PLYLIBASSERT( pType >= POSITION_PAGE_TYPE && pType < TOTAL_PAGE_TYPES );
    if (!mapHasKey(key, pType))
        return;

    PLYLIBASSERT( _activePages[pType][key] <= 0 );
    _disposablePages[pType].remove(key);
    mapEraseKey(key, pType);
}

bool VirtualVertexBufferData::openMMap()
{
    bool result = false;

#ifdef WIN32
#else
    // try to open binary file
    if( _fd < 0 )
    {
        _fd = open( _fName.c_str(), O_RDONLY );
        if( _fd < 0 )
            return false;
    }

    // retrieving file information
    struct stat status;
    fstat( _fd, &status );
    _mmapSize = status.st_size;

    // create memory mapped file
    _mmapAddr  = static_cast< char* >( mmap( 0, _mmapSize, PROT_READ,
                                       MAP_SHARED, _fd, 0 ) );

    if( _mmapAddr != MAP_FAILED )
    {
        try
        {
            char* dataAddr = _mmapAddr;
            size_t version;
            memRead( reinterpret_cast< char* >( &version ), &dataAddr, sizeof( size_t ) );
            if( version != FILE_VERSION )
                throw MeshException( "Error reading binary file. Version in file "
                                     "does not match the expected version." );
            size_t nodeType;
            memRead( reinterpret_cast< char* >( &nodeType ), &dataAddr, sizeof( size_t ) );
            if( nodeType != ROOT_TYPE )
                throw MeshException( "Error reading binary file. Expected the root "
                                     "node, but found something else instead." );

            memRead( reinterpret_cast< char* >( &_boundingBox[0] ),
                     &dataAddr, 2*sizeof( Vertex ) );
            memRead( reinterpret_cast< char* >( &_totalElems[POSITION_PAGE_TYPE] ),
                     &dataAddr, sizeof( size_t ) );
            _dataAddr[POSITION_PAGE_TYPE] = dataAddr;

            dataAddr += _totalElems[POSITION_PAGE_TYPE] * sizeof(Vertex);
            memRead( reinterpret_cast< char* >( &_totalElems[COLOR_PAGE_TYPE] ),
                     &dataAddr, sizeof( size_t ) );
            _dataAddr[COLOR_PAGE_TYPE] = dataAddr;

            dataAddr += _totalElems[COLOR_PAGE_TYPE] * sizeof(Color);
            memRead( reinterpret_cast< char* >( &_totalElems[NORMAL_PAGE_TYPE] ),
                     &dataAddr, sizeof( size_t ) );
            _dataAddr[NORMAL_PAGE_TYPE] = dataAddr;

            dataAddr += _totalElems[NORMAL_PAGE_TYPE] * sizeof(Normal);
            memRead( reinterpret_cast< char* >( &_totalElems[SHORTINDEX_PAGE_TYPE] ),
                     &dataAddr, sizeof( size_t ) );
            _dataAddr[SHORTINDEX_PAGE_TYPE] = dataAddr;

            result = true;
        }
        catch( const std::exception& e )
        {
            PLYLIBERROR << "Unable to read binary file, an exception occured:  "
                        << e.what() << std::endl;
        }
    }
    else
    {
        PLYLIBERROR << "Unable to read binary file, memory mapping failed."
                  << std::endl;
    }
#endif

    _mmaped = result;
    return result;
}

bool VirtualVertexBufferData::closeMMap()
{
    bool result = false;

#ifdef WIN32
#else
    if( _mmapAddr != MAP_FAILED )
    {
        if (_mmaped)
            munmap( _mmapAddr, _mmapSize );
        _mmapAddr = static_cast<char*>(MAP_FAILED);
        result = true;
    }
    if( _fd >= 0 )
    {
        close( _fd );
        _fd = -1;
        result = true;
    }
#endif

    _mmaped = false;
    _dataAddr[POSITION_PAGE_TYPE] = reinterpret_cast<char*>(-1);
    _dataAddr[COLOR_PAGE_TYPE] = reinterpret_cast<char*>(-1);
    _dataAddr[NORMAL_PAGE_TYPE] = reinterpret_cast<char*>(-1);
    _dataAddr[SHORTINDEX_PAGE_TYPE] = reinterpret_cast<char*>(-1);

    return result;
}

} // namespace triply
