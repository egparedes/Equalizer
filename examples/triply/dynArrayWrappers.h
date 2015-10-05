
/* Copyright (c)      2015, Enrique G. Paredes <egparedes@ifi.uzh.ch>
 *                    2007, Tobias Wolf <twolf@access.unizh.ch>
 *               2009-2014, Stefan Eilemann <eile@equalizergraphics.com>
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


#ifndef TRIPLY_DYNARRAYWRAPPERS_H
#define TRIPLY_DYNARRAYWRAPPERS_H

#include "mmap.h"
#include <boost/filesystem.hpp>
#include <algorithm>

namespace triply
{

namespace bfs = boost::filesystem;

// Wrapper to use a dynamically allocated array as a generic container object
template < typename T >
class DynArrayWrapper
{
public:
    typedef T DataType;

    DynArrayWrapper( )
        : _data( 0 ), _dataSize( 0 ), _owner( false )
    { }

    DynArrayWrapper( size_t arraySize, const T& defaultValue=T() )
        : _data( 0 ), _dataSize( 0 ), _owner( false )
    {
        resize( arraySize, defaultValue );
    }

    DynArrayWrapper( T* ptr, size_t arraySize )
        : _data( 0 ), _dataSize( 0 ), _owner( false )
    {
        init( ptr, arraySize );
    }

    ~DynArrayWrapper( )
    {
        clear();
    }

    inline void init( T* ptr, size_t arraySize )
    {
        TRIPLYASSERT( ptr != 0 || arraySize == 0 );

        clear();
        _data = ptr;
        _dataSize = arraySize;
        _owner = false;
    }

    inline void resize( size_t arraySize, const T& defaultValue=T() )
    {
        TRIPLYASSERT( _data == 0 || _owner );

        if( arraySize == _dataSize )
        {
            return;
        }
        else if( arraySize == 0 )
        {
            clear();
            return;
        }

        T* newData = new T[arraySize];
        std::copy( _data, _data + std::min( _dataSize, arraySize ), newData );
        delete[] _data;
        std::swap( _data, newData );
        for( unsigned i=_dataSize; i < arraySize; _data[i++] = defaultValue );

        _dataSize = arraySize;
        _owner = true;
    }

    inline void clear( bool forceFree=false )
    {
        if( ( _owner || forceFree ) && _data != 0 )
            delete[] _data;
        _data = 0;
        _dataSize = 0;
        _owner = false;
    }

    inline T& operator[]( const size_t i )
    {
        TRIPLYASSERT( i < _dataSize );
        return _data[i];
    }

    inline const T& operator[]( const size_t i ) const
    {
        TRIPLYASSERT( i < _dataSize );
        return _data[i];
    }

    inline T at( const size_t i )
    {
        return ( i < _dataSize ) ? _data[i] : T();
    }

    inline const T at( const size_t i ) const
    {
        return ( i < _dataSize ) ? _data[i] : T();
    }

    inline T* begin()
    {
        return _data;
    }

    inline const T* begin() const
    {
        return _data;
    }

    inline T* end()
    {
        return _data + _dataSize;
    }

    inline const T* end() const
    {
        return _data + _dataSize;
    }

    inline size_t size() const
    {
        return _dataSize;
    }

protected:
    T*          _data;
    size_t      _dataSize;
    bool        _owner;
};


// Wrapper to use a memory mapped (when needed) data array as a generic
// container object
template < typename T >
class LargeDynArrayWrapper : public DynArrayWrapper< T >
{
public:
    using typename DynArrayWrapper< T >::DataType;

    LargeDynArrayWrapper()
        : DynArrayWrapper< T >(), _outOfCore( false ), _mmapName( )
    { }

    LargeDynArrayWrapper( size_t arraySize, bool outOfCore=true, const T& defaultValue=T() )
        : DynArrayWrapper< T >( ), _outOfCore( outOfCore )
    {
        init( arraySize, outOfCore, defaultValue );
    }

    ~LargeDynArrayWrapper( )
    {
        clear();
    }

    inline void resize( size_t arraySize, bool outOfCore=true, const T& defaultValue=T() )
    {
        clear();
        _outOfCore = outOfCore;
        if( _outOfCore )
        {
            // create temporary file and mmap it
            _mmapName = bfs::unique_path("%%%%-%%%%-%%%%.tmpfile").native();
            std::ofstream output( _mmapName, std::ios::out | std::ios::binary );
            if( output )
            {
                const size_t TempMMapMagic = 0xABCD;
                output.write( reinterpret_cast< const char* >( &TempMMapMagic ),
                              sizeof( TempMMapMagic ) );
                output.close();
            }

            TRIPLYASSERT( bfs::exists( _mmapName ) );

            size_t fileSize = sizeof( size_t ) + arraySize * sizeof( T );
            bfs::resize_file( _mmapName, fileSize );

            TRIPLYASSERT( bfs::file_size( _mmapName ) == fileSize);

            _mmap.open( _mmapName );
            DynArrayWrapper< T >::_data = reinterpret_cast< T* >( _mmap.getPtr() );
        }
        else
        {
            DynArrayWrapper< T >::resize( arraySize, defaultValue );
        }
    }

    inline void clear()
    {
        if ( _outOfCore )
        {
            _mmap.close();
            if( bfs::exists( _mmapName ))
                bfs::remove( _mmapName );
            _mmapName = "";
            _outOfCore = false;
        }
        DynArrayWrapper< T >::clear();
    }

protected:
    bool                _outOfCore;
    std::string         _mmapName;
    MMap                _mmap;
};

// Wrapper to use a pool as a generic container object
template < typename T >
class PooledArray
{
public:
    typedef T DataType;

    PooledArray( )
        : _data( 0 ), _size( 0 )
    { }

    PooledArray( T* data, size_t aSize )
        : _data( data ), _size( aSize )
    { }

    inline void init( T* data, size_t aSize )
    {
        _data = data;
        _size = aSize;
    }

    inline void clear( const T& initValue=T() )
    {
        for( size_t i=0; i < _size; ++i )
        {
            _data[i] = initValue;
        }
    }

    inline T& operator[]( const size_t i )
    {
        TRIPLYASSERT( i < _size );
        return _data[i];
    }

    inline const T& operator[]( const size_t i ) const
    {
        TRIPLYASSERT( i < _size );
        return _data[i];
    }

    inline T at( const size_t i )
    {
        return ( i < _size ) ? _data[i] : T();
    }

    inline const T at( const size_t i ) const
    {
        return ( i < _size ) ? _data[i] : T();
    }

    inline T* begin()
    {
        return _data;
    }

    inline const T* begin() const
    {
        return _data;
    }

    inline T* end()
    {
        return _data + _size;
    }

    inline const T* end() const
    {
        return _data + _size;
    }

    inline size_t size() const
    {
        return _size;
    }

protected:
    T*          _data;
    size_t      _size;
};


} //namespace triply

#endif // TRIPLY_DYNARRAYWRAPPERS_H
