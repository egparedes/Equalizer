
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

#include "mmap.h"

//#include "typedefs.h"

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifndef WIN32
#   include <sys/mman.h>
#   include <unistd.h>
#endif

namespace triply
{
#ifdef WIN32
    const char* MMap::MMAP_BAD_ADDRESS = 0;
#else
    const char* MMap::MMAP_BAD_ADDRESS = static_cast< char* >( MAP_FAILED );
#endif

MMap::MMap()
    : _address( const_cast< char* >( MMap::MMAP_BAD_ADDRESS )),
      _filename( "" ),
#ifdef WIN32
    _handle( INVALID_HANDLE_VALUE )
#else
    _fd( -1 ),
    _mmapSize( 0 )
#endif
{ }

MMap::~MMap()
{
    if( isValid() )
        close();
}

bool MMap::open( const std::string &filename )
{
    if( isValid() )
        return false;

    _filename = filename;

#ifdef WIN32
    // replace dir delimiters since '\' is often used as escape char
    for( size_t i=0; i<filename.length(); ++i )
        if( filename[i] == '\\' )
            filename[i] = '/';

    // try to open binary file
    HANDLE file = CreateFile( filename.c_str(), GENERIC_READ | GENERIC_WRITE,
                              FILE_SHARE_READ,
                              0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 );
    if( file == INVALID_HANDLE_VALUE )
        return false;

    TRIPLYINFO << "Reading cached binary representation." << std::endl;

    // create a file mapping
    _handle = CreateFileMapping( file, 0, PAGE_READWRITE, 0, 0,
                                     filename.c_str( ));
    CloseHandle( file );
    if( !_handle )
    {
        TRIPLYERROR << "Unable to read binary file, file mapping failed."
                  << std::endl;
        return false;
    }

    // get a view of the mapping
    _address   = static_cast< char* >( MapViewOfFile( _handle, FILE_MAP_READ, 0,
                                                          0, 0 ) );
    if( _address == MMAP_BAD_ADDRESS )
    {
        CloseHandle( _handle );
    }
#else
    // try to open binary file
    _fd = ::open( filename.c_str(), O_RDWR );
    if( _fd < 0 )
        return false;

    // retrieving file information
    struct stat status;
    fstat( _fd, &status );
    _mmapSize = status.st_size;

    // create memory mapped file
    _address   = static_cast< char* >( mmap( 0, status.st_size,
                                             PROT_READ | PROT_WRITE,
                                             MAP_SHARED, _fd, 0 ) );
    if( _address == MMAP_BAD_ADDRESS )
	{
        ::close( _fd );
	}
#endif

    return isValid();
}

void MMap::close( )
{
    if( _address != MMAP_BAD_ADDRESS )
    {
#ifdef WIN32
        UnmapViewOfFile( _address );
        CloseHandle( _handle );
#else
        munmap( _address, _mmapSize );
        if( _fd >= 0 )
            ::close( _fd );
#endif
        _address = const_cast< char* >( MMAP_BAD_ADDRESS );
    }
}

} //namespace triply


