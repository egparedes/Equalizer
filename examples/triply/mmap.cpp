
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

#include "typedefs.h"

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

namespace triply
{

namespace detail
{
	struct MMapInfo
	{
        bool mapped;
#ifdef WIN32
		HANDLE	handle;
#else
		int fd;
		size_t mmapSize;
#endif
	};

	static std::map< char*, detail::MMapInfo > MMInfos;

} // namespace detail


bool openMMap( std::string filename, char** mmapAddrPtr )
{
	bool  result = false;
    detail::MMapInfo info;

#ifdef WIN32
    // replace dir delimiters since '\' is often used as escape char
    for( size_t i=0; i<filename.length(); ++i )
        if( filename[i] == '\\' )
            filename[i] = '/';

    // try to open binary file
    HANDLE file = CreateFile( filename.c_str(), GENERIC_READ, FILE_SHARE_READ,
                              0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 );
    if( file == INVALID_HANDLE_VALUE )
        return false;

    TRIPLYINFO << "Reading cached binary representation." << std::endl;

    // create a file mapping
    info.handle = CreateFileMapping( file, 0, PAGE_READONLY, 0, 0,
                                     filename.c_str( ));
    CloseHandle( file );
    if( !info.handle )
    {
        TRIPLYERROR << "Unable to read binary file, file mapping failed."
                  << std::endl;
        return false;
    }

    // get a view of the mapping
    *mmapAddrPtr   = static_cast< char* >( MapViewOfFile( info.handle, FILE_MAP_READ, 0,
                                                          0, 0 ) );
    if( *mmapAddrPtr != MMAP_BAD_ADDRESS )
    {
    	info.mapped = true;
    	detail::MMInfos[*mmapAddrPtr] = info;
    	result = true;
    }
    else
    {
    	CloseHandle( info.handle );
    }
#else
    // try to open binary file
    info.fd = open( filename.c_str(), O_RDONLY );
    if( info.fd < 0 )
        return false;

    // retrieving file information
    struct stat status;
    fstat( info.fd, &status );
    info.mmapSize = status.st_size;

    // create memory mapped file
    *mmapAddrPtr   = static_cast< char* >( mmap( 0, status.st_size, PROT_READ,
                                                 MAP_SHARED, info.fd, 0 ) );
    if( *mmapAddrPtr != MMAP_BAD_ADDRESS )
    {
    	info.mapped = true;
    	detail::MMInfos[*mmapAddrPtr] = info;
		result = true;
    }
	else
	{
		close( info.fd );
	}
#endif

    return result;
}

void closeMMap( char** mmapAddrPtr )
{
	if( mmapAddrPtr == 0 )
		return;	

	char* mmapAddr = *mmapAddrPtr;
	if( detail::MMInfos.count( mmapAddr ) > 0 )
		return;	

    if( mmapAddr != MMAP_BAD_ADDRESS )
    {
        detail::MMapInfo info = detail::MMInfos[mmapAddr];
#ifdef WIN32
        if( info.mmapped )
        	UnmapViewOfFile( mmapAddr );
    	CloseHandle( info.handle );
#else
        if( info.mapped )
            munmap( mmapAddr, info.mmapSize );
        if( info.fd >= 0 )
            close( info.fd );
#endif
        *mmapAddrPtr = const_cast< char* >( MMAP_BAD_ADDRESS );
    }

    detail::MMInfos.erase( mmapAddr );
}

} //namespace triply
