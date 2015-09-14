
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


#ifndef TRIPLY_SEGMENTEDBUFFER_H
#define TRIPLY_SEGMENTEDBUFFER_H

#include "typedefs.h"
#include "modelTreeData.h"
#include <lunchbox/refPtr.h>
#include <vector>
#include <utility>

namespace triply
{

class SegmentedBuffer
{
public:
    typedef size_t iterator;

    struct Segment
    {
        Segment( char* ptrArg=0, size_t sizeArg=0)
            : ptr( ptrArg ), size( sizeArg )
        { }

        char* ptr;
        size_t size;
    };

    inline SegmentedBuffer( size_t segmentSize=0, size_t start=0 )
        : _startOffset( start ), _endOffset( segmentSize ),
          _segmentSize( segmentSize ), _isClosed( false )
    { }

    inline bool isValid() const
    {
        return _segmentPtrs.size() > 0
                && _startOffset < _segmentSize
                && _startOffset < _endOffset
                && _endOffset <= _segmentSize;
    }

    inline bool isClosed() const
    {
        return _isClosed;
    }

    inline size_t size() const
    {
        return _segmentSize * ( _segmentPtrs.size() - 1 ) - _startOffset + _endOffset;
    }

    inline iterator begin() const { return 0; }
    inline iterator end() const { return size(); }
    inline iterator rbegin() const { return size() - 1; }
    inline iterator rend() const { return -1; }

    inline size_t numSegments() const
    {
        return _segmentPtrs.size();
    }

    inline size_t getSegmentSize() const
    {
        return _segmentSize;
    }

    inline Segment getSegment( const size_t idx ) const
    {
        TRIPLYASSERT( idx < _segmentPtrs.size() );

        Segment result( 0, 0 );
        result.ptr = _segmentPtrs[idx];
        result.size = _segmentSize;

        if( idx == 0 )
        {
            result.ptr += _startOffset;
            result.size -= _startOffset;
        }
        if( idx == _segmentPtrs.size() - 1 )
        {
            result.size -= ( _segmentSize - _endOffset );
        }

        return result;
    }

    inline char* ptr( size_t i ) const
    {
        TRIPLYASSERT( isValid() && i < size() );

        i += _startOffset;
        return _segmentPtrs[i /_segmentSize] + ( i % _segmentSize );
    }

    template < typename T >
    inline T& at( size_t i ) const
    {
        return *( reinterpret_cast< T* >( ptr( i * sizeof( T ) )) );
    }

    inline void reset( size_t segmentSize=0, size_t start=0 )
    {
        _segmentSize = segmentSize;
        _startOffset = start;
        _endOffset = _segmentSize;
        _segmentPtrs.clear();
        _isClosed = false;
    }

    // Set single segment buffer
    template < typename T >
    inline void set( T* segmentPtr, size_t segmentSize )
    {        
        _startOffset = 0;
        _segmentSize = segmentSize;
        _endOffset = segmentSize;
        _segmentPtrs.push_back( reinterpret_cast< char* >( segmentPtr ));
        _isClosed = true;
    }

    template < typename T >
    inline void addSegment( T* segmentPtr )
    {
        TRIPLYASSERT( !_isClosed );

        if( !_isClosed )
        {
            _segmentPtrs.push_back( reinterpret_cast< char* >( segmentPtr ));
        }
    }

    template < typename T >
    inline void addTail( T* segmentPtr, size_t tailSize )
    {
        TRIPLYASSERT( !_isClosed );
        TRIPLYASSERT( tailSize <= _segmentSize );

        if( !_isClosed )
        {
            _endOffset = tailSize + _startOffset * ( _segmentPtrs.size() == 0 );
            _segmentPtrs.push_back( reinterpret_cast< char* >( segmentPtr ));
            _isClosed = true;
        }
    }

    inline void popSegment()
    {
        TRIPLYASSERT( isValid() );

        _endOffset = _segmentSize;
        _segmentPtrs.pop_back();
        _isClosed = false;
    }

private:
    size_t                  _startOffset;
    size_t                  _endOffset;
    size_t                  _segmentSize;
    std::vector< char* >    _segmentPtrs;
    bool                    _isClosed;
};

} // namespace triply


#endif // TRIPLY_SEGMENTEDBUFFER_H
