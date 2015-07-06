
/* Copyright (c) 2013, David Steiner <steiner@ifi.uzh.ch>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef EQFABRIC_STEALABLE_H
#define EQFABRIC_STEALABLE_H

namespace eq
{
namespace fabric
{
/** @internal */
#pragma pack(push, 1)
class Stealable
{
public:
    Stealable()
        : stolen( false ) {}

    bool stolen;
};
#pragma pack(pop)

// std::ostream& operator << ( std::ostream& os, const Stealable& stealable )
// {
//     os << "[ " << stealable.stolen << " ]";
//     return os;
// }
}
}

namespace lunchbox
{
template<> inline void byteswap( eq::fabric::Stealable& stealable )
{
    byteswap( stealable.stolen );
}
}

#endif // EQFABRIC_STEALABLE_H



