
/* Copyright (c) 2013-2015, David Steiner <steiner@ifi.uzh.ch>
 *               2011-2015, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2011, Carsten Rohn <carsten.rohn@rtt.ag>
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

#ifndef EQS_PACKAGEEQUALIZER_H
#define EQS_PACKAGEEQUALIZER_H

#include "equalizer.h"

namespace eq
{
namespace server
{

template < class T, class S > class PackageEqualizer : public Equalizer
{
public:
    EQSERVER_API PackageEqualizer();
    PackageEqualizer( const PackageEqualizer& from );
    ~PackageEqualizer() {}

    /** @sa CompoundListener::notifyUpdatePre */
    virtual void notifyUpdatePre( Compound* compound,
                                  const uint32_t frameNumber );

    virtual void toStream( std::ostream& os ) const { os << this; }
    void setName( const std::string& name ) { _name = name; }

    const std::string& getName() const { return _name; }

protected:
    void notifyChildAdded( Compound*, Compound* ) override {}
    void notifyChildRemove( Compound*, Compound* ) override {}

private:
    std::string _getQueueName() const;
    void _destroyQueues( Compound* compound );
    void _createQueues( Compound* compound );

    bool _created;
    std::string _name;
};

} //server
} //eq

#include "packageEqualizer.ipp" // template implementation

#endif // EQS_PACKAGEEQUALIZER_H
