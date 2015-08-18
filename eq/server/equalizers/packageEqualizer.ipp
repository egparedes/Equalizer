
/* Copyright (c) 2013-2015, David Steiner <steiner@ifi.uzh.ch>
 *               2011-2015, Stefan Eilemann <eile@equalizergraphics.com>
 *               2011, Carsten Rohn <carsten.rohn@rtt.ag>
 *               2011, Daniel Nachbaur <danielnachbaur@gmail.com>
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

#include "../compound.h"
#include "../compoundVisitor.h"
#include "../config.h"
#include "../server.h"
#include "../packageQueue.h"
#include "../view.h"

namespace eq
{
namespace server
{
namespace
{

template < class T, class S >
PackageQueue<T, S>* _findQueue( const std::string& name, const std::vector< PackageQueue<T, S>* >& queues )
{
    for( typename std::vector< PackageQueue<T, S>* >::const_iterator i = queues.begin();
         i != queues.end(); ++i )
    {
        if ((*i)->getName() == name)
            return *i;
    }
    return 0;
}

template < class T, class S >
class InputQueueCreator : public CompoundVisitor
{
public:
    InputQueueCreator( const S& size,
                       const std::string& name )
        : CompoundVisitor()
        , _tileSize( size )
        , _name( name )
    {}

    /** Visit a leaf compound. */
    virtual VisitorResult visitLeaf( Compound* compound )
    {
        const std::vector< PackageQueue<T, S>* >* inputPackageQueues;
        compound->getInputPackageQueues( &inputPackageQueues );
        if( _findQueue( _name, *inputPackageQueues ))
            return TRAVERSE_CONTINUE;

        // reset compound viewport to (0, 0, 1, 1) (#108)
        if( !compound->getViewport().hasArea() )
            compound->setViewport( Viewport( ));

        PackageQueue<T, S>* input = new PackageQueue<T, S>;
        ServerPtr server = compound->getServer();
        server->registerObject( input );
        input->setPackageSize( _tileSize );
        input->setName( _name );
        input->setAutoObsolete( compound->getConfig()->getLatency( ));

        compound->addInputPackageQueue( input );
        return TRAVERSE_CONTINUE;
    }

private:
    const S& _tileSize;
    const std::string& _name;
};

template < class T, class S >
class InputQueueDestroyer : public CompoundVisitor
{
public:
    explicit InputQueueDestroyer( const std::string& name )
        : CompoundVisitor()
        , _name( name )
    {
    }

    /** Visit a leaf compound. */
    virtual VisitorResult visitLeaf( Compound* compound )
    {
        const std::vector< PackageQueue<T, S>* >* inputPackageQueues;
        compound->getInputPackageQueues( &inputPackageQueues );
        PackageQueue<T, S>* q = _findQueue( _name, *inputPackageQueues );
        if( q )
        {
            compound->removeInputPackageQueue( q );
            ServerPtr server = compound->getServer();
            q->flush();
            server->deregisterObject( q );
            delete q;
        }

        return TRAVERSE_CONTINUE;
    }
private:
    const std::string& _name;
};

}

template < class T, class S >
PackageEqualizer<T, S>::PackageEqualizer()
    : Equalizer()
    , _created( false )
    , _name( "PackageEqualizer" )
{
}

template < class T, class S >
PackageEqualizer<T, S>::PackageEqualizer( const PackageEqualizer<T, S>& from )
    : Equalizer( from )
    , _created( from._created )
    , _name( from._name )
{
}

template < class T, class S >
std::string PackageEqualizer<T, S>::_getQueueName() const
{
    std::ostringstream name;
    name << "queue." << _name << (void*)this;
    return name.str();
}

template < class T, class S >
void PackageEqualizer<T, S>::_createQueues( Compound* compound )
{
    _created = true;
    const std::string& name = _getQueueName();
    const std::vector< PackageQueue<T, S>* >* outputPackageQueues;
    compound->getOutputPackageQueues( &outputPackageQueues );
    const S *packageSize;
    getPackageSize( &packageSize );
    if( !_findQueue( name, *outputPackageQueues ))
    {
        PackageQueue<T, S>* output = new PackageQueue<T, S>;
        ServerPtr server = compound->getServer();
        server->registerObject( output );
        output->setPackageSize( *packageSize );
        output->setDistributionStrategy( getDistributionStrategy( ));
        output->setPerfLogger(compound->getServer().get( ));
        output->setName( name );
        output->setAutoObsolete( compound->getConfig()->getLatency( ));

        compound->addOutputPackageQueue( output );
    }

    InputQueueCreator<T, S> creator( *packageSize, name );
    compound->accept( creator );
}

template < class T, class S >
void PackageEqualizer<T, S>::_destroyQueues( Compound* compound )
{
    const std::string& name = _getQueueName();
    const std::vector< PackageQueue<T, S>* >* outputPackageQueues;
    compound->getOutputPackageQueues( &outputPackageQueues );
    PackageQueue<T, S>* q = _findQueue( name, *outputPackageQueues );
    if ( q )
    {
        compound->removeOutputPackageQueue( q );
        ServerPtr server = compound->getServer();
        q->flush();
        server->deregisterObject( q );
        delete q;
    }

    InputQueueDestroyer<T, S> destroyer( name );
    compound->accept( destroyer );
    _created = false;
}

template < class T, class S >
void PackageEqualizer<T, S>::notifyUpdatePre( Compound* compound,
                                        const uint32_t /*frame*/ )
{
    if( isActive() && !_created )
        _createQueues( compound );

    if( !isActive() && _created )
        _destroyQueues( compound );
}

} //server
} //eq
