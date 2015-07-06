
/* Copyright (c) 2011-2014, Stefan Eilemann <eile@eyescale.ch>
 *               2011-2012, Daniel Nachbaur <danielnachbaur@googlemail.com>
 *               2014-2015, David Steiner <steiner@ifi.uzh.ch>
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

#include <co/stealingQueueMaster.h>
#include <co/centLoadAwareDistributor.h>
#include <co/latencyAwareDistributor.h>
#include <co/loadAwareDistributor.h>
#include <co/randomDistributor.h>
#include <co/equalDistributor.h>
#include <co/dataIStream.h>
#include <co/dataOStream.h>
#include <co/queueItem.h>

#include <boost/crc.hpp>

namespace eq
{
namespace server
{

// static int calcCrc32(void const *buffer, std::size_t byteCount)   // TEST
// {
//     boost::crc_32_type result;
//     result.process_bytes(buffer, byteCount);
//     return result.checksum();
// }

template < class T, class S >
PackageQueue<T, S>::PackageQueue()
        : co::Object()
        , _compound( 0 )
        , _name()
        , _packageSize( 0 )
        , _perfLogger( NULL )
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        _queueMaster[i] = 0;
        _outputQueue[i] = 0;
    }
}

template < class T, class S >
PackageQueue<T, S>::PackageQueue( const PackageQueue<T, S>& from )
        : co::Object()
        , _compound( 0 )
        , _name( from._name )
        , _packageSize( from._packageSize )
        , _perfLogger( NULL )
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        _queueMaster[i] = 0;
        _outputQueue[i] = 0;
    }
}

template < class T, class S >
PackageQueue<T, S>::~PackageQueue()
{
    _compound = 0;
}

template < class T, class S >
void PackageQueue<T, S>::addPackage( T package, float position, const fabric::Eye eye )
{
    uint32_t index = lunchbox::getIndexOfLastBit(eye);
    LBASSERT( index < NUM_EYES );

    package.stolen = false;
//     package.checksum = calcCrc32(&package.frustum, sizeof(package) - sizeof(package.checksum) - sizeof(package.stolen));     // HACK

    _queueMaster[index]->_queue.push().setPositionHint( position ) << package;
}

template < class T, class S >
void PackageQueue<T, S>::addEnd( const fabric::Eye eye )
{
//     std::cerr << "Eq: Pushing queue end" << std::endl;

       uint32_t index = lunchbox::getIndexOfLastBit(eye);
       LBASSERT( index < NUM_EYES );

       co::QueueMaster& qm = _queueMaster[index]->_queue;
//     qm.push();

       qm.notifyEnd();
}

template < class T, class S >
void PackageQueue<T, S>::setSlaveNodes( const co::Nodes& nodes, const fabric::Eye eye )
{
    uint32_t index = lunchbox::getIndexOfLastBit(eye);
    LBASSERT( index < NUM_EYES );

    co::QueueMaster& qm = _queueMaster[index]->_queue;
    qm.setSlaveNodes( nodes );
}

template < class T, class S >
void PackageQueue<T, S>::selectPackageDistributor( const Eye eye )
{
    std::string distName("None");

    uint32_t index = lunchbox::getIndexOfLastBit(eye);
    LBASSERT( index < NUM_EYES );
    co::Producer &producer = _queueMaster[index]->_queue;
    co::PackageDistributorPtr distributor = producer.getPackageDistributor();
    if( !distributor )
    {
        switch (_distributionStrategy)
        {
        case fabric::DISTRIBUTION_STRATEGY_NONE:
//             distributor = new co::PackageDistributor(producer);
//             distName = "DummyDistributor";
            break;

        case fabric::DISTRIBUTION_STRATEGY_EQUAL:
            distributor = new co::EqualDistributor(producer);
            distName = "EqualDistributor";
            break;

        case fabric::DISTRIBUTION_STRATEGY_RANDOM:
            distributor = new co::RandomDistributor(producer);
            distName = "RandomDistributor";
            break;

        case fabric::DISTRIBUTION_STRATEGY_LOAD_AWARE:
            distributor = new co::LoadAwareDistributor(producer);
            distName = "LoadAwareDistributor";
            break;

        case fabric::DISTRIBUTION_STRATEGY_LATENCY_AWARE:
            distributor = new co::LatencyAwareDistributor(producer);
            distName = "LatencyAwareDistributor";
            break;

        case fabric::DISTRIBUTION_STRATEGY_CENT_LOAD_AWARE:
            distributor = new co::CentLoadAwareDistributor(producer);
            distName = "CentLoadAwareDistributor";
            break;
        }
        
        distributor->setPerfLogger( _perfLogger );
        producer.setPackageDistributor(distributor);

        std::cerr << "Eq: Selected package distributor: " << distName << "." << std::endl;
    }
}

template < class T, class S >
void PackageQueue<T, S>::cycleData( const uint32_t frameNumber, const Compound* compound)
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        if( !compound->isInheritActive( Eye( 1<<i )))// eye pass not used
        {
            _queueMaster[i] = 0;
            continue;
        }

        // reuse unused queues
        LatencyQueue* queue    = _queues.empty() ? 0 : _queues.back();
        const uint32_t latency = getAutoObsolete();
        const uint32_t dataAge = queue ? queue->_frameNumber : 0;

        if( queue && dataAge < frameNumber-latency && frameNumber > latency )
            // not used anymore
            _queues.pop_back();
        else // still used - allocate new data
        {
            queue = new LatencyQueue;

            getLocalNode()->registerObject( &queue->_queue );
            queue->_queue.setAutoObsolete( 1 ); // current + in use by render nodes
        }

        queue->_queue.clear();
        queue->_frameNumber = frameNumber;

        _queues.push_front( queue );
        _queueMaster[i] = queue;
    }
}

template < class T, class S >
void PackageQueue<T, S>::setOutputQueue( PackageQueue<T, S>* queue, const Compound* compound )
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        // eye pass not used && no output frame for eye pass
        if( compound->isInheritActive( Eye( 1<<i )))
            _outputQueue[i] =queue;
    }
}

template < class T, class S >
void PackageQueue<T, S>::getInstanceData( co::DataOStream& )
{
}

template < class T, class S >
void PackageQueue<T, S>::applyInstanceData( co::DataIStream& )
{
}

template < class T, class S >
void PackageQueue<T, S>::flush()
{
    unsetData();

    while( !_queues.empty( ))
    {
        LatencyQueue* queue = _queues.front();
        _queues.pop_front();
        getLocalNode()->deregisterObject( &queue->_queue );
        delete queue;
    }

}

template < class T, class S >
void PackageQueue<T, S>::unsetData()
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        _queueMaster[i] = 0;
        _outputQueue[i] = 0;
    }
}

template < class T, class S >
uint128_t PackageQueue<T, S>::getQueueMasterID( const Eye eye ) const
{
    uint32_t index = lunchbox::getIndexOfLastBit(eye);
    LatencyQueue* queue = _queueMaster[ index ];
    if ( queue )
        return queue->_queue.getID();
    return uint128_t();
}

}
}
