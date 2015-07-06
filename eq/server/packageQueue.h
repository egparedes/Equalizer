
/* Copyright (c) 2011-2012, Stefan Eilemann <eile@eyescale.ch>
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

#ifndef EQSERVER_PACKAGEQUEUE_H
#define EQSERVER_PACKAGEQUEUE_H

#include "compound.h"
#include "types.h"

#include <lunchbox/bitOperation.h> // function getIndexOfLastBit
#include <co/perfLogger.h>
#include <co/queueMaster.h>
#include <co/stealingQueueMaster.h>
#include <eq/fabric/distributionStrategy.h>

namespace eq
{
namespace server
{
    /** A holder for work package data and parameters. */
    template < class T, class S > class PackageQueue : public co::Object
    {
    public:
        /**
         * Constructs a new PackageQueue.
         */
        EQSERVER_API PackageQueue();
        EQSERVER_API virtual ~PackageQueue();
        PackageQueue( const PackageQueue<T, S>& from );

        /**
         * @name Data Access
         */
        //@{
        void setCompound( Compound* compound )
            { LBASSERT( !_compound ); _compound = compound; }
        Compound* getCompound() const { return _compound; }
        Channel* getChannel() const
            { return _compound ? _compound->getChannel() :0; }
        Node* getNode() const { return _compound ? _compound->getNode() : 0; }

        void setName( const std::string& name ) { _name = name; }
        const std::string& getName() const      { return _name; }

        /** Set the size of the packages. */
        void setPackageSize( const Vector2i& size ) { _packageSize = size; }

        /** Set the size of the packages. */
        void setPackageSize( float size ) { _packageSize = size; }

        /** @return the package size. */
        const S& getPackageSize() const { return _packageSize; }

        /** Add a package to the queue. */
        void addPackage( T package, float position, const Eye eye );

        /** Add an end to the queue. */
        void addEnd( const fabric::Eye eye );

        /** Set slave nodes. */
        void setSlaveNodes( const co::Nodes& nodes, const fabric::Eye eye );
        
        /** set distribution strategy for the queue. */
        void setDistributionStrategy( const fabric::DistributionStrategy strategy )
            { _distributionStrategy = strategy; }
        
        /** set performance logger for the queue. */
        void setPerfLogger( co::PerfLogger* perfLogger )
            { _perfLogger = perfLogger; }

        /** select package distributor, if any, for the queue. */
        void selectPackageDistributor( const Eye eye );

        /**
         * Cycle the current package queue.
         *
         * Used for output package queues to allocate/recycle queue masters.
         *
         * @param frameNumber the current frame number.
         * @param compound the compound holding the output frame.
         */
        void cycleData( const uint32_t frameNumber, const Compound* compound );

        void setOutputQueue( PackageQueue<T, S>* queue, const Compound* compound );
        const PackageQueue<T, S>* getOutputQueue( const Eye eye ) const
            { return _outputQueue[ lunchbox::getIndexOfLastBit( eye ) ]; }

        /**
         * @name Operations
         */
        //@{
        /** Unset the package data. */
        void unsetData();

        /** Reset the frame and delete all package datas. */
        void flush();
        //@}

        uint128_t getQueueMasterID( const Eye eye ) const;

    protected:
        EQSERVER_API virtual ChangeType getChangeType() const { return INSTANCE; }
        EQSERVER_API virtual void getInstanceData( co::DataOStream& os );
        EQSERVER_API virtual void applyInstanceData( co::DataIStream& is );

    private:
        struct LatencyQueue
        {
            uint32_t _frameNumber;
            co::QueueMaster _queue;
        };

        /** The parent compound. */
        Compound* _compound;

        /** The name which associates input to output frames. */
        std::string _name;

        /** The size of each package in the queue. */
        S _packageSize;

        /** The collage queue pool. */
        std::deque< LatencyQueue* > _queues;

        /** the currently used package queues */
        LatencyQueue* _queueMaster[ NUM_EYES ];

        /** The current output queue. */
        PackageQueue<T, S>* _outputQueue[ NUM_EYES ];
        
        /** The queue's distribution strategy */
        fabric::DistributionStrategy _distributionStrategy;

        /** The queue's performance logger */
        co::PerfLogger* _perfLogger;
    };
}
}

#include "packageQueue.ipp" // template implementation

#endif // EQSERVER_PACKAGEQUEUE_H
