#pragma once

#include <swarmio/Node.h>
#include <swarmio/data/telemetry/Status.pb.h>

namespace swarmio::services::telemetry
{
    /**
     * @brief Interface for discovery observers
     * 
     */
    class SWARMIO_API Observer  
    {
        public:

            /**
             * @brief Called to notify the observer when a node
             *        has updated its status.
             * 
             * @param node Node
             * @param status Status
             */
            virtual void CachedStatusWasUpdated(const Node* node, const data::telemetry::Status& status) = 0;
    };
}