#pragma once

#include <swarmio/Node.h>
#include <swarmio/data/discovery/Response.pb.h>

namespace swarmio::services::discovery 
{
    /**
     * @brief Interface for discovery observers
     * 
     */
    class SWARMIO_API Observer  
    {
        public:

            /**
             * @brief Called to notify the observer when new discovery 
             *        information was received for a node.
             * 
             * @param node Node
             * @param response Discovery response
             */
            virtual void CachedDiscoveryResponseWasUpdated(const Node* node, const data::discovery::Response& response) = 0;
    };
}