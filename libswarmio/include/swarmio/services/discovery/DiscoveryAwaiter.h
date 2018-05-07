#pragma once

#include <swarmio/services/Awaiter.h>
#include <swarmio/data/discovery/Response.pb.h>

namespace swarmio::services::discovery
{
    /**
     * @brief An Awaiter that returns discovery data on a remote node
     * 
     */
    class SWARMIO_API DiscoveryAwaiter : public Awaiter<data::discovery::Response>
    {
        protected:

            /**
             * @brief Called when a response for the original message has been received.
             * 
             * @param node Sender node
             * @param message Message
             */
            virtual data::discovery::Response ExtractResponse(const Node* node, const data::Message* message) override
            {
                return message->ds_response();
            }

        public:

            /**
             * @brief Inherit constructor
             * 
             */
            using Awaiter::Awaiter;
    };
}
