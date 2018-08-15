#pragma once

#include <swarmio/services/Awaiter.h>
#include <swarmio/data/discovery/Response.pb.h>

namespace swarmio::services::discovery
{
    /**
     * @brief An Awaiter that returns discovery data on a remote node
     * 
     */
    class SWARMIO_API DiscoveryAwaiter final : public Awaiter<data::discovery::Response>
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
             * @brief Construct a new DiscoveryAwaiter object
             * 
             * @param endpoint Endpoint
             * @param requestIdentifier Original message identifier
             */
            DiscoveryAwaiter(Endpoint* endpoint, uint64_t requestIdentifier)
                : Awaiter(endpoint, requestIdentifier) 
            {
                FinishConstruction();
            }

            /**
             * @brief Construct a DiscoveryAwaiter with a cached value
             * 
             * @param value Value
             */
            DiscoveryAwaiter(const data::discovery::Response& value)
                : Awaiter(value)
            {
                FinishConstruction();
            }
    };
}
