#pragma once

#include <swarmio/services/Awaiter.h>

namespace swarmio::services 
{
    /**
     * @brief An Awaiter that checks whether the operation was a success.
     * 
     */
    class SWARMIO_API ErrorAwaiter final : public Awaiter<bool>
    {

        protected:

            /**
             * @brief Called when a response for the original message has been received.
             * 
             * @param node Sender node
             * @param message Message
             */
            virtual bool ExtractResponse(const Node* node, const data::Message* message) override;

        public:

            /**
             * @brief Construct a new ErrorAwaiter object
             * 
             * @param endpoint Endpoint
             * @param requestIdentifier Original message identifier
             */
            ErrorAwaiter(Endpoint* endpoint, uint64_t requestIdentifier)
                : Awaiter(endpoint, requestIdentifier)
            {
                FinishConstruction();
            }
    };
}
