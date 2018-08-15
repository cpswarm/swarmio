#pragma once

#include <swarmio/services/Awaiter.h>
#include <swarmio/data/Variant.pb.h>

namespace swarmio::services::keyvalue
{
    /**
     * @brief An Awaiter that returns the requested value.
     * 
     */
    class SWARMIO_API ValueAwaiter final : public Awaiter<data::Variant>
    {
        private:

            /**
             * @brief The expected key in the telemetry response
             * 
             */
            std::string _key;

        protected:

            /**
             * @brief Called when a response for the original message has been received.
             * 
             * @param node Sender node
             * @param message Message
             */
            virtual data::Variant ExtractResponse(const Node* node, const data::Message* message) override;

        public:

            /**
             * @brief Construct a new ValueAwaiter object
             * 
             * @param endpoint Endpoint
             * @param requestIdentifier Original message identifier
             * @param key Expected key for the response
             */
            ValueAwaiter(Endpoint* endpoint, uint64_t requestIdentifier, const std::string& key)
                : Awaiter(endpoint, requestIdentifier) 
            {
                _key = key;
                FinishConstruction();
            }
    };
}
