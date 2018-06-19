#pragma once

#include <swarmio/services/Awaiter.h>
#include <chrono>

namespace swarmio::services::ping
{
    /**
     * @brief An Awaiter that returns the roundtrip time of the Echo message.
     * 
     */
    class SWARMIO_API TimingAwaiter final : public Awaiter<std::chrono::nanoseconds>
    {
        private:

            /**
             * @brief When the ping request was sent
             * 
             */
            std::chrono::time_point<std::chrono::high_resolution_clock> _start;

        protected:

            /**
             * @brief Called when a response for the original message has been received.
             * 
             * @param node Sender node
             * @param message Message
             */
            virtual std::chrono::nanoseconds ExtractResponse(const Node* node, const data::Message* message) override;

        public:

            /**
             * @brief Construct a new TimingAwaiter object
             * 
             * @param endpoint Endpoint
             * @param requestIdentifier Original message identifier
             */
            TimingAwaiter(Endpoint* endpoint, uint64_t requestIdentifier);

            /**
             * @brief Translate the precise response to a
             *        floating point number of milliseconds
             * 
             * @return double 
             */
            double GetResponseInMilliseconds();
    };
}
