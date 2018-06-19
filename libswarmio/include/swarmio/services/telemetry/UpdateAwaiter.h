#pragma once

#include <swarmio/services/Awaiter.h>
#include <chrono>

namespace swarmio::services::telemetry
{
    /**
     * @brief An Awaiter that has a longer lifetime and is updated periodically.
     * 
     */
    class SWARMIO_API UpdateAwaiter final : public Awaiter<data::telemetry::Update>
    {
        private:

            /**
             * @brief Target node
             * 
             */
            const Node* _target;

        protected:

            /**
             * @brief Called when a response for the original message has been received.
             * 
             * @param node Sender node
             * @param message Message
             */
            virtual data::telemetry::Update ExtractResponse(const Node* node, const data::Message* message) override
            {
                if (message->content_case() == data::Message::ContentCase::kTmUpdate)
                {
                    return message->tm_update();
                }
                else
                {
                    throw Exception("Last update received is invalid");
                }
            }

            /**
             * @brief Telemetry update awaiters remain valid for their entire lifetime.
             * 
             * @return Always true 
             */
            virtual bool IsFinished() override
            {
                return false;
            }

        public:

            /**
             * @brief Construct a new UpdateAwaiter object
             * 
             * @param endpoint Endpoint
             * @param requestIdentifier Original message identifier
             */
            UpdateAwaiter(Endpoint* endpoint, uint64_t requestIdentifier, const Node* target)
                : Awaiter(endpoint, requestIdentifier), _target(target) { }

            /**
             * @brief Intercepted to handle unsubscriptions gracefully
             * 
             */
            virtual void Disconnect() override
            {
                if (GetEndpoint() != nullptr)
                {
                    data::Message request;
                    request.mutable_tm_unsubscribe_request()->set_identifier(GetIdentifier());
                    GetEndpoint()->Send(&request, _target);
                }
                Awaiter::Disconnect();
            }

            /**
             * @brief Get target node
             * 
             * @return const Node* Target
             */
            const Node* GetTarget() const
            {
                return _target;
            }
    };
}
