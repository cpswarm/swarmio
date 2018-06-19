#pragma once

#include <swarmio/Endpoint.h>
#include <swarmio/data/Variant.pb.h>
#include <swarmio/data/telemetry/SubscribeRequest.pb.h>
#include <map>
#include <string>

namespace swarmio::services::telemetry 
{
    /**
     * @brief Telemetry Service can subscribe to receive
     *        updates from remote nodes on named values.
     * 
     */
    class Tracker final
    {
        private:

            /**
             * @brief Subscriber
             * 
             */
            const Node* _node;

            /**
             * @brief The identifier of the original request
             * 
             */
            uint64_t _identifier;

            /**
             * @brief The original subscribe request
             * 
             */
            data::telemetry::SubscribeRequest _request;

            /**
             * @brief Tick counter for the updates
             * 
             */
            std::atomic<uint32_t> _currentTick;

            /**
             * @brief Is the tracker still valid
             * 
             */
            std::atomic<bool> _valid;

        public:

            /**
             * @brief Construct a new Tracker object
             * 
             * @param node Subscriber
             * @param identifier Message identifier
             * @param request Request body
             */
            Tracker(const Node* node, uint64_t identifier, const data::telemetry::SubscribeRequest& request)
                : _identifier(identifier), _request(request), _node(node), _currentTick(0), _valid(true) { }


            /**
             * @brief Get original message identifier
             * 
             * @return uint64_t Identifier
             */
            uint64_t GetIdentifier() const
            {
                return _identifier;
            }

            /**
             * @brief Get target node
             * 
             * @return const Node* Node
             */
            const Node* GetNode() const
            {
                return _node;
            }

            /**
             * @brief Get original request
             * 
             * @return const data::telemetry::SubscribeRequest& 
             */
            const data::telemetry::SubscribeRequest& GetRequest() const
            {
                return _request;
            }

            /**
             * @brief Get current tick
             * 
             * @return uint32_t 
             */
            uint32_t GetTick() const
            {
                return _currentTick;
            }

            /**
             * @brief Increment current tick and return value
             * 
             * @return uint32_t 
             */
            uint32_t GetAndIncrementTick()
            {
                return _currentTick++;
            }

            /**
             * @brief Is the tracker still valid?
             * 
             * @return True if updates should still be sent
             */
            bool IsValid() const
            {
                return _valid;
            }

            /**
             * @brief Mark the tracker as invalid
             * 
             */
            void Invalidate()
            {
                _valid = false;
            }
    };
}
