#pragma once

#include <swarmio/Mailbox.h>
#include <swarmio/services/discovery/Discoverable.h>
#include <swarmio/services/telemetry/UpdateAwaiter.h>
#include <swarmio/services/telemetry/Tracker.h>
#include <swarmio/data/Variant.pb.h>
#include <list>
#include <string>
#include <shared_mutex>
#include <thread>

namespace swarmio::services::telemetry 
{
    /**
     * @brief Telemetry Service can subscribe to receive
     *        updates from remote nodes on named values.
     * 
     */
    class SWARMIO_API Service final : public Mailbox, public discovery::Discoverable
    {
        private:

            /**
             * @brief Worker thread
             * 
             */
            std::thread* _worker = nullptr;

            /**
             * @brief Worker thread shutdown requested
             * 
             */
            std::atomic<bool> _shutdownRequested;

            /**
             * @brief List of published telemetry values
             * 
             */
            std::map<std::string, data::Variant> _values;

            /**
             * @brief Mutex to protect the map of values
             * 
             */
            std::shared_mutex _valuesMutex;

            /**
             * @brief Trackers for each remote subscription
             * 
             */
            std::list<Tracker> _trackers;

            /**
             * @brief Mutex to protect the list of trackers
             * 
             */
            std::shared_mutex _trackersMutex;

            /**
             * @brief Worker thread entry point
             * 
             */
            void Worker();

            /**
             * @brief Send an update to all subscribers.
             * 
             */
            void Update();

        public:

            /**
             * @brief Subscribe to all named values on the remote node.
             * 
             * @param endpoint Endpoint to use
             * @param node Remote node
             * @param interval Interval
             * @return UpdateAwaiter Awaiter for the updates
             */
            static UpdateAwaiter Subscribe(Endpoint* endpoint, const Node* node, uint32_t interval = 1)
            {
                std::list<std::string> emptyList;
                return Subscribe(endpoint, node, interval, emptyList);
            }

            
            /**
             * @brief Subscribe to soecific named values on the remote node.
             * 
             * @param endpoint Endpoint to use
             * @param node Remote node
             * @param interval Interval
             * @param keys Keys to subscribe to
             * @return UpdateAwaiter Awaiter for the updates
             */
            static UpdateAwaiter Subscribe(Endpoint* endpoint, const Node* node, uint32_t interval, const std::list<std::string>& keys);

            /**
             * @brief Construct a new Service object
             * 
             * @param endpoint Endpoint
             */
            Service(Endpoint* endpoint);

            /**
             * @brief Add or update a value in the local telemetry cache.
             * 
             * @param key Key
             * @param value Value
             */
            void SetValue(const std::string& key, data::Variant value)
            {
                std::unique_lock<std::shared_mutex> guard(_valuesMutex);
                _values[key] = value;
            }

            /**
             * @brief Remove a value from the local telemetry cache.
             * 
             * @param key Key
             */
            void RemoveValue(const std::string& key)
            {
                std::unique_lock<std::shared_mutex> guard(_valuesMutex);
                _values.erase(key);
            }

            /**
             * @brief Delivery point of all messages
             * 
             * @param sender The node that has sent the message
             * @param message The message itself
             * @returns True if the message had been processed and should 
             *          not be forwarded to other mailboxes
             */
            virtual bool ReceiveMessage(const Node* sender, const data::Message* message) override;

            /**
             * @brief Add descriptors for the service to
             *        the discovery descriptor
             * 
             * @param descriptor The proposed response to load 
             *                   the description into
             */
            virtual void DescribeService(data::discovery::Response& descriptor) override;

            /**
             * @brief Destructor
             * 
             */
            virtual ~Service() override;
    };
}
