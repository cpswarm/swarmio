#pragma once

#include <swarmio/services/PeriodicService.h>
#include <swarmio/services/discovery/Discoverable.h>
#include <swarmio/services/telemetry/UpdateAwaiter.h>
#include <swarmio/services/telemetry/Tracker.h>
#include <swarmio/services/telemetry/Observer.h>
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
    class SWARMIO_API Service final : public PeriodicService, public discovery::Discoverable
    {
        private:

            /**
             * @brief List of published telemetry values
             * 
             */
            std::map<std::string, data::Variant> _values;

            /**
             * @brief Mutex to protect the map of values
             * 
             */
            std::shared_timed_mutex _valuesMutex;

            /**
             * @brief Schema
             * 
             */
            data::discovery::Schema _schema;

            /**
             * @brief List of keys to include in the status broadcast
             * 
             */
            std::set<std::string> _statusKeys;

            /**
             * @brief Mutex to protect the schema
             * 
             */
            std::shared_timed_mutex _schemaMutex;

            /**
             * @brief Trackers for each remote subscription
             * 
             */
            std::list<Tracker> _trackers;

            /**
             * @brief Mutex to protect the list of trackers
             * 
             */
            std::shared_timed_mutex _trackersMutex;

            /**
             * @brief List of observers
             * 
             */
            std::set<Observer*> _observers;

            /**
             * @brief Mutex to protect the list of observers
             * 
             */
            std::shared_timed_mutex _observersMutex;

            /**
             * @brief Cached values for remote status reports
             * 
             */
            std::map<const Node*, data::telemetry::Status> _reports;

            /**
             * @brief Mutex to protect remote status reports
             * 
             */
            std::shared_timed_mutex _reportsMutex;

        protected:

            /**
             * @brief Send an update to all subscribers.
             * 
             */
            virtual void Update() override final;

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
             * @param period Tick period
             */
            Service(Endpoint* endpoint, std::chrono::milliseconds period = std::chrono::milliseconds(10))
                : PeriodicService(endpoint, period)
            { 
                FinishConstruction();
            }

            /**
             * @brief Add or update a value in the local telemetry cache.
             * 
             * @param key Key
             * @param value Value
             */
            void SetValue(const std::string& key, const data::Variant& value)
            {
                std::unique_lock<std::shared_timed_mutex> guard(_valuesMutex);
                _values[key] = value;
            }

            /**
             * @brief Remove a value from the local telemetry cache.
             * 
             * @param key Key
             */
            void RemoveValue(const std::string& key)
            {
                std::unique_lock<std::shared_timed_mutex> guard(_valuesMutex);
                _values.erase(key);
            }

            /**
             * @brief Add field to the schema
             * 
             * @param key Key
             * @param field Field
             */
            void SetFieldDefinitionForKey(const std::string& key, const data::discovery::Field& field, bool includeInStatus)
            {
                std::unique_lock<std::shared_timed_mutex> guard(_schemaMutex);
                (*_schema.mutable_fields())[key] = field;
                if (includeInStatus)
                {
                    _statusKeys.insert(key);
                }
                else
                {
                    _statusKeys.erase(key);
                }
            }

            /**
             * @brief Remove a field from the schema
             * 
             * @param key Key
             */
            void RemoveFieldDefinitionForKey(const std::string& key)
            {
                std::unique_lock<std::shared_timed_mutex> guard(_schemaMutex);
                _schema.mutable_fields()->erase(key);
                _statusKeys.erase(key);
            }

            /**
             * @brief Register a new status observer
             * 
             * @param observer Observer
             */
            void RegisterObserver(Observer* observer)
            {
                std::unique_lock<std::shared_timed_mutex> guard(_observersMutex);
                _observers.insert(observer);
            }

            /**
             * @brief Unregister a status observer
             * 
             * @param observer Observer
             */
            void UnregisterObserver(Observer* observer)
            {
                std::unique_lock<std::shared_timed_mutex> guard(_observersMutex);
                _observers.erase(observer);
            }

            data::telemetry::Status GetCachedStatus(const Node* node)
            {
                std::shared_lock<std::shared_timed_mutex> guard(_reportsMutex);
                return _reports[node];
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
    };
}
