#pragma once

#include <swarmio/Mailbox.h>
#include <swarmio/services/discovery/Observer.h>
#include <swarmio/services/discovery/Discoverable.h>
#include <swarmio/services/discovery/DiscoveryAwaiter.h>
#include <set>
#include <map>
#include <mutex>

namespace swarmio::services::discovery 
{
    /**
     * @brief The Discovery Service makes it possible to
     *        query remote nodes for services.
     * 
     */
    class SWARMIO_API Service final : public Mailbox  
    {
        private:

            /**
             * @brief Should we send discovery requests automatically?
             * 
             */
            bool _performActiveDiscovery;

            /**
             * @brief Cache of remote node discovery responses
             * 
             */
            std::map<const Node*, data::discovery::Response> _remotes;

             /**
             * @brief Mutex to protect remotes list
             * 
             */
            std::mutex _remotesMutex;

            /**
             * @brief List of discovery observers
             * 
             */
            std::set<Observer*> _observers;

            /**
             * @brief Mutex to protect observer list
             * 
             */
            std::mutex _observersMutex;

            /**
             * @brief List of discoverable services
             * 
             */
            std::set<Discoverable*> _discoverables;

            /**
             * @brief Mutex to protect discoverable list
             * 
             */
            std::mutex _discoverablesMutex;

            /**
             * @brief Tracks whether the current cache is valid
             * 
             */
            bool _cacheValid = true;

            /**
             * @brief Cached discovery response
             * 
             */
            data::discovery::Response _cachedResponse;

            /**
             * @brief Handle incoming discovery requests
             * 
             * @param sender The node that has sent the message
             * @param message The message itself
             */
            void HandleDiscoveryRequest(const Node* node, const data::Message* message);

            /**
             * @brief Handle incoming discovery requests
             * 
             * @param sender The node that has sent the message
             * @param message The message itself
             */
            void CacheDiscoveryResponse(const Node* node, const data::Message* message);

            /**
             * @brief Invalidate information on a 
             *        previously discovered node
             * 
             * @param sender The node that has sent the message
             * @param message The message itself
             */
            void HandleInvalidationRequest(const Node* node, const data::Message* message);

        public:

            /**
             * @brief Construct a new Service object
             * 
             * @param endpoint Endpoint
             * @param performActiveDiscovery Send automatic discovery
             *                               requests to newly joined nodes
             */
            Service(Endpoint* endpoint, bool performActiveDiscovery)
                : Mailbox(endpoint), _performActiveDiscovery(performActiveDiscovery)
            { 
                FinishConstruction();
            }

            /**
             * @brief Send a Discovery query to a remote node
             * 
             * @param node Remote node
             * @return DiscoveryAwaiter 
             */
            static DiscoveryAwaiter Query(Endpoint* endpoint, const Node* node);

            /**
             * @brief Send a Discovery query to a remote node, or if the 
             *        information already exists in the cache, return that
             * 
             * @param node Remote node
             * @return DiscoveryAwaiter 
             */
            DiscoveryAwaiter CachedQuery(const Node* node);

            /**
             * @brief Sends a global Discovery request
             * 
             */
            void GlobalQuery();

            /**
             * @brief Returns a map containing all currently valid 
             *        node information in the cache
             * 
             * @return std::map<const Node*, data::discovery::Response> 
             */
            std::map<const Node*, data::discovery::Response> GetCachedNodeInformation();

            /**
             * @brief Register a new discovery observer
             * 
             * @param observer Observer
             */
            void RegisterObserver(Observer* observer)
            {
                std::unique_lock<std::mutex> guard(_observersMutex);
                _observers.insert(observer);
            }
            /**
             * @brief Unregister a discovery observer
             * 
             * @param observer Observer
             */
            void UnregisterObserver(Observer* observer)
            {
                std::unique_lock<std::mutex> guard(_observersMutex);
                _observers.erase(observer);
            }

            /**
             * @brief Invalidate the current cached descriptor.
             * 
             */
            void Invalidate();

            /**
             * @brief Register a new Discoverable service
             * 
             * @param discoverable Service
             */
            void RegisterDiscoverable(Discoverable* discoverable);

            /**
             * @brief Unregister a Discoverable service
             * 
             * @param discoverable Service
             */
            void UnregisterDiscoverable(Discoverable* discoverable);

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
             * @brief Called when a new Node has joined the group
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeDidJoin(const Node* node) noexcept override;

            /**
             * @brief Called when a Node signals that it will leave.
             * 
             * @param node The node that has left
             */
            virtual void NodeWillLeave(const Node* node) noexcept override;
    };
}