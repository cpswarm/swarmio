#pragma once

#include <swarmio/transport/BasicEndpoint.h>
#include <swarmio/transport/zyre/ZyreNode.h>
#include <swarmio/transport/zyre/ZyreControlSocket.h>
#include <readerwriterqueue.h>
#include <zyre.h>
#include <map>
#include <list>
#include <thread>
#include <shared_mutex>

namespace swarmio::transport::zyre
{
    /**
     * @brief An Endpoint implementation using the
     *        Zyre protocol.
     * 
     * Zyre provides reliable group messaging over local 
     * area networks. It uses UDP beacons for discovery and
     * full-fledged TCP based ZeroMQ messages for reliable
     * communication within the group. Zyre endpoints leverage
     * the protocol for both transmission and node discovery.
     * 
     */
    class SWARMIO_API ZyreEndpoint : public BasicEndpoint
    {
        private:

            /**
             * @brief Reference to the Zyre structure
             * 
             */
            zyre_t* _zyre;

            /**
             * @brief Control pipe to shut down event processing
             * 
             */
            ZyreControlSocket _control;

            /**
             * @brief Node registry
             * 
             */
            std::map<std::string, ZyreNode> _nodes;

            /**
             * @brief Mutex protecting the Nodes registry
             * 
             */
            std::shared_timed_mutex _mutex;

            /**
             * @brief Worker thread
             * 
             */
            std::thread* _worker = nullptr;

            /**
             * @brief Entry point for the worker thread
             * 
             */
            void Process();

            /**
             * @brief Entry point for the delivery thread
             * 
             */
            void Deliver(moodycamel::BlockingReaderWriterQueue<zyre_event_t*>* queue);

        protected:

            /**
             * @brief Called by BasicEndpoint to send serialized messages.
             *        Called with node set to nullptr to send a message
             *        to all members of the swarm.
             * 
             * @param data Raw message data
             * @param size Length of the message data
             * @param node Node the message will be sent to
             */            
            virtual void Send(const void* data, size_t size, const Node* node) override;

        public:

            /**
             * @brief Construct a new ZyreEndpoint object
             * 
             * @param name The discoverable name of the endpoint
             * @param ifname Network interface name
             */
            ZyreEndpoint(const char* name);

            /**
             * @brief Set the port used by the endpoint.
             * 
             * Will throw an exception if the node is running.
             * 
             * @param port Port
             */
            void SetPort(uint16_t port);

            /**
             * @brief Set the network interface to bind to.
             * 
             * Will throw an exception if the node is running.
             * 
             * @param ifname Interface name
             */
            void SetInterface(const char* ifname);

            /**
             * @brief Start the background thread, announce the 
             *        Zyre node and start processing messages.
             * 
             */
            virtual void Start() override;

             /**
             * @brief Send a termination signal and wait until the
             *        endpoint finished processing messages.
             * 
             */
            virtual void Stop() override;

            /**
             * @brief Destroy the ZyreEndpoint object
             * 
             */
            virtual ~ZyreEndpoint() override;

            /**
             * @brief Get the UUID of the local node.
             * 
             * @return std::string UUID
             */
            std::string GetUUID();

            /**
             * @brief Get a list of known Nodes
             * 
             * @return std::list<ZyreNode> 
             */
            std::list<const ZyreNode*> GetNodes();
    };
}