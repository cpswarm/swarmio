#pragma once

#include <swarmio/Mailbox.h>
#include <swarmio/services/ping/TimingAwaiter.h>
#include <swarmio/services/discovery/Discoverable.h>

namespace swarmio::services::ping 
{
    /**
     * @brief Ping Service dispatches and responds to ping 
     *        requests, and measures the time between the 
     *        request and the response.
     * 
     */
    class SWARMIO_API Service final : public Mailbox, public discovery::Discoverable
    {
        public:

            /**
             * @brief Measure the latency to a remote node.
             * 
             * @param endpoint Endpoint to use
             * @param node Node to ping
             * @param size Number of bytes to generate for the payload
             * @return TimingAwaiter Awaiter for the latency value
             */
            static TimingAwaiter Ping(Endpoint* endpoint, const Node* node, size_t size);

            /**
             * @brief Construct a new Service object
             * 
             * @param endpoint Endpoint
             */
            Service(Endpoint* endpoint)
                : Mailbox(endpoint) { }

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
