#pragma once

#include <swarmio/Mailbox.h>
#include <swarmio/services/StatusAwaiter.h>
#include <swarmio/services/keyvalue/Target.h>
#include <swarmio/services/keyvalue/ValueAwaiter.h>
#include <swarmio/services/discovery/Discoverable.h>
#include <string>
#include <memory>
#include <mutex>
#include <map>

namespace swarmio::services::keyvalue 
{
    /**
     * @brief The Key-Value Service is responsible for getting
     *        and setting named values.
     * 
     */
    class SWARMIO_API Service final : public Mailbox, public discovery::Discoverable
    {
        private:

            /**
             * @brief Key-target map
             * 
             */
            std::map<std::string, Target*> _targets;

            /**
             * @brief Mutex to protect target list
             * 
             */
            std::mutex _mutex;

            /**
             * @brief Handle remote Get requests
             * 
             * @param request Request message
             * @param reply_to Reply address
             * @return True if the request was handled
             */
            bool HandleGetRequest(const Node* sender, const data::Message* message);

            /**
             * @brief Handle remote Set requests
             * 
             * @param request Request message
             * @return True if the request was handled
             */
            bool HandleSetRequest(const Node* sender, const data::Message* message);

        public:

            /**
             * @brief Get a remote value
             * 
             * Thread-safe.
             * 
             * @param endpoint Endpoint to use
             * @param node Node to get the value from
             * @param path Resource path
             * @return ValueAwaiter Async value
             */
            static ValueAwaiter Get(Endpoint* endpoint, const Node* node, const std::string& path);

            /**
             * @brief Set a remote value
             * 
             * Thread-safe.
             * 
             * @param endpoint Endpoint to use
             * @param node Node to set the value at
             * @param path Resource path
             * @return StatusAwaiter Async result
             */
            static StatusAwaiter Set(Endpoint* endpoint, const Node* node, const std::string& path, const data::Variant& value);

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
             * @brief Register a new Target with the specified path
             * 
             * Thread-safe.
             * 
             * @param path Resource path
             * @param target Target
             */
            void RegisterTarget(const std::string& path, Target* target);

            /**
             * @brief Unregister a Target
             * 
             * Thread-safe.
             * 
             * @param path Resource path
             */
            void UnregisterTarget(const std::string& path);

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
