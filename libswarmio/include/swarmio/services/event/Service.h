#pragma once

#include <swarmio/Mailbox.h>
#include <swarmio/services/ErrorAwaiter.h>
#include <swarmio/services/event/Handler.h>
#include <swarmio/services/discovery/Discoverable.h>
#include <string>
#include <memory>
#include <map>

namespace swarmio::services::event 
{
    /**
     * @brief The Event service is responsible for 
     *        triggering, handling and monitoring events.
     * 
     */
    class SWARMIO_API Service final : public Mailbox, public discovery::Discoverable
    {
        private:

            /**
             * @brief Map of handlers
             * 
             */
            std::map<std::string, Handler*> _handlers;

            /**
             * @brief Mutex to protect handler list
             * 
             */
            std::mutex _mutex;

        public:

            /**
             * @brief Trigger an event globally.
             * 
             * Thread-safe.
             * 
             * @param endpoint Endpoint to use
             * @param event Event to propagate
             */
            static void Trigger(Endpoint* endpoint, const data::event::Notification& event);

            /**
             * @brief Trigger an event on a remote Node.
             * 
             * Thread-safe.
             * 
             * @param endpoint Endpoint to use
             * @param event Event to propagate
             * @param event Target node
             * @return ErrorAwaiter Async result
             */
            static ErrorAwaiter Trigger(Endpoint* endpoint, const data::event::Notification& event, const Node* node);

            /**
             * @brief Construct a new Service object
             * 
             * @param endpoint Endpoint
             */
            Service(Endpoint* endpoint)
                : Mailbox(endpoint)
            { 
                FinishConstruction();
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
             * @brief Subscribe to events.
             * 
             * Thread-safe. 
             * 
             * @param name Event name
             * @param handler Handler
             */
            void RegisterHandler(const std::string& name, Handler* handler);

            /**
             * @brief Unsubscribe from events.
             * 
             * Thread-safe.
             * 
             * @param name Event name
             */
            void UnregisterHandler(const std::string& name);

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