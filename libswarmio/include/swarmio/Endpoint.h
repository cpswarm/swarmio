#pragma once

#include <swarmio/Node.h>
#include <swarmio/data/Message.pb.h>

namespace swarmio
{

    /**
     * @brief Forward declare Mailbox
     * 
     */
    class Mailbox;

    /**
     * @brief Abstract base class for Endpoint implementations.
     * 
     * When an Endpoint is created and started, it announces itself
     * to all members of the swarm and start sending and receiving
     * messages. Different implementations might provide different 
     * services for discovery, authentication, authorization and 
     * encryption.
     * 
     */
    class SWARMIO_API Endpoint
    {
        protected:

            /**
             * @brief Allow Mailboxes to register themselves
             * 
             */
            friend Mailbox;

            /**
             * @brief Register a Mailbox to receive messages.
             * 
             * Each registered Mailbox receives each message sent to
             * the endpoint, and can choose to handle it or ignore it.
             * 
             * Thread-safe. Messages will be delivered on a separate thread.
             * 
             * @param mailbox Mailbox to register.
             */
            virtual void RegisterMailbox(Mailbox* mailbox) = 0;

            /**
             * @brief Unregister a Mailbox from receiving messages.
             * 
             * Thread-safe.
             * 
             * @param mailbox Mailbox to unregister.
             */
            virtual void UnregisterMailbox(Mailbox* mailbox)= 0;

            /**
             * @brief Relocate a mailbox to another in-memory location.
             * 
             * Supports thread-safe move operations on Mailboxes.
             * 
             * @param mailbox Mailbox to register.
             */
            virtual void ReplaceMailbox(Mailbox* oldMailbox, Mailbox* newMailbox) = 0;

        public:

            /**
             * @brief Start a background thread and begin processing
             *        messages on this endpoint.
             * 
             */
            virtual void Start() = 0;

            /**
             * @brief Send a termination signal and wait until the
             *        endpoint finished processing messages.
             * 
             * Thread-safe. 
             * 
             */
            virtual void Stop() = 0;

            /**
             * @brief Set the message identifier for a message.
             * 
             * Thread safe.
             * 
             * @param message Message
             */
            virtual void Tag(data::Message* message) = 0;

            /**
             * @brief Send a message to a specific member of the swarm. 
             *        Call with node set to nullptr to send a message
             *        to all members of the swarm.
             * 
             * Thread-safe.
             * 
             * @param message Message
             * @param node Node the message will be sent to
             */
            virtual void Send(data::Message* message, const Node* node) = 0;

            /**
             * @brief Retreive a node by its UUID
             * 
             * @param uuid UUID
             * @return const Node* 
             */
            virtual const Node* NodeForUUID(const std::string& uuid) = 0;

            /**
             * @brief Destroy the Endpoint object
             * 
             */
            virtual ~Endpoint() { }
    };
}