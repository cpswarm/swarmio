#pragma once

#include <swarmio/Endpoint.h>
#include <swarmio/Mailbox.h>
#include <mutex>
#include <set>
#include <atomic>

namespace swarmio::transport
{
    /**
     * @brief An helper class for Endpoint implementations
     *        which implements some common basic functionality.
     */
    class SWARMIO_API BasicEndpoint : public Endpoint
    {
        private:

            /**
             * @brief Mutex used to synchronize access 
             *        to the list of mailboxes.
             */
            std::recursive_mutex _mutex;

            /**
             * @brief Container for registered mailboxes
             */
            std::set<Mailbox*> _mailboxes;

            /**
             * @brief Atomic counter for message identifiers
             * 
             */
            std::atomic<uint64_t> _counter = 1;

            /**
             * @brief Send a reply to a message that only contains a status code.
             * 
             * @param sender Sender of the original message
             * @param message The original message
             * @param error Error code to send back
             */
            void ReplyWithError(const Node* sender, const data::Message* message, data::Error error);

        protected:

            /**
             * @brief Called by this class to send serialized messages.
             *        Called with node set to nullptr to send a message
             *        to all members of the swarm.
             * 
             * @param data Raw message data
             * @param size Length of the message data
             * @param node Node the message will be sent to
             */
            virtual void Send(const void* data, size_t size, const Node* node) = 0;

            /**
             * @brief Called by implementations to deliver decoded messages.
             * 
             * @param sender Sender node
             * @param message The message itself
             * @return True if the message was handled
             */
            virtual bool ReceiveMessage(const Node* sender, const data::Message* message) noexcept;

            /**
             * @brief Called by implementations to deliver raw messages.
             * 
             * @param sender Sender node
             * @param data Raw message data
             * @param size Length of the message data
             * @return True if the message was handled 
             */
            virtual bool ReceiveMessage(const Node* sender, const void* data, size_t size) noexcept;

            /**
             * @brief Called when a new Node has been discovered.
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeWasDiscovered(const Node* node) noexcept;

            /**
             * @brief Called when a new Node has joined the group
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeDidJoin(const Node* node) noexcept;

            /**
             * @brief Called when a Node signals that it will leave.
             * 
             * @param node The node that has left
             */
            virtual void NodeWillLeave(const Node* node) noexcept;

            /**
             * @brief Register a Mailbox to receive messages.
             * 
             * May be called more than once, and has no effect
             * if the mailbox has already been registered.
             * 
             * @param mailbox Mailbox to register.
             */
            virtual void RegisterMailbox(Mailbox* mailbox) override;

            /**
             * @brief Unregister a Mailbox from receiving messages.
             * 
             * May be called more than once, and has no effect
             * if the mailbox has already been unregistered.
             * 
             * @param mailbox Mailbox to unregister.
             */
            virtual void UnregisterMailbox(Mailbox* mailbox) override;

            /**
             * @brief Relocate a mailbox to another in-memory location.
             * 
             * Supports thread-safe move operations on Mailboxes.
             * 
             * @param mailbox Mailbox to register.
             */
            virtual void ReplaceMailbox(Mailbox* oldMailbox, Mailbox* newMailbox) override;

        public:

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
            virtual void Send(data::Message* message, const Node* node) override;

            /**
             * @brief Set the message identifier for a message.
             * 
             * Thread safe.
             * 
             * @param message Message
             */
            virtual void Tag(data::Message* message) override;
    };
}