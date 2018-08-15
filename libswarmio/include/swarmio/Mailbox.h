#pragma once

#include <swarmio/Node.h>
#include <swarmio/Endpoint.h>
#include <swarmio/data/Message.pb.h>

namespace swarmio
{
    /**
     * @brief Abstract base class for Mailbox implementations.
     * 
     */
    class SWARMIO_API Mailbox
    {
        private:

            /**
             * @brief Associated endpoint
             * 
             */
            Endpoint* _endpoint;

        protected:

            /**
             * @brief Construct a disconnected Mailbox
             * 
             */
            Mailbox()
                : _endpoint(nullptr) { }

            /**
             * @brief Construct a new Mailbox object
             * 
             * @param endpoint Endpoint
             */
            Mailbox(Endpoint* endpoint)
            {
                _endpoint = endpoint;
            }

            /**
             * @brief Called when the last constructor has finished its job.
             * 
             */
            void FinishConstruction()
            {
                if (_endpoint != nullptr)
                {
                    _endpoint->RegisterMailbox(this);
                }
            }
            
            /**
             * @brief Move a Mailbox object
             * 
             */
            Mailbox(Mailbox&& other)
            {
                // Copy over endpoint reference
                _endpoint = other.GetEndpoint();
            }

            /**
             * @brief Called when message handling should be 
             *        passed onto the new instance
             * 
             * @param other New Mailbox
             */
            void FinishMovingTo(Mailbox* other)
            {
                if (_endpoint != nullptr)
                {
                    _endpoint->ReplaceMailbox(this, other);
                    _endpoint = nullptr;
                }
            }

        public:

            /**
             * @brief Remove copy constructor
             * 
             */
            Mailbox(const Mailbox&) = delete;

            /**
             * @brief Remove assignment operator
             * 
             * @return Awaiter& 
             */
            Mailbox& operator=(const Mailbox&) = delete;

            /**
             * @brief Delivery point of all messages
             * 
             * @param sender The node that has sent the message
             * @param message The message itself
             * @returns True if the message had been processed and should 
             *          not be forwarded to other mailboxes
             */
            virtual bool ReceiveMessage(const Node* sender, const data::Message* message) { return false; }

            /**
             * @brief Called when a new Node has been discovered.
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeWasDiscovered(const Node* node) noexcept { }

            /**
             * @brief Called when a new Node has joined the group
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeDidJoin(const Node* node) noexcept { }

            /**
             * @brief Called when a Node signals that it will leave.
             * 
             * @param node The node that has left
             */
            virtual void NodeWillLeave(const Node* node) noexcept { }

            /**
             * @brief Called when the mailbox is attached to an already running
             *        endpoint or if the attached endpoint has just started.
             * 
             */
            virtual void MailboxWasConnected() noexcept { }

            /**
             * @brief Called right before the mailbox is disconnected from its
             *        endpoint or if the attached endpoint is about to stop.
             * 
             */
            virtual void MailboxWillBeDisconnected() noexcept { }

            /**
             * @brief Get the associated Endpoint
             * 
             * @return Endpoint* 
             */
            Endpoint* GetEndpoint()
            {
                return _endpoint;
            }

            /**
             * @brief Disconnect this Mailbox from the Endpoint
             * 
             */
            virtual void Disconnect()
            {
                if (_endpoint != nullptr)
                {
                    _endpoint->UnregisterMailbox(this);
                    _endpoint = nullptr;
                }
            }

            /**
             * @brief Destroy the Mailbox object
             */
            virtual ~Mailbox() 
            {
                Disconnect();
            }
    };
}