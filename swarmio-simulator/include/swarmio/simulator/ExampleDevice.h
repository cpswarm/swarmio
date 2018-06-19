#pragma once

#include <swarmio/profiles/MemberProfile.h>
#include <swarmio/simulator/FauxEventHandler.h>
#include <swarmio/simulator/InMemoryParameter.h>
#include <list>

namespace swarmio::simulator 
{
    /**
     * @brief Represents a collection of standard
     *        services that can be used to simulate 
     *        an ordinary device.
     * 
     */
    class ExampleDevice : private profiles::MemberProfile
    {
        private:

            /**
             * @brief Number of incoming messages
             * 
             */
            uint32_t _messageCounter = 0;

            /**
             * @brief Event handlers
             * 
             */
            std::list<FauxEventHandler*> _eventHandlers;

            /**
             * @brief Parameters
             * 
             */
            std::list<InMemoryParameter*> _parameters;

        public:

            /**
             * @brief Construct a new Example Device object
             * 
             * @param endpoint Endpoint to use
             */
            ExampleDevice(Endpoint* endpoint)
                : MemberProfile(endpoint) { }

            /**
             * @brief Add an event handler and register it.
             * 
             * @param eventHandler Event handler
             */
            void AddFauxEventHandler(FauxEventHandler* eventHandler);

            /**
             * @brief Add an In-Memory Parameter and register it.
             * 
             * @param parameter Parameter
             */
            void AddInMemoryParameter(InMemoryParameter* parameter);

            /**
             * @brief Update a telemetry value
             * 
             * @param key Key
             * @param value Value
             */
            void SetTelemetryValue(const std::string& key, const data::Variant& value);

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
             * @brief Destroy the Example Device object
             * 
             */
            ~ExampleDevice();
    };
}

