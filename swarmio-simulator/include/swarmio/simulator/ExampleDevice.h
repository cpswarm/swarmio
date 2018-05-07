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
             * @brief Destroy the Example Device object
             * 
             */
            ~ExampleDevice();
    };
}

