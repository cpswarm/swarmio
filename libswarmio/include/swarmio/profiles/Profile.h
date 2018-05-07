#pragma once

#include <swarmio/Endpoint.h>
#include <swarmio/Mailbox.h>
#include <swarmio/services/discovery/Service.h>
#include <swarmio/services/keyvalue/Service.h>
#include <swarmio/services/event/Service.h>
#include <swarmio/services/ping/Service.h>

namespace swarmio::profiles 
{
    /**
     * @brief Base class for profiles
     * 
     */
    class SWARMIO_API Profile : public Mailbox
    {
        protected:

            /**
             * @brief Discovery service
             * 
             */
            swarmio::services::discovery::Service _discoveryService;

            /**
             * @brief Ping service
             * 
             */
            swarmio::services::ping::Service _pingService;

            /**
             * @brief Construct a new Profile
             * 
             * @param endpoint Endpoint to use
             * @param performActiveDiscovery Enable active discovery 
             *                               for the Discovery service
             */
            Profile(Endpoint* endpoint, bool performActiveDiscovery)
                : Mailbox(endpoint), 
                _discoveryService(endpoint, performActiveDiscovery), 
                _pingService(endpoint) 
            {
                 _discoveryService.RegisterDiscoverable(&_pingService);
            }

        public:

            /**
             * @brief Get a reference for the Discovery service
             * 
             * @return swarmio::services::discovery::Service& 
             */
            swarmio::services::discovery::Service& GetDiscoveryService()
            {
                return _discoveryService;
            }

            /**
             * @brief Get a reference for the Ping service
             * 
             * @return swarmio::services::ping::Service& 
             */
            swarmio::services::ping::Service& GetPingService()
            {
                return _pingService;
            }

            /**
             * @brief Destroy the Profile object
             * 
             */
            virtual ~Profile()
            {
                _discoveryService.UnregisterDiscoverable(&_pingService);
            }
    };
}