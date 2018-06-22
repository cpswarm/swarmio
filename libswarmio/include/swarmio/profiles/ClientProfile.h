#pragma once

#include <swarmio/profiles/Profile.h>
#include <swarmio/services/discovery/Observer.h>

namespace swarmio::profiles 
{
    /**
     * @brief Service profile for non-member clients
     * 
     */
    class SWARMIO_API ClientProfile : public Profile, public services::discovery::Observer
    {
        public:

            /**
             * @brief Construct a new ClientProfile
             * 
             * @param endpoint Endpoint
             */
            ClientProfile(Endpoint* endpoint)
                : Profile(endpoint, true)
            {
                GetDiscoveryService().RegisterObserver(this);
            }

            /**
             * @brief Receives notifications when discovery information becomes available
             * 
             * @param node Node
             * @param response Discovery response
             */
            virtual void CachedDiscoveryResponseWasUpdated(const Node* node, const data::discovery::Response& response) override { }

            /**
             * @brief Destructor
             * 
             */
            virtual ~ClientProfile() override
            {
                GetDiscoveryService().UnregisterObserver(this);
            }
    };
}