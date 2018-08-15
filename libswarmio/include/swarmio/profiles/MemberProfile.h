#pragma once

#include <swarmio/profiles/Profile.h>
#include <swarmio/services/event/Service.h>
#include <swarmio/services/keyvalue/Service.h>
#include <swarmio/services/telemetry/Service.h>

namespace swarmio::profiles 
{
    /**
     * @brief Service profile for swarm members
     * 
     */
    class MemberProfile : public Profile
    {
        protected:

            /**
             * @brief Key-Value service
             * 
             */
            swarmio::services::keyvalue::Service _keyvalueService;

            /**
             * @brief Event service
             * 
             */
            swarmio::services::event::Service _eventService;

        public:

            /**
             * @brief Construct a new MemberProfile
             * 
             * @param endpoint 
             */
            MemberProfile(Endpoint* endpoint)
                : Profile(endpoint, false), _keyvalueService(endpoint), _eventService(endpoint)
            {
                _discoveryService.RegisterDiscoverable(&_eventService);
                _discoveryService.RegisterDiscoverable(&_keyvalueService);
            }

             /**
             * @brief Get a reference for the Event service
             * 
             * @return swarmio::services::event::Service& 
             */
            swarmio::services::event::Service& GetEventService()
            {
                return _eventService;
            }

            /**
             * @brief Get a reference for the Key-Value service
             * 
             * @return swarmio::services::keyvalue::Service& 
             */
            swarmio::services::keyvalue::Service& GetKeyValueService()
            {
                return _keyvalueService;
            }
   
            /**
             * @brief Destroy the MemberProfile object
             * 
             */
            virtual ~MemberProfile()
            {
                _discoveryService.UnregisterDiscoverable(&_keyvalueService);
                _discoveryService.UnregisterDiscoverable(&_eventService);
            }
    };
}