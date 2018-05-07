#pragma once

#include <swarmio/profiles/Profile.h>

namespace swarmio::profiles 
{
    /**
     * @brief Service profile for non-member clients
     * 
     */
    class SWARMIO_API ClientProfile : public Profile
    {
        public:

            /**
             * @brief Construct a new ClientProfile
             * 
             * @param endpoint 
             */
            ClientProfile(Endpoint* endpoint)
                : Profile(endpoint, true) { }
    };
}