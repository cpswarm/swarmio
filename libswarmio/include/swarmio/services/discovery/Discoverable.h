#pragma once

#include <swarmio/Node.h>
#include <swarmio/data/discovery/Response.pb.h>

namespace swarmio::services::discovery 
{
    /**
     * @brief Interface for discoverable services
     * 
     */
    class SWARMIO_API Discoverable  
    {
        public:

            /**
             * @brief Add descriptors for the service to
             *        the discovery descriptor
             * 
             * @param descriptor The proposed response to load 
             *                   the description into
             */
            virtual void DescribeService(data::discovery::Response& descriptor) = 0;
    };
}