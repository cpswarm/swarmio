#pragma once

#include <swarmio/Exception.h>

namespace swarmros
{
    /**
     * @brief Exception base class for swarmros exceptions
     * 
     */
    class Exception : public swarmio::Exception
    {
        public:

            /**
             * @brief Inherit constructors
             * 
             */
            using swarmio::Exception::Exception;
    };
}
