#pragma once

#include <swarmros/Exception.h>

namespace swarmros
{
    /**
     * @brief An exception that is expected to be caught and 
     *        translated into another exception.
     * 
     */
    class UnqualifiedException : public Exception
    {
        public:

            /**
             * @brief  Inherit constructors
             * 
             */
            using Exception::Exception;
    };
}
