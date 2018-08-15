#pragma once

#include <swarmros/Exception.h>

namespace swarmros::introspection
{
    /**
     * @brief Exception thrown when the schema 
     *        does not match the variant.
     * 
     */
    class SchemaMismatchException : public Exception
    {
        private:

            /**
             * @brief The location where the exception has occurred
             * 
             */
            std::string _location;

        public:

            /**
             * @brief Constructor
             * 
             * @param message Message
             * @param location Location
             */
            SchemaMismatchException(const char* message, const std::string& location) 
                : Exception(message), _location(location) { }                        

            /**
             * @brief Get the location of the mismatch
             * 
             * @return const char* 
             */
            const char* location() const
            {
                return _location.c_str();
            }
    };
}
