#pragma once

#include <exception>
#include <string>

namespace swarmio
{
    /**
     * @brief Exception class thrown by all library classes.
     * 
     */
    class SWARMIO_API Exception : public std::exception
    {
        private:
        
            /**
             * @brief The buffer where the message is stored
             * 
             */
            std::string _message;

        public:

            /**
             * @brief Construct a new Exception
             * 
             * @param message Human readable error message
             */
            Exception(const char* message)
                : _message(message) { }

            /**
             * @brief Get the error message
             * 
             * @return const char* 
             */
            const char* what() const override
            {
                return _message.c_str();
            }
    };
}