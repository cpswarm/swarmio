#pragma once

#include <swarmros/Exception.h>

namespace swarmros::bridge
{
    /**
     * @brief Exception thrown when the expected type of a
     *        message does not match the actual type.
     * 
     */
    class MessageMismatchException : public Exception
    {
        private:

            /**
             * @brief Topic
             * 
             */
            std::string _topic;

            /**
             * @brief The expected type of the variant
             * 
             */
            std::string _expectedMessage;

            /**
             * @brief The actual type of the variant
             * 
             */
            std::string _actualMessage;

        public:

            /**
             * @brief Constructor
             * 
             * @param message Message
             * @param topic Topic
             * @param expectedMessage Expected message type
             * @param actualMessage Actual message type
             */
            MessageMismatchException(const char* message, const std::string& topic, const std::string& expectedMessage, const std::string& actualMessage) 
                : Exception(message), _topic(topic), _expectedMessage(expectedMessage), _actualMessage(actualMessage) { }       

            /**
             * @brief Get the expected message type
             * 
             * @return const char*
             */
            const char* expected_message() const
            {
                return _expectedMessage.c_str();
            }                        

            /**
             * @brief Get the actual message type
             * 
             * @return const char*
             */
            const char* actual_message() const
            {
                return _actualMessage.c_str();
            }

            /**
             * @brief Get the actual message type
             * 
             * @return const char*
             */
            const char* topic() const
            {
                return _topic.c_str();
            }
    };
}
