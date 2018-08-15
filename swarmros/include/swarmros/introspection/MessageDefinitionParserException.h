#pragma once

#include <swarmros/Exception.h>

namespace swarmros::introspection
{
    /**
     * @brief Exception thrown while parsing 
     *        a message definition file.
     * 
     */
    class MessageDefinitionParserException : public Exception
    {
        private:

            /**
             * @brief Path of the file
             * 
             */
            std::string _path;

            /**
             * @brief Affected line number
             * 
             */
            uint32_t _line;

        public:

            /**
             * @brief Constructor
             * 
             * @param message Message
             * @param path Path
             * @param line Line
             */
            MessageDefinitionParserException(const char* message, const std::string& path, uint32_t line) 
                : Exception(message), _path(path), _line(line) { }                        

            /**
             * @brief Get the path of the definition file
             * 
             * @return const char* 
             */
            const char* path() const
            {
                return _path.c_str();
            }

            /**
             * @brief Get the affected line
             * 
             * @return uint32_t
             */
            uint32_t line() const
            {
                return _line;
            }
    };
}
