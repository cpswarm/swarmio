#pragma once

#include <g3log/g3log.hpp>
#include <list>
#include <string>
#include <mutex>

namespace swarmio::tool 
{
    class LogBuffer final 
    {
        private:

            /**
             * @brief Internal list of messages
             * 
             */
            std::list<g3::LogMessage> _messages;

            /**
             * @brief Mutex to protect the list of messages
             * 
             */
            std::mutex _mutex;

        public:

            /**
             * @brief Receive log messages
             * 
             * @param logEntry Log message
             */
            void ReceiveLogMessage(g3::LogMessageMover logEntry)
            {
                std::unique_lock<std::mutex> guard(_mutex);

                // Add to list                
                _messages.push_back(logEntry.get());

                // Remove one if we are over the limit
                if (_messages.size() > 15)
                {
                    _messages.pop_front();
                }
            }

            /**
             * @brief Get a copy of the list of messages
             * 
             * @return std::list<std::string> 
             */
            std::list<g3::LogMessage> GetMessages()
            {
                std::unique_lock<std::mutex> guard(_mutex);

                // Copy list
                return _messages;
            }
    };

}
