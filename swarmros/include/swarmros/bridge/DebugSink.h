#pragma once

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>
#include <ros/console.h>

namespace swarmros::bridge 
{
    /**
     * @brief A sink for g3log that forwards messages 
     *        to the standard ROS logging mechanism.
     * 
     */
    class DebugSink final
    {
        public:

            /**
             * @brief Receives messages from g3log and
             *        forwards them to ROS.
             * 
             * @param logEntry Log message
             */
            void ReceiveLogMessage(g3::LogMessageMover logEntry);
    };
}