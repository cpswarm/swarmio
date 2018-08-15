#include <swarmros/bridge/DebugSink.h>
#include <g3log/loglevels.hpp>
#include <ros/console.h>

using namespace swarmros;
using namespace swarmros::bridge;

void DebugSink::ReceiveLogMessage(g3::LogMessageMover logEntry)
{
    const auto& entry = logEntry.get();
    if (entry._level == DBUG)
    {
        ROS_DEBUG_STREAM(entry.message());
    }
    else if (entry._level == INFO)
    {
        ROS_INFO_STREAM(entry.message());
    }
    else if (entry._level == WARNING)
    {
        ROS_WARN_STREAM(entry.message());
    }
    else if (entry._level == FATAL)
    {
        ROS_FATAL_STREAM(entry.message());
    }
}