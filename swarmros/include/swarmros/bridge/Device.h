#pragma once

#include <swarmio/profiles/MemberProfile.h>
#include <swarmros/bridge/EventForwarder.h>
#include <swarmros/bridge/ParameterTarget.h>
#include <swarmros/bridge/TelemetrySource.h>
#include <ros/ros.h>
#include <list>

namespace swarmros::bridge
{
    /**
     * @brief Represents a collection of standard
     *        services that can be used to bridge 
     *        a ROS based member to the swarm.
     * 
     */
    class Device final : private swarmio::profiles::MemberProfile
    {
        private:

            /**
             * @brief Node handle
             * 
             */
            ros::NodeHandle _nodeHandle;

            /**
             * @brief Topic handle for events
             * 
             */
            ros::Publisher _eventPublisher;

            /**
             * @brief Event forwarders
             * 
             */
            std::list<EventForwarder> _eventForwarders;

            /**
             * @brief Parameter targets
             * 
             */
            std::list<ParameterTarget> _parameterTargets;

            /**
             * @brief Telemetry sources
             * 
             */
            std::list<TelemetrySource> _telemetrySources;

        public:

            /**
             * @brief Construct a new Device object
             * 
             * @param endpoint Endpoint to use
             */
            Device(swarmio::Endpoint* endpoint);

            /**
             * @brief Get ROS node handle
             * 
             * @return ros::NodeHandle& Node handle
             */
            ros::NodeHandle& GetNodeHandle()
            {
                return _nodeHandle;
            }

            /**
             * @brief Forward incoming events to a ROS topic.
             * 
             * @param name Event name
             * @param parameters Event parameters
             */
            void ForwardEvent(const std::string& name, const std::map<std::string, swarmio::data::discovery::Type>& parameters);

            /**
             * @brief Forward updates on a topic as telemetry.
             * 
             * @param name Telemetry key
             * @param path ROS topic path
             * @param type Data type
             */
            void ForwardTelemetry(const std::string& name, const std::string& path, swarmio::data::discovery::Type type);

            /**
             * @brief Connect a remote parameter with a ROS 
             *        topic and a parameter server entry.
             * 
             * @param name Parameter name
             * @param path ROS parameter path
             * @param isWritable True if Set requests are allowed
             * @param defaultValue Default value
             */
            void PublishParameter(const std::string& name, const std::string& path, bool isWritable, const swarmio::data::Variant& defaultValue);

            /**
             * @brief Destructor
             * 
             */
            virtual ~Device() override;
    };
}

