#pragma once

#include <swarmros/bridge/Pylon.h>
#include <swarmros/introspection/AnyMessage.h>
#include <swarmio/services/telemetry/Service.h>
#include <ros/ros.h>

namespace swarmros::bridge
{
    /**
     * @brief A ROS topic subscriber that bridges 
     *        telemetry data to ROS topics.
     * 
     */
    class TelemetryForwarder final : public Pylon
    {
        private:

            /**
             * @brief ROS topic subscriber
             * 
             */
            ros::Subscriber _subscriber;

            /**
             * @brief Message type
             * 
             */
            std::string _message;
            
            /**
             * @brief Telemetry service
             * 
             */
            swarmio::services::telemetry::Service& _telemetryService;

            /**
             * @brief Telemetry key
             * 
             */
            std::string _name;

            /**
             * @brief Called whenever the topic is updated
             * 
             * @param message Message
             */
            void UpdateReceived(const introspection::AnyMessage::ConstPtr& message);

        public:

            /**
             * @brief Construct a new TelemetryForwarder object
             * 
             * @param nodeHandle Node handle
             * @param source ROS topic
             * @param message Message type
             * @param telemetryService Telemetry service
             * @param name Telemetry key
             * @param includeInStatus Include in status broadcast
             */
            TelemetryForwarder(ros::NodeHandle& nodeHandle, const std::string& source, const std::string& message, swarmio::services::telemetry::Service& telemetryService, const std::string& name, bool includeInStatus);

            /**
             * @brief Destroy the TelemetryForwarder object
             * 
             */
            virtual ~TelemetryForwarder() override;
    };
}

