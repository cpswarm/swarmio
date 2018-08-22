#pragma once

#include <swarmros/bridge/Pylon.h>
#include <swarmros/bridge/EventMessage.h>
#include <swarmio/Endpoint.h>
#include <ros/ros.h>

namespace swarmros::bridge
{
    /**
     * @brief A ROS topic subscriber that bridges 
     *        events from ROS topics to the swarm.
     * 
     */
    class EventForwarder final : public Pylon
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
            swarmio::Endpoint* _endpoint;

            /**
             * @brief Called whenever the topic is updated
             * 
             * @param message Message
             */
            void EventReceived(const EventMessage::ConstPtr& message);

        public:

            /**
             * @brief Construct a new EventForwarder object
             * 
             * @param nodeHandle Node handle
             * @param source ROS topic
             * @param message Message type
             * @param endpoint Endpoint
             */
            EventForwarder(ros::NodeHandle& nodeHandle, const std::string& source, const std::string& message, swarmio::Endpoint* endpoint);
    };
}

