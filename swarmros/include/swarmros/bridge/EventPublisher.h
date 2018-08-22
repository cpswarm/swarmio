#pragma once

#include <swarmros/bridge/Pylon.h>
#include <swarmros/bridge/EventMessage.h>
#include <swarmros/introspection/MessageSerializer.h>
#include <swarmio/services/event/Service.h>
#include <swarmio/services/event/Handler.h>
#include <ros/ros.h>

namespace swarmros::bridge
{
    /**
     * @brief A Handler implementation that publishes
     *        events from the swarm to a ROS topic.
     * 
     */
    class EventPublisher final : public Pylon, public swarmio::services::event::Handler
    {
        private:

            /**
             * @brief Publisher
             * 
             */
            ros::Publisher _publisher;

            /**
             * @brief Event name
             * 
             */
            std::string _name;

            /**
             * @brief Event name
             * 
             */
            const introspection::MessageSerializer* _serializer;

            /**
             * @brief Event service
             * 
             */
            swarmio::services::event::Service& _eventService;

        public:

            /**
             * @brief Construct a new EventPublisher object
             * 
             * @param nodeHandle Node handle
             * @param message Event message tyoe
             * @param eventService Event service
             * @param name Event name
             */
            EventPublisher(ros::NodeHandle& nodeHandle, const std::string& suffix, const std::string& message, swarmio::services::event::Service& eventService, const std::string& name);

            /**
             * @brief Publish the event in the ROS topic.
             * 
             * @param node Source nodel
             * @param event Event
             */
            virtual void EventWasTriggered(const swarmio::Node* node, const swarmio::data::event::Notification& event) override;

            /**
             * @brief Describe the event based on the configuration of this instance.
             * 
             * @param name Event name
             */
            virtual swarmio::data::discovery::Schema DescribeEvent(const std::string& name) override;

            /**
             * @brief Destructor
             * 
             */
            virtual ~EventPublisher() override;
    };
}

