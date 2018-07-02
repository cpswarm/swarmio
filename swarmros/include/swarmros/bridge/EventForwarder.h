#pragma once

#include <swarmio/services/event/Handler.h>
#include <ros/ros.h>
#include <map>

namespace swarmros::bridge
{
    /**
     * @brief A Handler implementation that forwards
     *        matching events to a ROS topic.
     * 
     */
    class EventForwarder final : public swarmio::services::event::Handler
    {
        private:

            /**
             * @brief Publisher
             * 
             */
            ros::Publisher _thisPublisher;

            /**
             * @brief Publisher
             * 
             */
            ros::Publisher _allPublisher;

            /**
             * @brief Event name
             * 
             */
            std::string _name;

            /**
             * @brief Parameters
             * 
             */
            std::map<std::string, swarmio::data::discovery::Type> _parameters;

        public:

            /**
             * @brief Construct a new EventForwarder
             * 
             * @param nodeHandle Node handle
             * @param allPublisher All events topic handle
             * @param name Event name
             * @param parameters Parameter description
             */
            EventForwarder(ros::NodeHandle& nodeHandle, ros::Publisher& allPublisher, const std::string& name, const std::map<std::string, swarmio::data::discovery::Type>& parameters);

            /**
             * @brief Publish the event in the ROS topic.
             * 
             * @param node Source node
             * @param event Event
             */
            virtual void EventWasTriggered(const swarmio::Node* node, const swarmio::data::event::Notification& event) override;

            /**
             * @brief Describe the event based on the configuration of this instance.
             * 
             * @param descriptor Event descriptor
             */
            virtual void DescribeEvent(const std::string& name, swarmio::data::event::Descriptor& descriptor) override;

            /**
             * @brief Get the name of the event
             * 
             * @return const std::string& 
             */
            const std::string& GetName() const
            {
                return _name;
            }
    };
}

