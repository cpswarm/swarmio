#pragma once

#include <swarmros/bridge/Pylon.h>
#include <swarmio/profiles/MemberProfile.h>
#include <ros/ros.h>
#include <map>
#include <list>
#include <memory>

namespace swarmros::bridge
{
    /**
     * @brief Represents a collection of standard
     *        services that can be used to bridge 
     *        a ROS based member to the swarm.
     * 
     */
    class Node final : private swarmio::profiles::MemberProfile
    {
        private:

            /**
             * @brief Active bridging services
             * 
             */
            std::list<std::unique_ptr<Pylon>> _pylons;

            /**
             * @brief Node handle
             * 
             */
            ros::NodeHandle _nodeHandle;
        

            /**
             * @brief The publisher for the nodes topic
             * 
             */
            ros::Publisher _nodesPublisher;

            /**
             * @brief Publish a generic info update for a node
             * 
             * @param node Node
             */
            void PublishUpdateForNode(const swarmio::Node* node);

        public:

            /**
             * @brief Construct a new Node object
             * 
             * @param endpoint Endpoint to use
             */
            Node(swarmio::Endpoint* endpoint);

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
             * @brief Forward telemetry data to the swarm
             * 
             * @param source Source ROS topic
             * @param message ROS message type
             * @param name Telemetry name
             * @param includeInStatus Include in status broadcast
             */
            void ForwardTelemetry(const std::string& source, const std::string& message, const std::string& name, bool includeInStatus);

            /**
             * @brief Forward events from a topic to the swarm
             * 
             * @param source Source ROS topic
             * @param message ROS message type
             */
            void ForwardEvent(const std::string& source, const std::string& message);

            /**
             * @brief Forward events from the swarm to a ROS topic
             * 
             * @param suffix Local topic suffix
             * @param message ROS message type
             * @param name Global event name
             */
            void PublishEvent(const std::string& suffix, const std::string& message, const std::string& name);

            /**
             * @brief Bridge and publish a parameter between ROS and the swarm
             * 
             * @param suffix Local topic suffix
             * @param message ROS message type
             * @param name Global parameter name
             * @param parameter ROS parameter name
             * @param isWritable True if set requests are accepted
             */
            void PublishParameter(const std::string& suffix, const std::string& message, const std::string& name, const std::string& parameter, bool isWritable);

            /**
             * @brief Bridge a parameter between the ROS parameter server and the swarm
             * 
             * @param name Global parameter name
             * @param parameter ROS parameter name
             * @param isWritable True if set requests are accepted
             */
            void BridgeParameter(const std::string& name, const std::string& parameter, bool isWritable);

            /**
             * @brief Called when a new Node has been discovered.
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeWasDiscovered(const swarmio::Node* node) noexcept override;

            /**
             * @brief Called when a new Node has joined the group
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeDidJoin(const swarmio::Node* node) noexcept override;

            /**
             * @brief Called when a Node signals that it will leave.
             * 
             * @param node The node that has left
             */
            virtual void NodeWillLeave(const swarmio::Node* node) noexcept override;
    };
}

