#pragma once

#include <swarmio/services/telemetry/Service.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

namespace swarmros::bridge
{
    /**
     * @brief A ROS topic subscriber that bridges 
     *        telemetry data to ROS topics.
     * 
     */
    class TelemetrySource final
    {
        private:
            
            /**
             * @brief Telemetry service
             * 
             */
            swarmio::services::telemetry::Service* _telemetryService;
        
            /**
             * @brief ROS topic subscriber
             * 
             */
            ros::Subscriber _subscriber;

            /**
             * @brief Telemetry key
             * 
             */
            std::string _name;

            /**
             * @brief ROS topic path
             * 
             */
            std::string _path;

            /**
             * @brief Callback for Bool messages
             * 
             * @param value Value
             */
            void ReceiveBoolUpdate(const std_msgs::Bool& message);

            /**
             * @brief Callback for Bool messages
             * 
             * @param value Value
             */
            void ReceiveFloat64Update(const std_msgs::Float64& message);

            /**
             * @brief Callback for Bool messages
             * 
             * @param value Value
             */
            void ReceiveInt32Update(const std_msgs::Int32& message);

            /**
             * @brief Callback for Bool messages
             * 
             * @param value Value
             */
            void ReceiveStringUpdate(const std_msgs::String& message);

        public:

            /**
             * @brief Construct a new TelemetrySource object
             * 
             * @param nodeHandle ROS node handle
             * @param telemetryService Telemetry service
             * @param name Telemetry key
             * @param path ROS topic path
             * @param type Data type
             */
           TelemetrySource(ros::NodeHandle& nodeHandle, swarmio::services::telemetry::Service* telemetryService, const std::string& name, const std::string& path, swarmio::data::discovery::Type type);

            /**
             * @brief Get telemetry key
             * 
             * @return const std::string& Name
             */
            const std::string& GetName() const
            {
                return _name;
            }

            /**
             * @brief Get ROS topic path
             * 
             * @return const std::string& Path
             */
            const std::string& GetPath() const
            {
                return _path;
            }

            /**
             * @brief Destructor
             * 
             */
           ~TelemetrySource();
    };
}

