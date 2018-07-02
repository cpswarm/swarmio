#pragma once

#include <swarmio/services/keyvalue/Target.h>
#include <ros/ros.h>

namespace swarmros::bridge
{
    /**
     * @brief A Target implementation that uses
     *        the ROS parameter server. 
     * 
     */
    class ParameterTarget final : public swarmio::services::keyvalue::Target
    {
        private:

            /**
             * @brief Node handle
             * 
             */
            ros::NodeHandle _nodeHandle;

            /**
             * @brief Publisher
             * 
             */
            ros::Publisher _publisher;

            /**
             * @brief Discoverable parameter name
             * 
             */
            std::string _name;

            /**
             * @brief ROS parameter path
             * 
             */
            std::string _path;

            /**
             * @brief True if the property can be set
             * 
             */
            bool _isWritable;

            /**
             * @brief Data type
             * 
             */
            swarmio::data::Variant _defaultValue;

            /**
             * @brief Publish an update to the associated ROS topic
             * 
             * @param value Value
             */
            void PublishValue(const swarmio::data::Variant& value);

        public:

            /**
             * @brief Construct a new ParameterTarget object
             * 
             * @param nodeHandle ROS node handle
             * @param name Parameter name
             * @param path ROS parameter path
             * @param isWritable True if the parameter can be set
             * @param defaultValue Default value
             */
            ParameterTarget(ros::NodeHandle& nodeHandle, const std::string& name, const std::string& path, bool isWritable, const swarmio::data::Variant& defaultValue);
            
            /**
             * @brief Get the current value of the target parameter
             * 
             * @return swarmio::data::Variant Value
             */
            virtual swarmio::data::Variant Get(const std::string& path) override;

            /**
             * @brief Set the current value of the target
             * 
             * @param value New value
             */
            virtual void Set(const std::string& path, const swarmio::data::Variant& value) override;

            /**
             * @brief Get the data type of the target
             * 
             * @return swarmio::data::discovery::Type 
             */
            virtual swarmio::data::discovery::Type GetType(const std::string& name) const override;

            /**
             * @brief Determines whether the value can be written
             * 
             * @return True if Set operations are allowed
             */
            virtual bool CanWrite(const std::string& name) const noexcept override
            {
                return _isWritable;
            }

            /**
             * @brief Determines whether the value can be read
             * 
             * @return True if Get operations are allowed
             */
            virtual bool CanRead(const std::string& name) const noexcept override
            {
                return true;
            }

            /**
             * @brief Get parameter name
             * 
             * @return const std::string& Name
             */
            const std::string& GetName() const
            {
                return _name;
            }

            /**
             * @brief Get ROS parameter path
             * 
             * @return const std::string& Path
             */
            const std::string& GetPath() const
            {
                return _path;
            }
    };
}

