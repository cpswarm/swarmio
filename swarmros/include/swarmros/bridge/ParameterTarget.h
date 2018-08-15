#pragma once

#include <swarmros/bridge/Pylon.h>
#include <swarmio/services/keyvalue/Service.h>
#include <swarmio/services/keyvalue/Target.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace swarmros::bridge
{
    /**
     * @brief A Target implementation that uses
     *        the ROS parameter server.
     * 
     */
    class ParameterTarget : public Pylon, public swarmio::services::keyvalue::Target
    {
        private:

            /**
             * @brief Node handle
             * 
             */
            ros::NodeHandle _nodeHandle;

            /**
             * @brief Parameter name
             * 
             */
            std::string _parameter;

            /**
             * @brief Key-value service
             * 
             */
            swarmio::services::keyvalue::Service& _keyvalueService;

            /**
             * @brief Discoverable keyvalue name
             * 
             */
            std::string _path;

            /**
             * @brief True if the property can be set
             * 
             */
            bool _isWritable;

            /**
             * @brief Current value
             * 
             */
            swarmio::data::Variant _value;

            /**
             * @brief Convert an XmlRpc value to a variant
             * 
             * @param value Value
             * @return swarmio::data::Variant 
             */
            static swarmio::data::Variant VariantFromXmlRpcValue(XmlRpc::XmlRpcValue& value);

            /**
             * @brief Convert a variant to an XmlRpc value
             * 
             * @param value Value
             * @return XmlRpc::XmlRpcValue 
             */
            static XmlRpc::XmlRpcValue XmlRpcValueFromVariant(const swarmio::data::Variant& value);

        public:

            /**
             * @brief Construct a new ParameterTarget object
             * 
             * @param nodeHandle Node handle
             * @param parameter Parameter name
             * @param keyvalueService Key-value service
             * @param path Parameter name
             * @param isWritable True if can be set remotely
             */
            ParameterTarget(ros::NodeHandle& nodeHandle, swarmio::services::keyvalue::Service& keyvalueService, const std::string& name, const std::string& parameter, bool isWritable);
            
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
            virtual swarmio::data::discovery::Field GetFieldDescriptor(const std::string& path) const override;

            /**
             * @brief Determines whether the value can be written
             * 
             * @return True if Set operations are allowed
             */
            virtual bool CanWrite(const std::string& name) const noexcept override;

            /**
             * @brief Determines whether the value can be read
             * 
             * @return True if Get operations are allowed
             */
            virtual bool CanRead(const std::string& name) const noexcept override;

            /**
             * @brief Destructor
             * 
             */
            virtual ~ParameterTarget();
    };
}

