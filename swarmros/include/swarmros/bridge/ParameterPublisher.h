#pragma once

#include <swarmros/bridge/ParameterTarget.h>
#include <swarmros/introspection/MessageSerializer.h>

namespace swarmros::bridge
{
    /**
     * @brief An extension of ParameterTarget that publishes
     *        updates as custom messages.
     * 
     */
    class ParameterPublisher final : public ParameterTarget
    {
        private:

            /**
             * @brief Publisher
             * 
             */
            ros::Publisher _publisher;

            /**
             * @brief Serializer
             * 
             */
            const introspection::MessageSerializer* _serializer;

            /**
             * @brief Update the currently latched value
             * 
             * @param value New value
             */
            void UpdateValue(const swarmio::data::Variant& value);

        public:

            /**
             * @brief Construct a new ParameterPublisher object
             * 
             * @param nodeHandle Node handle
             * @param suffix Topic suffix
             * @param message Message type
             * @param keyvalueService Key-value service
             * @param name Global parameter name
             * @param parameter ROS parameter name
             * @param isWritable True if set requests should be accepted
             */
            ParameterPublisher(ros::NodeHandle& nodeHandle, const std::string& suffix, const std::string& message, swarmio::services::keyvalue::Service& keyvalueService, const std::string& name,  const std::string& parameter, bool isWritable);

            /**
             * @brief Set the current value of the target
             * 
             * @param value New value
             */
            virtual void Set(const std::string& path, const swarmio::data::Variant& value) override;
    };
}

