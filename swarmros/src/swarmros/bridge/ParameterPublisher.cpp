#include <swarmros/bridge/ParameterPublisher.h>
#include <swarmros/introspection/VariantMessage.h>

using namespace swarmros;
using namespace swarmros::bridge;

ParameterPublisher::ParameterPublisher(ros::NodeHandle& nodeHandle, const std::string& suffix, const std::string& message, swarmio::services::keyvalue::Service& keyvalueService, const std::string& name,  const std::string& parameter, bool isWritable)
    : ParameterTarget(nodeHandle, keyvalueService, name, parameter, isWritable)
{
    // Register schema
    _serializer = &introspection::MessageSerializer::MessageSerializerForType(message, "swarmros");

    // Create publisher
    ros::AdvertiseOptions options("bridge/parameters/" + suffix, 1, _serializer->GetHash(), _serializer->GetFullName(), _serializer->GetCanonicalDefinition());
    options.latch = true;
    _publisher = nodeHandle.advertise(options);

    // Update value
    UpdateValue(Get(name));
}

void ParameterPublisher::Set(const std::string& path, const swarmio::data::Variant& value)
{
    // Call superclass implementation
    ParameterTarget::Set(path, value);

    // Send upate
    UpdateValue(value);
}

void ParameterPublisher::UpdateValue(const swarmio::data::Variant& value)
{
    // Construct message
    introspection::VariantMessage message(_serializer->GetFullName(), value);

    // Publish message
    _publisher.publish(message);
}