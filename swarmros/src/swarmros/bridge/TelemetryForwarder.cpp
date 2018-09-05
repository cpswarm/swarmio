#include <swarmros/bridge/TelemetryForwarder.h>
#include <swarmros/bridge/MessageMismatchException.h>
#include <swarmros/introspection/VariantMessage.h>

using namespace swarmros;
using namespace swarmros::bridge;

void TelemetryForwarder::UpdateReceived(const introspection::VariantMessage::ConstPtr& message)
{
    if (message->GetType() == _message)
    {
        _telemetryService.SetValue(_name, message->GetValue());
    }
    else
    {
        throw MessageMismatchException("Invalid message type received from topic", _subscriber.getTopic(), _message, message->GetType());
    }
}

TelemetryForwarder::TelemetryForwarder(ros::NodeHandle& nodeHandle, const std::string& source, const std::string& message, swarmio::services::telemetry::Service& telemetryService, const std::string& name, bool includeInStatus)
    : _telemetryService(telemetryService), _name(name)
{
    // Register schema
    const auto& serializer = introspection::MessageSerializer::MessageSerializerForType(message, "swarmros");
    swarmio::data::discovery::Field fieldDescriptor;
    *fieldDescriptor.mutable_schema() = serializer.GetSchemaDescriptor(serializer.HasHeader() ? 1 : 0);
    _telemetryService.SetFieldDefinitionForKey(name, fieldDescriptor, includeInStatus);

    // Save message type
    _message = serializer.GetFullName();

    // Subscribe
    _subscriber = nodeHandle.subscribe<introspection::VariantMessage>(source, 1, &TelemetryForwarder::UpdateReceived, this);
}

TelemetryForwarder::~TelemetryForwarder()
{
    // Stop subscription
    _subscriber.shutdown();

    // Remove definition and value
    _telemetryService.RemoveFieldDefinitionForKey(_name);
    _telemetryService.RemoveValue(_name);
}