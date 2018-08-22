#include <swarmros/bridge/EventPublisher.h>
#include <swarmros/bridge/MessageMismatchException.h>

using namespace swarmros;
using namespace swarmros::bridge;

EventPublisher::EventPublisher(ros::NodeHandle& nodeHandle, const std::string& suffix, const std::string& message, swarmio::services::event::Service& eventService, const std::string& name)
    : _eventService(eventService), _name(name)
{
    // Register schema
    _serializer = &introspection::MessageSerializer::MessageSerializerForType(message, "swarmros");

    // Construct topic name
    auto topic = "events/" + suffix;

    // Check message format
    if (!EventMessage::IsEventSerializer(*_serializer))
    {
        throw MessageMismatchException("Message type has no valid event header", topic, "swarmros/EventHeader", _serializer->GetFullName());
    }

    // Create publisher
    ros::AdvertiseOptions options(topic, 32, _serializer->GetHash(), _serializer->GetFullName(), _serializer->GetCanonicalDefinition());
    _publisher = nodeHandle.advertise(options);

    // Start handline
    _eventService.RegisterHandler(name, this);
}

void EventPublisher::EventWasTriggered(const swarmio::Node* node, const swarmio::data::event::Notification& event)
{
    if (event.name() == _name)
    {
        EventMessage message;

        // Set type
        message.SetType(_serializer->GetFullName());

        // Fill header                
        message.SetName(event.name());
        message.SetNode(node->GetUUID());

        // Fill parameters
        message.GetMutableParameters().mutable_pairs()->insert(event.parameters().begin(), event.parameters().end());

        // Publish message
        _publisher.publish(message);
    }
    else
    {
        throw Exception("Trying to deliver unknown event");
    }
}

swarmio::data::discovery::Schema EventPublisher::DescribeEvent(const std::string& name)
{
    if (name == _name)
    {
        return _serializer->GetSchemaDescriptor(_serializer->HasHeader() ? 2 : 1);
    }
    else
    {
        throw Exception("Trying to query event descriptor for unknown event");
    }
}

EventPublisher::~EventPublisher()
{
    _eventService.UnregisterHandler(_name);
}