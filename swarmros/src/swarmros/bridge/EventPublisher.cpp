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

    // Check header
    const auto& headerType = _serializer->GetHeaderMessageSerializer().GetFullName();
    if (headerType != "swarmros/EventHeader")
    {
        throw MessageMismatchException("Invalid header type for event", topic, "swarmros/EventHeader", headerType);
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
        introspection::HeadedMessage message;
        auto& header = *message.GetMutableHeader().mutable_pairs();
        auto& content = *message.GetMutableContent().mutable_pairs();

        // Set type
        message.SetType(_serializer->GetFullName());

        // Fill header                
        header["node"].set_string_value(node->GetUUID());
        header["name"].set_string_value(event.name());;

        // Fill parameters
        content.insert(event.parameters().begin(), event.parameters().end());

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
        return _serializer->GetContentSchemaDescriptor();
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