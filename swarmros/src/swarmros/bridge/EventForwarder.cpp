#include <swarmros/bridge/EventForwarder.h>
#include <swarmros/bridge/MessageMismatchException.h>
#include <swarmio/services/event/Service.h>

using namespace swarmros;
using namespace swarmros::bridge;

EventForwarder::EventForwarder(ros::NodeHandle& nodeHandle, const std::string& source, const std::string& message, swarmio::Endpoint* endpoint)
        : _endpoint(endpoint)
{
    // Register schema
    const auto& serializer = introspection::MessageSerializer::MessageSerializerForType(message, "swarmros");

    // Save message type
    _message = serializer.GetFullName();

    // Check message format
    if (!EventMessage::IsEventSerializer(serializer))
    {
        throw MessageMismatchException("Message type has no valid event header", source, "swarmros/EventHeader", _message);
    }

    // Subscribe
    _subscriber = nodeHandle.subscribe<EventMessage>(source, 1, &EventForwarder::EventReceived, this);
}

void EventForwarder::EventReceived(const EventMessage::ConstPtr& message)
{
    if (message->GetType() == _message)
    {
        // Prepare notification
        swarmio::data::event::Notification notification;
        notification.set_name(message->GetName());
        notification.mutable_parameters()->insert(message->GetParameters().pairs().begin(), message->GetParameters().pairs().end());

        // Send message
        if (message->GetNode().empty())
        {
            swarmio::services::event::Service::Trigger(_endpoint, notification);
        }
        else
        {
            auto node = _endpoint->NodeForUUID(message->GetNode());
            swarmio::services::event::Service::Trigger(_endpoint, notification, node);
        }
    }
    else
    {
        throw MessageMismatchException("Invalid message type received from topic", _subscriber.getTopic(), _message, message->GetType());
    }
}