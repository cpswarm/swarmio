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

    // Check header
    const auto& headerType = serializer.GetHeaderMessageSerializer().GetFullName();
    if (headerType != "swarmros/EventHeader")
    {
        throw MessageMismatchException("Invalid header type for event", source, "swarmros/EventHeader", headerType);
    }

    // Save message type
    _message = serializer.GetFullName();

    // Subscribe
    _subscriber = nodeHandle.subscribe<introspection::HeadedMessage>(source, 1, &EventForwarder::EventReceived, this);
}

void EventForwarder::EventReceived(const introspection::HeadedMessage::ConstPtr& message)
{
    if (message->GetType() == _message)
    {
        // Fetch header fields
        const auto& header = message->GetHeader().pairs();
        const auto& content = message->GetContent().pairs();

        // Prepare notification
        swarmio::data::event::Notification notification;
        notification.set_name(header.at("name").string_value());
        notification.mutable_parameters()->insert(content.begin(), content.end());

        // Send message
        auto uuid = header.at("node").string_value();
        if (uuid.size() == 0)
        {
            swarmio::services::event::Service::Trigger(_endpoint, notification);
        }
        else
        {
            auto node = _endpoint->NodeForUUID(uuid);
            swarmio::services::event::Service::Trigger(_endpoint, notification, node);
        }
    }
    else
    {
        throw MessageMismatchException("Invalid message type received from topic", _subscriber.getTopic(), _message, message->GetType());
    }
}