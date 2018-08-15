#include <swarmio/services/event/Service.h>
#include <g3log/g3log.hpp>

using namespace swarmio;
using namespace swarmio::services;
using namespace swarmio::services::event;

void Service::Trigger(Endpoint* endpoint, const data::event::Notification& event)
{
    // Sanity checks
    CHECK(endpoint != nullptr) << "No endpoint specified";

    // Build message
    data::Message request;
    request.mutable_ev_notification()->CopyFrom(event);

    // Log outgoing event
    LOG(DBUG) << "Event '" << event.name() << "' will be triggered globally";

    // Send message
    endpoint->Send(&request, nullptr);
}

ErrorAwaiter Service::Trigger(Endpoint* endpoint, const data::event::Notification& event, const Node* node)
{
    // Sanity checks
    CHECK(endpoint != nullptr) << "No endpoint specified";
    CHECK(node != nullptr) << "No target node specified";

    // Build message
    data::Message request;
    request.mutable_header()->set_reliability(data::Reliability::ACK_REQUESTED);
    request.mutable_ev_notification()->CopyFrom(event);

    // Log outgoing event
    LOG(DBUG) << "Event '" << event.name() << "' will be triggered on node [" << node->GetUUID() << "]";

    // Send and await response
    endpoint->Tag(&request);
    ErrorAwaiter awaiter(endpoint, request.header().identifier());
    endpoint->Send(&request, node);
    return awaiter;
}

bool Service::ReceiveMessage(const Node* sender, const data::Message* message)
{
    // Sanity checks
    CHECK(sender != nullptr) << "Sender address missing";
    CHECK(message != nullptr) << "Message is missing";

    // Check type
    if (message->content_case() == data::Message::ContentCase::kEvNotification)
    {
        // Log incoming event
        LOG(DBUG) << "Event '" << message->ev_notification().name() << "' was received";

        // Find handler
        std::lock_guard<std::mutex> guard(_mutex);
        auto handler = _handlers.find(message->ev_notification().name());
        if (handler != _handlers.end())
        {
            handler->second->EventWasTriggered(sender, message->ev_notification());
            return true;
        }
        else
        {
            LOG(INFO) << "Event '" << message->ev_notification().name() << "' was not handled by any of the registered event handlers";
            return false;
        }
    }
    else
    {
        return false;
    }
}

void Service::RegisterHandler(const std::string& name, Handler* handler)
{
    // Sanity checks
    CHECK(handler != nullptr) << "Nullptr was supplied in shared pointer";

    // Add to map
    std::lock_guard<std::mutex> guard(_mutex);
    if (_handlers.find(name) == _handlers.end())
    {
        _handlers[name] = handler;
    }
    else
    {
        throw Exception("Event handler for this event is already registered");
    }
}

void Service::UnregisterHandler(const std::string& name)
{
    // Remove from map
    std::lock_guard<std::mutex> guard(_mutex);
    _handlers.erase(name);
}

void Service::DescribeService(data::discovery::Response& descriptor)
{
    std::lock_guard<std::mutex> guard(_mutex);

    // Collect information from handlers
    auto& fields = *descriptor.mutable_event_schema()->mutable_fields();
    for (auto handler : _handlers)
    {
        *fields[handler.first].mutable_schema() = handler.second->DescribeEvent(handler.first);
    }
}
