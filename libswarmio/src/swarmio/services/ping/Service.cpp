#include <swarmio/services/ping/Service.h>
#include <swarmio/data/Message.pb.h>
#include <g3log/g3log.hpp>
#include <chrono>

using namespace swarmio;
using namespace swarmio::services;
using namespace swarmio::services::ping;

TimingAwaiter Service::Ping(Endpoint* endpoint, const Node* node, size_t size)
{
    // Build message
    data::Message message;
    message.mutable_echo()->mutable_payload()->resize(size, 'P');

    // Log outgoing message
    LOG(DBUG) << "A ping request will be sent to node [" << node->GetUUID() << "]";

    // Create awaiter and send message
    endpoint->Tag(&message);
    TimingAwaiter awaiter(endpoint, message.header().identifier());
    endpoint->Send(&message, node);
    return awaiter;
}

bool Service::ReceiveMessage(const Node* sender, const data::Message* message)
{
    // Sanity checks
    CHECK(sender != nullptr) << "Sender address missing";
    CHECK(message != nullptr) << "Message is missing";

    // Check message type
    if (message->content_case() == data::Message::ContentCase::kEcho &&
        message->header().reply_to() == 0)
    {
        // Log message
        LOG(DBUG) << "A ping request was received from node [" << sender->GetUUID() << "] and will be answered";

        // Build message
        data::Message reply;
        reply.mutable_header()->set_reply_to(message->header().identifier());
        reply.mutable_echo()->set_payload(message->echo().payload());

        // Send message
        GetEndpoint()->Send(&reply, sender);

        // Mark as handled
        return true;
    }
    else
    {
        return false;
    }
}

void Service::DescribeService(data::discovery::Response& descriptor)
{
    // Mark the ECHO service as available
    descriptor.set_echo_enabled(true);
}