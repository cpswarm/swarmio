#include <swarmio/services/keyvalue/Service.h>
#include <swarmio/Exception.h>
#include <g3log/g3log.hpp>

using namespace swarmio;
using namespace swarmio::services;
using namespace swarmio::services::keyvalue;

ValueAwaiter Service::Get(Endpoint* endpoint, const Node* node, const std::string& path)
{
    // Sanity checks
    CHECK(endpoint != nullptr) << "No endpoint specified";
    CHECK(node != nullptr) << "No target node specified";

    // Build message
    data::Message request;
    request.mutable_header()->set_reliability(data::Reliability::NACK_REQUESTED);
    request.mutable_kv_get_request()->set_key(path);

    // Log outgoing request
    LOG(DBUG) << "A GET request was sent for the remote path '" << path << "' on node [" << node->GetUUID() << "]";

    // Send and await response
    endpoint->Tag(&request);
    ValueAwaiter awaiter(endpoint, request.header().identifier(), path);
    endpoint->Send(&request, node);
    return awaiter;
}

ErrorAwaiter Service::Set(Endpoint* endpoint, const Node* node, const std::string& path, const data::Variant& value)
{
    // Sanity checks
    CHECK(endpoint != nullptr) << "No endpoint specified";
    CHECK(node != nullptr) << "No target node specified";

    // Build message
    data::Message request;
    request.mutable_header()->set_reliability(data::Reliability::ACK_REQUESTED);
    request.mutable_kv_set_request()->set_key(path);
    request.mutable_kv_set_request()->mutable_value()->CopyFrom(value);

    // Log outgoing request
    LOG(DBUG) << "A SET request was sent for the remote path '" << path << "' on node [" << node->GetUUID() << "]";

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

    // Forward to handler for message type
    switch (message->content_case())
    {
        case data::Message::ContentCase::kKvGetRequest:
            return HandleGetRequest(sender, message);

        case data::Message::ContentCase::kKvSetRequest:
            return HandleSetRequest(sender, message);

        default:
            return false;
    }
}

bool Service::HandleGetRequest(const Node* sender, const data::Message* message)
{
    std::unique_lock<std::mutex> guard(_mutex);

    // Check if target exists
    auto target = _targets.find(message->kv_get_request().key());
    if (target != _targets.end())
    {
        // Fetch value and unlock
        data::Message reply;
        reply.mutable_kv_get_response()->mutable_value()->CopyFrom(target->second->Get(message->kv_get_request().key()));
        guard.unlock();

        // Build the rest of the response and send message
        reply.mutable_header()->set_reply_to(message->header().identifier());
        reply.mutable_kv_get_response()->set_key(message->kv_get_request().key());
        GetEndpoint()->Send(&reply, sender);

        // Log incoming request
        LOG(DBUG) << "A GET request for the local path '" << message->kv_get_request().key() << "' from node [" << sender->GetUUID() << "] was handled";

        // Mark as handled
        return true;
    }
    else
    {
        return false;
    }
}

bool Service::HandleSetRequest(const Node* sender, const data::Message* message)
{
    std::lock_guard<std::mutex> guard(_mutex);
    
    // Check if target exists
    auto target = _targets.find(message->kv_set_request().key());
    if (target != _targets.end())
    {
        // Set value
        target->second->Set(message->kv_set_request().key(), message->kv_set_request().value());

        // Log incoming request
        LOG(DBUG) << "A SET request for the local path '" << message->kv_set_request().key() << "' from node [" << sender->GetUUID() << "] was handled";

        // Mark as handled
        return true;
    }
    else
    {
        return false;
    }
}

void Service::RegisterTarget(const std::string& path, Target* target)
{
    std::lock_guard<std::mutex> guard(_mutex);

    // Check if path is already registered
    if (_targets.find(path) == _targets.end())
    {
        // Add to map
        _targets[path] = target;
    }
    else
    {
        throw Exception("Path already registered");
    }
}

void Service::UnregisterTarget(const std::string& path)
{
    std::lock_guard<std::mutex> guard(_mutex);

    // Remove path
    _targets.erase(path);
}

void Service::DescribeService(data::discovery::Response& descriptor)
{
    std::lock_guard<std::mutex> guard(_mutex);

    // Collect information from targets
    auto& fields = *descriptor.mutable_keyvalue_schema()->mutable_fields();
    for (auto target : _targets)
    {
        fields[target.first] = target.second->GetFieldDescriptor(target.first);
    }
}