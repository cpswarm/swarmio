#include <swarmio/services/discovery/Service.h>
#include <g3log/g3log.hpp>

using namespace swarmio;
using namespace swarmio::services;
using namespace swarmio::services::discovery;

void Service::Invalidate()
{
    std::unique_lock<std::mutex> guard(_discoverablesMutex);

    // Mark cache as invalid
    _cacheValid = false;
    guard.unlock();

    // Send invalidation request
    data::Message message;
    message.mutable_ds_request()->set_action(data::discovery::Action::INVALIDATE);
    GetEndpoint()->Send(&message, nullptr);

    // Log outgoing event
    LOG(DBUG) << "An invalidation request was sent globally.";
}

void Service::RegisterDiscoverable(Discoverable* discoverable)
{
    std::lock_guard<std::mutex> guard(_discoverablesMutex);

    // Insert new Discoverable and invalidate cache
    _discoverables.insert(discoverable);
    _cacheValid = false;
}

void Service::UnregisterDiscoverable(Discoverable* discoverable)
{
    std::lock_guard<std::mutex> guard(_discoverablesMutex);

    // Remove Discoverable and invalidate cache
    _discoverables.erase(discoverable);
    _cacheValid = false;
}

bool Service::ReceiveMessage(const Node* sender, const data::Message* message)
{
    // Sanity checks
    CHECK(sender != nullptr) << "Sender address missing";
    CHECK(message != nullptr) << "Message is missing";

    // Handle message types
    if (message->content_case() == data::Message::ContentCase::kDsRequest)
    {
        switch (message->ds_request().action())
        {
            case data::discovery::Action::DISCOVER:

                // Answer discovery
                HandleDiscoveryRequest(sender, message);

                // Mark as handled
                return true;

            case data::discovery::Action::INVALIDATE:

                // Invalidate cached information
                HandleInvalidationRequest(sender, message);

                // Mark as handled
                return true;

            default:
            
                // Unknown action
                LOG(WARNING) << "A discovery request with the unknown action '" << message->ds_request().action() << "' was received and promptly ignored.";
                return false;
        }
    }
    else if (message->content_case() == data::Message::ContentCase::kDsResponse)
    {
        // Cache
        CacheDiscoveryResponse(sender, message);

        // Forward to awaiter
        return false;
    }
    else
    {
        return false;
    }
}

void Service::HandleDiscoveryRequest(const Node* node, const data::Message* message)
{
    std::unique_lock<std::mutex> guard(_discoverablesMutex);

    // Sanity checks
    CHECK(node != nullptr) << "Sender address missing";
    CHECK(message != nullptr) << "Message is missing";

    // Make sure the cache is valid
    if (!_cacheValid)
    {
        // Clear existing cache
        _cachedResponse.Clear();

        // Query services for information
        for (auto discoverable : _discoverables)
        {
            discoverable->DescribeService(_cachedResponse);
        }

        // Mark cache as valid
        _cacheValid = true;
    }

    // Wrap into message and unlock
    data::Message response;
    response.mutable_ds_response()->CopyFrom(_cachedResponse);
    guard.unlock();

    // Send message
    response.mutable_header()->set_reply_to(message->header().identifier());
    GetEndpoint()->Send(&response, node);

    // Log outgoing response
    LOG(DBUG) << "A discovery request from node [" << node->GetUUID() << "] was answered.";
}

void Service::CacheDiscoveryResponse(const Node* node, const data::Message* message)
{
    // Sanity checks
    CHECK(node != nullptr) << "Sender address missing";
    CHECK(message != nullptr) << "Message is missing";

    // Cache response
    std::unique_lock<std::mutex> remotesGuard(_remotesMutex);
    _remotes[node] = message->ds_response();
    remotesGuard.unlock();

    // Log
    LOG(DBUG) << "A discovery response from node [" << node->GetUUID() << "] was cached.";

    // Call observers
    std::unique_lock<std::mutex> observersGuard(_observersMutex);
    for (auto observer : _observers)
    {
        observer->CachedDiscoveryResponseWasUpdated(node, message->ds_response());
    }
}

void Service::HandleInvalidationRequest(const Node* node, const data::Message* message)
{
    // Sanity checks
    CHECK(node != nullptr) << "Sender address missing";

    // Erase cached information
    std::unique_lock<std::mutex> guard(_remotesMutex);
    _remotes.erase(node);
    _remotesMutex.unlock();

    // Log
    LOG(DBUG) << "An invalidation request from node [" << node->GetUUID() << "] was received.";

    // If active discovery is enabled, immediately send a discovery request
    if (_performActiveDiscovery)
    {
        // Build message
        data::Message request;
        request.mutable_ds_request()->set_action(data::discovery::Action::DISCOVER);

        // Send message
        GetEndpoint()->Send(&request, node);

        // Log outgoing request
        LOG(DBUG) << "An automatic discovery request was sent to [" << node->GetUUID() << "] after the node requested invalidation.";
    }
}

void Service::NodeWillLeave(const Node* node) noexcept
{
    // Sanity checks
    CHECK(node != nullptr) << "Node address missing";

    // Erase cached information
    std::lock_guard<std::mutex> guard(_remotesMutex);
    _remotes.erase(node);

    // Log
    LOG(DBUG) << "Discovery information on [" << node->GetUUID() << "] was erased after the node left.";
}

void Service::NodeDidJoin(const Node* node) noexcept
{
    // Sanity checks
    CHECK(node != nullptr) << "Node address missing";

    // If active discovery is enabled, immediately send a discovery request
    if (_performActiveDiscovery)
    {
        // Build message
        data::Message request;
        request.mutable_ds_request()->set_action(data::discovery::Action::DISCOVER);

        // Send message
        GetEndpoint()->Send(&request, node);

        // Log outgoing request
        LOG(DBUG) << "An automatic discovery request was sent to [" << node->GetUUID() << "] after the node joined.";
    }
}

DiscoveryAwaiter Service::Query(Endpoint* endpoint, const Node* node)
{
    // Sanity checks
    CHECK(node != nullptr) << "Destination address missing";
    CHECK(endpoint != nullptr) << "Endpoint is missing";

    // Build message
    data::Message request;
    request.mutable_header()->set_reliability(data::Reliability::NACK_REQUESTED);
    request.mutable_ds_request()->set_action(data::discovery::Action::DISCOVER);

    // Send and await response
    endpoint->Tag(&request);
    DiscoveryAwaiter awaiter(endpoint, request.header().identifier());
    endpoint->Send(&request, node);

    // Log outgoing message
    LOG(DBUG) << "A discovery request was sent to [" << node->GetUUID() << "].";

    return awaiter;
}

DiscoveryAwaiter Service::CachedQuery(const Node* node)
{
    // Sanity checks
    CHECK(node != nullptr) << "Destination address missing";

    // Try finding the cached value
    std::unique_lock<std::mutex> guard(_remotesMutex);
    auto existing = _remotes.find(node);
    if (existing != _remotes.end())
    {
        // Log
        LOG(DBUG) << "Cached discovery information was returned for [" << node->GetUUID() << "].";

        // Get it from the cache
        return DiscoveryAwaiter(existing->second);
    }
    else
    {
        guard.unlock();

        // Issue request
        return Query(GetEndpoint(), node);
    }
}

void Service::GlobalQuery()
{
    // Build message
    data::Message request;
    request.mutable_ds_request()->set_action(data::discovery::Action::DISCOVER);

    // Send message
    GetEndpoint()->Send(&request, nullptr);
    
    // Log request
    LOG(DBUG) << "A global discovery request was sent.";
}

std::map<const Node*, data::discovery::Response> Service::GetCachedNodeInformation()
{
    std::lock_guard<std::mutex> guard(_remotesMutex);

    // Return a copy of the map
    return _remotes;
}