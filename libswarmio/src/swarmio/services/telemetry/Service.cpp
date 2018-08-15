#include <swarmio/services/telemetry/Service.h>
#include <swarmio/data/Helper.h>
#include <g3log/g3log.hpp>
#include <chrono>

using namespace swarmio;
using namespace swarmio::services;
using namespace swarmio::services::telemetry;
using namespace std::literals::chrono_literals;

UpdateAwaiter Service::Subscribe(Endpoint* endpoint, const Node* node, uint32_t interval, const std::list<std::string>& keys)
{
    // Sanity checks
    CHECK(endpoint != nullptr) << "No endpoint specified";
    CHECK(node != nullptr) << "No target node specified";

    // Build message
    data::Message request;
    request.mutable_header()->set_reliability(data::Reliability::NACK_REQUESTED);
    request.mutable_tm_subscribe_request()->set_interval(interval);

    // Add keys
    for (auto& key : keys)
    {
        request.mutable_tm_subscribe_request()->add_keys(key);
    }

    // Log outgoing request
    LOG(DBUG) << "A SUBSCRIBE request " << ((keys.size() > 0) ? "for specific keys" : "for all keys") << " was sent to node [" << node->GetUUID() << "]";

    // Send and await response
    endpoint->Tag(&request);
    UpdateAwaiter awaiter(endpoint, request.header().identifier(), node);
    endpoint->Send(&request, node);
    return awaiter;
}

void Service::Update()
{
    // Make copy of current list of values
    std::shared_lock guardValues(_valuesMutex);
    auto cache = _values;
    guardValues.unlock();

    // Make copy of the status keys
    std::shared_lock guardSchema(_schemaMutex);
    auto statusKeys = _statusKeys;
    guardSchema.unlock();

    // Send status broadcast
    if (!statusKeys.empty())
    {
        // Build message
        data::Message message;
        auto& pairs = *message.mutable_tm_status()->mutable_values();
        for (const auto& key : statusKeys)
        {                
            auto entry = cache.find(key);
            if (entry != cache.end())
            {
                pairs[key] = (*entry).second;
            }
            else
            {
                // Quietly ignore keys not found locally
            }
        }
        
        // Send message
        if (!pairs.empty())
        {
            GetEndpoint()->Send(&message, nullptr);
        }
    }

    // Send updates to telemetry subscribers
    std::shared_lock<std::shared_timed_mutex> guardTrackers(_trackersMutex);
    bool hasInvalidTrackers = false;
    for (auto& tracker : _trackers)
    {
        if (tracker.IsValid())
        {
            try
            {
                // Get current tick
                uint32_t tick = tracker.GetAndIncrementTick();

                // Check if we need to send an update
                if (tick % tracker.GetRequest().interval() == 0)
                {
                    // Build message
                    data::Message response;
                    response.mutable_header()->set_reply_to(tracker.GetIdentifier());
                    response.mutable_tm_update()->set_tick(tick);

                    // Add values
                    auto values = response.mutable_tm_update()->mutable_values();
                    if (tracker.GetRequest().keys_size() > 0)
                    {
                        // Only take keys as specified in the request
                        for (auto& key : tracker.GetRequest().keys())
                        {
                            auto entry = cache.find(key);
                            if (entry != cache.end())
                            {
                                (*values)[key] = (*entry).second;
                            }
                            else
                            {
                                // Quietly ignore keys not found locally
                            }
                        }
                    }
                    else
                    {
                        // Take all keys
                        for (auto& pair : cache)
                        {
                            (*values)[pair.first] = pair.second;
                        }
                    }

                    // Send update
                    LOG(DBUG) << "An UPDATE " << ((tracker.GetRequest().keys_size() > 0) ? "for specific keys" : "for all keys") << " was sent to node [" << tracker.GetNode()->GetUUID() << "]";
                    GetEndpoint()->Send(&response, tracker.GetNode());
                }
            }
            catch (const std::exception& e)
            {
                LOG(WARNING) << "An error has occurred while trying to send an update to node [" << tracker.GetNode()->GetUUID() << "]: " << e.what();
            }
        }
        else
        {
            LOG(DBUG) << "Tracker #" << tracker.GetIdentifier() << " of node [" << tracker.GetNode()->GetUUID() << "] was marked for removal";
            hasInvalidTrackers = false;
        }
    }
    guardTrackers.unlock();

    // Remove invalid trackers
    if (hasInvalidTrackers)
    {
        std::unique_lock<std::shared_timed_mutex> guard(_trackersMutex);
        _trackers.remove_if([](const Tracker& tracker){ return !tracker.IsValid(); });
        LOG(DBUG) << "Invalidated trackers have been removed";
    }
}

bool Service::ReceiveMessage(const Node* sender, const data::Message* message)
{
    if (message->content_case() == data::Message::ContentCase::kTmSubscribeRequest)
    {  
        if (message->tm_subscribe_request().interval() > 0)
        {
            std::unique_lock<std::shared_timed_mutex> guard(_trackersMutex);

            // Add new tracker
            _trackers.emplace_back(sender, message->header().identifier(), message->tm_subscribe_request());
            return true;
        }
        else
        {
            // Invalid request
            throw Exception("Invalid interval for SUBSCRIBE request");
        }
    }
    else if (message->content_case() == data::Message::ContentCase::kTmUnsubscribeRequest)
    {
        std::shared_lock<std::shared_timed_mutex> guardTrackers(_trackersMutex);

        // Try and find the tracker
        for (auto& tracker : _trackers)
        {
            if (tracker.GetIdentifier() == message->tm_unsubscribe_request().identifier())
            {
                // Invalidate tracker
                tracker.Invalidate();
                return true;
            }
        }

        // No such tracker found
        return false;
    }
    else if (message->content_case() == data::Message::ContentCase::kTmStatus)
    {
        // Cache report
        std::unique_lock<std::shared_timed_mutex> reportsGuard(_reportsMutex);
        _reports[sender] = message->tm_status();
        reportsGuard.unlock();

        // Log
        LOG(DBUG) << "A status update from node [" << sender->GetUUID() << "] was cached.";

        // Call observers
        std::shared_lock<std::shared_timed_mutex> observersGuard(_observersMutex);
        for (auto observer : _observers)
        {
            observer->CachedStatusWasUpdated(sender, message->tm_status());
        }

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
    std::lock_guard guard(_schemaMutex);

    // Simply return with the cached schema
    *descriptor.mutable_telemetry_schema() = _schema;
}