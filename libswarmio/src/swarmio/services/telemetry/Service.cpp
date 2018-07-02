#include <swarmio/services/telemetry/Service.h>
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

Service::Service(Endpoint* endpoint)
    : Mailbox(endpoint), _shutdownRequested(false)
{
    _worker = new std::thread(&Service::Worker, this);
}

Service::~Service()
{
    // Request shutdown
    _shutdownRequested = true;

    // Wait for worker thread
    _worker->join();

    // Delete worker thread
    delete _worker;
}

void Service::Worker()
{
    // Create clock
    std::chrono::high_resolution_clock timer;
    auto next = timer.now();

    // Go
    for (;;)
    {
        // Check if we need to stop
        if (_shutdownRequested)
        {
            break;
        }

        // Perform update
        Update();

        // Wait 100 seconds for the next iteration
        next += 100ms;
        std::this_thread::sleep_until(next);
    }
}

void Service::Update()
{
    // Make copy of current list of values
    std::shared_lock<std::shared_timed_mutex> guardValues(_valuesMutex);
    auto cache = _values;
    guardValues.unlock();

    // Send updates
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
    else
    {
        return false;
    }
}

void Service::DescribeService(data::discovery::Response& descriptor)
{
    std::shared_lock<std::shared_timed_mutex> guard(_valuesMutex);
    for (auto& pair : _values)
    {
        auto entry = descriptor.add_telemetry();

        // Set name
        entry->set_name(pair.first);

        // Determine type
        switch (pair.second.value_case())
        {
            case data::Variant::ValueCase::kIntValue:
                entry->set_type(data::discovery::Type::INT);
                break;

            case data::Variant::ValueCase::kDoubleValue:
                entry->set_type(data::discovery::Type::DOUBLE);
                break;

            case data::Variant::ValueCase::kBoolValue:
                entry->set_type(data::discovery::Type::BOOL);
                break;

            case data::Variant::ValueCase::kStringValue:
                entry->set_type(data::discovery::Type::STRING);
                break;

            default:
                LOG(WARNING) << "Unexpected variant data type for key '" << pair.first << "'";
                break;
        }        
    }
}