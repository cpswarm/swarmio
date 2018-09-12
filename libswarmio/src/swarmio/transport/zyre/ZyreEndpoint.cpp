#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/Exception.h>
#include <atomic>
#include <cstdio>

using namespace swarmio;
using namespace swarmio::transport;
using namespace swarmio::transport::zyre;

#define SWARMIO_ZYRE_GROUP_MESSAGES "messages"
#define SWARMIO_ZYRE_HEADER_DEVICE_CLASS "X-DeviceClass"

ZyreEndpoint::ZyreEndpoint(const char* name, const char* deviceClass)
{
    // Create Zyre node
    _zyre = zyre_new(name);
    if (_zyre == NULL)
    {
        throw Exception("Unable to create node");
    }

    // Set class
    if (deviceClass != nullptr)
    {
        zyre_set_header(_zyre, SWARMIO_ZYRE_HEADER_DEVICE_CLASS, "%s", deviceClass);
    }
}

void ZyreEndpoint::SetPort(uint16_t port)
{
    // Check if we are already running
    if (_worker != nullptr)
    {
        throw Exception("Node is already running");
    }

    // Set port
    zyre_set_port(_zyre, port);
}

void ZyreEndpoint::SetInterface(const char* ifname)
{
    // Check if we are already running
    if (_worker != nullptr)
    {
        throw Exception("Node is already running");
    }

    // Set interface
    zyre_set_interface(_zyre, ifname);
}

void ZyreEndpoint::Start()
{
    // Check if we are already running
    if (_worker != nullptr)
    {
        throw Exception("Node is already running");
    }

    // Start instance
    if (zyre_start(_zyre) != 0)
    {
        throw Exception("Unable to start node");
    }

    // Start the thread
    _worker = new std::thread(&ZyreEndpoint::Process, this);

    // Call base implementation
    BasicEndpoint::Start();
}

void ZyreEndpoint::Stop()
{
    // Call base implementation
    BasicEndpoint::Stop();

    // Check if we are running
    if (_worker == nullptr)
    {
        throw Exception("Node is not running");
    }

    // Create client socket
    ZyreControlSocket client(&_control);

    // Send request
    zsock_send(client, "p", "SHUTDOWN");

    // Await response
    if (zsock_wait(client) != 0)
    {
        throw Exception("Invalid response");
    }

    // Join thread and remove reference
    _worker->join();
    delete _worker;
    _worker = nullptr;
}

void ZyreEndpoint::Send(const void* data, size_t size, const Node* node)
{
    // Try to convert to a Zyre node
    const ZyreNode* zyreNode = dynamic_cast<const ZyreNode*>(node);
    if (node != nullptr && zyreNode == nullptr)
    {
        throw Exception("Invalid target node type");
    }

    // Allocate message and append data
    zmsg_t* message = zmsg_new();
    zmsg_addmem(message, data, (size_t)size);

    // Create client socket
    ZyreControlSocket client(&_control);

    // Send request
    if (zsock_send(client, "ppp", "SEND", zyreNode, message) != 0)
    {
        throw Exception("Cannot send control message");
    }

    // Await response
    if (zsock_wait(client) != 0)
    {
        throw Exception("Cannot send message");
    }
}

ZyreEndpoint::~ZyreEndpoint()
{
    // Stop thread
    if (_worker != nullptr)
    {
        Stop();
    }

    // Destroy Zyre node
    if (_zyre != NULL)
    {
        zyre_destroy(&_zyre);
    }
}

void ZyreEndpoint::Process()
{
    // Set up queue and delivery thread
    moodycamel::BlockingReaderWriterQueue<zyre_event_t*> deliveryQueue;
    std::thread deliveryThread(&ZyreEndpoint::Deliver, this, &deliveryQueue);

    // Join group
    zyre_join(_zyre, SWARMIO_ZYRE_GROUP_MESSAGES);

    // Set up poller
    zsock_t* zyreSocket = zyre_socket(_zyre);
    zsock_t* controlSocket = _control.GetSocket();
    zpoller_t* poller = zpoller_new(controlSocket, zyreSocket, nullptr);

    // Process events
    while (true)
    {
        // Wait for event
        void* result = zpoller_wait(poller, -1);

        // Check if we have a read event
        if (result == zyreSocket)
        {
            // Get event
            zyre_event_t* event = zyre_event_new(_zyre);
            if (event != NULL)
            {
                // Process event
                deliveryQueue.enqueue(event);
            }
        }
        else if (result == controlSocket)
        {
            // Receive command
            const char* command = NULL;
            const ZyreNode* target = NULL;
            zmsg_t* message = NULL;
            if (zsock_recv(_control, "ppp", &command, &target, &message) == 0)
            {
                if (strcmp(command, "SHUTDOWN") == 0)
                {
                    // Graceful shutdown request
                    break;
                }
                else if (strcmp(command, "SEND") == 0 && message != nullptr)
                {
                    // Send message
                    if (target == nullptr)
                    {
                        zyre_shout(_zyre, SWARMIO_ZYRE_GROUP_MESSAGES, &message);
                    }
                    else
                    {
                        zyre_whisper(_zyre, target->GetUUID().c_str(), &message);
                    }

                    // OK
                    zsock_signal(_control, 0);
                }
                else if (command == NULL)
                {
                    // Malformed message, send error
                    zsock_signal(_control, -1);
                }
            }
        }
    }

    // Finish the queue
    deliveryQueue.enqueue(nullptr);
    deliveryThread.join();

    // Leave group
    zyre_leave(_zyre, SWARMIO_ZYRE_GROUP_MESSAGES);

    // Stop instance
    zyre_stop(_zyre);

    // Signal terminator
    zsock_signal(_control, 0);
}

void ZyreEndpoint::Deliver(moodycamel::BlockingReaderWriterQueue<zyre_event_t*>* queue)
{
    zyre_event_t* event = nullptr;
    while (true)
    {
        // Get next event
        queue->wait_dequeue(event);

        // Check if we need to exit
        if (event == nullptr)
        {
            // Set all remote nodes as offline and send notifications
            std::shared_lock<std::shared_timed_mutex> guard(_mutex);
            for (auto& element : _nodes)
            {
                if (element.second.IsOnline())
                {
                    element.second.SetOnline(false);
                    NodeWillLeave(&element.second);
                }
            }

            // Exit loop
            break;
        }

        // Resolve UUID
        const char* uuid = zyre_event_peer_uuid(event);
        if (uuid != nullptr)
        {
            // Handle event
            const char* type = zyre_event_type(event);
            if (strcmp(type, "ENTER") == 0)
            {
                // Lock map
                std::unique_lock<std::shared_timed_mutex> guard(_mutex);

                // Try to resolve node
                auto result = _nodes.find(uuid);
                if (result == _nodes.end())
                {
                    // Detect device class
                    const char* deviceClass = zyre_event_header(event, SWARMIO_ZYRE_HEADER_DEVICE_CLASS);
                    if (deviceClass == nullptr)
                    {
                        deviceClass = "unknown";
                    }

                    // Construct node
                    // (It would be nice to use try_emplace, but there is no way to
                    // get it working with older C++ standard libraries.)
                    auto it = _nodes.emplace(
                        std::piecewise_construct, 
                        std::forward_as_tuple(uuid), 
                        std::forward_as_tuple(uuid, zyre_event_peer_name(event), deviceClass, zyre_event_peer_addr(event))
                    );

                    // Unlock map
                    guard.unlock();

                    // Send discovery message
                    if (it.second)
                    {
                        NodeWasDiscovered(&it.first->second);
                    }
                }
            }
            else
            {
                // Lock map
                std::shared_lock<std::shared_timed_mutex> guard(_mutex);

                // Resolve node
                auto result = _nodes.find(uuid);
                if (result != _nodes.end())
                {
                    // Unlock map
                    guard.unlock();

                    if (strcmp(type, "JOIN") == 0 && strcmp(zyre_event_group(event), SWARMIO_ZYRE_GROUP_MESSAGES) == 0)
                    {
                        // Mark as online and notify mailboxes
                        if (!result->second.IsOnline())
                        {
                            result->second.SetOnline(true);
                            NodeDidJoin(&result->second);
                        }
                    }
                    else if (strcmp(type, "LEAVE") == 0 && strcmp(zyre_event_group(event), SWARMIO_ZYRE_GROUP_MESSAGES) == 0)
                    {
                        // Mark as offline and notify mailboxes
                        if (result->second.IsOnline())
                        {
                            result->second.SetOnline(false);
                            NodeWillLeave(&result->second);
                        }
                    }
                    else if (strcmp(type, "EXIT") == 0)
                    {
                        // Mark as offline and notify mailboxes
                        if (result->second.IsOnline())
                        {
                            result->second.SetOnline(false);
                            NodeWillLeave(&result->second);
                        }
                    }
                    else if (strcmp(type, "SHOUT") == 0 || 
                            strcmp(type, "WHISPER") == 0)
                    {
                        // Get a reference to the raw message
                        zmsg_t* raw = zyre_event_msg(event);
                        if (raw != NULL)
                        {
                            // Each frame needs to be processed individually
                            while (zmsg_size(raw) > 0)
                            {
                                // Decode message using the implementation in BasicEndpoint
                                zframe_t* frame = zmsg_pop(raw);

                                // Decode and dispatch, log errors and go on
                                ReceiveMessage(&result->second, zframe_data(frame), zframe_size(frame));

                                // Destroy frame
                                zframe_destroy(&frame);
                            }
                        }
                    }            
                }
            }
        }

        // Destroy event
        zyre_event_destroy(&event);
    }
}

std::list<const ZyreNode*> ZyreEndpoint::GetNodes()
{
    // Lock map
    std::shared_lock<std::shared_timed_mutex> guard(_mutex);

    // Create copy
    std::list<const ZyreNode*> nodes;
    transform(_nodes.begin(), _nodes.end(), back_inserter(nodes), [] (const std::map<std::string, ZyreNode>::value_type& value) { return &value.second; } );

    // Return copy
    return nodes;
}

std::string ZyreEndpoint::GetUUID()
{
    return zyre_uuid(_zyre);
}

const Node* ZyreEndpoint::NodeForUUID(const std::string& uuid)
{
    // Lock map
    std::shared_lock<std::shared_timed_mutex> guard(_mutex);

    // Find element
    auto it = _nodes.find(uuid);
    if (it != _nodes.end())
    {
        return &it->second;
    }
    else
    {
        return nullptr;
    }
}