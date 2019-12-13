#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/transport/base64.h>
#include <swarmio/Exception.h>
#include <atomic>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>

using namespace swarmio;
using namespace swarmio::transport;
using namespace swarmio::transport::zyre;

#define SWARMIO_ZYRE_GROUP_MESSAGES "messages"
#define SWARMIO_ZYRE_HEADER_DEVICE_CLASS "X-DeviceClass"
#define SODIUM_STATIC

ZyreEndpoint::ZyreEndpoint(const char *name, const char *deviceClass)
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

    // Get UUID
    _uuid = zyre_uuid(_zyre);
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

void ZyreEndpoint::SetInterface(const char *ifname, bool security)
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

    // Reading config file
    std::ifstream cFile("config.json");
    if (cFile.is_open())
    {
        std::string line;
        while (getline(cFile, line))
        {
            line.erase(std::remove_if(line.begin(), line.end(), isspace),
                       line.end());
            if (line[0] == '#' || line.empty() || line[0] == '{' || line[0] == '}')
                continue;
            auto first = line.find('\"');
            auto second = line.find('\"', first + 1);
            auto third = line.find('\"', second + 1);
            auto fourth = line.find('\"', third + 1);
            auto name = line.substr(first + 1, second - first - 1);
            auto value = line.substr(third + 1, fourth - third - 1);
            std::string decoded = base64_decode(value);
            std::cout << name << ": " << value << ", " << decoded << '\n';

            if (strcmp(name.c_str(), "publicKey") == 0) //TODO: add to my_publickey, my_secretkey
            {
                memcpy(seed1, reinterpret_cast<unsigned char *>(const_cast<char *>(decoded.c_str())), crypto_box_SEEDBYTES);
                security_enabled = true;
            }
            if (strcmp(name.c_str(), "signature") == 0)
                memcpy(signature, reinterpret_cast<unsigned char *>(const_cast<char *>(decoded.c_str())), crypto_sign_SECRETKEYBYTES);
            if (strcmp(name.c_str(), "ca") == 0)
                memcpy(server_public, reinterpret_cast<unsigned char *>(const_cast<char *>(decoded.c_str())), crypto_sign_PUBLICKEYBYTES);
        }
    }
    else
    {
        std::cerr << "Couldn't open config file for reading.\n";
    }

    // Start crypto
    if (security_enabled)
    {
        if (sodium_init() < 0)
        {
            throw Exception("Unable to init sodium");
        }
        // For testing purposes always the same keys are generated with the same nonce
        // TODO: replace this with keys from Deployment tool
        crypto_box_seed_keypair(my_publickey, my_secretkey, seed1);
        std::cout << 'Me: p: ' << my_publickey << std::endl
                  << ' s: ' << my_secretkey << std::endl;
        // Create broadcast keys
        crypto_box_seed_keypair(bcast_publickey, bcast_secretkey, server_public);
        std::cout << 'Broadcast: p: ' << bcast_publickey << std::endl
                  << ' s: ' << bcast_secretkey << std::endl;
        auto it = _nodes.emplace(
            std::piecewise_construct,
            std::forward_as_tuple("0"),
            std::forward_as_tuple("0", "broadcast", "broadcast", "0"));
        unsigned char k[crypto_box_BEFORENMBYTES];
        if (crypto_box_beforenm(k, my_publickey, bcast_secretkey) != 0)
            throw Exception("Cannot create broadcast keys");
        else
        {
            _nodes.at("0").SetKey(k);
            unsigned char n[crypto_box_NONCEBYTES];
            randombytes_buf(n, crypto_box_NONCEBYTES);
            _nodes.at("0").SetCtr(n);
            _nodes.at("0").SetVerified();
        }
        //Test the signed certificate
        if (crypto_sign_verify_detached(signature, seed1, crypto_sign_PUBLICKEYBYTES, server_public) != 0)
        {
            throw Exception("Unable to verify own certificate");
        }
        else
        {
            memcpy(certificate, seed1, crypto_sign_PUBLICKEYBYTES);
            memcpy(&certificate[crypto_sign_PUBLICKEYBYTES], signature, crypto_sign_SECRETKEYBYTES);
        }
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

void ZyreEndpoint::Send(const void *data, size_t size, const Node *node)
{
    // Try to convert to a Zyre node
    const ZyreNode *zyreNode = dynamic_cast<const ZyreNode *>(node);
    if (node != nullptr && zyreNode == nullptr)
    {
        throw Exception("Invalid target node type");
    }

    // Allocate message and append data
    zmsg_t *message = zmsg_new();
    if (security_enabled && !isJoining)
    {
        size_t c_len = crypto_box_MACBYTES + (size_t)size;
        unsigned char *ciphertext = new unsigned char[c_len + crypto_box_NONCEBYTES];
        if (node == nullptr)
        { // Broadcast node should be selected
            zyreNode = (ZyreNode *)NodeForUUID("0");
        }
        dynamic_cast<const ZyreNode *>(NodeForUUID(zyreNode->GetUUID()))->IncrementCtr();
        if (crypto_box_easy_afternm(ciphertext, (const unsigned char *)data, (size_t)size, zyreNode->GetCtr(), zyreNode->GetKey()) != 0)
        {
            throw Exception("Could not encrypt message");
        }
        memcpy(&ciphertext[c_len], zyreNode->GetCtr(), crypto_box_NONCEBYTES);
        zmsg_addmem(message, ciphertext, c_len + crypto_box_NONCEBYTES);
        delete[] ciphertext;
        if (node == nullptr)
        {
            zyreNode = nullptr;
        }
    }
    else
    {
        zmsg_addmem(message, data, (size_t)size);
    }

    // Create client socket
    ZyreControlSocket client(&_control);

    // Send request
    if (zsock_send(client, "ppp", "SEND", zyreNode, message) != 0)
    {
        throw Exception("Cannot send control message");
    }
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
    moodycamel::BlockingReaderWriterQueue<zyre_event_t *> deliveryQueue;
    std::thread deliveryThread(&ZyreEndpoint::Deliver, this, &deliveryQueue);

    // Join group
    zyre_join(_zyre, SWARMIO_ZYRE_GROUP_MESSAGES);

    // Set up poller
    zsock_t *zyreSocket = zyre_socket(_zyre);
    zsock_t *controlSocket = _control.GetSocket();
    zpoller_t *poller = zpoller_new(controlSocket, zyreSocket, nullptr);

    // Process events
    while (true)
    {
        // Wait for event
        void *result = zpoller_wait(poller, -1);

        // Check if we have a read event
        if (result == zyreSocket)
        {
            // Get event
            zyre_event_t *event = zyre_event_new(_zyre);
            if (event != NULL)
            {
                // Process event
                deliveryQueue.enqueue(event);
            }
        }
        else if (result == controlSocket)
        {
            // Receive command
            const char *command = NULL;
            const ZyreNode *target = NULL;
            zmsg_t *message = NULL;
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

void ZyreEndpoint::Deliver(moodycamel::BlockingReaderWriterQueue<zyre_event_t *> *queue)
{
    zyre_event_t *event = nullptr;
    while (true)
    {
        // Get next event
        queue->wait_dequeue(event);

        // Check if we need to exit
        if (event == nullptr)
        {
            // Set all remote nodes as offline and send notifications
            std::shared_lock<std::shared_timed_mutex> guard(_mutex);
            for (auto &element : _nodes)
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
        const char *uuid = zyre_event_peer_uuid(event);
        if (uuid != nullptr)
        {
            // Handle event
            const char *type = zyre_event_type(event);
            if (strcmp(type, "ENTER") == 0)
            {
                // Lock map
                std::unique_lock<std::shared_timed_mutex> guard(_mutex);

                // Try to resolve node
                auto result = _nodes.find(uuid);
                if (result == _nodes.end())
                {
                    // Detect device class
                    const char *deviceClass = zyre_event_header(event, SWARMIO_ZYRE_HEADER_DEVICE_CLASS);
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
                        std::forward_as_tuple(uuid, zyre_event_peer_name(event), deviceClass, zyre_event_peer_addr(event)));

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
                            if (security_enabled)
                            {
                            isJoining = true;
                            LOG(WARNING) << "Sending certificate to join";
                            Send(certificate, crypto_box_PUBLICKEYBYTES + crypto_sign_SECRETKEYBYTES, &result->second);
                            isJoining = false;
                        }
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
                        zmsg_t *raw = zyre_event_msg(event);
                        if (raw != NULL)
                        {
                            // Each frame needs to be processed individually
                            while (zmsg_size(raw) > 0)
                            {
                                // Handle certificate handshake
                                zframe_t *frame = zmsg_pop(raw);
                                if (security_enabled && zframe_size(frame) == 96 && !((ZyreNode *)&result->second)->GetVerified())
                                {
                                    // Handle certificate registration
                                    if (crypto_sign_verify_detached(&zframe_data(frame)[crypto_sign_PUBLICKEYBYTES], zframe_data(frame), crypto_sign_PUBLICKEYBYTES, server_public) != 0)
                                    {
                                        LOG(WARNING) << "Unable to verify joining node certificate";
                                    }
                                    else
                                    {
                                        unsigned char k[crypto_box_BEFORENMBYTES];
                                        // TODO: use sent key (after parsing is ok)
                                        if (crypto_box_beforenm(k, my_publickey, bcast_secretkey) != 0)
                                        {
                                            throw Exception("Cannot create broadcast keys");
                                        }
                                        else
                                        {
                                            ((ZyreNode *)&result->second)->SetKey(k);
                                            ((ZyreNode *)&result->second)->SetVerified();
                                            LOG(WARNING) << "Adding joining node certificate";
                                        }
                                        // Send back own certificate
                                        isJoining = true;
                                        LOG(WARNING) << "Sending back own certificate";
                                        Send(certificate, crypto_box_PUBLICKEYBYTES + crypto_sign_SECRETKEYBYTES, &result->second);
                                        isJoining = false;
                                    }
                                }

                                // Decrypt message
                                else if (security_enabled)
                                {
                                    size_t d_size = zframe_size(frame) - crypto_box_MACBYTES - crypto_box_NONCEBYTES;
                                    unsigned char *decrypted = new unsigned char[d_size];
                                    //if (crypto_box_open_easy(decrypted, data, d_size + crypto_box_MACBYTES, nonce, alice_publickey, bob_secretkey) != 0)
                                    unsigned char *nonce = &zframe_data(frame)[d_size + crypto_box_MACBYTES];
                                    const ZyreNode *zyreNode = dynamic_cast<const ZyreNode *>(&(result->second));
                                    dynamic_cast<const ZyreNode *>(NodeForUUID(zyreNode->GetUUID()))->IncrementCtr();
                                    if (memcmp(nonce, zyreNode->GetCtr(), crypto_box_NONCEBYTES) < 0)
                                    {
                                        LOG(WARNING) << "Nonce was smaller than expected";
                                    }
                                    //Synchronizing nonce
                                    zyreNode->SetCtr(nonce);
                                    if (crypto_box_open_easy_afternm(decrypted, zframe_data(frame), d_size + crypto_box_MACBYTES, nonce, zyreNode->GetKey()) != 0)
                                    {
                                        LOG(WARNING) << "Message cannot be decrypted";
                                    }
                                    else
                                    {
                                        LOG(WARNING) << "Message successfully decrypted";
                                        ReceiveMessage(&result->second, decrypted, d_size);
                                    }
                                    delete[] decrypted;
                                }
                                else
                                {
                                    // Decode and dispatch, log errors and go on
                                    ReceiveMessage(&result->second, zframe_data(frame), zframe_size(frame));
                                }

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

std::list<const ZyreNode *> ZyreEndpoint::GetNodes()
{
    // Lock map
    std::shared_lock<std::shared_timed_mutex> guard(_mutex);

    // Create copy
    std::list<const ZyreNode *> nodes;
    transform(_nodes.begin(), _nodes.end(), back_inserter(nodes), [](const std::map<std::string, ZyreNode>::value_type &value) { return &value.second; });

    // Return copy
    return nodes;
}

const Node *ZyreEndpoint::NodeForUUID(const std::string &uuid)
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