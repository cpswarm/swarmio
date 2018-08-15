#include <swarmio/transport/BasicEndpoint.h>
#include <swarmio/Exception.h>
#include <g3log/g3log.hpp>
#include <memory>

using namespace swarmio;
using namespace swarmio::transport;

void BasicEndpoint::Start()
{
    std::lock_guard guard(_mutex);

    // Check if we are already running
    if (_isRunning)
    {
        throw Exception("Endpoint is already running");
    }
    else
    {
        // Mark as running
        _isRunning = true;   

        // Fire callbacks
        for (auto mailbox : _mailboxes)
        {
            mailbox->MailboxWasConnected();
        }
    }
}

void BasicEndpoint::Stop()
{
    std::lock_guard guard(_mutex);

    // Check if we are already running
    if (_isRunning)
    {
        // Fire callbacks
        for (auto mailbox : _mailboxes)
        {
            mailbox->MailboxWillBeDisconnected();
        }

        // Mark as stopped
        _isRunning = false;
    }
    else
    {
        throw Exception("Endpoint has not been started");
    }
}

void BasicEndpoint::RegisterMailbox(Mailbox* mailbox)
{
    std::lock_guard guard(_mutex);

    // Add mailbox
    _mailboxes.insert(mailbox);

    // Fire callback
    if (_isRunning)
    {
        mailbox->MailboxWasConnected();
    }
}

void BasicEndpoint::UnregisterMailbox(Mailbox* mailbox)
{
    std::lock_guard guard(_mutex);

    // Fire callback
    if (_isRunning)
    {
        mailbox->MailboxWillBeDisconnected();
    }

    // Remove mailbox
    _mailboxes.erase(mailbox);
}

void BasicEndpoint::ReplaceMailbox(Mailbox* oldMailbox, Mailbox* newMailbox)
{
    std::lock_guard guard(_mutex);

    // Remove the old one, add the new one
    _mailboxes.erase(oldMailbox);
    _mailboxes.insert(newMailbox);
}

void BasicEndpoint::NodeWasDiscovered(const Node* node) noexcept
{
    std::lock_guard guard(_mutex);

    // Fire callbacks
    for(auto mailbox : _mailboxes)
    {
        mailbox->NodeWasDiscovered(node);
    }
}

void BasicEndpoint::NodeDidJoin(const Node* node) noexcept
{
    std::lock_guard guard(_mutex);

    // Fire callbacks
    for(auto mailbox : _mailboxes)
    {
        mailbox->NodeDidJoin(node);
    }
}

void BasicEndpoint::NodeWillLeave(const Node* node) noexcept
{
    std::lock_guard guard(_mutex);

    // Fire callbacks
    for(auto mailbox : _mailboxes)
    {
        mailbox->NodeWillLeave(node);
    }
}

bool BasicEndpoint::ReceiveMessage(const Node* sender, const void* data, size_t size) noexcept
{
    data::Message message;
    if (message.ParseFromArray(data, (int)size))
    {
        return ReceiveMessage(sender, &message);
    }
    else
    {
        LOG(WARNING) << "Message received from node [" << sender->GetUUID() << "] cannot be parsed";
        return false;
    }
}

void BasicEndpoint::ReplyWithError(const Node* sender, const data::Message* message, data::Error error)
{
    try
    {
        // Send reply
        data::Message reply;
        reply.mutable_header()->set_reply_to(message->header().identifier());
        reply.set_error(error);
        Send(&reply, sender);
    }
    catch (const Exception& e)
    {
        // Log and ignore error
        LOG(WARNING) << "An error has occurred while trying to reply to the message received from node [" << sender->GetUUID() << "]: " << e.what();
    }
}
            
bool BasicEndpoint::ReceiveMessage(const Node* sender, const data::Message* message) noexcept
{
    // Drop malformed messages without a valid identifier
    if (message->header().identifier() == 0)
    {
        return false;
    }

    // Find mailbox to handle message
    std::unique_lock<std::recursive_mutex> guard(_mutex);
    for(auto mailbox : _mailboxes)
    {
        try
        {
            if (mailbox->ReceiveMessage(sender, message))
            {
                // Unlock
                guard.unlock();

                // Send ACK
                if (message->header().reliability() == data::Reliability::ACK_REQUESTED)
                {
                    ReplyWithError(sender, message, data::Error::NONE);
                }

                // Mark as processed
                return true;
            }
        }
        catch (const Exception& e)
        {
            // Unlock
            guard.unlock();

            // Log error
            LOG(WARNING) << "An error has occurred while processing the message received from node [" << sender->GetUUID() << "]: " << e.what();

            // Send error
            if (message->header().reliability() == data::Reliability::ACK_REQUESTED || 
                message->header().reliability() == data::Reliability::NACK_REQUESTED)
            {
                ReplyWithError(sender, message, data::Error::UNKNOWN);
            }

            // Stop processing
            return false;
        }
    }

    // Unlock
    guard.unlock();

    // Send error
    if (message->header().reliability() == data::Reliability::ACK_REQUESTED || 
        message->header().reliability() == data::Reliability::NACK_REQUESTED)
    {
        ReplyWithError(sender, message, data::Error::DELIVERY);
    }

    // Mark as unprocessed
    return false;
}

void BasicEndpoint::Tag(data::Message* message)
{
    message->mutable_header()->set_identifier(_counter++);
}

void BasicEndpoint::Send(data::Message* message, const Node* node)
{
    // Set identifier if missing
    if (!message->has_header() || message->header().identifier() == 0)
    {
        Tag(message);
    }

    // Serialize
    std::vector<char> buffer(message->ByteSizeLong());
    if (message->SerializeToArray(buffer.data(), (int)buffer.size()))
    {
        Send((const void*)buffer.data(), buffer.size(), node);
    }
    else
    {
        throw Exception("Cannot serialize message");
    }
}