#include <swarmio/services/keyvalue/ValueAwaiter.h>
#include <swarmio/Exception.h>

using namespace swarmio;
using namespace swarmio::services;
using namespace swarmio::services::keyvalue;

data::Variant ValueAwaiter::ExtractResponse(const Node* node, const data::Message* message)
{
    if (message->content_case() == data::Message::ContentCase::kError)
    {
        // Ignore 
        if (message->error() != data::Error::NONE)
        {
            throw Exception("A remote error has occurred");
        }
        else
        {
            throw Exception("Unexpected acknowledge received");
        }
    }
    else if (message->content_case() == data::Message::ContentCase::kKvGetResponse)
    {
        if (message->kv_get_response().key() == _key)
        {
            return message->kv_get_response().value();
        }
        else
        {
            throw Exception("Invalid key");    
        }
    }
    else
    {
        throw Exception("Invalid response type");
    }
}