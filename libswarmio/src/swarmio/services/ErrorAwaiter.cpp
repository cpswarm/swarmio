#include <swarmio/services/ErrorAwaiter.h>
#include <swarmio/Exception.h>

using namespace swarmio;
using namespace swarmio::services;

bool ErrorAwaiter::ExtractResponse(const Node* node, const data::Message* message)
{
    if (message->content_case() == data::Message::ContentCase::kError)
    {
        return message->error() == data::Error::NONE;
    }
    else
    {
        throw Exception("Invalid response type");
    }
}