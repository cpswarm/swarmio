#include <swarmio/services/ping/TimingAwaiter.h>
#include <swarmio/Exception.h>
#include <cstdio>

using namespace swarmio;
using namespace swarmio::services;
using namespace swarmio::services::ping;

std::chrono::nanoseconds TimingAwaiter::ExtractResponse(const Node* node, const data::Message* message)
{
    if (message->content_case() == data::Message::ContentCase::kEcho)
    {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(now - _start);
    }
    else
    {
        throw Exception("Invalid response type");
    }
}

double TimingAwaiter::GetResponseInMilliseconds()
{
    std::chrono::duration<double, std::milli> response = GetResponse();   
    return response.count();
}