#include <swarmio/tool/Loop.h>
#include <swarmio/tool/Command.h>
#include <swarmio/services/telemetry/Service.h>
#include <swarmio/Exception.h>
#include <iomanip>
#include <iostream>
#include <exception>
#include <chrono>
#include <regex>
 
using namespace swarmio;
using namespace swarmio::tool;
using namespace std::literals::chrono_literals;

Loop::Loop(Endpoint* endpoint, LogBuffer* logBuffer)
    : ClientProfile(endpoint), _logBuffer(logBuffer)
{
    _repl.install_window_change_handler();
    _repl.set_highlighter_callback(&Command::HighlighterCallback, nullptr);
}

void Loop::NodeWasDiscovered(const Node* node) noexcept
{
    std::lock_guard<std::mutex> guard(_mutex);

    // Add to vector
    _nodes.push_back(node);
}

std::string Loop::GetPrompt()
{
    if (_selectedNode == nullptr)
    {
        return std::string("\x1b[1;32mswarm\x1b[0m> ");
    }
    else
    {
        return std::string("\x1b[1;32mmembr\x1b[0m> ");
    }
}

void Loop::Run()
{
    for (;;)
    {
        // Get command
        const char* input = _repl.input(GetPrompt());
        if (input == nullptr)
        {
            if (errno == EAGAIN)
            {
                // Retry
                continue;
            }
            else
            {
                // Exit
                break;
            }
        }
        else if (*input == '\0')
        {
            // Empty command
            continue;
        }

        // Add to history
        std::string command = input;
        _repl.history_add(command);

        // Parse and execute command
        if (!ExecuteCommand(command))
        {
            break;
        }
    }
}

void Loop::ExecuteInfoCommand(const Command& command)
{
    if (_selectedNode != nullptr)
    {  
        // UUID
        std::cout << "UUID: " << _selectedNode->GetUUID() << std::endl;

        // Send request
        auto awaiter = _discoveryService.CachedQuery(_selectedNode);

        // Wait for request
        if (awaiter.WaitForResponse(5s))
        {
            auto response = awaiter.GetResponse();

            // Echo support
            std::cout << "Pingable: " << (response.echo() ? "true" : "false") << std::endl;

            // Keys
            if (response.keyvalue_size() > 0)
            {
                std::cout << "Parameters:" << std::endl;
                for (auto kv : response.keyvalue())
                {
                    std::cout << " - " << kv.name() << ":" << kv.type();
                    if (kv.can_read() && kv.can_write())
                    {
                        std::cout << " (rw)";
                    }
                    else if (kv.can_read())
                    {
                        std::cout << " (ro)";
                    }
                    else if (kv.can_write())
                    {
                        std::cout << " (wo)";
                    }
                    else
                    {
                        std::cout << " (na)";
                    }
                    std::cout << std::endl;
                }
            }

            // Events
            if (response.event_size() > 0)
            {
                std::cout << "Events:" << std::endl;
                for (auto ev : response.event())
                {
                    std::cout << " - " << ev.name();
                    if (ev.parameters_size() > 0)
                    {
                        std::cout << "(";
                        bool first = true;
                        for (auto p : ev.parameters())
                        {
                            if (first)
                            {
                                first = false;
                            }
                            else 
                            {
                                std::cout << ", ";
                            }
                            std::cout << p.first << ":" << p.second;
                        }
                        std::cout << ")";
                    }
                    std::cout << std::endl;
                }
            }

            // Telemetry
            if (response.telemetry_size() > 0)
            {
                std::cout << "Telemetry:" << std::endl;
                for (auto tm : response.telemetry())
                {
                    std::cout << " - " << tm.name() << ":" << tm.type() << std::endl;
                }
            }
        }
        else
        {
            std::cout << "Discovery timed out." << std::endl;
        }
    }
    else
    {
        std::cout << "No member selected." << std::endl;
    }
}

void Loop::ExecuteRediscoverCommand(const Command& command)
{
    _discoveryService.GlobalQuery();
    std::cout << "A global discovery request was sent." << std::endl;
}

void Loop::ExecuteGetCommand(const Command& command)
{
    if (_selectedNode != nullptr)
    {
        if (command.HasPath())
        {
            auto awaiter = services::keyvalue::Service::Get(GetEndpoint(), _selectedNode, command.GetPath());
            if (awaiter.WaitForResponse(5s))
            {
                auto value = awaiter.GetResponse();
                switch (value.value_case())
                {
                    case data::Variant::ValueCase::kBoolValue:
                        std::cout << (value.bool_value() ? "true" : "false") << std::endl;
                        break;

                    case data::Variant::ValueCase::kDoubleValue:
                        std::cout << value.double_value() << std::endl;
                        break;

                    case data::Variant::ValueCase::kIntValue:
                        std::cout << value.int_value() << std::endl;
                        break;

                    case data::Variant::ValueCase::kStringValue:
                        std::cout << "\"" << value.string_value() << "\"" << std::endl;
                        break;

                    default:
                        std::cout << "<unknown>" << std::endl;
                        break;
                }
            }
            else
            {
                std::cout << "Timeout." << std::endl;
            }
        }
        else
        {
            std::cout << "No resource path specified." << std::endl;
        }
    }
    else
    {
        std::cout << "No member selected." << std::endl;
    }
}

void Loop::ExecuteSetCommand(const Command& command)
{
    if (_selectedNode != nullptr)
    {
        if (command.GetParameters().size() == 1)
        {
            auto pair = command.GetParameters().begin();
            auto awaiter = services::keyvalue::Service::Set(GetEndpoint(), _selectedNode, pair->first, ConvertToVariant(pair->second));
            if (awaiter.WaitForResponse(5s))
            {
                if (awaiter.GetResponse())
                {
                     std::cout << "Value was set." << std::endl;
                }
                else
                {
                    std::cout << "Request rejected." << std::endl;
                }
            }
            else
            {
                std::cout << "Timeout." << std::endl;
            }
        }
        else
        {
            std::cout << "Invalid number of parameters." << std::endl;
        }
    }
    else
    {
        std::cout << "No member selected." << std::endl;
    }
}

void Loop::ExecutePingCommand(const Command& command)
{
    if (_selectedNode != nullptr)
    {
        // Determine packet size
        int size = 1024;
        if (command.HasPath())
        {
            // Parse
            size = std::stoi(command.GetPath());

            // Check
            if (size < 0 || size > 1024 * 1024 * 256)
            {
                std::cout << "Invalid ping packet size." << std::endl;
            }
        }
        
        // Send request
        auto awaiter = _pingService.Ping(GetEndpoint(), _selectedNode, size);

        // Wait for request
        if (awaiter.WaitForResponse(5s))
        {
            std::cout << "Response time: " << std::fixed << std::setprecision(2) << awaiter.GetResponseInMilliseconds() << "ms" << std::endl;
        }
        else
        {
            std::cout << "Timeout." << std::endl;
        }
    }
    else
    {
        std::cout << "No member selected." << std::endl;
    }
}

void Loop::ExecuteEventCommand(const Command& command)
{
    if (command.HasPath())
    {
        // Build events
        data::event::Notification event;
        event.set_name(command.GetPath());
        
        // Add parameters
        for (auto p : command.GetParameters())
        {
            (*event.mutable_parameters())[p.first] = ConvertToVariant(p.second);
        }

        // Send event
        if (_selectedNode == nullptr)
        {
            services::event::Service::Trigger(GetEndpoint(), event);
            std::cout << "Event broadcasted." << std::endl;
        }
        else
        {
            auto awaiter = services::event::Service::Trigger(GetEndpoint(), event, _selectedNode);
            if (awaiter.WaitForResponse(5s))
            {
                if (awaiter.GetResponse())
                {
                    std::cout << "Event handled remotely." << std::endl;
                }
                else
                {
                    std::cout << "Event rejected." << std::endl; 
                }
            }
            else
            {
                std::cout << "Timeout." << std::endl;
            }
        }
    }
    else
    {
        std::cout << "No event specified." << std::endl;
    }
}

void Loop::ExecuteMembersCommand(const Command& command)
{
    std::lock_guard<std::mutex> guard(_mutex);
    if (_nodes.size() > 0)
    {
        // Header
        std::cout << "MID" << "\t" << "UUID" << std::endl;

        // List nodes
        for (int i = 0; i < _nodes.size(); ++i)
        {
            std::cout << i << "\t" << _nodes[i]->GetUUID() << "\t" << _nodes[i]->GetDescription() << std::endl;
        }
    }
    else
    {
        std::cout << "No swarm members found." << std::endl;
    }
}

void Loop::ExecuteSelectCommand(const Command& command)
{
    // Check if we have a parameter
    if (command.HasPath())
    {
        // Parse
        int idx = std::stoi(command.GetPath());

        // Retreive from nodes vector
        std::lock_guard<std::mutex> guard(_mutex);
        if (idx >= 0 && idx < _nodes.size())
        {
            _selectedNode = _nodes[idx];
            std::cout << "Member selected: " << _selectedNode->GetUUID() << std::endl;
        }
        else
        {
            std::cout << "Invalid member index." << std::endl;
        }
    }
    else if (_selectedNode != nullptr)
    {
        _selectedNode = nullptr;
        std::cout << "Member unselected." << std::endl;
    }
    else
    {
        std::cout << "No effect, no member was selected." << std::endl
                    << "To select a member, specify a MID." << std::endl;
    }
}

void Loop::ExecuteSubscriptionsCommand(const Command& command)
{
    if (_subscriptions.size() > 0)
    {
        for (auto& subscription : _subscriptions)
        {
            std::cout << "Subscription #" << subscription.GetIdentifier() << std::endl;
            std::cout << "  " << "Node: " << subscription.GetTarget()->GetUUID() << std::endl;
            if (subscription.WaitForResponse(0ms))
            {
                auto response = subscription.GetResponse();

                // Tick
                std::cout << "  " << "Tick: " << response.tick() << std::endl;

                // Values
                if (response.values_size() > 0)
                {
                    std::cout << "  " << "Current values:" << std::endl;
                    for (auto& pair : subscription.GetResponse().values())
                    {
                        std::cout << "  " << " - " << pair.first << "=";
                        switch (pair.second.value_case())
                        {
                            case swarmio::data::Variant::ValueCase::kBoolValue:
                                std::cout << pair.second.bool_value() ? "true" : "false";
                                break;
                            case swarmio::data::Variant::ValueCase::kDoubleValue:
                                std::cout << pair.second.double_value();
                                break;
                            case swarmio::data::Variant::ValueCase::kIntValue:
                                std::cout << pair.second.int_value();
                                break;
                            case swarmio::data::Variant::ValueCase::kStringValue:
                                std::cout << "\"" << pair.second.string_value() << "\"";
                                break;
                            default:
                                std::cout << "<unknown>";
                                break;
                        }
                        std::cout << std::endl;
                    }
                }
                else
                {
                    std::cout << "  " << "Last update was empty." << std::endl;
                }
            }
            else
            {
                std::cout << "  " << "No update received so far." << std::endl;
            }
        }
    }
    else
    {
        std::cout << "No subscriptions." << std::endl;
    }
}

void Loop::ExecuteSubscribeCommand(const Command& command)
{
    if (_selectedNode != nullptr)
    {
        // Determine interval
        uint32_t interval = 1;
        auto intervalParameter = command.GetParameters().find("interval");
        if (intervalParameter != command.GetParameters().end())
        {
            interval = std::stoi(intervalParameter->second);
        }

        // Determine keys
        std::list<std::string> keys;
        auto keyParameter = command.GetParameters().find("key");
        if (keyParameter != command.GetParameters().end())
        {
            keys.push_back(keyParameter->second);
        }

        // Submit request
        _subscriptions.emplace_back(services::telemetry::Service::Subscribe(GetEndpoint(), _selectedNode, interval, keys));

        // Success
        std::cout << "Subscribed." << std::endl;
    }
    else
    {
        // Error
        std::cout << "No member selected." << std::endl;
    }
}

void Loop::ExecuteUnsubscribeCommand(const Command& command)
{
    uint64_t identifier = std::stoi(command.GetPath());
    auto element = std::find_if(_subscriptions.begin(), _subscriptions.end(), [identifier](const services::telemetry::UpdateAwaiter& awaiter){ return awaiter.GetIdentifier() == identifier; });
    if (element != _subscriptions.end())
    {
        _subscriptions.erase(element);
        std::cout << "Unsubcribed." << std::endl;
    }    
    else
    {
        std::cout << "Not found." << std::endl;
    }
}

void Loop::ExecuteLogCommand(const Command& command)
{
    if (_logBuffer != nullptr)
    {
        for (auto& message : _logBuffer->GetMessages())
        {
            std::cout << message.timestamp() << "\t" << message.level() << "\t" << message.message() << std::endl;
        }
    }
    else
    {
        std::cout << "No log buffer found." << std::endl;
    }
}

bool Loop::ExecuteCommand(const std::string& input)
{
    try
    {
        // Parse command
        auto command = Command::Parse(input);

        // Handle verbs
        if (command.Is("members"))
        {
            ExecuteMembersCommand(command);
        }
        else if (command.Is("select"))
        {
            ExecuteSelectCommand(command);
        }
        else if (command.Is("info"))
        {
            ExecuteInfoCommand(command);
        }
        else if (command.Is("ping"))
        {
            ExecutePingCommand(command);
        }
        else if (command.Is("event"))
        {
            ExecuteEventCommand(command);
        }
        else if (command.Is("set"))
        {
            ExecuteSetCommand(command);
        }
        else if (command.Is("rediscover"))
        {
            ExecuteRediscoverCommand(command);
        }
        else if (command.Is("get"))
        {
            ExecuteGetCommand(command);
        }
        else if (command.Is("subscriptions"))
        {
            ExecuteSubscriptionsCommand(command);
        }
        else if (command.Is("subscribe"))
        {
            ExecuteSubscribeCommand(command);
        }
        else if (command.Is("unsubscribe"))
        {
            ExecuteUnsubscribeCommand(command);
        }
        else if (command.Is("log"))
        {
            ExecuteLogCommand(command);
        }
        else if (command.Is("help"))
        {
            // Display help
            std::cout << "Available commands:"                       << std::endl
                      << " - members"                                << std::endl
                      << " - rediscover"                             << std::endl
                      << " - info"                                   << std::endl
                      << " - select [MID]"                           << std::endl
                      << " - event NAME [KEY=VALUE]..."              << std::endl
                      << " - get KEY"                                << std::endl
                      << " - set KEY=VALUE"                          << std::endl
                      << " - subscriptions"                          << std::endl
                      << " - subscribe [key=KEY] [interval=N]"       << std::endl
                      << " - unsubscribe [SID]"                      << std::endl
                      << " - ping [SIZE]"                            << std::endl
                      << " - help"                                   << std::endl
                      << " - log"                                    << std::endl
                      << " - exit"                                   << std::endl;
        }
        else if (command.Is("exit"))
        {
            // Say goodbye
            std::cout << "Goodbye!" << std::endl;

            // Exit
            return false;
        }
        else
        {
            // Unknown command
            std::cout << "Unknown command verb: " << command.GetVerb() << std::endl
                      << "Try 'help' to list available commands." << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

    // Continue
    return true;
}

data::Variant Loop::ConvertToVariant(const std::string& value)
{
    if (std::regex_match(value, std::regex("[0-9]+", std::regex_constants::icase)))
    {
        // Integer
        data::Variant variant;
        variant.set_int_value(std::stoi(value));
        return variant;
    }
    else if (std::regex_match(value, std::regex("[0-9]+\\.[0-9]+", std::regex_constants::icase)))
    {
        // Double
        data::Variant variant;
        variant.set_double_value(std::stod(value));
        return variant;
    }
    else if (std::regex_match(value, std::regex("true", std::regex_constants::icase)))
    {
        // Bool - true
        data::Variant variant;
        variant.set_bool_value(true);
        return variant;
    }
    else if (std::regex_match(value, std::regex("false", std::regex_constants::icase)))
    {
        // Bool - false
        data::Variant variant;
        variant.set_bool_value(false);
        return variant;
    }
    else
    {
        // String
        data::Variant variant;
        variant.set_string_value(value);
        return variant;
    }
}