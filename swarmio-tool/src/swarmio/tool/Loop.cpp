#include <swarmio/tool/Loop.h>
#include <swarmio/tool/Command.h>
#include <swarmio/Exception.h>
#include <iomanip>
#include <iostream>
#include <exception>
#include <chrono>
#include <regex>
 
using namespace swarmio;
using namespace swarmio::tool;
using namespace std::literals::chrono_literals;

Loop::Loop(Endpoint* endpoint)
    : ClientProfile(endpoint)
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
        std::cout << "UUID: " << _selectedNode->GetUUID() << "\n";

        // Send request
        auto awaiter = _discoveryService.CachedQuery(_selectedNode);

        // Wait for request
        if (awaiter.WaitForResponse(5s))
        {
            auto response = awaiter.GetResponse();

            // Echo support
            std::cout << "Pingable: " << (response.echo() ? "true" : "false") << "\n";

            // Keys
            if (response.keyvalue_size() > 0)
            {
                std::cout << "Parameters:" << "\n";
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
                    std::cout << "\n";
                }
            }

            // Events
            if (response.event_size() > 0)
            {
                std::cout << "Events:" << "\n";
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
                    std::cout << "\n";
                }
            }
        }
        else
        {
            std::cout << "Discovery timed out." << "\n";
        }
    }
    else
    {
        std::cout << "No member selected." << "\n";
    }
}

void Loop::ExecuteRediscoverCommand(const Command& command)
{
    _discoveryService.GlobalQuery();
    std::cout << "A global discovery request was sent." << "\n";
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
                        std::cout << (value.bool_value() ? "true" : "false") << "\n";
                        break;

                    case data::Variant::ValueCase::kDoubleValue:
                        std::cout << value.double_value() << "\n";
                        break;

                    case data::Variant::ValueCase::kIntValue:
                        std::cout << value.int_value() << "\n";
                        break;

                    case data::Variant::ValueCase::kStringValue:
                        std::cout << value.string_value() << "\n";
                        break;

                    default:
                        std::cout << "Unknown value type." << "\n";
                        break;
                }
            }
            else
            {
                std::cout << "Timeout." << "\n";
            }
        }
        else
        {
            std::cout << "No resource path specified." << "\n";
        }
    }
    else
    {
        std::cout << "No member selected." << "\n";
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
                     std::cout << "Value was set." << "\n";
                }
                else
                {
                    std::cout << "Request rejected." << "\n";
                }
            }
            else
            {
                std::cout << "Timeout." << "\n";
            }
        }
        else
        {
            std::cout << "Invalid number of parameters." << "\n";
        }
    }
    else
    {
        std::cout << "No member selected." << "\n";
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
                std::cout << "Invalid ping packet size." << "\n";
            }
        }
        
        // Send request
        auto awaiter = _pingService.Ping(GetEndpoint(), _selectedNode, size);

        // Wait for request
        if (awaiter.WaitForResponse(5s))
        {
            std::cout << "Response time: " << std::fixed << std::setprecision(2) << awaiter.GetResponseInMilliseconds() << "ms" << "\n";
        }
        else
        {
            std::cout << "Timeout." << "\n";
        }
    }
    else
    {
        std::cout << "No member selected." << "\n";
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
            std::cout << "Event broadcasted." << "\n";
        }
        else
        {
            auto awaiter = services::event::Service::Trigger(GetEndpoint(), event, _selectedNode);
            if (awaiter.WaitForResponse(5s))
            {
                if (awaiter.GetResponse())
                {
                    std::cout << "Event handled remotely." << "\n";
                }
                else
                {
                    std::cout << "Event rejected." << "\n";
                }
            }
            else
            {
                std::cout << "Timeout." << "\n";
            }
        }
    }
    else
    {
        std::cout << "No event specified." << "\n";
    }
}

void Loop::ExecuteMembersCommand(const Command& command)
{
    std::lock_guard<std::mutex> guard(_mutex);
    if (_nodes.size() > 0)
    {
        // Header
        std::cout << "MID" << "\t" << "UUID" << "\n";

        // List nodes
        for (int i = 0; i < _nodes.size(); ++i)
        {
            std::cout << i << "\t" << _nodes[i]->GetUUID() << "\t" << _nodes[i]->GetDescription() << "\n";
        }
    }
    else
    {
        std::cout << "No swarm members found." << "\n";
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
            std::cout << "Member selected: " << _selectedNode->GetUUID() << "\n";
        }
        else
        {
            std::cout << "Invalid member index." << "\n";
        }
    }
    else if (_selectedNode != nullptr)
    {
        _selectedNode = nullptr;
        std::cout << "Member unselected." << "\n";
    }
    else
    {
        std::cout << "No effect, no member was selected." << "\n"
                    << "To select a member, specify a MID." << "\n";
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
        else if (command.Is("help"))
        {
            // Display help
            std::cout << "Available commands:"           << "\n"
                      << " - members"                    << "\n"
                      << " - rediscover"                 << "\n"
                      << " - info"                       << "\n"
                      << " - select [MID]"               << "\n"
                      << " - event NAME [KEY=VALUE]..."  << "\n"
                      << " - get KEY"                    << "\n"
                      << " - set KEY=VALUE"              << "\n"
                      << " - ping [SIZE]"                << "\n"
                      << " - help"                       << "\n"
                      << " - exit"                       << "\n";
        }
        else if (command.Is("exit"))
        {
            // Say goodbye
            std::cout << "Goodbye!" << "\n";

            // Exit
            return false;
        }
        else
        {
            // Unknown command
            std::cout << "Unknown command verb: " << command.GetVerb() << "\n"
                      << "Try 'help' to list available commands." << "\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << "\n";
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