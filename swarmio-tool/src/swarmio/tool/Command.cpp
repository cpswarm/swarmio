#include <swarmio/tool/Command.h>
#include <swarmio/Exception.h>
#include <regex>

using namespace swarmio;
using namespace swarmio::tool;

void Command::HighlighterCallback(const std::string& input, replxx::Replxx::colors_t& colors, void* unused) 
{
    // Workaround for a known bug with UTF-8 special characters
    colors.resize(input.size());

    // Verbs are MAGENTA, so we start with that
    size_t location = 0;
    auto currentColor = replxx::Replxx::Color::BRIGHTMAGENTA;

    // Skip whitespace
	for (; location < input.size() && input[location] == ' '; ++location)
    {
        colors[location] = replxx::Replxx::Color::DEFAULT;
    }

    // Process verb
    for (; location < input.size(); ++location)
    {
        if (input[location] != ' ')
        {
            colors[location] = replxx::Replxx::Color::BRIGHTMAGENTA;
        }
        else
        {
            break;
        }
	}

    // Process the rest of the components
    bool first = true;
    while (location < input.size())
    {
        // Look ahead
        size_t nextSpace = input.find(' ', location);
        size_t nextAssignment = input.find('=', location);
        if (nextSpace == location)
        {
            // Skip leading space
            colors[location] = replxx::Replxx::Color::DEFAULT;
            ++location;
        }
        else 
        {
            // Determine color
            auto color = replxx::Replxx::Color::WHITE;
            if (nextAssignment == location)
            {
                // Error, starts with =
                color = replxx::Replxx::Color::BRIGHTRED;
            }
            else
            {
                if (nextAssignment == std::string::npos)
                {
                    if (first)
                    {
                        // Path
                        color =  replxx::Replxx::Color::BRIGHTCYAN;
                    }
                    else
                    {
                        // Error, invalid location for path
                        color = replxx::Replxx::Color::BRIGHTRED;
                    }
                }
                else
                {
                    // Parameter
                    if (nextAssignment < nextSpace || nextSpace == std::string::npos)
                    {
                        color = replxx::Replxx::Color::WHITE;
                    }
                    else 
                    {
                        if (first)
                        {
                            // Path
                            color =  replxx::Replxx::Color::BRIGHTCYAN;
                        }
                        else
                        {
                            // Error, invalid location for path
                            color = replxx::Replxx::Color::BRIGHTRED;
                        }
                    }
                }
            }

            // Apply color
            for (; location < input.size() && input[location] != ' '; ++location)
            {
                colors[location] = color;
            }

            // Mark as not first
            first = false;
        }
    }
}

Command Command::Parse(std::string input)
{
    for (;;)
    {
        size_t location = input.find(' ');
        if (input.empty())
        {
            throw Exception("Syntax error: empty command");
        }
        else if (location == std::string::npos)
        {
            // Verb only command
            return Command(input);
        }
        else if (location == 0)
        {
            // Remove leading space
            input = input.substr(1);

            // Start over
            continue;
        }
        else
        {
            // Create command with verb
            Command command(input.substr(0, location));

            // Strip verb
            input = input.substr(location + 1);

            // Process the rest
            bool first = true;
            while (!input.empty())
            {
                // Find next space and assignment operator
                size_t nextSpace = input.find(' ');
                size_t nextAssignment = input.find('=');

                // Determine type of parameter
                if (nextSpace == 0)
                {
                    // Skip extra space
                    input = input.substr(1);
                }
                else if (nextAssignment == 0)
                {
                    throw Exception("Syntax error: empty key for parameter");
                }
                else if (nextSpace == std::string::npos)
                {
                    if (nextAssignment == std::string::npos)
                    {
                        // Parse as path
                        if (first)
                        {
                            command._path = input;
                        }
                        else
                        {
                            throw Exception("Syntax error: stray name or path");
                        }
                    }
                    else
                    {
                        // Parse as parameter
                        std::string key = input.substr(0, nextAssignment);
                        if (command._parameters.find(key) == command._parameters.end())
                        {
                            command._parameters[key] = input.substr(nextAssignment + 1);
                        }
                        else
                        {
                            throw Exception("Syntax error: duplicate parameter");
                        }
                    }

                    // No more
                    break;
                }
                else
                {
                    if (nextAssignment > nextSpace)
                    {
                        // Parse as path
                        if (first)
                        {
                            command._path = input.substr(0, nextSpace);
                        }
                        else                    
                        {
                            throw Exception("Syntax error: stray name or path");
                        }
                    }
                    else
                    {
                        // Parse as parameter
                        std::string key = input.substr(0, nextAssignment);
                        if (command._parameters.find(key) == command._parameters.end())
                        {
                            command._parameters[key] = input.substr(nextAssignment + 1, nextSpace - nextAssignment - 1);
                        }
                        else
                        {
                            throw Exception("Syntax error: duplicate parameter");
                        }
                    }

                    // Strip component
                    input = input.substr(nextSpace + 1);

                    // Mark as not first
                    first = false;
                }
            }

            // Return command
            return command;
        }
    }
}

bool Command::Is(const char* pattern) const
{
    // Build expression
    std::regex matcher(pattern, std::regex_constants::icase);

    // Match
    return std::regex_match(_verb, matcher);
}