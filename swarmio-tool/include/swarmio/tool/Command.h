#pragma once

#include <string>
#include <map>
#include <replxx.hxx>

namespace swarmio::tool 
{
    /**
     * @brief Parser for commands of the default form.
     * 
     * Basic format is: VERB [PATH] [NAME=VALUE]...
     * 
     */
    class Command final
    {
        private:

            /**
             * @brief Verb
             * 
             */
            std::string _verb;

            /**
             * @brief Path
             * 
             */
            std::string _path;

            /**
             * @brief Parameters
             * 
             */
            std::map<std::string, std::string> _parameters;

            /**
             * @brief Construct a new Command object
             * 
             * @param verb Verb
             */
            Command(const std::string& verb)
            {
                _verb = verb;
            } 

        public:

            /**
             * @brief Parse a string as a command
             * 
             * @param command Command string
             * @return Command 
             */
            static Command Parse(std::string command);

            /**
             * @brief Highlighter callback for commands
             * 
             * @param input Input string
             * @param colors Colors
             * @param unused Unused parameter
             */
            static void HighlighterCallback(const std::string& input, replxx::Replxx::colors_t& colors, void* unused);

            /**
             * @brief Do a case insensitive regex 
             *        match on the verb.
             *        
             * @param pattern Regex pattern
             */
            bool Is(const char* pattern) const;
            
            /**
             * @brief Does the command have a path specified?
             * 
             * @return True if a path was specified
             */
            bool HasPath() const
            {
                return !_path.empty();
            }

            /**
             * @brief Does the command have parameters?
             * 
             * @return True if parameters were specified
             */
            bool HasParameters() const
            {
                return !_parameters.empty();
            }

            /**
             * @brief Get the verb
             * 
             * @return const std::string& 
             */
            const std::string& GetVerb() const
            {
                return _verb;
            }

            /**
             * @brief Get the path
             * 
             * @return const std::string& 
             */
            const std::string& GetPath() const
            {
                return _path;
            }

            /**
             * @brief Get the map of parameters
             * 
             * @return const std::map& 
             */
            const std::map<std::string, std::string>& GetParameters() const
            {
                return _parameters;
            }
    };

}