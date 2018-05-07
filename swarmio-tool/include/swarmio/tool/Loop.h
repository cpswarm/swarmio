#pragma once

#include <string>
#include <mutex>
#include <replxx.hxx>
#include <swarmio/tool/Command.h>
#include <swarmio/profiles/ClientProfile.h>
#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/data/Variant.pb.h>

namespace swarmio::tool 
{
    /**
     * @brief The command line interface for the tool
     * 
     */
    class Loop final : private profiles::ClientProfile
    {
        private:

            /**
             * @brief REPL interface
             * 
             */
            replxx::Replxx _repl;

            /**
             * @brief Selected node
             * 
             */
            const Node* _selectedNode = nullptr;

            /**
             * @brief Build prompt string
             * 
             * @return std::string 
             */
            std::string GetPrompt();

            /**
             * @brief Nodes and their indices
             * 
             */
            std::vector<const Node*> _nodes;

            /**
             * @brief Mutex to protect nodes vector
             * 
             */
            std::mutex _mutex;

            /**
             * @brief Called when a new Node has been discovered.
             * 
             * @param node The node that has been discovered
             */
            virtual void NodeWasDiscovered(const Node* node) noexcept override;

            /**
             * @brief Convert a string parameter to a variant
             * 
             * @param value Original value
             * @return data::Variant Variant value
             */
            data::Variant ConvertToVariant(const std::string& value);

            /**
             * @brief Execute a command given by the user
             * 
             * @param command Command string
             * @return True if the loop should continue.
             */
            bool ExecuteCommand(const std::string& command);

            /**
             * @brief Execute a 'members' command
             * 
             * @param command Command
             */
            void ExecuteMembersCommand(const Command& command);

            /**
             * @brief Execute a 'select' command
             * 
             * @param command Command
             */
            void ExecuteSelectCommand(const Command& command);

            /**
             * @brief Execute a 'ping' command
             * 
             * @param command Command
             */
            void ExecutePingCommand(const Command& command);

            /**
             * @brief Execute an 'info' command
             * 
             * @param command Command
             */
            void ExecuteInfoCommand(const Command& command);

            /**
             * @brief Execute an 'event' command
             * 
             * @param command Command
             */
            void ExecuteEventCommand(const Command& command);

            /**
             * @brief Execute an 'set' command
             * 
             * @param command Command
             */
            void ExecuteSetCommand(const Command& command);

            /**
             * @brief Execute an 'get' command
             * 
             * @param command Command
             */
            void ExecuteGetCommand(const Command& command);

            /**
             * @brief Execute an 'rediscover' command
             * 
             * @param command Command
             */
            void ExecuteRediscoverCommand(const Command& command);

        public:

            /**
             * @brief Construct a new Loop object
             * 
             * @param endpoint Endpoint to use
             */
            Loop(Endpoint* endpoint);

            /**
             * @brief Run the command loop
             * 
             */
            void Run();
    };

}