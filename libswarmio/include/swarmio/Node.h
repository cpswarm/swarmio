#pragma once

#include <swarmio/data/Message.pb.h>

namespace swarmio
{
    /**
     * @brief Represents a Node the Endpoint knows about
     *        and can send messages to.
     */
    class SWARMIO_API Node
    {
        public:

            /**
             * @brief Returns the unique identifier of the node
             * 
             * @return const std::string&
             */
            virtual const std::string& GetUUID() const = 0;

            /**
             * @brief Get a user-readable description of the node.
             * 
             * @return std::string 
             */
            virtual std::string GetDescription() const = 0;

            /**
             * @brief Destroy the Node object
             * 
             */
            virtual ~Node() { }
    };
}