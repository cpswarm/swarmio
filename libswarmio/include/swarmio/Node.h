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
             * @brief Returns the (possibly non-unique) name of the node
             * 
             * @return const std::string& 
             */
            virtual const std::string& GetName() const = 0;

            /**
             * @brief Get the class of the underlying device
             * 
             * @return const std::string& 
             */
            virtual const std::string& GetDeviceClass() const = 0;

            /**
             * @brief Get a user-readable description of the node.
             * 
             * @return std::string 
             */
            virtual std::string GetDescription() const = 0;

            /**
             * @brief Checks whether the Node is reachable.
             * 
             * @return True if the Node is online.
             */
            virtual bool IsOnline() const = 0;

            /**
             * @brief Destroy the Node object
             * 
             */
            virtual ~Node() { }
    };
}