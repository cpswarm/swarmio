#pragma once

#include <swarmio/transport/BasicNode.h>
#include <string>
#include <sstream>
#include <atomic>

namespace swarmio::transport::zyre
{
    /**
     * @brief Forward declare ZyreEndpoint
     * 
     */
    class ZyreEndpoint;

    /**
     * @brief A Node as discovered by the Zyre protocol
     * 
     */
    class SWARMIO_API ZyreNode final : public BasicNode
    {
        private:

            /**
             * @brief Only ZyreEdpoint can manage these objects
             * 
             */
            friend ZyreEndpoint;

            /**
             * @brief IP address of the node
             * 
             */
            std::string _address;

            /**
             * @brief Is the Node currently online?
             * 
             */
            std::atomic<bool> _online;

            /**
             * @brief Mark the Node as online or offline
             * 
             * @param online Current status
             */
            void SetOnline(bool online)
            {
                _online = online;
            }

        public:

            /**
             * @brief Construct a new ZyreNode object
             * 
             * @param uuid UUID
             * @param name Discoverable name
             * @param address IP address
             */
            ZyreNode(const std::string& uuid, const std::string& name, const std::string& address)
                : BasicNode(uuid, name), _address(address), _online(false) { }

            /**
             * @brief Is the Node currently online?
             * 
             * @return True if online
             */
            bool IsOnline() const override
            {
                return _online;
            }

            /**
             * @brief Get the IP address of the node
             * 
             * @return const std::string& 
             */
            const std::string& GetAddress() const
            {
                return _address;
            }

            /**
             * @brief Get a user-readable description of the node.
             * 
             * @return std::string 
             */
            virtual std::string GetDescription() const override
            {
                std::ostringstream stream;
                stream << "online=" << (_online ? "true" : "false") << ", "
                       << "address=" << _address;
                return stream.str();
            }
    };
}