#pragma once

#include <swarmio/transport/BasicNode.h>
#include <string>
#include <sstream>
#include <atomic>
namespace swarmio::transport::zyre
{
    /**
     * @brief A Node as discovered by the Zyre protocol
     * 
     */
    class SWARMIO_API ZyreNode final : public BasicNode
    {
        private:

            /**
             * @brief The UUID of the node
             * 
             */
            std::string _name;

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

        public:

            /**
             * @brief Construct a new ZyreNode object
             * 
             * @param uuid UUID
             * @param name Discoverable name
             * @param address IP address
             */
            ZyreNode(const std::string& uuid, const std::string& name, const std::string& address)
                : BasicNode(uuid), _name(name), _address(address), _online(false) { }

            /**
             * @brief Get the UUID of the node
             * 
             * @return const std::string& 
             */
            const std::string& GetName() const
            {
                return _name;
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
             * @brief Is the Node currently online?
             * 
             * @return True if online
             */
            bool IsOnline() const
            {
                return _online;
            }

            /**
             * @brief Mark the Node as online or offline
             * 
             * @param online Current status
             */
            void SetOnline(bool online)
            {
                _online = online;
            }

            /**
             * @brief Get a user-readable description of the node.
             * 
             * @return std::string 
             */
            virtual std::string GetDescription() const override
            {
                std::ostringstream stream;
                stream << "name=" << _name << ", "
                       << "online=" << (_online ? "true" : "false") << ", "
                       << "address=" << _address;
                return stream.str();
            }
    };
}