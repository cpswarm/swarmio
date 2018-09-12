#pragma once

#include <swarmio/Node.h>

namespace swarmio::transport
{
    /**
     * @brief A simple Node with a UUID
     * 
     */
    class SWARMIO_API BasicNode : public Node
    {
        private:

            /**
             * @brief The unique identifier of the node
             * 
             */
            std::string _uuid;

            /**
             * @brief The name of the node
             * 
             */
            std::string _name;

            /**
             * @brief Device class
             * 
             */
            std::string _deviceClass;

        public:

            /**
             * @brief Construct a new BasicNode object
             * 
             * @param uuid Unique identifier
             * @param name Discoverable name
             * @param deviceClass Device class
             */
            BasicNode(const std::string& uuid, const std::string& name, const std::string& deviceClass)
                : _uuid(uuid), _name(name), _deviceClass(deviceClass) { }

            /**
             * @brief Get the unique identifier of the node
             * 
             * @return const std::string&
             */
            virtual const std::string& GetUUID() const override
            {
                return _uuid;
            }

            /**
             * @brief Returns the (possibly non-unique) name of the node
             * 
             * @return const std::string& 
             */
            virtual const std::string& GetName() const override
            {
                return _name;
            }

            /**
             * @brief Get the class of the underlying device
             * 
             * @return const std::string& 
             */
            virtual const std::string& GetDeviceClass() const override
            {
                return _deviceClass;
            }
    };
}