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

        public:

            /**
             * @brief Construct a new BasicNode object
             * 
             * @param uuid Unique identifier
             * @param name Discoverable name
             */
            BasicNode(const std::string& uuid, const std::string& name)
                : _uuid(uuid), _name(name) { }

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
    };
}