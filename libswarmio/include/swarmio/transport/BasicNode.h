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

        public:

            /**
             * @brief Construct a new BasicNode object
             * 
             * @param name Discoverable name
             */
            BasicNode(const std::string& uuid)
                : _uuid(uuid) { }

            /**
             * @brief Get the unique identifier of the node
             * 
             * @return const std::string&
             */
            virtual const std::string& GetUUID() const override
            {
                return _uuid;
            }
    };
}