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
             * @brief Message counter as nonce and replay attack
             * 
             */
            unsigned char _message_counter[24];

            /**
             * @brief Shared key
             * 
             */
            unsigned char _key[32];

            /**
             * @brief Public key
             * 
             */
            unsigned char _pubkey[32];

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
             * @brief Is the Node verified?
             * 
             */
            std::atomic<bool> _verified;

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
             * @param deviceClass Device class
             * @param address IP address
             */
            ZyreNode(const std::string& uuid, const std::string& name, const std::string& deviceClass, const std::string& address)
                : BasicNode(uuid, name, deviceClass), _address(address), _online(false) { }

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
             * @brief Get the shared cryptographic key of the node
             * 
             * @return const unsigned char& 
             */
            const unsigned char* GetKey() const
            {
                return _key;
            }

            /**
             * @brief Get the  public key of the node
             * 
             * @return const unsigned char& 
             */
            const unsigned char* GetPubKey() const
            {
                return _pubkey;
            }

            /**
             * @brief Set the shared cryptographic key and the public key of the node
             * 
             */
            void SetKeys(unsigned char* k, unsigned char* pk)
            {
                memcpy(_key, k, 32);
                memcpy(_pubkey, pk, 32);
            }

            /**
             * @brief Get the nonce counter of the node
             * 
             * @return unsigned char& 
             */
            const unsigned char* GetCtr() const
            {
                return _message_counter;
            }

                        /**
             * @brief Is the node verified?
             * 
             * @return unsigned char& 
             */
            const bool GetVerified() const
            {
                return _verified;
            }
        
                    /**
             * @brief Set the verified status of the node
             * 
             */
            void SetVerified()
            {
                _verified = true;
            }

            /**
             * @brief Increment the nonce counter the node
             * 
             */
            void IncrementCtr() const
            {
                for (int i = 23; i >= 0; i--) if (++const_cast <ZyreNode*>(this)->_message_counter[i]) break;
            }

            /**
             * @brief Set the nonce (counter)
             * 
             */
            void SetCtr(unsigned char* c) const
            {
                memcpy(const_cast <ZyreNode*>(this)->_message_counter, c, 24);
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