#pragma once

#include <swarmros/introspection/MessageSerializer.h>
#include <swarmio/Exception.h>
#include <ros/ros.h>
#include <ros/message_traits.h>

namespace swarmros::introspection 
{
    /**
     * @brief Message is the base class for message
     *        types supported by the introspection library.
     * 
     */
    class Message
    {
        private:

            /**
             * @brief Reference to the serializer 
             *        used to construct this message
             * 
             */
            const MessageSerializer* _serializer;

            /**
             * @brief ROS type
             * 
             */
            std::string _type;

        protected:

            /**
             * @brief Construct an empty Message instance
             * 
             */
            Message() 
                : _serializer(nullptr), _type("*") { }

            /**
             * @brief Build an Message instance for a specific type
             * 
             * @param type ROS type
             */
            Message(const std::string& type)
            {
                SetType(type);
            }

        public:

            /**
             * @brief Set the type of the object
             *        and fetch its serializer
             * 
             * @param type ROS type
             */
            void SetType(const std::string& type)
            {
                _serializer = &MessageSerializer::MessageSerializerForType(type, "swarmros");
                _type = _serializer->GetFullName();
            }

            /**
             * @brief Get the fully qualified
             *        type of the message
             * 
             * @return const std::string& 
             */
            const std::string& GetType() const
            {
                return _type;
            }

            /**
             * @brief Erase type information from message
             * 
             */
            void EraseType()
            {
                _serializer = nullptr;
                _type = "*";
            }

            /**
             * @brief Get the associated serializer
             * 
             * @return const MessageSerializer* 
             */
            const MessageSerializer* GetSerializer() const
            {
                return _serializer;
            }
    };
}

namespace ros::message_traits
{
    /**
     * @brief Trait to mark Message as a message
     * 
     */
    template<> 
    struct IsMessage<swarmros::introspection::Message> : TrueType { };

    /**
     * @brief Trait to mark const Message as a message
     * 
     */
    template<> 
    struct IsMessage<const swarmros::introspection::Message> : TrueType { };

    /**
     * @brief Trait to accept messages with any MD5 sum and 
     *        to calcualte the MD5 sum of outgoing messages
     * 
     */
    template<> 
    struct MD5Sum<swarmros::introspection::Message>
    {
        /**
         * @brief Uses the message serializer to retreive 
         *        the MD5 hash of the message definition
         * 
         * @param message Message
         * @return const char* 
         */
        static const char* value(const swarmros::introspection::Message& message) 
        { 
            auto serializer = message.GetSerializer();
            if (serializer != nullptr)
            {
                return serializer->GetHash().c_str();
            }
            else
            {
                throw swarmio::Exception("Trying to serialize Message without serializer");
            }
        }

        /**
         * @brief Returns a wildcard MD5 hash to accept all messages
         * 
         * @return const char* 
         */
        static const char* value() 
        { 
            return "*"; 
        }
    };

    /**
     * @brief Trait to retreive the full data type of the message
     * 
     */
    template<> 
    struct DataType<swarmros::introspection::Message>
    {
        /**
         * @brief Uses the message serializer to retreive 
         *        the full name of the underlying data type.
         * 
         * @param message Message
         * @return const char* 
         */
        static const char* value(const swarmros::introspection::Message& message) 
        {
            auto serializer = message.GetSerializer();
            if (serializer != nullptr)
            {
                return message.GetType().c_str();
            }
            else
            {
                throw swarmio::Exception("Trying to serialize Message without serializer");
            }             
        }

        /**
         * @brief Returns a wildcard data type to accept all messages
         * 
         * @return const char* 
         */
        static const char* value() 
        {
                return "*"; 
        }
    };

    /**
     * @brief Trait to retreive the definition of an Message instance
     * 
     */
    template<> 
    struct Definition<swarmros::introspection::Message>
    {
        /**
         * @brief Uses the message serializer to retreive 
         *        the canonical definition of the underlying data type.
         * 
         * @param message Message
         * @return const char* 
         */
        static const char* value(const swarmros::introspection::Message& message) 
        {
            auto serializer = message.GetSerializer();
            if (serializer != nullptr)
            {
                return serializer->GetCanonicalDefinition().c_str();
            }
            else
            {
                throw swarmio::Exception("Trying to serialize Message without serializer");
            }
        }
    };
}