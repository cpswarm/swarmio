#pragma once

#include <swarmros/introspection/Message.h>
#include <swarmros/introspection/MessageSerializer.h>
#include <swarmros/introspection/RootFieldStack.h>
#include <swarmio/Exception.h>
#include <ros/ros.h>
#include <ros/message_traits.h>

namespace swarmros::introspection 
{
    /**
     * @brief AnyMessage implements an interface with
     *        the ROS type system to send and receive
     *        messages of any known type.
     */
    class AnyMessage final : public Message
    {
        private:

            /**
             * @brief Current value
             * 
             */
            swarmio::data::Variant _value;

        public:

            /**
             * @brief Constant pointer type
             * 
             */
            typedef boost::shared_ptr<AnyMessage const> ConstPtr;
 
            /**
             * @brief Pointer type
             * 
             */
            typedef boost::shared_ptr<AnyMessage> Ptr;

            /**
             * @brief Construct an empty AnyMessage instance
             * 
             */
            AnyMessage() { }

            /**
             * @brief Build an AnyMessage instance from a Variant
             * 
             * @param type ROS type
             * @param value Variant value
             */
            AnyMessage(const std::string& type, const swarmio::data::Variant& value)
                : Message(type), _value(value) { }

            /**
             * @brief Get value
             * 
             * @return const swarmio::data::Variant& 
             */
            const swarmio::data::Variant& GetValue() const
            {
                return _value;
            }

            /**
             * @brief Get mutable value
             * 
             * @return const swarmio::data::Variant& 
             */
            swarmio::data::Variant& GetMutableValue()
            {
                return _value;
            }

            /**
             * @brief Set value
             * 
             * @param value 
             */
            void SetValue(const swarmio::data::Variant& value)
            {
                _value = value;
            }
    };
}

namespace ros
{
    namespace message_traits
    {
        /**
         * @brief Trait to mark Message as a message
         * 
         */
        template<> 
        struct IsMessage<swarmros::introspection::AnyMessage> : public IsMessage<swarmros::introspection::Message> { };

        /**
         * @brief Trait to mark const Message as a message
         * 
         */
        template<> 
        struct IsMessage<const swarmros::introspection::AnyMessage> : public IsMessage<const swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to accept messages with any MD5 sum and 
         *        to calcualte the MD5 sum of outgoing messages
         * 
         */
        template<> 
        struct MD5Sum<swarmros::introspection::AnyMessage> : public MD5Sum<swarmros::introspection::Message> { };
        
        /**
         * @brief Trait to retreive the full data type of the message
         * 
         */
        template<> 
        struct DataType<swarmros::introspection::AnyMessage> : public DataType<swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to retreive the definition of an Message instance
         * 
         */
        template<> 
        struct Definition<swarmros::introspection::AnyMessage> : public Definition<swarmros::introspection::Message> { };
    }

    namespace serialization
    {
        /**
         * @brief Trait to set up callbacks before 
         *        deserialization of an AnyMessage
         */
        template<>
        struct PreDeserialize<swarmros::introspection::AnyMessage>
        {
            /**
             * @brief Before deserialization, sets the data
             *        type of an AnyMessage instance
             * 
             * @param params Deserialization parameters
             */
            static void notify(const PreDeserializeParams<swarmros::introspection::AnyMessage>& params)
            {
                params.message->SetType((*params.connection_header)["type"]);
            }
        };

        /**
         * @brief Trait to perform the serialization and
         *        deserialization of an AnyMessage instance
         */
        template<> 
        struct Serializer<swarmros::introspection::AnyMessage>
        {
            /**
             * @brief Uses the message serializer to calculate the
             *        serialized size of the underlying variant.
             * 
             * @param message Message
             * @return uint32_t
             */
            inline static uint32_t serializedLength(const swarmros::introspection::AnyMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    return serializer->CalculateSerializedLength(message.GetValue(), stack);
                }
                else
                {
                    throw swarmio::Exception("Trying to serialize AnyMessage without serializer");
                }
            }

            /**
             * @brief Uses the message serializer to write the
             *        serialized representation of the underlying 
             *        variant onto an output stream.
             * 
             * @param stream Stream
             * @param message Message
             */
            template<typename Stream>
            inline static void write(Stream& stream, const swarmros::introspection::AnyMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    serializer->Serialize(dynamic_cast<ros::serialization::OStream&>(stream), message.GetValue(), stack);
                }
                else
                {
                    throw swarmio::Exception("Trying to serialize AnyMessage without serializer");
                }
            }

            /**
             * @brief Uses the message serializer to read the
             *        serialized representation of the underlying 
             *        variant from an output stream.
             * 
             * @param stream Stream
             * @param message Message
             */
            template<typename Stream>
            inline static void read(Stream& stream, swarmros::introspection::AnyMessage& message)
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    message.SetValue(serializer->Deserialize(dynamic_cast<ros::serialization::IStream&>(stream), stack));
                }
                else
                {
                    throw swarmio::Exception("Trying to deserialize AnyMessage without serializer");
                }
            }
        };
    }
}