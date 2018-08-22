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
     * @brief VariantMessage implements an interface with
     *        the ROS type system to send and receive
     *        messages of any known type.
     */
    class VariantMessage : public Message
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
            typedef boost::shared_ptr<VariantMessage const> ConstPtr;
 
            /**
             * @brief Pointer type
             * 
             */
            typedef boost::shared_ptr<VariantMessage> Ptr;

            /**
             * @brief Construct an empty VariantMessage instance
             * 
             */
            VariantMessage() { }

            /**
             * @brief Build an VariantMessage instance from a Variant
             * 
             * @param type ROS type
             * @param value Variant value
             */
            VariantMessage(const std::string& type, const swarmio::data::Variant& value)
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
        struct IsMessage<swarmros::introspection::VariantMessage> : public IsMessage<swarmros::introspection::Message> { };

        /**
         * @brief Trait to mark const Message as a message
         * 
         */
        template<> 
        struct IsMessage<const swarmros::introspection::VariantMessage> : public IsMessage<const swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to accept messages with any MD5 sum and 
         *        to calcualte the MD5 sum of outgoing messages
         * 
         */
        template<> 
        struct MD5Sum<swarmros::introspection::VariantMessage> : public MD5Sum<swarmros::introspection::Message> { };
        
        /**
         * @brief Trait to retreive the full data type of the message
         * 
         */
        template<> 
        struct DataType<swarmros::introspection::VariantMessage> : public DataType<swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to retreive the definition of an Message instance
         * 
         */
        template<> 
        struct Definition<swarmros::introspection::VariantMessage> : public Definition<swarmros::introspection::Message> { };
    }

    namespace serialization
    {
        /**
         * @brief Trait to set up callbacks before 
         *        deserialization of an VariantMessage
         */
        template<>
        struct PreDeserialize<swarmros::introspection::VariantMessage>
        {
            /**
             * @brief Before deserialization, sets the data
             *        type of an VariantMessage instance
             * 
             * @param params Deserialization parameters
             */
            static void notify(const PreDeserializeParams<swarmros::introspection::VariantMessage>& params)
            {
                params.message->SetType((*params.connection_header)["type"]);
            }
        };

        /**
         * @brief Trait to perform the serialization and
         *        deserialization of an VariantMessage instance
         */
        template<> 
        struct Serializer<swarmros::introspection::VariantMessage>
        {
            /**
             * @brief Uses the message serializer to calculate the
             *        serialized size of the underlying variant.
             * 
             * @param message Message
             * @return uint32_t
             */
            inline static uint32_t serializedLength(const swarmros::introspection::VariantMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    // Handle header
                    uint32_t headerLength = Serializer<swarmros::introspection::Message>::serializedLength(message);

                    // Handle contents
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    return headerLength + serializer->CalculateSerializedLength(message.GetValue(), serializer->HasHeader() ? 1 : 0, stack);
                }
                else
                {
                    throw Exception("Trying to serialize VariantMessage without serializer");
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
            inline static void write(Stream& stream, const swarmros::introspection::VariantMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    // Handle header
                    Serializer<swarmros::introspection::Message>::write(stream, message);

                    // Handle contents
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    serializer->Serialize(dynamic_cast<ros::serialization::OStream&>(stream), message.GetValue(), serializer->HasHeader() ? 1 : 0, stack);
                }
                else
                {
                    throw Exception("Trying to serialize VariantMessage without serializer");
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
            inline static void read(Stream& stream, swarmros::introspection::VariantMessage& message)
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    // Handle header
                    Serializer<swarmros::introspection::Message>::read(stream, message);

                    // Handle contents
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    message.SetValue(serializer->Deserialize(dynamic_cast<ros::serialization::IStream&>(stream), serializer->HasHeader() ? 1 : 0, stack));
                }
                else
                {
                    throw Exception("Trying to deserialize VariantMessage without serializer");
                }
            }
        };
    }
}