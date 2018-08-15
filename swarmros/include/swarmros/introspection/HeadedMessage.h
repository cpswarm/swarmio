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
     * @brief HeadedMessage implements an interface with
     *        the ROS type system to send and receive
     *        messages of with a header field.
     */
    class HeadedMessage final : public Message
    {
        private:

            /**
             * @brief Header
             * 
             */
            swarmio::data::Map _header;

            /**
             * @brief Content
             * 
             */
            swarmio::data::Map _content;

        public:

            /**
             * @brief Constant pointer type
             * 
             */
            typedef boost::shared_ptr<HeadedMessage const> ConstPtr;
 
            /**
             * @brief Pointer type
             * 
             */
            typedef boost::shared_ptr<HeadedMessage> Ptr;

            /**
             * @brief Construct an empty HeadedMessage instance
             * 
             */
            HeadedMessage() { }

            /**
             * @brief Build an HeadedMessage instance from a header and content map
             * 
             * @param type ROS type
             * @param header Header
             * @param header Content
             */
            HeadedMessage(const std::string& type, const swarmio::data::Map& header, const swarmio::data::Map& content)
                : Message(type), _header(header), _content(content) { }

            /**
             * @brief Get content
             * 
             * @return const swarmio::data::Map& 
             */
            const swarmio::data::Map& GetContent() const
            {
                return _content;
            }

            /**
             * @brief Get mutable content
             * 
             * @return const swarmio::data::Map& 
             */
            swarmio::data::Map& GetMutableContent()
            {
                return _content;
            }

            /**
             * @brief Set content
             * 
             * @param value 
             */
            void SetContent(const swarmio::data::Map& value)
            {
                _content = value;
            }


            /**
             * @brief Get header
             * 
             * @return const swarmio::data::Map& 
             */
            const swarmio::data::Map& GetHeader() const
            {
                return _header;
            }

            /**
             * @brief Get mutable header
             * 
             * @return const swarmio::data::Map& 
             */
            swarmio::data::Map& GetMutableHeader()
            {
                return _header;
            }

            /**
             * @brief Set header
             * 
             * @param value 
             */
            void SetHeader(const swarmio::data::Map& value)
            {
                _header = value;
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
        struct IsMessage<swarmros::introspection::HeadedMessage> : public IsMessage<swarmros::introspection::Message> { };

        /**
         * @brief Trait to mark const Message as a message
         * 
         */
        template<> 
        struct IsMessage<const swarmros::introspection::HeadedMessage> : public IsMessage<const swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to accept messages with any MD5 sum and 
         *        to calcualte the MD5 sum of outgoing messages
         * 
         */
        template<> 
        struct MD5Sum<swarmros::introspection::HeadedMessage> : public MD5Sum<swarmros::introspection::Message> { };
        
        /**
         * @brief Trait to retreive the full data type of the message
         * 
         */
        template<> 
        struct DataType<swarmros::introspection::HeadedMessage> : public DataType<swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to retreive the definition of an Message instance
         * 
         */
        template<> 
        struct Definition<swarmros::introspection::HeadedMessage> : public Definition<swarmros::introspection::Message> { };
    }

    namespace serialization
    {
        /**
         * @brief Trait to set up callbacks before 
         *        deserialization of an Message
         */
        template<>
        struct PreDeserialize<swarmros::introspection::HeadedMessage>
        {
            /**
             * @brief Before deserialization, sets the data
             *        type of an HeadedMessage instance
             * 
             * @param params Deserialization parameters
             */
            static void notify(const PreDeserializeParams<swarmros::introspection::HeadedMessage>& params)
            {
                params.message->SetType((*params.connection_header)["type"]);
            }
        };

        /**
         * @brief Trait to perform the serialization and
         *        deserialization of an HeadedMessage instance
         */
        template<> 
        struct Serializer<swarmros::introspection::HeadedMessage>
        {
            /**
             * @brief Uses the message serializer to calculate the
             *        serialized size of the underlying variant.
             * 
             * @param message Message
             * @return uint32_t
             */
            inline static uint32_t serializedLength(const swarmros::introspection::HeadedMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    return serializer->CalculateSerializedHeaderLength(message.GetHeader(), stack) + 
                            serializer->CalculateSerializedContentLength(message.GetContent(), stack);
                }
                else
                {
                    throw swarmio::Exception("Trying to serialize HeadedMessage without serializer");
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
            inline static void write(Stream& stream, const swarmros::introspection::HeadedMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    serializer->SerializeHeader(dynamic_cast<ros::serialization::OStream&>(stream), message.GetHeader(), stack);
                    serializer->SerializeContent(dynamic_cast<ros::serialization::OStream&>(stream), message.GetContent(), stack);
                }
                else
                {
                    throw swarmio::Exception("Trying to serialize HeadedMessage without serializer");
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
            inline static void read(Stream& stream, swarmros::introspection::HeadedMessage& message)
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    message.SetHeader(serializer->DeserializeHeader(dynamic_cast<ros::serialization::IStream&>(stream), stack));
                    message.SetContent(serializer->DeserializeContent(dynamic_cast<ros::serialization::IStream&>(stream), stack));
                }
                else
                {
                    throw swarmio::Exception("Trying to deserialize HeadedMessage without serializer");
                }
            }
        };
    }
}