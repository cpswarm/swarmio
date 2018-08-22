#pragma once

#include <swarmros/introspection/Message.h>
#include <swarmros/introspection/MessageSerializer.h>
#include <swarmros/introspection/RootFieldStack.h>
#include <swarmros/EventHeader.h>

namespace swarmros::bridge 
{
    /**
     * @brief EventMessage implements an interface with
     *        the ROS type system to send and receive
     *        events.
     */
    class EventMessage : public introspection::Message
    {
        private:

            /**
             * @brief Event header
             * 
             */
            EventHeader _eventHeader;

            /**
             * @brief Parameters
             * 
             */
            swarmio::data::Map _parameters;

        public:

            /**
             * @brief Checks whether a serializer has the correct layout.
             * 
             * @param serializer Serializer
             * @return True if the serializer can be used to serialize events.
             */
            static bool IsEventSerializer(const introspection::MessageSerializer& serializer)
            {
                bool hasHeader = false;
                serializer.EnumerateFields([&hasHeader, &serializer](unsigned n, const introspection::Field& field) {
                    if (n == 0 && serializer.HasHeader())
                    {
                        // Skip this field
                        return true;
                    }
                    else if (field.GetSerializer().GetFullName() == "swarmros/EventHeader")
                    {
                        // We found it!
                        hasHeader = true;
                        return false;
                    }
                    else
                    {
                        // No point looking anymore
                        return false;
                    }                    
                });
                return hasHeader;
            }

            /**
             * @brief Constant pointer type
             * 
             */
            typedef boost::shared_ptr<EventMessage const> ConstPtr;
 
            /**
             * @brief Pointer type
             * 
             */
            typedef boost::shared_ptr<EventMessage> Ptr;

            /**
             * @brief Construct an empty EventMessage instance
             * 
             */
            EventMessage() { }

            /**
             * @brief Build an EventMessage instance from an event
             * 
             * @param type ROS type
             * @param node Node UUID
             * @param event Event
             */
            EventMessage(const std::string& type, const std::string& name, const std::string& node, const swarmio::data::Map& parameters)
                : Message(type), _parameters(parameters)
            {
                _eventHeader.name = name;
                _eventHeader.node = node;   
            }

            /**
             * @brief Get event name
             * 
             * @return const std::string& 
             */
            const std::string& GetName() const
            {
                return _eventHeader.name;
            }

            /**
             * @brief Set event name
             * 
             * @param name Name
             */
            void SetName(const std::string& name)
            {
                _eventHeader.name = name;
            }

            /**
             * @brief Get node UUID
             * 
             * @return const std::string& 
             */
            const std::string& GetNode() const
            {
                return _eventHeader.node;
            }

            /**
             * @brief Set node UUID
             * 
             * @param node Node
             */
            void SetNode(const std::string& node)
            {
                _eventHeader.node = node;
            }

            /**
             * @brief Get event parameters
             * 
             * @return const swarmio::data::event::Notification& 
             */
            const swarmio::data::Map& GetParameters() const
            {
                return _parameters;
            }

            /**
             * @brief Get mutable event parameters
             * 
             * @return const swarmio::data::event::Notification& 
             */
            swarmio::data::Map& GetMutableParameters()
            {
                return _parameters;
            }

            /**
             * @brief Set event parameters
             * 
             * @param event Event
             */
            void SetParameters(const swarmio::data::Map& parameters)
            {
                _parameters = parameters;
            }

            /**
             * @brief Get event header
             * 
             * @return const EventHeader& 
             */
            const EventHeader& GetEventHeader() const
            {
                return _eventHeader;
            }

            /**
             * @brief Get mutable event header
             * 
             * @return EventHeader& 
             */
            EventHeader& GetMutableEventHeader()
            {
                return _eventHeader;
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
        struct IsMessage<swarmros::bridge::EventMessage> : public IsMessage<swarmros::introspection::Message> { };

        /**
         * @brief Trait to mark const Message as a message
         * 
         */
        template<> 
        struct IsMessage<const swarmros::bridge::EventMessage> : public IsMessage<const swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to accept messages with any MD5 sum and 
         *        to calcualte the MD5 sum of outgoing messages
         * 
         */
        template<> 
        struct MD5Sum<swarmros::bridge::EventMessage> : public MD5Sum<swarmros::introspection::Message> { };
        
        /**
         * @brief Trait to retreive the full data type of the message
         * 
         */
        template<> 
        struct DataType<swarmros::bridge::EventMessage> : public DataType<swarmros::introspection::Message> { };
 
        /**
         * @brief Trait to retreive the definition of an Message instance
         * 
         */
        template<> 
        struct Definition<swarmros::bridge::EventMessage> : public Definition<swarmros::introspection::Message> { };
    }

    namespace serialization
    {
        /**
         * @brief Trait to set up callbacks before 
         *        deserialization of an EventMessage
         */
        template<>
        struct PreDeserialize<swarmros::bridge::EventMessage>
        {
            /**
             * @brief Before deserialization, sets the data
             *        type of an EventMessage instance
             * 
             * @param params Deserialization parameters
             */
            static void notify(const PreDeserializeParams<swarmros::bridge::EventMessage>& params)
            {
                params.message->SetType((*params.connection_header)["type"]);
            }
        };

        /**
         * @brief Trait to perform the serialization and
         *        deserialization of an EventMessage instance
         */
        template<> 
        struct Serializer<swarmros::bridge::EventMessage>
        {
            /**
             * @brief Uses the message serializer to calculate the
             *        serialized size of the underlying variant.
             * 
             * @param message Message
             * @return uint32_t
             */
            inline static uint32_t serializedLength(const swarmros::bridge::EventMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    // Handle header
                    uint32_t headerLength = Serializer<swarmros::introspection::Message>::serializedLength(message);

                    // Handle event header
                    headerLength += Serializer<swarmros::EventHeader>::serializedLength(message.GetEventHeader());

                    // Handle contents
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    return headerLength + serializer->CalculateSerializedLength(message.GetParameters(), serializer->HasHeader() ? 2 : 1, stack);
                }
                else
                {
                    throw Exception("Trying to serialize EventMessage without serializer");
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
            inline static void write(Stream& stream, const swarmros::bridge::EventMessage& message) 
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    // Handle header
                    Serializer<swarmros::introspection::Message>::write(stream, message);

                    // Handle event header
                    Serializer<swarmros::EventHeader>::write(stream, message.GetEventHeader());

                    // Handle contents
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    serializer->Serialize(dynamic_cast<ros::serialization::OStream&>(stream), message.GetParameters(), serializer->HasHeader() ? 2 : 1, stack);
                }
                else
                {
                    throw Exception("Trying to serialize EventMessage without serializer");
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
            inline static void read(Stream& stream, swarmros::bridge::EventMessage& message)
            {
                auto serializer = message.GetSerializer();
                if (serializer != nullptr)
                {
                    // Handle header
                    Serializer<swarmros::introspection::Message>::read(stream, message);

                    // Handle event header
                    Serializer<swarmros::EventHeader>::read(stream, message.GetMutableEventHeader());

                    // Handle contents
                    swarmros::introspection::RootFieldStack stack(serializer->GetFullName());
                    serializer->Deserialize(dynamic_cast<ros::serialization::IStream&>(stream), message.GetMutableParameters(), serializer->HasHeader() ? 2 : 1, stack);
                }
                else
                {
                    throw Exception("Trying to deserialize EventMessage without serializer");
                }
            }
        };
    }
}