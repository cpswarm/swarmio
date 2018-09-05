#pragma once

#include <swarmros/introspection/Serializer.h>
#include <swarmros/introspection/Field.h>
#include <swarmros/introspection/ConstantField.h>
#include <list>
#include <functional>

namespace swarmros::introspection 
{
    /**
     * @brief Serializer for full-fledged message types
     * 
     */
    class MessageSerializer : public Serializer
    {
        private:

            /**
             * @brief Map of message serializers
             * 
             */
            static std::map<std::string, std::unique_ptr<MessageSerializer>> _messageSerializers;

            /**
             * @brief Variable fields
             * 
             */
            std::list<std::unique_ptr<Field>> _fields; 

            /**
             * @brief Constant fields
             * 
             */
            std::list<ConstantField> _constants; 

            /**
             * @brief Whether the message begins  
             *        with the standard header.
             * 
             */
            bool _hasHeader;

            /**
             * @brief For certain messages, the normal field processing 
             *        is skipped and a single field is used to deserialize
             *        the entire message.
             * 
             */
            const Field* _shortcut;

            /**
             * @brief Package name
             * 
             */
            std::string _package;

            /**
             * @brief Full name
             * 
             */
            std::string _fullName;

            /**
             * @brief Canonical definition of the message
             * 
             */
            std::string _canonicalDefinition;

            /**
             * @brief MD5 hash of the message
             * 
             */
            std::string _hash;

            /**
             * @brief Build a canonical definition for the message
             * 
             * @return std::string 
             */
            std::string BuildCanonicalDefinition();

            /**
             * @brief Calculate the MD5 hash used to identify the message type
             * 
             * @return std::string 
             */
            std::string CalculateHash();

        public:  

            /**
             * @brief Look up or build a reader for a message type
             * 
             * @param type Message type
             * @param parentPackage Package where the type was referenced
             * @return const Serializer& 
             */
            static const MessageSerializer& MessageSerializerForType(const std::string& type, const std::string& parentPackage);

            /**
             * @brief Construct a new MessageSerializer object
             * 
             * @param package Package name
             * @param name Message name
             * @param path Path to message definition file
             */
            MessageSerializer(const std::string& package, const std::string& name, const std::string& path);

            /**
             * @brief Get package name
             * 
             * @return const std::string& 
             */
            const std::string& GetPackage() const
            {
                return _package;
            }

            /**
             * @brief Get the fully qualified name for 
             *        the type behind the serializer
             * 
             * @return const std::string& 
             */
            virtual const std::string& GetFullName() const override
            {
                return _fullName;
            }

            /**
             * @brief Get the MD5 hash of the message definition file
             * 
             * @return const std::string& 
             */
            const std::string& GetHash() const
            {
                return _hash;
            }

            /**
             * @brief Get the canonical definition of the message
             * 
             * @return const std::string& 
             */
            const std::string& GetCanonicalDefinition() const
            {
                return _canonicalDefinition;
            }

            /**
             * @brief Checks whether the message fits the requirements 
             *        of a message with a header
             * 
             * @return True if the message has a header
             */
            bool HasHeader() const
            {
                return _hasHeader;
            }

            /**
             * @brief Get the length of the default value.
             * 
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t 
             */
            virtual uint32_t GetDefaultLength(const FieldStack& fieldStack) const override;

            /**
             * @brief Calculate the length of a serialized message in bytes
             * 
             * @param value Value
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t 
             */
            virtual uint32_t CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const override;

            /**
             * @brief Deserialize a stream into a variant
             * 
             * @param stream Input stream
             * @param fieldStack Stack of fields to determine location
             * @return swarmio::data::Variant 
             */
            virtual swarmio::data::Variant Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const override;

            /**
             * @brief Deserialize a stream into a variant array
             * 
             * @param stream Input stream
             * @param count Item count
             * @param fieldStack Stack of fields to determine location
             * @return swarmio::data::Variant 
             */
            virtual swarmio::data::Variant DeserializeArray(ros::serialization::IStream& stream, uint32_t count, const FieldStack& fieldStack) const override;

            /**
             * @brief Serialize a variant onto a stream
             * 
             * @param stream Output stream
             * @param value Value to serialize
             * @param fieldStack Stack of fields to determine location
             */
            virtual void Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const override;

            /**
             * @brief Write the default value to the stream
             * 
             * @param stream Output stream
             * @param fieldStack Stack of fields to determine location
             */
            virtual void EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const override;

            /**
             * @brief Build a field descriptor for the underlying type
             * 
             * @return swarmio::data::discovery::Field 
             */
            virtual swarmio::data::discovery::Field GetFieldDescriptor() const override;

            /**
             * @brief Calculate the length of a serialized message 
             *        in bytes from a variant, with or without its header.
             * 
             * @param value Value
             * @param skipCount Number of fields to skip
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t 
             */
            uint32_t CalculateSerializedLength(const swarmio::data::Variant& value, unsigned skipCount, const FieldStack& fieldStack) const;

            /**
             * @brief Calculate the length of a serialized message
             *        in bytes from a map, with or without its header.
             * 
             * @param value Value
             * @param skipCount Number of fields to skip
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t 
             */
            uint32_t CalculateSerializedLength(const swarmio::data::Map& value, unsigned skipCount, const FieldStack& fieldStack) const;

            /**
             * @brief Serialize a variant onto a stream, with or
             *        without its header.
             * 
             * @param stream Output stream
             * @param value Value to serialize
             * @param skipCount Number of fields to skip
             * @param fieldStack Stack of fields to determine location
             */
            void Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, unsigned skipCount, const FieldStack& fieldStack) const;

            /**
             * @brief Serialize a map onto a stream, with or 
             *        without its header.
             * 
             * @param stream Output stream
             * @param value Value to serialize             
             * @param skipCount Number of fields to skip
             * @param fieldStack Stack of fields to determine location
             */
            void Serialize(ros::serialization::OStream& stream, const swarmio::data::Map& value, unsigned skipCount, const FieldStack& fieldStack) const;

            /**
             * @brief Deserialize a stream into a variant, with or 
             *        without its header.
             * 
             * @param stream Input stream
             * @param fieldStack Stack of fields to determine location
             * @param skipCount Number of fields to skip
             * @return swarmio::data::Variant 
             */
            swarmio::data::Variant Deserialize(ros::serialization::IStream& stream, unsigned skipCount, const FieldStack& fieldStack) const;

            /**
             * @brief Deserialize a stream into a map, with or 
             *        without its header.
             * 
             * @param stream Input stream
             * @param map Map to fill values with
             * @param fieldStack Stack of fields to determine location
             * @param skipCount Number of fields to skip
             */
            void Deserialize(ros::serialization::IStream& stream, swarmio::data::Map& map, unsigned skipCount, const FieldStack& fieldStack) const;

            /**
             * @brief Build a field descriptor for the underlying type,
             *        with or without its header.
             * 
             * @param skipCount Number of fields to skip
             * @return swarmio::data::discovery::Schema 
             */
            swarmio::data::discovery::Schema GetSchemaDescriptor(unsigned skipCount) const; 

            /**
             * @brief Enumerate the fields of the message
             *        with a function.
             * 
             * @param enumerator Enumerator function
             */
            void EnumerateFields(std::function<bool(unsigned, const Field&)> enumerator) const;
    };
}