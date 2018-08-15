#pragma once

#include <swarmros/introspection/Serializer.h>
#include <swarmio/data/Variant.pb.h>
#include <string>
#include <sstream>

namespace swarmros::introspection 
{
    /**
     * @brief A Field represents an entry in a message
     *        reader that can read one of its fields.
     * 
     */
    class Field : public Serializer
    {
        protected:

            /**
             * @brief Serializer
             * 
             */
            const Serializer& _serializer;

            /**
             * @brief Get the type name for the serializer
             * 
             * @param stream Stream
             * @param forHash True if the type name will be used for the MD5 hash
             */
            void WriteTypeName(std::stringstream& stream, bool forHash) const;

        public:

            /**
             * @brief Construct a new Field object
             * 
             * @param name Name
             * @param serializer Serializer
             */
            Field(const std::string& name, const Serializer& serializer)
                : Serializer(name), _serializer(serializer) { }

            /**
             * @brief Write the definition of this field 
             *        to a string stream
             * 
             * @param stream Stream
             * @param forHash True if the definition will be used for the MD5 hash
             */
            virtual void WriteDefinition(std::stringstream& stream, bool forHash) const;

            /**
             * @brief Get the associated serializer
             * 
             * @return const Serializer&
             */
            const Serializer& GetSerializer()
            {
                return _serializer;
            }

            /**
             * @brief Calculate the length of a serialized message in bytes
             * 
             * @param value Value
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t 
             */
            virtual uint32_t CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const override;

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
             * @brief Deserialize a stream into a variant
             * 
             * @param stream Input stream
             * @param fieldStack Stack of fields to determine location
             * @return swarmio::data::Variant 
             */
            virtual swarmio::data::Variant Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const override;

            /**
             * @brief Deserialize a stream into a variant array - not 
             *        supported on Fields.
             * 
             * @param stream Input stream
             * @param count Item count
             * @param fieldStack Stack of fields to determine location
             * @return swarmio::data::Variant 
             */
            virtual swarmio::data::Variant DeserializeArray(ros::serialization::IStream& stream, uint32_t count, const FieldStack& fieldStack) const override;

            /**
             * @brief Build a field descriptor for the underlying type
             * 
             * @return swarmio::data::discovery::Field 
             */
            virtual swarmio::data::discovery::Field GetFieldDescriptor() const override;
    };
}