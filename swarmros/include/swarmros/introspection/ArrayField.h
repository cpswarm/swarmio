#pragma once

#include <swarmros/introspection/Field.h>

namespace swarmros::introspection 
{
    /**
     * @brief Field for dynamic arrays
     * 
     */
    class ArrayField : public Field
    {
        public:  

            /**
             * @brief Inherit constructor
             * 
             */
            using Field::Field;

            /**
             * @brief Write the definition of this field 
             *        to a string stream
             * 
             * @param stream Stream
             * @param forHash True if the definition will be used for the MD5 hash
             */
            virtual void WriteDefinition(std::stringstream& stream, bool forHash) const override;

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
             * @brief Serialize the count of the array onto a stream
             * 
             * @param stream Output stream
             * @param count Array count
             * @param fieldStack Stack of fields to determine location
             */
            virtual void SerializeCount(ros::serialization::OStream& stream, uint32_t count, const FieldStack& fieldStack) const;

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
             * @brief Deserialize the count of the array from a stream
             * 
             * @param stream Input stream
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t
             */
            virtual uint32_t DeserializeCount(ros::serialization::IStream& stream, const FieldStack& fieldStack) const;

            /**
             * @brief Build a field descriptor for the underlying type
             * 
             * @return swarmio::data::discovery::Field 
             */
            virtual swarmio::data::discovery::Field GetFieldDescriptor() const override;
    };
}