#pragma once

#include <swarmros/introspection/Serializer.h>
#include <swarmros/UnqualifiedException.h>
#include <string>

namespace swarmros::introspection 
{
    /**
     * @brief Values correspond to builtin types
     *        of the ROS msg specification.
     * 
     */ 
    enum PrimitiveType
    {
        UNKNOWN = 0,
        BOOL,
        INT8,
        UINT8,
        INT16,
        UINT16,
        INT32,
        UINT32,
        INT64,
        UINT64,
        FLOAT32,
        FLOAT64,
        STRING,
        TIME,
        DURATION
    };

    /**
     * @brief Serializer for primitive builtin types
     * 
     */
    class PrimitiveSerializer : public Serializer
    {
        private:

            /**
             * @brief Type
             * 
             */
            PrimitiveType _type;

        public:  

            /**
             * @brief Construct a new PrimitiveSerializer object
             * 
             * @param type Type
             */
            PrimitiveSerializer(const std::string& name, PrimitiveType type)
                : Serializer(name), _type(type) { }

            /**
             * @brief Get underlying data type
             * 
             * @return PrimitiveType 
             */
            PrimitiveType GetType() const 
            {
                return _type;
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
             * @brief Deserialize a stream into a variant array
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