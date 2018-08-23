#pragma once

#include <swarmros/introspection/ArrayField.h>

namespace swarmros::introspection 
{
    /**
     * @brief Serializer for fixed arrays
     * 
     */
    class FixedArrayField : public ArrayField
    {
        private:

            /**
             * @brief Element count
             * 
             */
            uint32_t _count;

        public:  

            /**
             * @brief Construct a new FixedArrayField object
             * 
             * @param name Name
             * @param serializer Serializer
             * @param count Item count
             */
            FixedArrayField(const std::string& name, const Serializer& serializer, uint32_t count) 
                : ArrayField(name, serializer), _count(count) { }

            /**
             * @brief Write the definition of this field 
             *        to a string stream
             * 
             * @param stream Stream
             * @param forHash True if the definition will be used for the MD5 hash
             */
            virtual void WriteDefinition(std::stringstream& stream, bool forHash) const override;

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
             * @brief Serialize the count of the array onto a stream
             * 
             * @param stream Output stream
             * @param count Array count
             * @param fieldStack Stack of fields to determine location
             */
            virtual void SerializeCount(ros::serialization::OStream& stream, uint32_t count, const FieldStack& fieldStack) const override;

            /**
             * @brief Write the default value to the stream
             * 
             * @param stream Output stream
             * @param fieldStack Stack of fields to determine location
             */
            virtual void EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const override;

            /**
             * @brief Deserialize the count of the array from a stream
             * 
             * @param stream Input stream
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t
             */
            virtual uint32_t DeserializeCount(ros::serialization::IStream& stream, const FieldStack& fieldStack) const override;

            /**
             * @brief Build a field descriptor for the underlying type
             * 
             * @return swarmio::data::discovery::Field 
             */
            virtual swarmio::data::discovery::Field GetFieldDescriptor() const override;
    };
}