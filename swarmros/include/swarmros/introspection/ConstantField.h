#pragma once

#include <swarmros/introspection/Field.h>
#include <swarmio/data/Variant.pb.h>
#include <string>

namespace swarmros::introspection 
{
    /**
     * @brief Field for constant values
     * 
     */
    class ConstantField : public Field
    {
        private:

            /**
             * @brief Value
             * 
             */
            swarmio::data::Variant _value;

            /**
             * @brief Raw string value
             * 
             */
            std::string _rawValue;

            /**
             * @brief Parse a boolean value
             * 
             * @param value Value
             */
            void ParseBool(const std::string& value);

            /**
             * @brief Parse a floating point value
             * 
             * @param value Value
             */
            void ParseFloat(const std::string& value);

            /**
             * @brief Parse a string value
             * 
             * @param value Value
             */
            void ParseString(const std::string& value);

            /**
             * @brief Parse a signed integer value
             * 
             * @param value Value 
             */
            void ParseSigned(const std::string& value);

            /**
             * @brief Parse an unsigned integer value
             * 
             * @param value Value
             */
            void ParseUnsigned(const std::string& value);

        public:  

            /**
             * @brief Construct a new ConstantField object
             * 
             * @param name Name
             * @param serializer Serializer
             * @param value Value
             */
            ConstantField(const std::string& name, const Serializer& serializer, const std::string& value);

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
    };
}