#pragma once

#include <swarmros/introspection/FieldStack.h>
#include <swarmio/data/Variant.pb.h>
#include <swarmio/data/discovery/Schema.pb.h>
#include <ros/serialization.h>
#include <string>
#include <memory>

namespace swarmros::introspection 
{
    /**
     * @brief Serializer is the base class for all 
     *        binary message interpreters.
     * 
     */
    class Serializer
    {
        private:

            /**
             * @brief List of built-in default serializers
             * 
             */
            static std::map<std::string, std::unique_ptr<Serializer>> _defaultSerializers;

            /**
             * @brief Load the list of default serializers
             * 
             * @return std::map<std::string, std::unique_ptr<Serializer>> 
             */
            static std::map<std::string, std::unique_ptr<Serializer>> LoadDefaultSerializers();

            /**
             * @brief Non-qualified name for the type behind the serializer
             * 
             */
            std::string _name;

        protected:

            /**
             * @brief Construct a new Serializer object
             * 
             */
            Serializer(const std::string& name)
                : _name(name) { }

            /**
             * @brief Look up or build a reader for a message type
             * 
             * @param type Message type
             * @param parentPackage Package where the type was referenced
             * @return const Serializer& 
             */
            static const Serializer& SerializerForType(const std::string& type, const std::string& parentPackage);

        public:

            /**
             * @brief Get the non-qualified name for 
             *        the type behind the serializer
             * 
             * @return const std::string& 
             */
            const std::string& GetName() const
            {
                return _name;
            }

            /**
             * @brief Get the fully qualified name for 
             *        the type behind the serializer
             * 
             * @return const std::string& 
             */
            virtual const std::string& GetFullName() const
            {
                return _name;
            }

            /**
             * @brief Calculate the length of a serialized message in bytes
             * 
             * @param value Value
             * @param fieldStack Stack of fields to determine location
             * @return uint32_t 
             */
            virtual uint32_t CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const = 0;

            /**
             * @brief Serialize a variant onto a stream
             * 
             * @param stream Output stream
             * @param value Value to serialize
             * @param fieldStack Stack of fields to determine location
             */
            virtual void Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const = 0;

            /**
             * @brief Deserialize a stream into a variant
             * 
             * @param stream Input stream
             * @param fieldStack Stack of fields to determine location
             * @return swarmio::data::Variant 
             */
            virtual swarmio::data::Variant Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const = 0;

            /**
             * @brief Deserialize a stream into a variant array
             * 
             * @param stream Input stream
             * @param count Item count
             * @param fieldStack Stack of fields to determine location
             * @return swarmio::data::Variant 
             */
            virtual swarmio::data::Variant DeserializeArray(ros::serialization::IStream& stream, uint32_t count, const FieldStack& fieldStack) const = 0;

            /**
             * @brief Write the default value to the stream
             * 
             * @param stream Output stream
             * @param fieldStack Stack of fields to determine location
             */
            virtual void EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const = 0;

            /**
             * @brief Build a field descriptor for the underlying type
             * 
             * @return swarmio::data::discovery::Field 
             */
            virtual swarmio::data::discovery::Field GetFieldDescriptor() const = 0;

            /**
             * @brief Destructor
             * 
             */
            virtual ~Serializer() { }
    };
}