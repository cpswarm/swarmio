#pragma once

#include <swarmros/introspection/SchemaMismatchException.h>
#include <swarmio/data/Variant.pb.h>

namespace swarmros::introspection
{
    /**
     * @brief Exception thrown when the expected type of the
     *        Variant does not match the actual type.
     * 
     */
    class TypeMismatchException : public SchemaMismatchException
    {
        private:

            /**
             * @brief The expected type of the variant
             * 
             */
            swarmio::data::Variant::ValueCase _expectedType;

            /**
             * @brief The actual type of the variant
             * 
             */
            swarmio::data::Variant::ValueCase _actualType;

        public:

            /**
             * @brief Constructor
             * 
             * @param message Message
             * @param location Location
             * @param expectedType Expected variant type
             * @param actualType Actual variant type
             */
            TypeMismatchException(const char* message, const std::string& location, swarmio::data::Variant::ValueCase expectedType, swarmio::data::Variant::ValueCase actualType) 
                : SchemaMismatchException(message, location), _expectedType(expectedType), _actualType(actualType) { }       

            /**
             * @brief Get the expected type of the Variant
             * 
             * @return swarmio::data::Variant::ValueCase 
             */
            swarmio::data::Variant::ValueCase expected_type() const
            {
                return _expectedType;
            }                        

            /**
             * @brief Get the actual type of the Variant
             * 
             * @return swarmio::data::Variant::ValueCase 
             */
            swarmio::data::Variant::ValueCase actual_type() const
            {
                return _actualType;
            }
    };
}
