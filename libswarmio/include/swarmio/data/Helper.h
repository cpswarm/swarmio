#pragma once

#include <swarmio/data/Variant.pb.h>
#include <swarmio/data/discovery/Schema.pb.h>
#include <sstream>

namespace swarmio::data
{
    /**
     * @brief This Helper class contains commonly
     *        used function to manipulate protobuf
     *        objects.
     * 
     */
    class SWARMIO_API Helper final
    {
        private:

            /**
             * @brief Disable construction
             * 
             */
            Helper() { }

            /**
             * @brief Look up the human readable name of a type
             * 
             * @param type Type
             * @return const char* 
             */
            static const char* TypeToString(discovery::Type type);

            /**
             * @brief Write an escaped and quoted string to the stream
             * 
             * @param stream Stream
             * @param value String
             */
            static void WriteEscapedAndQuotedStringToStream(std::ostream& stream, const std::string& value);

            /**
             * @brief Write the element separator between two elements of an array.
             * 
             * @param stream Stream
             * @param first First element marker
             * @param prettyPrint Add indentation and spacing
             * @param indentationLevel Current indentation level
             */
            static void WriteElementSeparatorToStream(std::ostream& stream, bool& first, bool prettyPrint, int indentationLevel);

        public: 

            /**
             * @brief Check whether the variant contains an array type
             * 
             * @param value Variant
             * @return True if the variant is an array
             */
            static bool IsArray(const Variant& value);

            /**
             * @brief Get the item count of the variant array,
             *        or 1 if it is not an array.
             * 
             * @param value Variant
             * @return size_t 
             */
            static size_t GetCount(const Variant& value);

            /**
             * @brief Get the base type of the variant array,
             *        or its type if it is not an array.
             * 
             * @param value Variant
             * @return Variant::ValueCase 
             */
            static Variant::ValueCase GetBaseType(const Variant& value);

            /**
             * @brief Get the schema for an existing map
             * 
             * @param value Value
             * @param includeArraySizes Mark arrays as fixed size
             * @return discovery::Schema 
             */
            static discovery::Schema GetSchemaDescriptor(const Map& value, bool includeArraySizes = false);

            /**
             * @brief Get the schema for an existing map
             * 
             * @param value Value
             * @param includeArraySizes Mark arrays as fixed size
             * @return discovery::Field 
             */
            static discovery::Field GetFieldDescriptor(const Variant& value, bool includeArraySizes = false);

            /**
             * @brief Get a string representation of a schema
             * 
             * @param value Schema
             * @param prettyPrint Add indentation and spacing
             * @return std::string              
             */
            static std::string ToString(const discovery::Schema& value, bool prettyPrint = true); 

            /**
             * @brief Get a string representation of a variant
             * 
             * @param value Variant
             * @param prettyPrint Add indentation and spacing
             * @return std::string 
             */
            static std::string ToString(const Variant& value, bool prettyPrint = true);   

            /**
             * @brief Get a string representation of a map
             * 
             * @param value Map
             * @param prettyPrint Add indentation and spacing
             * @return std::string 
             */
            static std::string ToString(const Map& value, bool prettyPrint = true);   

            /**
             * @brief Get a string representation of a map
             * 
             * @param value Map
             * @param prettyPrint Add indentation and spacing
             * @return std::string 
             */
            static std::string ToString(const google::protobuf::Map<std::string, swarmio::data::Variant>& value, bool prettyPrint = true);   

            /**
             * @brief Write a string representation of a variant to the stream
             * 
             * @param stream Stream
             * @param prettyPrint Add indentation and spacing
             * @param indentationLevel Current indentation level
             * @param value Variant
             */
            static void WriteToStream(std::ostream& stream, const Variant& value, bool prettyPrint = true, int indentationLevel = 0);

            /**
             * @brief Write a string representation of a map to the stream
             * 
             * @param stream Stream
             * @param prettyPrint Add indentation and spacing
             * @param indentationLevel Current indentation level
             * @param value Map
             */
            static void WriteToStream(std::ostream& stream, const Map& value, bool prettyPrint = true, int indentationLevel = 0);

            /**
             * @brief Write a string representation of a map to the stream
             * 
             * @param stream Stream
             * @param prettyPrint Add indentation and spacing
             * @param indentationLevel Current indentation level
             * @param value Map
             */
            static void WriteToStream(std::ostream& stream, const google::protobuf::Map<std::string, swarmio::data::Variant>& value, bool prettyPrint = true, int indentationLevel = 0);

            /**
             * @brief Write a string representation of a schema to the stream
             * 
             * @param stream Stream
             * @param prettyPrint Add indentation and spacing
             * @param indentationLevel Current indentation level
             * @param value Schema
             */
            static void WriteToStream(std::ostream& stream, const discovery::Schema& value, bool prettyPrint = true, int indentationLevel = 0);   
    };
}