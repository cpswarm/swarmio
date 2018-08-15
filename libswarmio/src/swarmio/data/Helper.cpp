#include <swarmio/data/Helper.h>
#include <swarmio/Exception.h>
#include <iomanip>
#include <list>
#include <algorithm>

using namespace swarmio;
using namespace swarmio::data;

bool Helper::IsArray(const Variant& value)
{
    switch (value.value_case())
    {
        case swarmio::data::Variant::ValueCase::kBoolArray:
        case swarmio::data::Variant::ValueCase::kIntArray:
        case swarmio::data::Variant::ValueCase::kUintArray:
        case swarmio::data::Variant::ValueCase::kDoubleArray:
        case swarmio::data::Variant::ValueCase::kStringArray:
        case swarmio::data::Variant::ValueCase::kMapArray:
            return true;
        default:
            return false;
    }
}

size_t Helper::GetCount(const Variant& value)
{
    switch (value.value_case())
    {
        case swarmio::data::Variant::ValueCase::kBoolArray:
            return value.bool_array().elements_size();

        case swarmio::data::Variant::ValueCase::kIntArray:
            return value.int_array().elements_size();

        case swarmio::data::Variant::ValueCase::kUintArray:
            return value.uint_array().elements_size();

        case swarmio::data::Variant::ValueCase::kDoubleArray:
            return value.double_array().elements_size();

        case swarmio::data::Variant::ValueCase::kStringArray:
            return value.string_array().elements_size();

        case swarmio::data::Variant::ValueCase::kMapArray:
            return value.map_array().elements_size();

        default:
            return 1;
    }
}

data::Variant::ValueCase Helper::GetBaseType(const Variant& value)
{
    switch (value.value_case())
    {
        case swarmio::data::Variant::ValueCase::kBoolArray:
            return swarmio::data::Variant::ValueCase::kBoolValue;

        case swarmio::data::Variant::ValueCase::kIntArray:
            return swarmio::data::Variant::ValueCase::kIntValue;

        case swarmio::data::Variant::ValueCase::kUintArray:
            return swarmio::data::Variant::ValueCase::kUintValue;

        case swarmio::data::Variant::ValueCase::kDoubleArray:
            return swarmio::data::Variant::ValueCase::kDoubleValue;

        case swarmio::data::Variant::ValueCase::kStringArray:
            return swarmio::data::Variant::ValueCase::kStringValue;

        case swarmio::data::Variant::ValueCase::kMapArray:
            return swarmio::data::Variant::ValueCase::kMapValue;

        default:
            return value.value_case();
    }
}

discovery::Schema Helper::GetSchemaDescriptor(const Map& value, bool includeArraySizes)
{
    discovery::Schema schema;
    auto& fields = *schema.mutable_fields();
    for (const auto& pair : value.pairs())
    {
        fields[pair.first] = GetFieldDescriptor(pair.second, includeArraySizes);
    }
    return schema;
}

discovery::Field Helper::GetFieldDescriptor(const Variant& value, bool includeArraySizes)
{
    discovery::Field field;
    switch (value.value_case())
    {
        case Variant::ValueCase::kBoolValue:
        case Variant::ValueCase::kBoolArray:
            field.set_type(discovery::Type::BOOL);
            break;

        case Variant::ValueCase::kIntValue:
        case Variant::ValueCase::kIntArray:
            field.set_type(discovery::Type::INT);
            break;

        case Variant::ValueCase::kUintValue:
        case Variant::ValueCase::kUintArray:
            field.set_type(discovery::Type::UINT);
            break;

        case Variant::ValueCase::kDoubleValue:
        case Variant::ValueCase::kDoubleArray:
            field.set_type(discovery::Type::DOUBLE);
            break;

        case Variant::ValueCase::kStringValue:
        case Variant::ValueCase::kStringArray:
            field.set_type(discovery::Type::STRING);
            break;

        case Variant::ValueCase::kMapValue:
            *field.mutable_schema() = GetSchemaDescriptor(value.map_value(), includeArraySizes);
            break;

        case Variant::ValueCase::kMapArray:
            if (value.map_array().elements_size() > 0)
            {
                *field.mutable_schema() = GetSchemaDescriptor(*value.map_array().elements().begin());
            }
            else
            {
                field.set_type(discovery::Type::UNKNOWN);
            }
            break;

        default:
                field.set_type(discovery::Type::UNKNOWN);
                break;
    }
    if (IsArray(value))
    {
        if (includeArraySizes)
        {
            field.set_fixed_size(GetCount(value));
        }
        else
        {
            field.set_is_variable_size(true);
        }
    }
    return field;
}

std::string Helper::ToString(const discovery::Schema& value, bool prettyPrint)
{
    std::ostringstream stream;
    WriteToStream(stream, value, prettyPrint);
    return stream.str();
}

std::string Helper::ToString(const Variant& value, bool prettyPrint)
{
    std::ostringstream stream;
    WriteToStream(stream, value, prettyPrint);
    return stream.str();
}

std::string Helper::ToString(const Map& value, bool prettyPrint)
{
    std::ostringstream stream;
    WriteToStream(stream, value, prettyPrint);
    return stream.str();
}

std::string Helper::ToString(const google::protobuf::Map<std::string, swarmio::data::Variant>& value, bool prettyPrint)
{
    std::ostringstream stream;
    WriteToStream(stream, value, prettyPrint);
    return stream.str();
}

void Helper::WriteEscapedAndQuotedStringToStream(std::ostream& stream, const std::string& value)
{
    stream << '"';
    for (auto c : value)
    {
        switch (c)
        {
            case '"': 
                stream << "\\\""; 
                break;

            case '\\': 
                stream << "\\\\"; 
                break;

            case '\b': 
                stream << "\\b";
                break;

            case '\f': 
                stream << "\\f"; 
                break;

            case '\n':
                stream << "\\n"; 
                break;

            case '\r': 
                stream << "\\r"; 
                break;

            case '\t': 
                stream << "\\t"; 
                break;

            default:
                if ('\x00' <= c && c <= '\x1f') 
                {
                    stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (int)c;
                } 
                else
                {
                    stream << c;
                }
                break;
        }
    }
    stream << '"';
}

void Helper::WriteElementSeparatorToStream(std::ostream& stream, bool& first, bool prettyPrint, int indentationLevel)
{
    // Separate elements
    if (first)
    {
        first = false;
    }
    else
    {
        stream << ",";
    }

    // Add indentation
    if (prettyPrint)
    {
        stream << std::endl;
        for (int i = 0; i <= indentationLevel; ++i)
        {
            stream << "  ";
        }
    }
}

void Helper::WriteToStream(std::ostream& stream, const Variant& value, bool prettyPrint, int indentationLevel)
{
    if (IsArray(value))
    {
        if (GetCount(value) == 0)
        {
            stream << "[]";
        }
        else
        {
            // Open array
            stream << "[";

            // Write elements
            bool first = true;
            switch (value.value_case())
            {
                case Variant::ValueCase::kMapArray:
                    for (const auto& e : value.map_array().elements())
                    {
                        WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);
                        WriteToStream(stream, e, prettyPrint, indentationLevel + 1);
                    }
                    break;

                case Variant::ValueCase::kStringArray:
                    for (const auto& e : value.string_array().elements())
                    {
                        WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);
                        WriteEscapedAndQuotedStringToStream(stream, e);
                    }
                    break;

                case Variant::ValueCase::kBoolArray:
                    for (auto e : value.bool_array().elements())
                    {
                        WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);
                        stream << std::boolalpha << e;
                    }
                    break;

                case Variant::ValueCase::kIntArray:
                    for (auto e : value.int_array().elements())
                    {
                        WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);
                        stream << e;
                    }
                    break;

                case Variant::ValueCase::kUintArray:
                    for (auto e : value.uint_array().elements())
                    {
                        WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);
                        stream << e;
                    }
                    break;

                case Variant::ValueCase::kDoubleArray:
                    for (auto e : value.double_array().elements())
                    {
                        WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);
                        stream << e;
                    }
                    break;

                default:
                    throw Exception("Unknown variant type");
            }

            // Close array
            if (prettyPrint)
            {
                stream << std::endl;
                for (int i = 0; i < indentationLevel; ++i)
                {
                    stream << "  ";
                }
            }
            stream << "]";
        }
    }
    else
    {
        switch (value.value_case())
        {
            case Variant::ValueCase::kMapValue:
                WriteToStream(stream, value.map_value(), prettyPrint, indentationLevel);
                break;

            case Variant::ValueCase::kStringValue:
                WriteEscapedAndQuotedStringToStream(stream, value.string_value());
                break;

            case Variant::ValueCase::kBoolValue:
                stream << std::boolalpha << value.bool_value();
                break;

            case Variant::ValueCase::kIntValue:
                stream << value.int_value();
                break;

            case Variant::ValueCase::kUintValue:
                stream << value.uint_value();
                break;

            case Variant::ValueCase::kDoubleValue:
                stream << value.double_value();
                break;

            default:
                throw Exception("Unknown variant type");
        }
    }
}

void Helper::WriteToStream(std::ostream& stream, const Map& value, bool prettyPrint, int indentationLevel)
{
    WriteToStream(stream, value.pairs(), prettyPrint, indentationLevel);
}

void Helper::WriteToStream(std::ostream& stream, const google::protobuf::Map<std::string, swarmio::data::Variant>& value, bool prettyPrint, int indentationLevel)
{
    // Check for empty map
    if (value.size() == 0)
    {
        stream << "{}";
    }
    else
    {
        // Open map
        stream << "{";

        // Protobuf intentionally randomizes the iteration order
        // of maps - so the same map would be printed differently 
        // each time this function is called. To work around that,
        // keys are manually sorted beforehand.
        std::list<std::string> keys;
        for (const auto& element : value)
        {
            keys.push_back(element.first);
        }
        keys.sort();

        // Add values
        bool first = true;
        for (const auto& key : keys)
        {
            // Write element separator
            WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);

            // Write key
            WriteEscapedAndQuotedStringToStream(stream, key);

            // Write key-value separator
            stream << ":"; 
            if (prettyPrint)
            {
                stream << " ";
            }

            // Write value
            WriteToStream(stream, value.at(key), prettyPrint, indentationLevel + 1);
        }

        // Close map and return string
        if (prettyPrint)
        {
            stream << std::endl;
            for (int i = 0; i < indentationLevel; ++i)
            {
                stream << "  ";
            }
        }
        stream << "}";
    }
}

const char* Helper::TypeToString(discovery::Type type)
{
    switch (type)
    {
        case discovery::Type::BOOL:
            return "bool";

        case discovery::Type::INT:
            return "int";

        case discovery::Type::UINT:
            return "uint";

        case discovery::Type::DOUBLE:
            return "double";

        case discovery::Type::STRING:
            return "string";

        default:
            return "unknown";
    }
}

void Helper::WriteToStream(std::ostream& stream, const discovery::Schema& value, bool prettyPrint, int indentationLevel)
{
    if (value.fields_size() == 0)
    {
        stream << "{}";
    }
    else
    {
        // Open map
        stream << "{";

        // Protobuf intentionally randomizes the iteration order
        // of maps - so the same map would be printed differently 
        // each time this function is called. To work around that,
        // keys are manually sorted beforehand.
        std::list<std::string> keys;
        for (const auto& element : value.fields())
        {
            keys.push_back(element.first);
        }
        keys.sort();

        // Add values
        bool first = true;
        for (const auto& key : keys)
        {
            // Fetch current element
            const auto& field = value.fields().at(key);

            // Write element separator
            WriteElementSeparatorToStream(stream, first, prettyPrint, indentationLevel);

            // Write key
            if (field.multiplicity_case() == discovery::Field::MultiplicityCase::kFixedSize)
            {
                WriteEscapedAndQuotedStringToStream(stream, key + "[" + std::to_string(field.fixed_size()) + "]");
            }
            else if (field.is_variable_size())
            {
                WriteEscapedAndQuotedStringToStream(stream, key + "[]");
            }
            else
            {
                WriteEscapedAndQuotedStringToStream(stream, key);
            }

            // Write key-value separator
            stream << ":"; 
            if (prettyPrint)
            {
                stream << " ";
            }

            // Write value
            if (field.descriptor_case() == discovery::Field::DescriptorCase::kSchema)
            {
                WriteToStream(stream, field.schema(), prettyPrint, indentationLevel + 1);
            }
            else
            {
                stream << '"' << TypeToString(field.type()) << '"';
            }
        }

        // Close map and return string
        if (prettyPrint)
        {
            stream << std::endl;
            for (int i = 0; i < indentationLevel; ++i)
            {
                stream << "  ";
            }
        }
        stream << "}";
    }
}



