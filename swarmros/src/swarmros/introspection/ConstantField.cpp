#include <swarmros/introspection/ConstantField.h>
#include <swarmros/introspection/PrimitiveSerializer.h>
#include <swarmros/UnqualifiedException.h>
#include <regex>

using namespace swarmros;
using namespace swarmros::introspection;

// Regex to match true values
static const std::regex TruePattern("^\\s*(true|[0-9]*[1-9][0-9]*)\\s*(?:#.*)?$");

// Regex to match false values
static const std::regex FalsePattern("^\\s*(false|0+)\\s*(?:#.*)?$");

// Regex to match unsigned values
static const std::regex UnsignedPattern("^\\s*([0-9]+)\\s*(?:#.*)?$");

// Regex to match unsigned values
static const std::regex SignedPattern("^\\s*(-?[0-9]+)\\s*(?:#.*)?$");

// Regex to match floating point values
static const std::regex FloatPattern("^\\s*(-?[0-9]+(?:\\.[0-9]+)?)\\s*(?:#.*)?$");

void ConstantField::ParseBool(const std::string& value)
{
    std::smatch match;
    if (std::regex_match(value, match, TruePattern))
    {
        _value.set_bool_value(true);
        _rawValue = match[1].str();
    }
    else if (std::regex_match(value, match, FalsePattern))
    {
        _value.set_bool_value(false);
        _rawValue = match[1].str();
    }
    else
    {
        throw UnqualifiedException("Invalid value for bool constant");
    }
}

void ConstantField::ParseUnsigned(const std::string& value)
{
    std::smatch match;
    if (std::regex_match(value, match, UnsignedPattern))
    {
        _value.set_uint_value(std::stoull(value));
        _rawValue = match[1].str();
    }
    else
    {
        throw UnqualifiedException("Invalid value for unsigned constant");
    }
}

void ConstantField::ParseSigned(const std::string& value)
{
    std::smatch match;
    if (std::regex_match(value, match, SignedPattern))
    {
        _value.set_int_value(std::stoll(value));
        _rawValue = match[1].str();
    }
    else
    {
        throw UnqualifiedException("Invalid value for signed constant");
    }
}

void ConstantField::ParseFloat(const std::string& value)
{
    std::smatch match;
    if (std::regex_match(value, match, SignedPattern))
    {
        _value.set_double_value(std::stod(value));
        _rawValue = match[1].str();
    }
    else
    {
        throw UnqualifiedException("Invalid value for double constant");
    }
}

void ConstantField::ParseString(const std::string& value)
{
    _rawValue = value;
    _value.set_string_value(value);
}

ConstantField::ConstantField(const std::string& name, const Serializer& serializer, const std::string& value)
    : Field(name, serializer)
{
    const PrimitiveSerializer* primitiveSerializer = dynamic_cast<const PrimitiveSerializer*>(&serializer);
    if (primitiveSerializer != nullptr)
    {
        switch (primitiveSerializer->GetType())
        {
            case PrimitiveType::BOOL:
                ParseBool(value);
                break;

            case PrimitiveType::INT8:
            case PrimitiveType::INT16:
            case PrimitiveType::INT32:
            case PrimitiveType::INT64:
                ParseSigned(value);
                break;

            case PrimitiveType::UINT8:
            case PrimitiveType::UINT16:
            case PrimitiveType::UINT32:
            case PrimitiveType::UINT64:
                ParseUnsigned(value);
                break;

            case PrimitiveType::FLOAT32:
            case PrimitiveType::FLOAT64:
                ParseFloat(value);
                break;

            case PrimitiveType::STRING:
                ParseString(value);
                break;

            case PrimitiveType::DURATION:
            case PrimitiveType::TIME:
                throw UnqualifiedException("Invalid primitive type for constant");

            default:
                throw Exception("Unknown primitive type");
        }
    }
    else
    {
        throw UnqualifiedException("Invalid type for constant");
    }
}

void ConstantField::WriteDefinition(std::stringstream& stream, bool forHash) const
{
    WriteTypeName(stream, forHash);
    if (forHash)
    {
        stream << " " << GetName() << "=" << _rawValue;
    }
    else
    {
        stream << " " << GetName() << " = " << _rawValue;
    }
}

uint32_t ConstantField::GetDefaultLength(const FieldStack& fieldStack) const
{
    return 0;
}

uint32_t ConstantField::CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    return 0;
}

void ConstantField::Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    // Do nothing
}

void ConstantField::EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const
{
    // Do nothing
}

swarmio::data::Variant ConstantField::Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const 
{
    return _value;
}