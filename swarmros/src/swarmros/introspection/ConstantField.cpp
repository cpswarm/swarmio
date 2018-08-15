#include <swarmros/introspection/ConstantField.h>
#include <swarmros/introspection/PrimitiveSerializer.h>
#include <swarmros/UnqualifiedException.h>

using namespace swarmros;
using namespace swarmros::introspection;

ConstantField::ConstantField(const std::string& name, const Serializer& serializer, const std::string& value)
    : Field(name, serializer), _rawValue(value)
{
    const PrimitiveSerializer* primitiveSerializer = dynamic_cast<const PrimitiveSerializer*>(&serializer);
    if (primitiveSerializer != nullptr)
    {
        _value = primitiveSerializer->ParseConstant(value);
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