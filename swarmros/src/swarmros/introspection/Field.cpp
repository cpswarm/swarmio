#include <swarmros/introspection/Field.h>
#include <swarmros/introspection/KeyedFieldStack.h>
#include <swarmros/introspection/MessageSerializer.h>
#include <swarmros/introspection/PrimitiveSerializer.h>
#include <swarmros/introspection/SchemaMismatchException.h>
#include <swarmros/Exception.h>
#include <swarmio/data/Helper.h>

using namespace swarmros;
using namespace swarmros::introspection;

void Field::WriteDefinition(std::stringstream& stream, bool forHash) const
{
    WriteTypeName(stream, forHash);
    stream << " " << GetName();
}

void Field::WriteTypeName(std::stringstream& stream, bool forHash) const
{
    if (forHash)
    {
        const auto messageSerializer = dynamic_cast<const MessageSerializer*>(&_serializer);
        if (messageSerializer != nullptr)
        {
            stream << messageSerializer->GetHash();
        }
        else
        {
            stream << _serializer.GetName();
        }
    }
    else
    {
        stream <<  _serializer.GetFullName();
    }
}

uint32_t Field::CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    if (!swarmio::data::Helper::IsArray(value))
    {
        KeyedFieldStack current(fieldStack, GetName());
        return _serializer.CalculateSerializedLength(value, current);
    }
    else
    {
        throw SchemaMismatchException("Array found where scalar value was expected", fieldStack.GetLocation());
    }
}

void Field::Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    if (!swarmio::data::Helper::IsArray(value))
    {
        KeyedFieldStack current(fieldStack, GetName());
        _serializer.Serialize(stream, value, current);
    }
    else
    {
        throw SchemaMismatchException("Array found where scalar value was expected", fieldStack.GetLocation());
    }
}

void Field::EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const
{
    KeyedFieldStack current(fieldStack, GetName());
    _serializer.EmitDefault(stream, current);
}

swarmio::data::Variant Field::Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const 
{
    KeyedFieldStack current(fieldStack, GetName());
    return _serializer.Deserialize(stream, current);
}

swarmio::data::Variant Field::DeserializeArray(ros::serialization::IStream& stream, uint32_t count, const FieldStack& fieldStack) const
{
    throw Exception("Fields cannot be deserialized as arrays");
}

swarmio::data::discovery::Field Field::GetFieldDescriptor() const
{
    return _serializer.GetFieldDescriptor();
}