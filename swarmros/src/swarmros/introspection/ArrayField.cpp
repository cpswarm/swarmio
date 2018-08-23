#include <swarmros/introspection/ArrayField.h>
#include <swarmros/introspection/KeyedFieldStack.h>
#include <swarmros/introspection/MessageSerializer.h>
#include <swarmros/introspection/TypeMismatchException.h>
#include <swarmio/data/Helper.h>

using namespace swarmros;
using namespace swarmros::introspection;

void ArrayField::WriteDefinition(std::stringstream& stream, bool forHash) const
{
    if (forHash)
    {
        // Amusingly, array definitions are ignored for hash calculations
        // https://github.com/ros/genmsg/issues/50
        Field::WriteDefinition(stream, forHash);
    }
    else
    {
        stream << _serializer.GetFullName() << "[] " << GetName();
    }
}

uint32_t ArrayField::GetDefaultLength(const FieldStack& fieldStack) const
{
    return 4;
}

uint32_t ArrayField::CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    KeyedFieldStack current(fieldStack, GetName());
    return 4 + _serializer.CalculateSerializedLength(value, current);
}

void ArrayField::Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    KeyedFieldStack current(fieldStack, GetName());
    SerializeCount(stream, swarmio::data::Helper::GetCount(value), fieldStack);
    _serializer.Serialize(stream, value, current);
}

void ArrayField::SerializeCount(ros::serialization::OStream& stream, uint32_t count, const FieldStack& fieldStack) const
{
    stream.next(count);
}

void ArrayField::EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const
{
    SerializeCount(stream, 0, fieldStack);
}

swarmio::data::Variant ArrayField::Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const 
{
    KeyedFieldStack current(fieldStack, GetName());
    uint32_t count = DeserializeCount(stream, fieldStack);
    return _serializer.DeserializeArray(stream, count, current);
}

uint32_t ArrayField::DeserializeCount(ros::serialization::IStream& stream, const FieldStack& fieldStack) const
{
    uint32_t count = 0;
    stream.next(count);
    return count;
}

swarmio::data::discovery::Field ArrayField::GetFieldDescriptor() const
{
    auto field = _serializer.GetFieldDescriptor();
    field.set_is_variable_size(true);
    return field;
}