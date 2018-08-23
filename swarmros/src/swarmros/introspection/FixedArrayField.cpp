#include <swarmros/introspection/FixedArrayField.h>
#include <swarmros/introspection/IndexedFieldStack.h>
#include <swarmros/introspection/KeyedFieldStack.h>
#include <swarmros/introspection/MessageSerializer.h>
#include <swarmros/introspection/CountMismatchException.h>
#include <swarmio/data/Helper.h>

using namespace swarmros;
using namespace swarmros::introspection;

void FixedArrayField::WriteDefinition(std::stringstream& stream, bool forHash) const
{
    if (forHash)
    {
        // Amusingly, array definitions are ignored for hash calculations
        // https://github.com/ros/genmsg/issues/50
        Field::WriteDefinition(stream, forHash);
    }
    else
    {
        stream << _serializer.GetFullName() << "[" << _count << "] " << GetName();
    }
}

uint32_t FixedArrayField::GetDefaultLength(const FieldStack& fieldStack) const
{
    KeyedFieldStack current(fieldStack, GetName());
    return _serializer.GetDefaultLength(fieldStack) * _count;
}

uint32_t FixedArrayField::CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    // Check array count
    uint32_t count = swarmio::data::Helper::GetCount(value);
    if (count != _count)
    {
        throw CountMismatchException(
            "Invalid count for FixedArrayField", 
            fieldStack.GetLocation(), 
            _count,
            count);
    }

    // Calculate as if it was a dynamic array but subtract 
    // the 4 bytes of count that would preceed it
    return ArrayField::CalculateSerializedLength(value, fieldStack) - 4;
}

void FixedArrayField::SerializeCount(ros::serialization::OStream& stream, uint32_t count, const FieldStack& fieldStack) const
{
    if (count != _count)
    {
        throw CountMismatchException(
            "Invalid count for FixedArrayField", 
            fieldStack.GetLocation(), 
            _count,
            count);
    }
}

void FixedArrayField::EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const
{
    IndexedFieldStack current(fieldStack);
    for (uint32_t i = 0; i < _count; ++i)
    {
        _serializer.EmitDefault(stream, current);
        ++current;
    }
}

uint32_t FixedArrayField::DeserializeCount(ros::serialization::IStream& stream, const FieldStack& fieldStack) const
{
    return _count;
}

swarmio::data::discovery::Field FixedArrayField::GetFieldDescriptor() const
{
    auto field = _serializer.GetFieldDescriptor();
    field.set_fixed_size(_count);
    return field;
}