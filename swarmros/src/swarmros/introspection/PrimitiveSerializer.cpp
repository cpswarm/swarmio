#include <swarmros/introspection/PrimitiveSerializer.h>
#include <swarmros/introspection/TypeMismatchException.h>
#include <swarmros/introspection/IndexedFieldStack.h>
#include <swarmros/UnqualifiedException.h>
#include <swarmio/data/Helper.h>
#include <boost/numeric/conversion/cast.hpp>
#include <limits>
#include <regex>

using namespace swarmros;
using namespace swarmros::introspection;

static inline void ThrowTypeMismatchException(swarmio::data::Variant::ValueCase sourceType, PrimitiveType targetPrimitiveType, const FieldStack& fieldStack)
{
    auto targetType = swarmio::data::Variant::ValueCase::VALUE_NOT_SET;
    switch (targetPrimitiveType)
    {
        case PrimitiveType::BOOL:
            targetType = swarmio::data::Variant::ValueCase::kBoolValue;
            break;

        case PrimitiveType::FLOAT32:
        case PrimitiveType::FLOAT64:
            targetType = swarmio::data::Variant::ValueCase::kDoubleValue;
            break; 

        case PrimitiveType::INT8:
        case PrimitiveType::INT16:
        case PrimitiveType::INT32:
        case PrimitiveType::INT64:
        case PrimitiveType::DURATION:
            targetType = swarmio::data::Variant::ValueCase::kIntValue;
            break; 

        case PrimitiveType::UINT8:
        case PrimitiveType::UINT16:
        case PrimitiveType::UINT32:
        case PrimitiveType::UINT64:
        case PrimitiveType::TIME:     
            targetType = swarmio::data::Variant::ValueCase::kUintValue;
            break; 

        case PrimitiveType::STRING:
            targetType = swarmio::data::Variant::ValueCase::kStringValue;
            break; 

        default:
            throw Exception("Unknown primitive type");
    }
    throw TypeMismatchException("Unexpected target type for PrimitiveSerializer", fieldStack.GetLocation(), targetType, sourceType);
}

template<typename T, typename O>
static inline void SerializeAs(ros::serialization::OStream& stream, O value, const FieldStack& fieldStack)
{
    try
    {
        stream.next((T)boost::numeric_cast<T>(value));
    }
    catch (const boost::numeric::bad_numeric_cast&)
    {
        throw SchemaMismatchException("Integer out-of-range", fieldStack.GetLocation());
    }
}

static inline void SerializeAsBool(ros::serialization::OStream& stream, bool value, const FieldStack& fieldStack)
{
    stream.next((uint8_t)(value ? 1 : 0));
}

static inline void SerializeAsString(ros::serialization::OStream& stream, const std::string& value, const FieldStack& fieldStack)
{
    uint32_t size = value.size();
    SerializeAs<uint32_t>(stream, size, fieldStack);
    memcpy(stream.advance(size), value.c_str(), size);
}

template<typename O>
static void SerializeAsTime(ros::serialization::OStream& stream, O value, const FieldStack& fieldStack)
{
    int32_t secs = value / 1000000000;
    SerializeAs<int32_t>(stream, secs, fieldStack);
    if (value < 0)
    {
        int32_t nsecs = -(-value % 1000000000);
        SerializeAs<int32_t>(stream, nsecs, fieldStack);
    }
    else
    {
        int32_t nsecs = value % 1000000000;        
        SerializeAs<int32_t>(stream, nsecs, fieldStack);
    }

}

template<typename O>
static void SerializeAsDuration(ros::serialization::OStream& stream, O value, const FieldStack& fieldStack)
{
    if (value < 0)
    {
        throw SchemaMismatchException("Integer out-of-range", fieldStack.GetLocation());
    }
    else
    {
        uint32_t secs = value / 1000000000;
        SerializeAs<uint32_t>(stream, secs, fieldStack);
        uint32_t nsecs = value % 1000000000;
        SerializeAs<uint32_t>(stream, nsecs, fieldStack);
    }
}

static inline void SerializeAs(ros::serialization::OStream& stream, int64_t value, PrimitiveType type, const FieldStack& fieldStack)
{
    switch (type)
    {
        case PrimitiveType::INT8:
            SerializeAs<int8_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::INT16:
            SerializeAs<int16_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::INT32:
            SerializeAs<int32_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::INT64:
            SerializeAs<int64_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::UINT8:
            SerializeAs<uint8_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::UINT16:
            SerializeAs<uint16_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::UINT32:
            SerializeAs<uint32_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::UINT64:
            SerializeAs<uint64_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::DURATION:
            SerializeAsDuration(stream, value, fieldStack);
            break;

        case PrimitiveType::TIME:
            SerializeAsTime(stream, value, fieldStack);
            break;

        default:
            ThrowTypeMismatchException(swarmio::data::Variant::ValueCase::kIntValue, type, fieldStack);
            break;
    }
}

static inline void SerializeAs(ros::serialization::OStream& stream, uint64_t value, PrimitiveType type, const FieldStack& fieldStack)
{
    switch (type)
    {
        case PrimitiveType::UINT8:
            SerializeAs<uint8_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::UINT16:
            SerializeAs<uint16_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::UINT32:
            SerializeAs<uint32_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::UINT64:
            SerializeAs<uint64_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::INT8:
            SerializeAs<int8_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::INT16:
            SerializeAs<int16_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::INT32:
            SerializeAs<int32_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::INT64:
            SerializeAs<int64_t>(stream, value, fieldStack);
            break;

        case PrimitiveType::DURATION:
            SerializeAsDuration(stream, value, fieldStack);
            break;

        case PrimitiveType::TIME:
            SerializeAsTime(stream, value, fieldStack);
            break;

        default:
            ThrowTypeMismatchException(swarmio::data::Variant::ValueCase::kUintValue, type, fieldStack);
            break;
    }
}

static inline void SerializeAs(ros::serialization::OStream& stream, bool value, PrimitiveType type, const FieldStack& fieldStack)
{
    switch (type)
    {
        case PrimitiveType::BOOL:
            SerializeAsBool(stream, value, fieldStack);
            break;

        default:
            ThrowTypeMismatchException(swarmio::data::Variant::ValueCase::kBoolValue, type, fieldStack);
            break;
    }
}

static inline void SerializeAs(ros::serialization::OStream& stream, double value, PrimitiveType type, const FieldStack& fieldStack)
{
    switch (type)
    {
        case PrimitiveType::FLOAT32:
            SerializeAs<float>(stream, value, fieldStack);
            break;

        case PrimitiveType::FLOAT64:
            SerializeAs<double>(stream, value, fieldStack);
            break;

        default:
            ThrowTypeMismatchException(swarmio::data::Variant::ValueCase::kDoubleValue, type, fieldStack);
            break;
    }
}

static inline void SerializeAs(ros::serialization::OStream& stream, const std::string& value, PrimitiveType type, const FieldStack& fieldStack)
{
    switch (type)
    {
        case PrimitiveType::STRING:
            SerializeAsString(stream, value, fieldStack);
            break;

        default:
            ThrowTypeMismatchException(swarmio::data::Variant::ValueCase::kDoubleValue, type, fieldStack);
            break;
    } 
}

void PrimitiveSerializer::Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    if (swarmio::data::Helper::IsArray(value))
    {
        IndexedFieldStack current(fieldStack);
        switch (value.value_case())
        {
            case swarmio::data::Variant::ValueCase::kBoolArray:
                for (const auto e : value.bool_array().elements())
                {
                    SerializeAs(stream, e, _type, current);
                    ++current;
                }
                break;

            case swarmio::data::Variant::ValueCase::kIntArray:
                for (const auto e : value.int_array().elements())
                {
                    SerializeAs(stream, e, _type, current);
                    ++current;
                }
                break;

            case swarmio::data::Variant::ValueCase::kUintArray:
                for (const auto e : value.uint_array().elements())
                {
                    SerializeAs(stream, e, _type, current);
                    ++current;
                }
                break;

            case swarmio::data::Variant::ValueCase::kDoubleArray:
                for (const auto e : value.double_array().elements())
                {
                    SerializeAs(stream, e, _type, current);
                    ++current;
                }
                break;

            case swarmio::data::Variant::ValueCase::kStringArray:
                for (const auto& e : value.string_array().elements())
                {
                    SerializeAs(stream, e, _type, current);
                    ++current;
                }
                break;

            default:
                ThrowTypeMismatchException(value.value_case(), _type, current);
                break;
        }
    }
    else
    {
        switch (value.value_case())
        {
            case swarmio::data::Variant::ValueCase::kBoolValue:
                SerializeAs(stream, value.bool_value(), _type, fieldStack);
                break;

            case swarmio::data::Variant::ValueCase::kIntValue:
                SerializeAs(stream, value.int_value(), _type, fieldStack);
                break;

            case swarmio::data::Variant::ValueCase::kUintValue:
                SerializeAs(stream, value.uint_value(), _type, fieldStack);
                break;

            case swarmio::data::Variant::ValueCase::kDoubleValue:
                SerializeAs(stream, value.double_value(), _type, fieldStack);
                break;

            case swarmio::data::Variant::ValueCase::kStringValue:
                SerializeAs(stream, value.string_value(), _type, fieldStack);
                break;

            default:
                ThrowTypeMismatchException(value.value_case(), _type, fieldStack);
                break;
        }
    }
}

template<typename T>
static inline T DeserializeAs(ros::serialization::IStream& stream)
{
    T value;
    stream.next(value);
    return value;            
}

template<>
inline bool DeserializeAs(ros::serialization::IStream& stream)
{
    uint8_t value;
    stream.next(value);
    return value ? true : false;            
}

template<>
inline std::string DeserializeAs(ros::serialization::IStream& stream)
{
    uint32_t length = DeserializeAs<uint32_t>(stream);
    return std::string((const char*)stream.advance(length), length);  
}

inline uint64_t DeserializeAsTime(ros::serialization::IStream& stream)
{
    uint32_t secs = DeserializeAs<uint32_t>(stream);
    uint32_t nsecs = DeserializeAs<uint32_t>(stream);
    return secs * 1000000000 + nsecs;
}

inline int64_t DeserializeAsDuration(ros::serialization::IStream& stream)
{
    int32_t secs = DeserializeAs<int32_t>(stream);
    int32_t nsecs = DeserializeAs<int32_t>(stream);
    return secs * 1000000000 + nsecs;
}

swarmio::data::Variant PrimitiveSerializer::Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const 
{
    swarmio::data::Variant value;
    switch (_type)
    {
        case PrimitiveType::BOOL:
            value.set_bool_value(DeserializeAs<bool>(stream));
            break;

        case PrimitiveType::INT8:
            value.set_int_value(DeserializeAs<int8_t>(stream));
            break;

        case PrimitiveType::UINT8:
            value.set_uint_value(DeserializeAs<uint8_t>(stream));
            break;

        case PrimitiveType::INT16:
            value.set_int_value(DeserializeAs<int16_t>(stream));
            break;

        case PrimitiveType::UINT16:
            value.set_uint_value(DeserializeAs<uint16_t>(stream));
            break;

        case PrimitiveType::INT32:
            value.set_int_value(DeserializeAs<int32_t>(stream));
            break;

        case PrimitiveType::DURATION:
            value.set_int_value(DeserializeAsDuration(stream));
            break;

        case PrimitiveType::UINT32:
            value.set_uint_value(DeserializeAs<uint32_t>(stream));
            break;

        case PrimitiveType::TIME:
            value.set_uint_value(DeserializeAsTime(stream));
            break;

        case PrimitiveType::INT64:
            value.set_int_value(DeserializeAs<int64_t>(stream));
            break;

        case PrimitiveType::UINT64:
            value.set_uint_value(DeserializeAs<uint64_t>(stream));
            break;

        case PrimitiveType::FLOAT32:
            value.set_double_value(DeserializeAs<float>(stream));
            break;

        case PrimitiveType::FLOAT64:
            value.set_double_value(DeserializeAs<double>(stream));
            break;

        case PrimitiveType::STRING:
            value.set_string_value(DeserializeAs<std::string>(stream));
            break;

        default:
            throw Exception("Unknown primitive type");
    }
    return value;
}

swarmio::data::Variant PrimitiveSerializer::DeserializeArray(ros::serialization::IStream& stream, uint32_t count, const FieldStack& fieldStack) const
{
    swarmio::data::Variant value;
    if (count > 0)
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            switch (_type)
            {
                case PrimitiveType::BOOL:
                    value.mutable_bool_array()->add_elements(DeserializeAs<bool>(stream));
                    break;

                case PrimitiveType::INT8:
                    value.mutable_int_array()->add_elements(DeserializeAs<int8_t>(stream));
                    break;

                case PrimitiveType::UINT8:
                    value.mutable_uint_array()->add_elements(DeserializeAs<uint8_t>(stream));
                    break;

                case PrimitiveType::INT16:
                    value.mutable_int_array()->add_elements(DeserializeAs<int16_t>(stream));
                    break;

                case PrimitiveType::UINT16:
                    value.mutable_uint_array()->add_elements(DeserializeAs<uint16_t>(stream));
                    break;

                case PrimitiveType::INT32:
                    value.mutable_int_array()->add_elements(DeserializeAs<int32_t>(stream));
                    break;

                case PrimitiveType::DURATION:
                    value.mutable_int_array()->add_elements(DeserializeAsDuration(stream));
                    break;

                case PrimitiveType::UINT32:
                    value.mutable_uint_array()->add_elements(DeserializeAs<uint32_t>(stream));
                    break;

                case PrimitiveType::TIME:
                    value.mutable_uint_array()->add_elements(DeserializeAsTime(stream));
                    break;

                case PrimitiveType::INT64:
                    value.mutable_int_array()->add_elements(DeserializeAs<int64_t>(stream));
                    break;

                case PrimitiveType::UINT64:
                    value.mutable_uint_array()->add_elements(DeserializeAs<uint64_t>(stream));
                    break;

                case PrimitiveType::FLOAT32:
                    value.mutable_double_array()->add_elements(DeserializeAs<float>(stream));
                    break;

                case PrimitiveType::FLOAT64:
                    value.mutable_double_array()->add_elements(DeserializeAs<double>(stream));
                    break;

                case PrimitiveType::STRING:
                    value.mutable_string_array()->add_elements(DeserializeAs<std::string>(stream));
                    break;

                default:
                    throw Exception("Unknown primitive type");
            }
        }
    }
    else
    {
        switch (_type)
        {
            case PrimitiveType::BOOL:
                value.mutable_bool_array();
                break;

            case PrimitiveType::INT8:
                value.mutable_int_array();
                break;

            case PrimitiveType::UINT8:
                value.mutable_uint_array();
                break;

            case PrimitiveType::INT16:
                value.mutable_int_array();
                break;

            case PrimitiveType::UINT16:
                value.mutable_uint_array();
                break;

            case PrimitiveType::INT32:
                value.mutable_int_array();
                break;

            case PrimitiveType::DURATION:
                value.mutable_int_array();
                break;

            case PrimitiveType::UINT32:
                value.mutable_uint_array();
                break;

            case PrimitiveType::TIME:
                value.mutable_uint_array();
                break;

            case PrimitiveType::INT64:
                value.mutable_int_array();
                break;

            case PrimitiveType::UINT64:
                value.mutable_uint_array();
                break;

            case PrimitiveType::FLOAT32:
                value.mutable_double_array();
                break;

            case PrimitiveType::FLOAT64:
                value.mutable_double_array();
                break;

            case PrimitiveType::STRING:
                value.mutable_string_array();
                break;

            default:
                throw Exception("Unknown primitive type");
        }
    }
    return value;
}

uint32_t PrimitiveSerializer::GetDefaultLength(const FieldStack& fieldStack) const
{
    switch (_type)
    {
        case PrimitiveType::BOOL:
        case PrimitiveType::INT8:
        case PrimitiveType::UINT8:
            return 1;

        case PrimitiveType::INT16:
        case PrimitiveType::UINT16:
            return 2;

        case PrimitiveType::INT32:
        case PrimitiveType::UINT32:
        case PrimitiveType::FLOAT32:
        case PrimitiveType::STRING:
            return 4;

        case PrimitiveType::INT64:
        case PrimitiveType::UINT64:
        case PrimitiveType::FLOAT64:
        case PrimitiveType::TIME:
        case PrimitiveType::DURATION:
            return 8;

        default:
            throw Exception("Unknown primitive type");
    }
}

uint32_t PrimitiveSerializer::CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    // Determine length and expected type
    uint32_t length = 0;
    swarmio::data::Variant::ValueCase expectedType;
    switch (_type)
    {
        case PrimitiveType::BOOL:
            expectedType = swarmio::data::Variant::ValueCase::kBoolValue;
            length = 1;
            break;

        case PrimitiveType::INT8:
            expectedType = swarmio::data::Variant::ValueCase::kIntValue;
            length = 1;
            break;

        case PrimitiveType::UINT8:
            expectedType = swarmio::data::Variant::ValueCase::kUintValue;
            length = 1;
            break;

        case PrimitiveType::INT16:
            expectedType = swarmio::data::Variant::ValueCase::kIntValue;
            length = 2;
            break;

        case PrimitiveType::UINT16:
            expectedType = swarmio::data::Variant::ValueCase::kUintValue;
            length = 2;
            break;

        case PrimitiveType::INT32:
            expectedType = swarmio::data::Variant::ValueCase::kIntValue;
            length = 4;
            break;

        case PrimitiveType::UINT32:
            expectedType = swarmio::data::Variant::ValueCase::kUintValue;
            length = 4;
            break;

        case PrimitiveType::FLOAT32:
            expectedType = swarmio::data::Variant::ValueCase::kDoubleValue;
            length = 4;
            break;

        case PrimitiveType::INT64:
        case PrimitiveType::DURATION:
            expectedType = swarmio::data::Variant::ValueCase::kIntValue;
            length = 8;
            break;

        case PrimitiveType::UINT64:
        case PrimitiveType::TIME:
            expectedType = swarmio::data::Variant::ValueCase::kUintValue;
            length = 8;
            break;
            
        case PrimitiveType::FLOAT64:
            expectedType = swarmio::data::Variant::ValueCase::kDoubleValue;
            length = 8;
            break;
            
        case PrimitiveType::STRING:
            expectedType = swarmio::data::Variant::ValueCase::kStringValue;
            length = 4;
            break;

        default:
            throw Exception("Unknown primitive type");
    }

    // Check type
    auto baseType = swarmio::data::Helper::GetBaseType(value);
    if (baseType == swarmio::data::Variant::ValueCase::kUintValue && expectedType == swarmio::data::Variant::ValueCase::kIntValue)
    {
        baseType = expectedType;
    }
    else if (baseType == swarmio::data::Variant::ValueCase::kIntValue && expectedType == swarmio::data::Variant::ValueCase::kUintValue)
    {
        baseType = expectedType;
    }   
    if (expectedType != baseType)
    {
        throw TypeMismatchException(
            "Unexpected source type for PrimitiveSerializer", 
            fieldStack.GetLocation(), 
            expectedType, 
            baseType);
    }

    // Handle string size, multiply by array count, then return value
    uint32_t count = swarmio::data::Helper::GetCount(value);
    if (_type == PrimitiveType::STRING)
    {
        if (count > 1)
        {
            // Add length definitions
            length = length * value.string_array().elements_size();

            // Add actual characters
            for (const auto& element : value.string_array().elements())
            {
                length += element.size();
            }
        }
        else
        {
            // Add actual characters
            length += value.string_value().size();
        }
    }
    else
    {
        // Multiply by array count
        length *= count;
    }
    return length;
}

void PrimitiveSerializer::EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const
{
    switch (_type)
    {
        case PrimitiveType::BOOL:
            SerializeAs<bool>(stream, false, fieldStack);
            break;

        case PrimitiveType::INT8:
            SerializeAs<int8_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::INT16:
            SerializeAs<int16_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::INT32:
            SerializeAs<int32_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::DURATION:
            SerializeAs<int32_t>(stream, 0, fieldStack);
            SerializeAs<int32_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::INT64:
            SerializeAs<int64_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::UINT8:
            SerializeAs<uint8_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::UINT16:
            SerializeAs<uint16_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::UINT32:
            SerializeAs<uint32_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::TIME:
            SerializeAs<uint32_t>(stream, 0, fieldStack);
            SerializeAs<uint32_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::UINT64:
            SerializeAs<uint64_t>(stream, 0, fieldStack);
            break;

        case PrimitiveType::FLOAT32:
            SerializeAs<float>(stream, 0.0, fieldStack);
            break;

        case PrimitiveType::FLOAT64:
            SerializeAs<double>(stream, 0.0, fieldStack);
            break;

        case PrimitiveType::STRING:
            SerializeAsString(stream, "", fieldStack);
            break;

        default:
            throw Exception("Unknown primitive type");
    }
}

swarmio::data::discovery::Field PrimitiveSerializer::GetFieldDescriptor() const
{
    swarmio::data::discovery::Field field;
    switch (_type)
    {
        case PrimitiveType::BOOL:
            field.set_type(swarmio::data::discovery::BOOL);
            break;

        case PrimitiveType::INT8:
        case PrimitiveType::INT16:
        case PrimitiveType::INT32:
        case PrimitiveType::INT64:
        case PrimitiveType::DURATION:
            field.set_type(swarmio::data::discovery::INT);
            break;

        case PrimitiveType::UINT8:
        case PrimitiveType::UINT16:
        case PrimitiveType::UINT32:
        case PrimitiveType::UINT64:
        case PrimitiveType::TIME:
            field.set_type(swarmio::data::discovery::UINT);
            break;

        case PrimitiveType::FLOAT32:
        case PrimitiveType::FLOAT64:
            field.set_type(swarmio::data::discovery::DOUBLE);
            break;
            
        case PrimitiveType::STRING:
            field.set_type(swarmio::data::discovery::STRING);
            break;

        default:
            throw Exception("Unknown primitive type");
    }
    return field;
}

