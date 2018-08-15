#include <swarmros/bridge/ParameterTarget.h>
#include <swarmros/Exception.h>
#include <swarmio/data/Helper.h>
#include <limits>

using namespace swarmros;
using namespace swarmros::bridge;

swarmio::data::Variant ParameterTarget::VariantFromXmlRpcValue(XmlRpc::XmlRpcValue& value)
{
    swarmio::data::Variant result;
    switch (value.getType())
    {
        case XmlRpc::XmlRpcValue::Type::TypeInt:
            result.set_int_value((int&)value);
            break;

        case XmlRpc::XmlRpcValue::Type::TypeDouble:
            result.set_double_value((double&)value);
            break;

        case XmlRpc::XmlRpcValue::Type::TypeBoolean:
            result.set_bool_value((bool&)value);
            break;

        case XmlRpc::XmlRpcValue::Type::TypeString:
            result.set_string_value((std::string&)value);
            break;

        case XmlRpc::XmlRpcValue::Type::TypeStruct:
            for (auto& pair : value)
            {
                (*result.mutable_map_value()->mutable_pairs())[pair.first] = VariantFromXmlRpcValue(pair.second);
            }
            break;

        case XmlRpc::XmlRpcValue::Type::TypeDateTime:
        case XmlRpc::XmlRpcValue::Type::TypeInvalid:
        case XmlRpc::XmlRpcValue::Type::TypeBase64:
            throw Exception("Unsupported XmlRpc data type");

        default:
            if (value.getType() == XmlRpc::XmlRpcValue::Type::TypeArray)
            {
                if (value.size() == 0)
                {
                    result.mutable_map_array();
                }
                else
                {
                    swarmio::data::Variant::ValueCase arrayType = swarmio::data::Variant::ValueCase::VALUE_NOT_SET;
                    for (int i = 0; i < value.size(); ++i)
                    {
                        auto current = VariantFromXmlRpcValue(value[i]);
                        if (arrayType == swarmio::data::Variant::ValueCase::VALUE_NOT_SET)
                        {
                            arrayType = current.value_case();
                        }
                        else if (arrayType != current.value_case())
                        {
                            throw Exception("XmlRpc heterogeneous arrays are not supported");
                        }
                        switch (current.value_case())
                        {
                            case swarmio::data::Variant::ValueCase::kBoolValue:
                                result.mutable_bool_array()->mutable_elements()->Add(current.bool_value());
                                break;

                            case swarmio::data::Variant::ValueCase::kIntValue:
                                result.mutable_int_array()->mutable_elements()->Add(current.int_value());
                                break;

                            case swarmio::data::Variant::ValueCase::kDoubleValue:
                                result.mutable_double_array()->mutable_elements()->Add(current.double_value());
                                break;

                            case swarmio::data::Variant::ValueCase::kStringValue:
                                *result.mutable_string_array()->mutable_elements()->Add() = current.string_value();
                                break;

                            case swarmio::data::Variant::ValueCase::kMapValue:
                                *result.mutable_map_array()->mutable_elements()->Add() = current.map_value();
                                break;

                            default:
                                throw Exception("Unsupported XmlRpc array element type");
                        }
                    }
                }
            }
            else
            {
                throw Exception("Unknown XmlRpc data type");
            }            
    }
    return result;
}

XmlRpc::XmlRpcValue ParameterTarget::XmlRpcValueFromVariant(const swarmio::data::Variant& value)
{
    if (swarmio::data::Helper::IsArray(value))
    {
        XmlRpc::XmlRpcValue result; 
        int i = 0;
        result.setSize(swarmio::data::Helper::GetCount(value));
        switch (value.value_case())
        {
            case swarmio::data::Variant::ValueCase::kBoolArray:
                for (bool e : value.bool_array().elements())
                {
                    result[i++] = e;
                }
                break;

            case swarmio::data::Variant::ValueCase::kIntArray:
                for (int64_t e : value.int_array().elements())
                {
                    if (e < std::numeric_limits<int>::min() ||
                        e > std::numeric_limits<int>::max())
                    {
                        throw Exception("Integer overflow in XmlRpc data type");
                    }
                    result[i++] = (int)e;
                }
                break;

            case swarmio::data::Variant::ValueCase::kUintArray:
                for (uint64_t e : value.uint_array().elements())
                {
                    if (e > std::numeric_limits<int>::max())
                    {
                        throw Exception("Integer overflow in XmlRpc data type");
                    }
                    result[i++] = (int)e;
                }
                break;

            case swarmio::data::Variant::ValueCase::kDoubleArray:
                for (double e : value.double_array().elements())
                {
                    result[i++] = e;
                }
                break;

            case swarmio::data::Variant::ValueCase::kStringValue:
                for (const std::string& e : value.string_array().elements())
                {
                    result[i++] = e;
                }
                break;

            case swarmio::data::Variant::ValueCase::kMapArray:
                for (const auto& e : value.map_array().elements())
                {
                    XmlRpc::XmlRpcValue subresult; 
                    for (const auto& pair : e.pairs())
                    {
                        subresult[pair.first] = XmlRpcValueFromVariant(pair.second);    
                    }
                    result[i++] = subresult;
                }
                break;

            default:
                throw Exception("Unknown Variant data type");
        }
        return result;
    }
    else
    {
        switch (value.value_case())
        {
            case swarmio::data::Variant::ValueCase::kBoolValue:
                return value.bool_value();

            case swarmio::data::Variant::ValueCase::kIntValue:
                if (value.int_value() < std::numeric_limits<int>::min() ||
                    value.int_value() > std::numeric_limits<int>::max())
                {
                    throw Exception("Integer overflow in XmlRpc data type");
                }
                return (int)value.int_value();

            case swarmio::data::Variant::ValueCase::kUintValue:
                if (value.uint_value() > std::numeric_limits<int>::max())
                {
                    throw Exception("Integer overflow in XmlRpc data type");
                }
                return (int)value.uint_value();

            case swarmio::data::Variant::ValueCase::kDoubleValue:
                return value.double_value();

            case swarmio::data::Variant::ValueCase::kStringValue:
                return value.string_value();

            default:
                if (value.value_case() == swarmio::data::Variant::ValueCase::kMapValue)
                {
                    XmlRpc::XmlRpcValue result; 
                    for (const auto& pair : value.map_value().pairs())
                    {
                        result[pair.first] = XmlRpcValueFromVariant(pair.second);    
                    }
                    return result;
                }
                else
                {
                    throw Exception("Unknown Variant data type");
                }
        }
    }
}

ParameterTarget::ParameterTarget(ros::NodeHandle& nodeHandle, swarmio::services::keyvalue::Service& keyvalueService, const std::string& name, const std::string& parameter, bool isWritable)
    : _nodeHandle(nodeHandle), _keyvalueService(keyvalueService), _path(name), _parameter(parameter), _isWritable(isWritable)
{
    XmlRpc::XmlRpcValue value;
    if (nodeHandle.getParam(parameter, value))
    {
        _value = VariantFromXmlRpcValue(value);
    }
    else
    {
        throw Exception("Parameter does not exist on parameter server");
    }
    keyvalueService.RegisterTarget(name, this);
}
            
swarmio::data::Variant ParameterTarget::Get(const std::string& path)
{
    if (path == _path)
    {
        return _value;
    }
    else
    {
        throw Exception("Trying to get value of unknown parameter");
    }
}

void ParameterTarget::Set(const std::string& path, const swarmio::data::Variant& value)
{
    if (path == _path)
    {        
        _nodeHandle.setParam(_parameter, XmlRpcValueFromVariant(value));
        _value = value;
    }
    else
    {
        throw Exception("Trying to set value of unknown parameter");
    }
}

swarmio::data::discovery::Field ParameterTarget::GetFieldDescriptor(const std::string& path) const
{
    if (path == _path)
    {        
        return swarmio::data::Helper::GetFieldDescriptor(_value);
    }
    else
    {
        throw Exception("Trying to get field descriptor of unknown parameter");
    }
}

bool ParameterTarget::CanWrite(const std::string& name) const noexcept
{
    return _isWritable;
}

bool ParameterTarget::CanRead(const std::string& name) const noexcept
{
    return true;
}

ParameterTarget::~ParameterTarget()
{
    _keyvalueService.UnregisterTarget(_path);
}