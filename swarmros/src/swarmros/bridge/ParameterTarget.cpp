#include <swarmros/bridge/ParameterTarget.h>
#include <swarmio/Exception.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

using namespace swarmros;
using namespace swarmros::bridge;

ParameterTarget::ParameterTarget(ros::NodeHandle& nodeHandle, const std::string& name, const std::string& path, bool isWritable, const swarmio::data::Variant& defaultValue)
    : _nodeHandle(nodeHandle), _name(name), _path(path), _isWritable(isWritable), _defaultValue(defaultValue)
{
    // Advertise topic
    switch (_defaultValue.value_case())
    {
        case swarmio::data::Variant::ValueCase::kBoolValue:
            _publisher = nodeHandle.advertise<std_msgs::Bool>("parameters/" + _name, 1, true);
            break;

        case swarmio::data::Variant::ValueCase::kDoubleValue:
            _publisher = nodeHandle.advertise<std_msgs::Float64>("parameters/" + _name, 1, true);
            break;

        case swarmio::data::Variant::ValueCase::kIntValue:
            _publisher = nodeHandle.advertise<std_msgs::Int32>("parameters/" + _name, 1, true);
            break;

        case swarmio::data::Variant::ValueCase::kStringValue:
            _publisher = nodeHandle.advertise<std_msgs::String>("parameters/" + _name, 1, true);
            break;
        
        default:
            throw swarmio::Exception("Unknown data type for variant");
    }

    // Publish initial value
    PublishValue(Get(name));
}

void ParameterTarget::PublishValue(const swarmio::data::Variant& value)
{
    // Build message and publish
    if (value.value_case() == swarmio::data::Variant::ValueCase::kBoolValue)
    {
        std_msgs::Bool message;
        message.data = value.bool_value();
        _publisher.publish(message);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kDoubleValue)
    {
        std_msgs::Float64 message;
        message.data = value.double_value();
        _publisher.publish(message);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kIntValue)
    {
        std_msgs::Int32 message;
        message.data = value.int_value();
        _publisher.publish(message);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kStringValue)
    {
        std_msgs::String message;
        message.data = value.string_value();
        _publisher.publish(message);
    }
    else 
    {
        throw swarmio::Exception("Unknown data type for variant");
    }
}

swarmio::data::Variant ParameterTarget::Get(const std::string& name)
{
    // Check name
    if (name != _name)
    {
        throw swarmio::Exception("Name mismatch for parameter get request.");
    }

    // Fetch value
    swarmio::data::Variant value;
    switch (_defaultValue.value_case())
    {
        case swarmio::data::Variant::ValueCase::kBoolValue:
            value.set_bool_value(_nodeHandle.param(_path, _defaultValue.bool_value()));
            break;

        case swarmio::data::Variant::ValueCase::kDoubleValue:
            value.set_double_value(_nodeHandle.param(_path, _defaultValue.double_value()));
            break;

        case swarmio::data::Variant::ValueCase::kIntValue:
            value.set_int_value(_nodeHandle.param(_path, _defaultValue.int_value()));
            break;

        case swarmio::data::Variant::ValueCase::kStringValue:
            value.set_string_value(_nodeHandle.param(_path, _defaultValue.string_value()));
            break;
        
        default:
            throw swarmio::Exception("Unknown data type for variant");
    }
    return value;
}

void ParameterTarget::Set(const std::string& name, const swarmio::data::Variant& value)
{
    // Check name
    if (name != _name)
    {
        throw swarmio::Exception("Name mismatch for parameter set request.");
    }

    // Check if writable
    if (!_isWritable)
    {
        throw swarmio::Exception("Parameter is read-only.");
    }

    // Check if type matches
    if (value.value_case() != _defaultValue.value_case())
    {
        throw swarmio::Exception("Type mismatch for parameter set request.");
    }

    // Set value on parameter server
    switch (_defaultValue.value_case())
    {
        case swarmio::data::Variant::ValueCase::kBoolValue:
            _nodeHandle.setParam(_path, value.bool_value());
            break;

        case swarmio::data::Variant::ValueCase::kDoubleValue:
            _nodeHandle.setParam(_path, value.double_value());
            break;

        case swarmio::data::Variant::ValueCase::kIntValue:
            _nodeHandle.setParam(_path, value.int_value());
            break;

        case swarmio::data::Variant::ValueCase::kStringValue:
            _nodeHandle.setParam(_path, value.string_value());
            break;
        
        default:
            throw swarmio::Exception("Unknown data type for variant");
    }

    // Publish value
    PublishValue(value);
}

swarmio::data::discovery::Type ParameterTarget::GetType(const std::string& name) const
{
    // Check name
    if (name != _name)
    {
        throw swarmio::Exception("Name mismatch for parameter get type request.");
    }

    // Determine type
    switch (_defaultValue.value_case())
    {
        case swarmio::data::Variant::ValueCase::kBoolValue:
            return swarmio::data::discovery::Type::BOOL;

        case swarmio::data::Variant::ValueCase::kDoubleValue:
            return swarmio::data::discovery::Type::DOUBLE;

        case swarmio::data::Variant::ValueCase::kIntValue:
            return swarmio::data::discovery::Type::INT;

        case swarmio::data::Variant::ValueCase::kStringValue:
            return swarmio::data::discovery::Type::STRING;
        
        default:
            throw swarmio::Exception("Unknown data type for variant");
    }
}