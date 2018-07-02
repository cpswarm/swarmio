#include <swarmros/bridge/TelemetrySource.h>
#include <swarmio/Exception.h>

using namespace swarmros;
using namespace swarmros::bridge;

TelemetrySource::TelemetrySource(ros::NodeHandle& nodeHandle, swarmio::services::telemetry::Service* telemetryService, const std::string& name, const std::string& path, swarmio::data::discovery::Type type)
    : _telemetryService(telemetryService), _name(name), _path(path)
{
    switch (type)
    {
        case swarmio::data::discovery::Type::BOOL:
            _subscriber = nodeHandle.subscribe(path, 1, &TelemetrySource::ReceiveBoolUpdate, this);
            break;

        case swarmio::data::discovery::Type::DOUBLE:
            _subscriber = nodeHandle.subscribe(path, 1, &TelemetrySource::ReceiveFloat64Update, this);
            break;

        case swarmio::data::discovery::Type::INT:
            _subscriber = nodeHandle.subscribe(path, 1, &TelemetrySource::ReceiveInt32Update, this);
            break;

        case swarmio::data::discovery::Type::STRING:
            _subscriber = nodeHandle.subscribe(path, 1, &TelemetrySource::ReceiveStringUpdate, this);
            break;

        default:
            throw swarmio::Exception("Unknown type for telemetry data");
    }
}

void TelemetrySource::ReceiveBoolUpdate(const std_msgs::Bool& message)
{
    swarmio::data::Variant value;
    value.set_bool_value(message.data);
    _telemetryService->SetValue(_name, value);
}

void TelemetrySource::ReceiveFloat64Update(const std_msgs::Float64& message)
{
    swarmio::data::Variant value;
    value.set_double_value(message.data);
    _telemetryService->SetValue(_name, value);
}

void TelemetrySource::ReceiveInt32Update(const std_msgs::Int32& message)
{
    swarmio::data::Variant value;
    value.set_int_value(message.data);
    _telemetryService->SetValue(_name, value);
}

void TelemetrySource::ReceiveStringUpdate(const std_msgs::String& message)
{
    swarmio::data::Variant value;
    value.set_string_value(message.data);
    _telemetryService->SetValue(_name, value);
}

TelemetrySource::~TelemetrySource()
{
    _telemetryService->RemoveValue(_name);
}