#include <swarmio/simulator/ExampleDevice.h>
#include <swarmio/data/Helper.h>

using namespace swarmio;
using namespace swarmio::simulator;

ExampleDevice::ExampleDevice(Endpoint* endpoint)
    : MemberProfile(endpoint)    
{ 
    // Add message counter to telemetry schema
    data::discovery::Field field;
    field.set_type(data::discovery::Type::UINT);
    GetTelemetryService().SetFieldDefinitionForKey("incoming_messages", field, true);

    // Finish construction
    FinishConstruction();
}

bool ExampleDevice::ReceiveMessage(const Node* sender, const data::Message* message)
{
    // Increment message counter
    ++_messageCounter;

    // Update message counter telemetry key
    data::Variant value;
    value.set_uint_value(_messageCounter);
    GetTelemetryService().SetValue("incoming_messages", value);

    // Leave message unhandled
    return false;
}

void ExampleDevice::AddConstantTelemetryValue(const std::string& key, const data::Variant& value, bool includeInStatus)
{
    GetTelemetryService().SetFieldDefinitionForKey(key, data::Helper::GetFieldDescriptor(value), includeInStatus);
    GetTelemetryService().SetValue(key, value);
}

void ExampleDevice::AddFauxEventHandler(FauxEventHandler* eventHandler)
{
    _eventHandlers.push_back(eventHandler);
    _eventService.RegisterHandler(eventHandler->GetName(), eventHandler);
}

void ExampleDevice::AddInMemoryParameter(InMemoryParameter* parameter)
{
    _parameters.push_back(parameter);
    _keyvalueService.RegisterTarget(parameter->GetPath(), parameter);
}

ExampleDevice::~ExampleDevice()
{
    for (auto handler : _eventHandlers)
    {
        _eventService.UnregisterHandler(handler->GetName());
    }
    for (auto parameter : _parameters)
    {
        _keyvalueService.UnregisterTarget(parameter->GetPath());
    }
}