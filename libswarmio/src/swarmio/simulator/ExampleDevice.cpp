#include <swarmio/simulator/ExampleDevice.h>

using namespace swarmio;
using namespace swarmio::simulator;

bool ExampleDevice::ReceiveMessage(const Node* sender, const data::Message* message)
{
    // Increment message counter
    ++_messageCounter;

    // Publish telemetry
    data::Variant value;
    value.set_int_value(_messageCounter);
    SetTelemetryValue("incoming_messages", value);

    // Leave message unhandled
    return false;
}

void ExampleDevice::SetTelemetryValue(const std::string& key, const data::Variant& value)
{
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