#include <swarmio/simulator/ExampleDevice.h>

using namespace swarmio;
using namespace swarmio::simulator;

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