#include <swarmros/bridge/Device.h>
#include <swarmros/Event.h>

using namespace swarmros;
using namespace swarmros::bridge;

Device::Device(swarmio::Endpoint* endpoint)
    : MemberProfile(endpoint), _nodeHandle("swarmros")
{
    _eventPublisher = _nodeHandle.advertise<swarmros::Event>("events/all", 64);
}

void Device::ForwardEvent(const std::string& name, const std::map<std::string, swarmio::data::discovery::Type>& parameters)
{
    if (name == "all")
    {
        throw swarmio::Exception("Reserved keyword 'all' cannot be used as an event name");
    }
    auto& element = _eventForwarders.emplace_back(_nodeHandle, _eventPublisher, name, parameters);
    GetEventService().RegisterHandler(element.GetName(), &element);
}

void Device::PublishParameter(const std::string& name, const std::string& path, bool isWritable, const swarmio::data::Variant& defaultValue)
{
    auto& element = _parameterTargets.emplace_back(_nodeHandle, name, path, isWritable, defaultValue);
    GetKeyValueService().RegisterTarget(element.GetName(), &element);
}

void Device::ForwardTelemetry(const std::string& name, const std::string& path, swarmio::data::discovery::Type type)
{
    auto& element = _telemetrySources.emplace_back(_nodeHandle, &GetTelemetryService(), name, path, type);
}

Device::~Device()
{
    for (auto& target : _parameterTargets)
    {
        GetKeyValueService().UnregisterTarget(target.GetName());
    }
    for (auto& forwarder : _eventForwarders)
    {
        GetEventService().UnregisterHandler(forwarder.GetName());
    }
}