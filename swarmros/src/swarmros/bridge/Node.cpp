#include <swarmros/bridge/Node.h>
#include <swarmros/bridge/EventForwarder.h>
#include <swarmros/bridge/EventPublisher.h>
#include <swarmros/bridge/TelemetryForwarder.h>
#include <swarmros/bridge/ParameterPublisher.h>
#include <swarmros/bridge/ParameterTarget.h>
#include <swarmros/NodeInfo.h>

using namespace swarmros;
using namespace swarmros::bridge;

Node::Node(swarmio::Endpoint* endpoint)
    : MemberProfile(endpoint)
{
    _nodesPublisher = _nodeHandle.advertise<NodeInfo>("bridge/nodes", 128, true);
    FinishConstruction();
}

void Node::PublishUpdateForNode(const swarmio::Node* node)
{
    swarmros::NodeInfo message;
    message.uuid = node->GetUUID();
    message.name = node->GetName();
    message.deviceClass = node->GetDeviceClass();
    message.online = node->IsOnline();
    _nodesPublisher.publish(message);
}

void Node::NodeWasDiscovered(const swarmio::Node* node) noexcept
{
    PublishUpdateForNode(node);
}

void Node::NodeDidJoin(const swarmio::Node* node) noexcept
{
    PublishUpdateForNode(node);
}

void Node::NodeWillLeave(const swarmio::Node* node) noexcept
{
    PublishUpdateForNode(node);
}

void Node::ForwardTelemetry(const std::string& source, const std::string& message, const std::string& name, bool includeInStatus)
{
    _pylons.push_back(std::make_unique<TelemetryForwarder>(_nodeHandle, source, message, GetTelemetryService(), name, includeInStatus));
}

void Node::ForwardEvent(const std::string& source, const std::string& message)
{
    _pylons.push_back(std::make_unique<EventForwarder>(_nodeHandle, source, message, GetEndpoint()));
}

void Node::PublishEvent(const std::string& suffix, const std::string& message, const std::string& name)
{
    _pylons.push_back(std::make_unique<EventPublisher>(_nodeHandle, suffix, message, GetEventService(), name));
}

void Node::PublishParameter(const std::string& suffix, const std::string& message, const std::string& name, const std::string& parameter, bool isWritable)
{
    _pylons.push_back(std::make_unique<ParameterPublisher>(_nodeHandle, suffix, message, GetKeyValueService(), name, parameter, isWritable));
}

void Node::BridgeParameter(const std::string& name, const std::string& parameter, bool isWritable)
{
    _pylons.push_back(std::make_unique<ParameterTarget>(_nodeHandle, GetKeyValueService(), name, parameter, isWritable));
}