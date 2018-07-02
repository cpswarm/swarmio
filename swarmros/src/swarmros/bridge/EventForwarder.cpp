#include <swarmros/bridge/EventForwarder.h>
#include <swarmros/Event.h>
#include <swarmio/Exception.h>
#include <google/protobuf/util/json_util.h>

using namespace swarmros;
using namespace swarmros::bridge;

EventForwarder::EventForwarder(ros::NodeHandle& nodeHandle, ros::Publisher& allPublisher, const std::string& name, const std::map<std::string, swarmio::data::discovery::Type>& parameters)
    : _allPublisher(allPublisher), _name(name), _parameters(parameters)
{
    _thisPublisher = nodeHandle.advertise<swarmros::Event>("events/" + name, 8);
}

void EventForwarder::EventWasTriggered(const swarmio::Node* node, const swarmio::data::event::Notification& event)
{
    // Serialize basic fields
    swarmros::Event message;
    message.node = node->GetUUID();
    message.name = event.name();

    // Include a JSON serialized version of the event
    google::protobuf::util::MessageToJsonString(event, &message.json);

    // Publish to both topics
    _thisPublisher.publish(message);
    _allPublisher.publish(message);
}

void EventForwarder::DescribeEvent(const std::string& name, swarmio::data::event::Descriptor& descriptor)
{
    if (name == _name)
    {
        auto& target = *descriptor.mutable_parameters();
        for (const auto& pair : _parameters)
        {
            target[pair.first] = pair.second;
        }
    }
    else
    {
        throw swarmio::Exception("Name mismatch for event description request.");
    }
}