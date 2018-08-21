#include <swarmros/example/Node.h>
#include <swarmros/ExampleComplexMessage.h>
#include <chrono>
#include <iostream>

using namespace swarmros;
using namespace swarmros::example;

Node::Node(const std::string& name)
    : _handle(name), _shutdownRequested(false), _reportInterval(50)
{
    // Add event sources
    _pongEventSource = _handle.advertise<SimpleEvent>("events/pong", 64, false);
    _heartbeatEventSource = _handle.advertise<SimpleEvent>("events/heartbeat", 64, false);
    _exampleEventSource = _handle.advertise<ExampleEvent>("events/report", 64, false);

    // Publish current counter value as telemetry
    _heartbeatCounterTelemetryPublisher = _handle.advertise<UInt>("telemetry/heartbeatCounter", 1, true);

    // Publish a complex message that will be filled with gibberish every time a new report is published
    _complexMessageTelemetryPublisher = _handle.advertise<ExampleComplexMessage>("telemetry/exampleComplexMessage", 1, true);

    // Add event sinks
    _pingEventSink = _handle.subscribe("/bridge/events/ping", 64, &Node::HandleSimpleEvent, this);
    _heartbeatEventSink = _handle.subscribe("/bridge/events/heartbeat", 64, &Node::HandleSimpleEvent, this);
    _exampleEventSink = _handle.subscribe("/bridge/events/report", 64, &Node::HandleExampleEvent, this);

    // Subscribe to parameter
    _reportIntervalParameterSubscriber = _handle.subscribe("/bridge/parameters/reportInterval", 1, &Node::HandleIntervalParameterChange, this);

    // Start worker thread
    _worker = std::thread(&Node::Worker, this);
}

void Node::Worker()
{
    for (uint64_t tick = 1; !_shutdownRequested; ++tick)
    {
        // Publish heartbeat event
        SimpleEvent heartbeatEvent;
        heartbeatEvent.header.name = "heartbeat";
        _heartbeatEventSource.publish(heartbeatEvent);

        // Every interval, report current counter value
        if (tick >= _reportInterval && _reportInterval != 0)
        {
            // Send report
            ExampleEvent exampleEvent;
            exampleEvent.header.name = "report";
            exampleEvent.counter = _heartbeatCounter;
            _exampleEventSource.publish(exampleEvent);

            // Reset tick counter
            tick = 0;

            // Create distributions
            std::uniform_int_distribution<uint64_t> intDistribution;
            std::uniform_real_distribution<double> doubleDistribution;

            // Publish complex message filled with random values
            ExampleComplexMessage complexMessage;
            complexMessage.submessage1.field1 = "random1";
            complexMessage.submessage2.field1 = "random2";
            complexMessage.field1 = intDistribution(_random);
            complexMessage.field2 = intDistribution(_random);
            complexMessage.field3 = doubleDistribution(_random);
            complexMessage.submessage1.field1 = intDistribution(_random);
            complexMessage.submessage1.field2 = intDistribution(_random);
            complexMessage.submessage2.field1 = intDistribution(_random);
            complexMessage.submessage2.field2 = intDistribution(_random);
            _complexMessageTelemetryPublisher.publish(complexMessage);
        }

        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Node::HandleSimpleEvent(const swarmros::SimpleEvent& event)
{
    if (event.header.name == "ping")
    {
        // Respond
        SimpleEvent pongEvent;
        pongEvent.header.name = "pong";
        pongEvent.header.node = event.header.node;
        _pongEventSource.publish(pongEvent);

        // Log
        std::cout << "A response was sent for a ping SimpleEvent to node " << pongEvent.header.node << std::endl;
    }
    else if (event.header.name == "heartbeat")
    {
        // Increment
        uint64_t current = ++_heartbeatCounter;

        // Publish update
        UInt value;
        value.value = current;
        _heartbeatCounterTelemetryPublisher.publish(value);
    }
    else
    {
        std::cout << "Unknown SimpleEvent received: " << event.header.name << std::endl;
    }
}

void Node::HandleExampleEvent(const swarmros::ExampleEvent& event)
{
    if (event.header.name == "report")
    {
        std::cout << "Node " << event.header.node << " reports that its counter is at " << event.counter << std::endl;
    }
    else
    {
        std::cout << "Unknown ExampleEvent received: " << event.header.name << std::endl;
    }
}

void Node::HandleIntervalParameterChange(const UInt& value)
{
    // Set interval
    _reportInterval = value.value;
}

Node::~Node()
{
    _shutdownRequested = true;
}
