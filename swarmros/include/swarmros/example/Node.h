#pragma once

#include <swarmros/SimpleEvent.h>
#include <swarmros/ExampleEvent.h>
#include <swarmros/UInt.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <atomic>

namespace swarmros::example 
{
    class Node final
    {
        private:
            
            /**
             * @brief ROS node handle
             * 
             */
            ros::NodeHandle _handle;

            /**
             * @brief Event sink for ping events
             * 
             */
            ros::Subscriber _pingEventSink;

            /**
             * @brief Event sink for heartbeat events
             * 
             */
            ros::Subscriber _heartbeatEventSink;

            /**
             * @brief Event sink for example events
             * 
             */
            ros::Subscriber _exampleEventSink;

            /**
             * @brief Parameter subscription for report interval
             * 
             */
            ros::Subscriber _reportIntervalParameterSubscriber;

            /**
             * @brief Event source for simple events
             * 
             */
            ros::Publisher _heartbeatEventSource;

            /**
             * @brief Event source for example events
             * 
             */
            ros::Publisher _exampleEventSource;

            /**
             * @brief Event source for pong events
             * 
             */
            ros::Publisher _pongEventSource;

            /**
             * @brief Telemetry publisher for the heartbeat counter
             * 
             */
            ros::Publisher _heartbeatCounterTelemetryPublisher;

            /**
             * @brief Telemetry publisher for the complex message
             * 
             */
            ros::Publisher _complexMessageTelemetryPublisher;

            /**
             * @brief Worker thread
             * 
             */
            std::thread _worker;

            /**
             * @brief Shutdown signal for worker thread
             * 
             */
            std::atomic_bool _shutdownRequested;

            /**
             * @brief Global heartbeat counter
             * 
             */
            std::atomic_uint64_t _heartbeatCounter;

            /**
             * @brief Report interval
             * 
             */
            std::atomic_uint64_t _reportInterval;

            /**
             * @brief Entry point for worker thread
             * 
             */
            void Worker();

        public:

            /**
             * @brief Construct a new Node objet
             * 
             * @param name Node name
             */
            Node(const std::string& name);

            /**
             * @brief Handle an incoming SimpleEvent
             * 
             * @param event Event
             */
            void HandleSimpleEvent(const SimpleEvent& event);

            /**
             * @brief Handle an incoming ExampleEvent
             * 
             * @param event Event
             */
            void HandleExampleEvent(const ExampleEvent& event);

            /**
             * @brief Handle parameter changes
             * 
             * @param value Value
             */
            void HandleIntervalParameterChange(const UInt& value);

            /**
             * @brief Destroy the Node object
             * 
             */
            ~Node();
    };
}