#include <swarmio/simulator/ExampleDevice.h>
#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/Exception.h>
#include <czmq.h>
#include <iostream>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

int main(int argc, const char* argv[])
{
    // Initialize logging
    auto worker = g3::LogWorker::createLogWorker();
    worker->addDefaultLogger("swarmio-simulator", ".");
    initializeLogging(worker.get());

    // Wrap to trigger destructors before shutdown
    {
        // Create a Zyre endpoint
        swarmio::transport::zyre::ZyreEndpoint endpoint("simulator");    

        // Print UUID
        std::cout << "Local node started with UUID: " << endpoint.GetUUID() << "\n";

        // Create device
        swarmio::simulator::ExampleDevice device(&endpoint);

        // Register static telemetry value
        swarmio::data::Variant value;
        value.set_string_value("This won't ever change.");
        device.AddConstantTelemetryValue("static_value", value, false);

        // Register static telemetry and status value
        value.set_string_value("And neither will this.");
        device.AddConstantTelemetryValue("static_status_value", value, true);

        // Register complex static telemetry value
        value.Clear();
        auto& pairs = *value.mutable_map_value()->mutable_pairs();
        pairs["key1"].set_string_value("value1");
        pairs["key2"].set_uint_value(2);
        pairs["key3"].set_int_value(-3);
        auto& pairs2 = *pairs["key4"].mutable_map_value()->mutable_pairs();
        pairs2["embedded1"].set_string_value("subvalue1");
        pairs2["embedded2"].set_uint_value(2);
        device.AddConstantTelemetryValue("complex_value", value, false);

        // Register array static telemetry value
        value.Clear();
        auto& elements = *value.mutable_int_array();
        elements.add_elements(30);
        elements.add_elements(55);
        elements.add_elements(0);
        elements.add_elements(-45);
        elements.add_elements(900);
        device.AddConstantTelemetryValue("array_value", value, false);

        // Register some parameters
        swarmio::simulator::InMemoryParameter p1("examples/boolParameter", false);
        device.AddInMemoryParameter(&p1);
        swarmio::simulator::InMemoryParameter p2("examples/stringParameter", "unknown");
        device.AddInMemoryParameter(&p2);
        swarmio::simulator::InMemoryParameter p3("examples/intParameter", 1024);
        device.AddInMemoryParameter(&p3);
        swarmio::simulator::InMemoryParameter p4("examples/doubleParameter", 2.5);
        device.AddInMemoryParameter(&p4);
        swarmio::simulator::InMemoryParameter p5("examples/readOnlyParameter", "Can't change this", true);
        device.AddInMemoryParameter(&p5);

        // Register some events
        swarmio::simulator::FauxEventHandler e1("emergency");
        e1.AddParameter("where", swarmio::data::discovery::Type::STRING);
        e1.AddParameter("severity", swarmio::data::discovery::Type::INT);
        device.AddFauxEventHandler(&e1);
        swarmio::simulator::FauxEventHandler e2("blackHawkDown");
        e2.AddParameter("really", swarmio::data::discovery::Type::BOOL);
        device.AddFauxEventHandler(&e2);

        // Start sending and receiving
        endpoint.Start();

        // Loop endlessly
        std::cout << "Press ENTER to stop the simulator." << "\n";

        // Get a character
        std::cin.get();

        // Stop endpoint
        endpoint.Stop();
    }

    // Shut down zsys manually to avoid assertion failure on Windows
    zsys_shutdown();

    // All is well
    return 0;
}
