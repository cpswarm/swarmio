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
        device.SetTelemetryValue("static_value", value);

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
    }

    // Shut down zsys manually to avoid assertion failure on Windows
    zsys_shutdown();

    // All is well
    return 0;
}
