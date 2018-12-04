#include <swarmio/simulator/ExampleDevice.h>
#include <swarmio/simulator/LinearPathTelemetrySimulator.h>
#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/Exception.h>
#include <czmq.h>
#include <iostream>
#include <chrono>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

using namespace swarmio;
using namespace swarmio::simulator;
using namespace std::chrono_literals;

int main(int argc, const char* argv[])
{
    // Initialize logging
    auto worker = g3::LogWorker::createLogWorker();
    worker->addDefaultLogger("swarmio-simulator", ".");
    initializeLogging(worker.get());

    // Wrap to trigger destructors before shutdown
    {
        // Create Zyre endpoint
        char* hostname = zsys_hostname();
        swarmio::transport::zyre::ZyreEndpoint endpoint(hostname, "simulator"); 
        zstr_free(&hostname);

        // Assign interface
        if (argc > 1)
        {
            endpoint.SetInterface(argv[1]);
            std::cout << "Selected interface: " << argv[1] << std::endl;
        }

        // Print UUID
        std::cout << "Local node started with UUID: " << endpoint.GetUUID() << "\n";

        // Create device
        ExampleDevice device(&endpoint);

        // Register static telemetry value
        data::Variant value;
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
        InMemoryParameter p1("examples/boolParameter", false);
        device.AddInMemoryParameter(&p1);
        InMemoryParameter p2("examples/stringParameter", "unknown");
        device.AddInMemoryParameter(&p2);
        InMemoryParameter p3("examples/intParameter", 1024);
        device.AddInMemoryParameter(&p3);
        InMemoryParameter p4("examples/doubleParameter", 2.5);
        device.AddInMemoryParameter(&p4);
        InMemoryParameter p5("examples/readOnlyParameter", "Can't change this", true);
        device.AddInMemoryParameter(&p5);

        // Register some events
        FauxEventHandler e1("emergency");
        e1.AddParameter("where", data::discovery::Type::STRING);
        e1.AddParameter("severity", data::discovery::Type::INT);
        device.AddFauxEventHandler(&e1);
        FauxEventHandler e2("blackHawkDown");
        e2.AddParameter("really", data::discovery::Type::BOOL);
        device.AddFauxEventHandler(&e2);

        // Simulate position
        LinearPathTelemetrySimulator pathSimulator(device.GetTelemetryService(), "location", SimulatedLocation(19.040235, 47.497912, 18.0), SimulatedLocation(48.210033, 16.363449, 23.0), 1h);

        // Start sending and receiving
        endpoint.Start();

        // Loop endlessly
        std::cout << "Press ENTER to stop the simulator." << "\n";

        // Get a character
        std::cin.get();

        // Stop telemetry
        pathSimulator.Stop();

        // Stop endpoint
        endpoint.Stop();
    }

    // Shut down zsys manually to avoid assertion failure on Windows
    zsys_shutdown();

    // All is well
    return 0;
}
