#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/tool/Loop.h>
#include <czmq.h>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <swarmio/tool/LogBuffer.h>
#include <memory>
#include <czmq.h>
 
int main(int argc, const char* argv[])
{
    // Initialize logging
    auto worker = g3::LogWorker::createLogWorker();
    auto buffer = std::make_unique<swarmio::tool::LogBuffer>();
    auto buffer_ptr = buffer.get();
    worker->addSink(std::move(buffer), &swarmio::tool::LogBuffer::ReceiveLogMessage);    
    initializeLogging(worker.get());

    // Wrap to trigger destructors before shutdown
    {
        // Create Zyre endpoint
        char* hostname = zsys_hostname();
        swarmio::transport::zyre::ZyreEndpoint endpoint(hostname, "tool"); 
        zstr_free(&hostname);

        // Assign interface
        if (argc > 1)
        {
            endpoint.SetInterface(argv[1]);
            std::cout << "Selected interface: " << argv[1] << std::endl;
        }
        
        // Print UUID
        std::cout << "Local node started with UUID: " << endpoint.GetUUID() << std::endl; 

        // Start endpoint
        endpoint.Start();

        // Start loop
        swarmio::tool::Loop loop(&endpoint, buffer_ptr);
        loop.Run();

        // Stop endpoint
        endpoint.Stop();
    }

    // Shut down zsys manually to avoid assertion failure on Windows
    zsys_shutdown();

    // All is well
    return 0;
}
