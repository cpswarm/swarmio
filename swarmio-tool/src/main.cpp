#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/tool/Loop.h>
#include <czmq.h>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
 
int main(int argc, const char* argv[])
{
    // Initialize logging
    auto worker = g3::LogWorker::createLogWorker();
    worker->addDefaultLogger("swarmio-tool", ".");
    initializeLogging(worker.get());

    // Wrap to trigger destructors before shutdown
    {
        // Create Zyre endpoint
        swarmio::transport::zyre::ZyreEndpoint endpoint("tool", nullptr); 
        
        // Print UUID
        std::cout << "Local node started with UUID: " << endpoint.GetUUID() << "\n";   

        // Start loop
        swarmio::tool::Loop loop(&endpoint);
        endpoint.Start();
        loop.Run();
    }

    // Shut down zsys manually to avoid assertion failure on Windows
    zsys_shutdown();

    // All is well
    return 0;
}
