#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/data/discovery/Schema.pb.h>
#include <swarmio/data/Variant.pb.h>
#include <swarmio/Exception.h>
#include <swarmros/bridge/Node.h>
#include <swarmros/bridge/DebugSink.h>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <libconfig.h++>
#include <ros/ros.h>
#include <czmq.h>
#include <map>

using namespace swarmros;

/**
 * Entry point for the node
 */
int main(int argc, const char *argv[])
{
    // Initialize ROS
    ros::init(argc, (char **)argv, "bridge");

    // Initialize logging
    auto worker = g3::LogWorker::createLogWorker();
    worker->addSink(std::make_unique<bridge::DebugSink>(), &bridge::DebugSink::ReceiveLogMessage);
    initializeLogging(worker.get());
    // Parse the rest of the arguments
    std::string configFilePath = SWARMROS_CONFIG_PATH;
    for (int i = 1; i < argc; ++i)
    {
        const char *current = argv[i];
        if (current[0] == '-')
        {
            if (current[1] == 'C')
            {
                // Config file remapping
                configFilePath = current + 2;
            }
            else if (current[1] == 'D' && current[2] == '\0')
            {
                // Enable debug mode
                if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
                {
                    ros::console::notifyLoggerLevelsChanged();
                }
            }
            else
            {
                LOG(FATAL) << "Command line argument " << i << " is unknown";
                return -10;
            }
        }
        else
        {
            LOG(FATAL) << "Command line argument " << i << " cannot be processed";
            return -10;
        }
    }

    // Load configuration and establish endpoint
    libconfig::Config config;
    std::unique_ptr<swarmio::Endpoint> endpoint;
    try
    {
        config.setIncludeDir(configFilePath.substr(0, configFilePath.find_last_of("/\\")).c_str());
        config.readFile(configFilePath.c_str());
        std::string type = (const char *)config.lookup("endpoint.type");
        if (type == "zyre")
        {
            // Create endpoint
            auto zyreEndpoint = std::make_unique<swarmio::transport::zyre::ZyreEndpoint>(config.lookup("endpoint.name"), config.lookup("endpoint.deviceClass"));
            zyreEndpoint->SetConfig(configFilePath);
            endpoint = std::move(zyreEndpoint);
        }
        else
        {
            LOG(FATAL) << "Unknown endpoint type specified in configuration file.";
            return -1;
        }
    }
    catch (const libconfig::SettingTypeException &e)
    {
        LOG(FATAL) << "Invalid type for setting at " << e.getPath() << ".";
        return -1;
    }
    catch (const libconfig::SettingNotFoundException &e)
    {
        LOG(FATAL) << "Missing setting at " << e.getPath() << ".";
        return -1;
    }
    catch (const libconfig::FileIOException &e)
    {
        LOG(DBUG) << e.what();
        LOG(FATAL) << "An exception has occurred while trying to read the configuration file.";
        return -1;
    }
    catch (const libconfig::ParseException &e)
    {
        LOG(DBUG) << e.getError();
        LOG(FATAL) << "An exception has occurred while trying to parse the configuration file (line " << e.getLine() << ").";
        return -1;
    }
    catch (const swarmio::Exception &e)
    {
        LOG(DBUG) << e.what();
        LOG(FATAL) << "An exception has occurred while trying to initialize the endpoint.";
        return -1;
    }

    // Create device
    auto device = std::make_unique<bridge::Node>(endpoint.get());

    // Process service descriptors
    try
    {
        if (config.exists("services"))
        {
            auto &services = config.lookup("services");
            for (int i = 0; i < services.getLength(); ++i)
            {
                auto &service = services[i];
                std::string name = service.getName();
                for (int j = 0; j < service.getLength(); ++j)
                {
                    auto &entry = service[j];
                    if (name == "publishedParameters")
                    {
                        device->PublishParameter(
                            entry.lookup("suffix"),
                            entry.lookup("message"),
                            entry.lookup("name"),
                            entry.lookup("path"),
                            entry.lookup("rw"));
                    }
                    else if (name == "bridgedParameters")
                    {
                        device->BridgeParameter(
                            entry.lookup("name"),
                            entry.lookup("path"),
                            entry.lookup("rw"));
                    }
                    else if (name == "telemetryTopics")
                    {
                        device->ForwardTelemetry(
                            entry.lookup("source"),
                            entry.lookup("message"),
                            entry.lookup("name"),
                            entry.lookup("status"));
                    }
                    else if (name == "incomingEvents")
                    {
                        device->PublishEvent(
                            entry.lookup("suffix"),
                            entry.lookup("message"),
                            entry.lookup("name"));
                    }
                    else if (name == "outgoingEvents")
                    {
                        device->ForwardEvent(
                            entry.lookup("source"),
                            entry.lookup("message"));
                    }
                    else
                    {
                        LOG(WARNING) << "Unknown service type: " << name;
                        break;
                    }
                }
            }
        }
    }
    catch (const libconfig::SettingTypeException &e)
    {
        LOG(FATAL) << "Invalid type for setting at " << e.getPath() << ".";
        return -2;
    }
    catch (const libconfig::SettingNotFoundException &e)
    {
        LOG(FATAL) << "Missing setting at " << e.getPath() << ".";
        return -2;
    }
    catch (const swarmio::Exception &e)
    {
        LOG(DBUG) << e.what();
        LOG(FATAL) << "An exception has occurred while trying to initialize the endpoint.";
        return -2;
    }

    // Start endpoint
    endpoint->Start();

    // Spin ROS
    ros::spin();

    // Stop gracefully
    endpoint->Stop();

    // Exit
    return 0;
}
