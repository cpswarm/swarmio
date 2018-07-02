#include <swarmio/transport/zyre/ZyreEndpoint.h>
#include <swarmio/data/discovery/Type.pb.h>
#include <swarmio/data/Variant.pb.h>
#include <swarmio/Exception.h>
#include <swarmros/bridge/Device.h>
#include <swarmros/bridge/DebugSink.h>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <libconfig.h++>
#include <ros/ros.h>
#include <czmq.h>
#include <map>

using namespace swarmros;

/**
 * @brief Get a map of interface names and addresses
 * 
 * @return std::map<std::string, std::string> Interfaces
 */
static std::map<std::string, std::string> GetInterfaceMap()
{
    std::map<std::string, std::string> map;

    // Get list of interfaces
    ziflist_t* ifaces = ziflist_new();
    if (ifaces != nullptr)
    {
        // Store interface names and current addresses
        for (const char* current = ziflist_first(ifaces); current != nullptr; current = ziflist_next(ifaces))
        {
            map[current] = ziflist_address(ifaces);
        }

        // Free list of interfaces
        ziflist_destroy(&ifaces);
    }

    return map;
}

/**
 * @brief Convert a setting to a data type
 * 
 * @param setting Setting
 * @return swarmio::data::discovery::Type Type
 */
static swarmio::data::discovery::Type SettingToType(const libconfig::Setting& setting)
{
    std::string value = setting;
    if (value == "bool")
    {
        return swarmio::data::discovery::Type::BOOL;
    }
    else if (value == "int")
    {
        return swarmio::data::discovery::Type::INT;
    }
    else if (value == "double")
    {
        return swarmio::data::discovery::Type::DOUBLE;
    }
    
    else if (value == "string")
    {
        return swarmio::data::discovery::Type::STRING;
    }
    else
    {
        throw swarmio::Exception("Invalid type definition");
    }
}

/**
 * @brief Convert a setting to a variant value
 * 
 * @param setting Setting
 * @return swarmio::data::Variant Variant
 */
static swarmio::data::Variant SettingToVariant(const libconfig::Setting& setting, swarmio::data::discovery::Type type)
{
    swarmio::data::Variant value;
    switch (type)
    {
        case swarmio::data::discovery::Type::BOOL:
            value.set_bool_value(setting);
            break;

        case swarmio::data::discovery::Type::INT:
            value.set_int_value(setting);
            break;

        case swarmio::data::discovery::Type::DOUBLE:
            value.set_double_value(setting);
            break;

        case swarmio::data::discovery::Type::STRING:
            value.set_string_value((const char*)setting);
            break;

        default:
            throw swarmio::Exception("Unknown type for variant");
    }
    return value;
}

/**
 * Entry point for the node
 */
int main(int argc, const char* argv[])
{
    // Initialize ROS
    ros::init(argc, (char**)argv, "bridge");

    // Initialize logging
    auto worker = g3::LogWorker::createLogWorker();
    worker->addSink(std::make_unique<bridge::DebugSink>(), &bridge::DebugSink::ReceiveLogMessage);
    initializeLogging(worker.get());

    // Load configuration and establish endpoint
    libconfig::Config config;
    std::unique_ptr<swarmio::Endpoint> endpoint;
    try
    {
        config.readFile(SWARMROS_CONFIG_PATH);
        std::string type = (const char*)config.lookup("endpoint.type");
        if (type == "zyre")
        {
            // Create endpoint
            auto zyreEndpoint = std::make_unique<swarmio::transport::zyre::ZyreEndpoint>(config.lookup("endpoint.name"));

            // Apply settings
            if (config.exists("endpoint.port"))
            {
                unsigned port = config.lookup("endpoint.port");
                if (port <= UINT16_MAX)
                {
                    zyreEndpoint->SetPort((uint16_t)port);
                } 
                else
                {
                    LOG(FATAL) << "Port is out of range.";
                    return -1;
                }
            }
            if (config.exists("endpoint.ifname"))
            {
                // Get interface name
                const char* name = config.lookup("endpoint.ifname");

                // Check that it exists
                auto map = GetInterfaceMap();
                if (map.find(name) != map.end())
                {
                    zyreEndpoint->SetInterface(name);
                }
                else
                {
                    LOG(DBUG) << "Available interfaces:";
                    for (auto& pair : map)
                    {
                        LOG(DBUG) << " - " << pair.first << " (" << pair.second << ")";
                    }
                    LOG(FATAL) << "An invalid interface name has been specified.";
                    return -1;
                }
            }

            // Move to base
            endpoint = std::move(zyreEndpoint);
        }
        else
        {
            LOG(FATAL) << "Unknown endpoint type specified in configuration file.";
            return -1;
        }
    }
    catch (const libconfig::FileIOException& e)
    {
        LOG(DBUG) << e.what();
        LOG(FATAL) << "An exception has occurred while trying to read the configuration file.";
        return -1;
    }
    catch (const libconfig::ParseException& e)
    {
        LOG(DBUG) << e.getError();
        LOG(FATAL) << "An exception has occurred while trying to parse the configuration file (line " << e.getLine() << ").";
        return -1;
    }
    catch(const swarmio::Exception& e)
    {
        LOG(DBUG) << e.what();
        LOG(FATAL) << "An exception has occurred while trying to initialize the endpoint.";
        return -1;
    }

    // Create device
    auto device = std::make_unique<bridge::Device>(endpoint.get());

    // Process service descriptors
    try
    {
        if (config.exists("services"))
        {
            auto& services = config.lookup("services");
            for (int i = 0; i < services.getLength(); ++i)
            {
                auto& service = services[i];
                std::string name = service.getName();
                for (int j = 0; j < service.getLength(); ++j)
                {
                    auto& entry = service[j];
                    if (name == "keyvalue")
                    {
                        auto type = SettingToType(entry.lookup("type"));
                        auto defaultValue = SettingToVariant(entry.lookup("defaultValue"), type);
                        device->PublishParameter(entry.lookup("name"), entry.lookup("path"), entry.lookup("writable"), defaultValue);
                    }
                    else if (name == "telemetry")
                    {
                        device->ForwardTelemetry(entry.lookup("name"), entry.lookup("path"), SettingToType(entry.lookup("type")));
                    }
                    else if (name == "event")
                    {
                        auto& parameters = entry.lookup("parameters");
                        std::map<std::string, swarmio::data::discovery::Type> mapping;
                        for (int k = 0; k < entry.getLength(); ++k)
                        {
                            auto& parameter = parameters[k];
                            mapping[parameter.lookup("name")] = SettingToType(parameter.lookup("type"));
                        }
                        device->ForwardEvent(entry.lookup("name"), mapping);
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
    catch (const libconfig::SettingTypeException & e)
    {
        LOG(FATAL) << "Invalid type for setting at " << e.getPath() << ".";
        return -2;
    }
    catch (const libconfig::SettingNotFoundException & e)
    {
        LOG(FATAL) << "Missing setting at " << e.getPath() << ".";
        return -2;
    }
    catch(const swarmio::Exception& e)
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
