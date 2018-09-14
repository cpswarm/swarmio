#include <swarmros/introspection/Serializer.h>
#include <swarmros/introspection/PrimitiveSerializer.h>
#include <swarmros/introspection/MessageSerializer.h>
#include <swarmros/UnqualifiedException.h>
#include <regex>
#include <ros/package.h>

using namespace swarmros;
using namespace swarmros::introspection;

std::map<std::string, std::unique_ptr<Serializer>> Serializer::_defaultSerializers = Serializer::LoadDefaultSerializers();

std::map<std::string, std::unique_ptr<Serializer>> Serializer::LoadDefaultSerializers()
{
    std::map<std::string, std::unique_ptr<Serializer>> map;

    // Primitive types
    map["bool"] = std::make_unique<PrimitiveSerializer>("bool", PrimitiveType::BOOL);
    map["int8"] = std::make_unique<PrimitiveSerializer>("int8", PrimitiveType::INT8);
    map["uint8"] = std::make_unique<PrimitiveSerializer>("uint8", PrimitiveType::UINT8);
    map["int16"] = std::make_unique<PrimitiveSerializer>("int16", PrimitiveType::INT16);
    map["uint16"] = std::make_unique<PrimitiveSerializer>("uint16", PrimitiveType::UINT16);
    map["int32"] = std::make_unique<PrimitiveSerializer>("int32", PrimitiveType::INT32);
    map["uint32"] = std::make_unique<PrimitiveSerializer>("uint32", PrimitiveType::UINT32);
    map["int64"] = std::make_unique<PrimitiveSerializer>("int64", PrimitiveType::INT64);
    map["uint64"] = std::make_unique<PrimitiveSerializer>("uint64", PrimitiveType::UINT64);
    map["float32"] = std::make_unique<PrimitiveSerializer>("float32", PrimitiveType::FLOAT32);
    map["float64"] = std::make_unique<PrimitiveSerializer>("float64", PrimitiveType::FLOAT64);
    map["string"] = std::make_unique<PrimitiveSerializer>("string", PrimitiveType::STRING);
    map["time"] = std::make_unique<PrimitiveSerializer>("time", PrimitiveType::TIME);
    map["duration"] = std::make_unique<PrimitiveSerializer>("duration", PrimitiveType::DURATION); 

    // Deprecated aliases
    map["byte"] = std::make_unique<PrimitiveSerializer>("byte", PrimitiveType::INT8);
    map["char"] = std::make_unique<PrimitiveSerializer>("char", PrimitiveType::UINT8);
    
    return map;  
}

const Serializer& Serializer::SerializerForType(const std::string& messageType, const std::string& parentPackage)
{
    auto found = _defaultSerializers.find(messageType);
    if (found != _defaultSerializers.end())
    {
        return *found->second;
    }
    else
    {
        return MessageSerializer::MessageSerializerForType(messageType, parentPackage);
    }
}