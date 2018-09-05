#include <swarmros/introspection/MessageSerializer.h>
#include <swarmros/introspection/ConstantField.h>
#include <swarmros/introspection/ArrayField.h>
#include <swarmros/introspection/IndexedFieldStack.h>
#include <swarmros/introspection/FixedArrayField.h>
#include <swarmros/introspection/TypeMismatchException.h>
#include <swarmros/introspection/FieldStack.h>
#include <swarmros/introspection/MessageDefinitionParserException.h>
#include <swarmros/UnqualifiedException.h>
#include <ros/package.h>
#include <external/md5.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <regex>

using namespace swarmros;
using namespace swarmros::introspection;

// Map of message serializers
std::map<std::string, std::unique_ptr<MessageSerializer>> MessageSerializer::_messageSerializers;

// Regex to match the name against
static const std::regex NamePattern("^(?:([a-zA-Z][\\w|/]*)\\/)?([a-zA-Z]\\w*)$");

// Regex to match an entire definition line against
static const std::regex LinePattern("^\\s*(?:([^\\s#]+)\\s+([a-zA-Z][a-zA-Z1-9_]*)\\s*(?:=\\s*(.*\\S)\\s*)?)?(?:#.*)?$");

// Regex to match the type against
static const std::regex TypePattern("^([^\\[]+)(\\[(\\d+)?\\])?$");

const MessageSerializer& MessageSerializer::MessageSerializerForType(const std::string& messageType, const std::string& parentPackage)
{
    // Check if it exists
    auto found = _messageSerializers.find(messageType);
    if (found != _messageSerializers.end())
    {
        return *found->second;
    }

    // If not, process name
    std::smatch match;
    if (std::regex_match(messageType, match, NamePattern))
    {
        // Get parts
        auto package = match[1].str();
        auto type = match[2].str();

        // Handle the special case of the Header field
        if (package.size() == 0)
        {
            if (parentPackage.size() > 0)
            {
                package = parentPackage;
            }
            else
            {
                throw UnqualifiedException("Relative package name with no parent package");
            }
        }

        // Check if we have a cached helper
        auto key = package + "/" + type;
        auto found = _messageSerializers.find(key);
        if (found != _messageSerializers.end())
        {
            return *found->second;
        }

        // Get base path
        auto path = ros::package::getPath(package);

        // Append message file path
        if (path.size() > 0)
        {
            path += "/msg/" + type + ".msg";
        }
        else
        {
            throw UnqualifiedException("Package for message not found");
        }

        // Build definition
        _messageSerializers[key] = std::make_unique<MessageSerializer>(package, type, path);

        // Return value
        return *_messageSerializers[key];
    }
    else
    {
        throw UnqualifiedException("Invalid message name");
    }
}

MessageSerializer::MessageSerializer(const std::string& package, const std::string& name, const std::string& path)
    : Serializer(name), _package(package), _fullName(package + "/" + name), _shortcut(nullptr), _hasHeader(false)
{
    // Read file line-by-line
    std::ifstream stream(path);
    std::string line;
    int currentLine = 0;
    while (std::getline(stream, line))
    {
        // Increment line counter and wrap exception qualifier
        ++currentLine;
        try
        {
            // Perform match
            std::smatch lineMatch;
            if (std::regex_match(line, lineMatch, LinePattern))
            {
                if (lineMatch[1].matched)
                {
                    // Get parts
                    auto type = lineMatch[1].str();
                    auto name = lineMatch[2].str();

                    // Determine if this is a constant field
                    if (lineMatch[3].matched)
                    {
                        // Extract literal value
                        auto value = lineMatch[3].str();

                        // Fetch serializer
                        const Serializer& serializer = Serializer::SerializerForType(type, package);

                        // Add constant field
                        _constants.emplace_back(name, serializer, value);
                    }
                    else
                    {
                        // Perform match
                        std::smatch typeMatch;
                        if (std::regex_match(type, typeMatch, TypePattern))
                        {
                            // Override parent package for header fields
                            const char* parentPackage = package.c_str();
                            if (typeMatch[1].str() == "Header" && _fields.empty())
                            {
                                parentPackage = "std_msgs";
                            }

                            // Fetch serializer
                            const Serializer& serializer = Serializer::SerializerForType(typeMatch[1].str(), parentPackage);

                            // Determine if this is an array
                            if (typeMatch[2].matched)
                            {
                                // Determine if dynamic or fixed
                                if (typeMatch[3].matched)
                                {
                                    // Fixed array
                                    auto length = std::stoul(typeMatch[3].str());
                                    if (length > 0)
                                    {
                                        _fields.push_back(std::make_unique<FixedArrayField>(name, serializer, length));
                                    }
                                    else
                                    {
                                        throw UnqualifiedException("Invalid length for array");
                                    }
                                }
                                else
                                {
                                    // Dynamic array
                                    _fields.push_back(std::make_unique<ArrayField>(name, serializer));
                                }
                            }
                            else
                            {
                                // Simple type
                                _fields.push_back(std::make_unique<Field>(name, serializer));
                            }
                        }
                        else
                        {
                            throw UnqualifiedException("Invalid type definition");
                        }
                    }
                }
                else
                {
                    // Empty line, ignore
                }
            }
            else
            {
                throw UnqualifiedException("Invalid line in message definition");
            }
        }
        catch (const UnqualifiedException& e)
        {
            // Rethrow with location information
            throw MessageDefinitionParserException(e.what(), path, currentLine);
        }
    }

    // Build canonical definition
    _canonicalDefinition = BuildCanonicalDefinition();

    // Calculate MD5 hash
    _hash = CalculateHash();

    // For primitives in the std_msgs package, and for
    // the parameter types in this package, a shortcut 
    // is used: the value / data field is deserialized
    // and used directly instead of being placed into
    // a map.    
    if (_fields.size() == 1)
    {
        const auto& name = _fields.front()->GetName();
        if ((package == "swarmros" && name == "value") || (package == "std_msgs" && name == "data"))
        {
            _shortcut = _fields.front().get();
        }
    }
    
    // Check if we have a standard message header
    if (!_fields.empty() && _fields.front()->GetName() == "header" && _fields.front()->GetSerializer().GetFullName() == "std_msgs/Header")
    {
        _hasHeader = true;
    }
}

std::string MessageSerializer::BuildCanonicalDefinition()
{
    // Just use fields as is
    std::stringstream stream;
    for (const auto& field : _fields)
    {
        field->WriteDefinition(stream, false);
        stream << std::endl;
    }

    // Return definition
    return stream.str();
}

std::string MessageSerializer::CalculateHash()
{
    // For the first round, we only take the constant fields
    bool firstLine = true;
    std::stringstream stream;
    for (const auto& constant : _constants)
    {
        if (firstLine)
        {
            firstLine = false;
        }
        else
        {
            stream << std::endl;
        }
        constant.WriteDefinition(stream, true);
    }

    // For the second round, we only take the dynamic fields
    for (const auto& field : _fields)
    {
        if (firstLine)
        {
            firstLine = false;
        }
        else
        {
            stream << std::endl;
        }
        field->WriteDefinition(stream, true);
    }
    
    // Calculate hash
    unsigned char rawHash[16];
    std::string base = stream.str();
    MD5_CTX context;
    MD5_Init(&context);
    MD5_Update(&context, base.c_str(), base.size());
    MD5_Final(rawHash, &context);

    // Convert to string
    std::stringstream hashStream;
    hashStream << std::hex << std::setfill('0');
    for (int i = 0; i < sizeof(rawHash); ++i)
    {
        hashStream << std::setw(2) << (int)rawHash[i];
    }
    return hashStream.str();
}

uint32_t MessageSerializer::CalculateSerializedLength(const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    return CalculateSerializedLength(value, 0, fieldStack);
}

uint32_t MessageSerializer::CalculateSerializedLength(const swarmio::data::Variant& value, unsigned skipCount, const FieldStack& fieldStack) const
{
    if (_shortcut != nullptr)
    {
        return _shortcut->CalculateSerializedLength(value, fieldStack);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kMapValue)
    {
        return CalculateSerializedLength(value.map_value(), skipCount, fieldStack);
    }
    else
    {
        throw TypeMismatchException(
            "Unexpected source type for MessageSerializer", 
            fieldStack.GetLocation(), 
            swarmio::data::Variant::ValueCase::kMapValue, 
            value.value_case());
    }
}

uint32_t MessageSerializer::GetDefaultLength(const FieldStack& fieldStack) const
{
    uint32_t length = 0;
    for (const auto& field : _fields)
    {  
        length += field->GetDefaultLength(fieldStack);
    }
    return length;
}

uint32_t MessageSerializer::CalculateSerializedLength(const swarmio::data::Map& value, unsigned skipCount, const FieldStack& fieldStack) const
{
    uint32_t length = 0;
    auto it = _fields.begin();

    // Skip fields
    while (skipCount > 0)
    {
        ++it;
        --skipCount;
    }

    // Fill the rest of the fields
    while (it != _fields.end())
    {
        auto& field = *it;
        auto entry = value.pairs().find(field->GetName());
        if (entry != value.pairs().end())
        {
            length += field->CalculateSerializedLength(entry->second, fieldStack);
        }
        else
        {
            length += field->GetDefaultLength(fieldStack);
        }    
        ++it;
    }

    // Return length
    return length;
}

void MessageSerializer::Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    // Check for an array
    if (value.value_case() == swarmio::data::Variant::ValueCase::kMapArray)
    {
        IndexedFieldStack current(fieldStack);
        for (const auto& element : value.map_array().elements())
        {
            // Serialize each element
            Serialize(stream, element, 0, current);
            ++current;
        }
    }
    else
    {
        // Serialize as a single item
        Serialize(stream, value, 0, fieldStack);
    }
}

void MessageSerializer::Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, unsigned skipCount, const FieldStack& fieldStack) const
{
    // Check for a shortcut
    if (_shortcut != nullptr)
    {
        _shortcut->Serialize(stream, value, fieldStack);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kMapValue)
    {
        // Serialize as a map value
        Serialize(stream, value.map_value(), skipCount, fieldStack);
    }
    else
    {
        throw TypeMismatchException(
            "Unexpected source type for MessageSerializer", 
            fieldStack.GetLocation(), 
            swarmio::data::Variant::ValueCase::kMapValue, 
            value.value_case());
    }
}

void MessageSerializer::Serialize(ros::serialization::OStream& stream, const swarmio::data::Map& value, unsigned skipCount, const FieldStack& fieldStack) const
{
    const auto& pairs = value.pairs();
    auto it = _fields.begin();

    // Skip fields
    while (skipCount > 0)
    {
        ++it;
        --skipCount;
    }

    // Fill the rest of the fields
    while (it != _fields.end())
    {
        auto& field = *it;
        auto value = pairs.find(field->GetName());
        if (value != pairs.end())
        {
            field->Serialize(stream, value->second, fieldStack);
        }
        else
        {
            field->EmitDefault(stream, fieldStack); 
        }
        ++it;
    }
}

void MessageSerializer::EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const
{
    // Emit default value for each field
    for (const auto& field : _fields)
    {
        field->EmitDefault(stream, fieldStack); 
    }
}

swarmio::data::Variant MessageSerializer::DeserializeArray(ros::serialization::IStream& stream, uint32_t count, const FieldStack& fieldStack) const
{
    // Check for a shortcut
    if (_shortcut != nullptr)
    {
        return _shortcut->DeserializeArray(stream, count, fieldStack);
    }
    else
    {
        // Perform array deserialization
        IndexedFieldStack current(fieldStack);
        swarmio::data::Variant value;
        for (uint32_t i = 0; i < count; ++i)
        {
            Deserialize(stream, *value.mutable_map_array()->add_elements(), 0, current);
        }
        return value;
    }
}

swarmio::data::Variant MessageSerializer::Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const
{
    return Deserialize(stream, 0, fieldStack);
}

swarmio::data::Variant MessageSerializer::Deserialize(ros::serialization::IStream& stream, unsigned skipCount, const FieldStack& fieldStack) const 
{
    // Check for a shortcut
    if (_shortcut != nullptr)
    {
        return _shortcut->Deserialize(stream, fieldStack);
    }
    else
    {
        // Deserialize as a map
        swarmio::data::Variant value;
        Deserialize(stream, *value.mutable_map_value(), skipCount, fieldStack);
        return value;
    }
}

void MessageSerializer::Deserialize(ros::serialization::IStream& stream, swarmio::data::Map& value, unsigned skipCount, const FieldStack& fieldStack) const
{
    auto& map = *value.mutable_pairs();
    auto it = _fields.begin();

    // Skip fields
    while (skipCount > 0)
    {
        ++it;
        --skipCount;
    }

    // Fill the rest of the fields
    while (it != _fields.end())
    {
        auto& field = *it;
        map[field->GetName()] = field->Deserialize(stream, fieldStack);
        ++it;
    }
}

swarmio::data::discovery::Field MessageSerializer::GetFieldDescriptor() const
{
    if (_shortcut != nullptr)
    {
        return _shortcut->GetFieldDescriptor();
    
    }
    else
    {
        swarmio::data::discovery::Field field;
        *field.mutable_schema() = GetSchemaDescriptor(0);
        return field;
    }
}

swarmio::data::discovery::Schema MessageSerializer::GetSchemaDescriptor(unsigned skipCount) const
{
    swarmio::data::discovery::Schema schema;
    auto& map = *schema.mutable_fields();
    auto it = _fields.begin();

    // Skip fields
    while (skipCount > 0)
    {
        ++it;
        --skipCount;
    }

    // Fill the rest of the fields
    while (it != _fields.end())
    {
        auto& field = *it;
        map[field->GetName()] = field->GetFieldDescriptor();
        ++it;
    }

    // Return schema
    return schema;
}

void MessageSerializer::EnumerateFields(std::function<bool(unsigned, const Field&)> enumerator) const
{
    unsigned n = 0;
    for (auto& field : _fields)
    {
        if (!enumerator(n++, *field))
        {
            break;
        }
    }
}