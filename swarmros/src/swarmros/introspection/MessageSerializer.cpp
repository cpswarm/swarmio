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
    : Serializer(name), _package(package), _fullName(package + "/" + name), _shortcut(nullptr)
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
                        _fields.push_back(std::make_unique<ConstantField>(name, serializer, value));
                    }
                    else
                    {
                        // Perform match
                        std::smatch typeMatch;
                        if (std::regex_match(type, typeMatch, TypePattern))
                        {
                            // Fetch serializer
                            const Serializer& serializer = Serializer::SerializerForType(typeMatch[1].str(), package);

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
    for (const auto& field : _fields)
    {
        if (dynamic_cast<const ConstantField*>(field.get()) != nullptr)
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
    }

    // For the second round, we only take the dynamic fields
    for (const auto& field : _fields)
    {
        if (dynamic_cast<const ConstantField*>(field.get()) == nullptr)
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
    if (_shortcut != nullptr)
    {
        return _shortcut->CalculateSerializedLength(value, fieldStack);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kMapValue)
    {
        return CalculateSerializedLength(value.map_value(), fieldStack);
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

uint32_t MessageSerializer::CalculateSerializedLength(const swarmio::data::Map& value, const FieldStack& fieldStack) const
{
    uint32_t length = 0;
    for (const auto& field : _fields)
    {
        length += field->CalculateSerializedLength(value.pairs().at(field->GetName()), fieldStack);
    }
    return length;
}

void MessageSerializer::Serialize(ros::serialization::OStream& stream, const swarmio::data::Variant& value, const FieldStack& fieldStack) const
{
    if (_shortcut != nullptr)
    {
        _shortcut->Serialize(stream, value, fieldStack);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kMapValue)
    {
        Serialize(stream, value.map_value(), fieldStack);
    }
    else if (value.value_case() == swarmio::data::Variant::ValueCase::kMapArray)
    {
        IndexedFieldStack current(fieldStack);
        for (const auto& element : value.map_array().elements())
        {
            Serialize(stream, element, current);
            ++current;
        }
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

void MessageSerializer::Serialize(ros::serialization::OStream& stream, const swarmio::data::Map& value, const FieldStack& fieldStack) const
{
    const auto& pairs = value.pairs();
    for (const auto& field : _fields)
    {
        auto it = pairs.find(field->GetName());
        if (it != pairs.end())
        {
            field->Serialize(stream, it->second, fieldStack);
        }
        else
        {
            field->EmitDefault(stream, fieldStack); 
        }
    }
}

void MessageSerializer::EmitDefault(ros::serialization::OStream& stream, const FieldStack& fieldStack) const
{
    for (const auto& field : _fields)
    {
        field->EmitDefault(stream, fieldStack); 
    }
}

swarmio::data::Variant MessageSerializer::Deserialize(ros::serialization::IStream& stream, const FieldStack& fieldStack) const 
{
    if (_shortcut != nullptr)
    {
        return _shortcut->Deserialize(stream, fieldStack);
    }
    else
    {
        swarmio::data::Variant value;
        auto& map = *value.mutable_map_value()->mutable_pairs();
        for (const auto& field : _fields)
        {
            map[field->GetName()] = field->Deserialize(stream, fieldStack);
        }
        return value;
    }
}

swarmio::data::Variant MessageSerializer::DeserializeArray(ros::serialization::IStream& stream, uint32_t count, const FieldStack& fieldStack) const
{
    if (_shortcut != nullptr)
    {
        return _shortcut->DeserializeArray(stream, count, fieldStack);
    }
    else
    {
        IndexedFieldStack current(fieldStack);
        swarmio::data::Variant value;
        for (uint32_t i = 0; i < count; ++i)
        {
            auto& map = *value.mutable_map_array()->add_elements()->mutable_pairs();
            for (const auto& field : _fields)
            {
                map[field->GetName()] = field->Deserialize(stream, current);
                ++current;
            }
        }
        return value;
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
        *field.mutable_schema() = GetSchemaDescriptor();
        return field;
    }
}

swarmio::data::discovery::Schema MessageSerializer::GetSchemaDescriptor() const
{
    swarmio::data::discovery::Schema schema;
    for (const auto& field : _fields)
    {
        (*schema.mutable_fields())[field->GetName()] = field->GetFieldDescriptor();
    }
    return schema;
}


bool MessageSerializer::HasHeader() const
{
    if (_shortcut == nullptr && _fields.size() > 0)
    {
        return _fields.front()->GetName() == "header";
    }
    else
    {
        return false;
    }
}

const MessageSerializer& MessageSerializer::GetHeaderMessageSerializer() const
{
    if (HasHeader())
    {
        const MessageSerializer* serializer = dynamic_cast<const MessageSerializer*>(&_fields.front()->GetSerializer());
        if (serializer != nullptr)
        {
            return *serializer;
        }
        else
        {
            throw SchemaMismatchException("Invalid header field type", GetFullName());
        }
    }
    else
    {
        throw SchemaMismatchException("Message has no header", GetFullName());
    }
}

uint32_t MessageSerializer::CalculateSerializedHeaderLength(const swarmio::data::Map& value, const FieldStack& fieldStack) const
{
    if (HasHeader())
    {
        const MessageSerializer* serializer = dynamic_cast<const MessageSerializer*>(&_fields.front()->GetSerializer());
        if (serializer != nullptr)
        {
            return serializer->CalculateSerializedLength(value, fieldStack);
        }
        else
        {
            throw SchemaMismatchException("Invalid header field type", fieldStack.GetLocation());
        }
    }
    else
    {
        throw SchemaMismatchException("Message has no header", fieldStack.GetLocation());
    }
}

void MessageSerializer::SerializeHeader(ros::serialization::OStream& stream, const swarmio::data::Map& value, const FieldStack& fieldStack) const
{
    if (HasHeader())
    {
        const MessageSerializer* serializer = dynamic_cast<const MessageSerializer*>(&_fields.front()->GetSerializer());
        if (serializer != nullptr)
        {
            serializer->Serialize(stream, value, fieldStack);
        }
        else
        {
            throw SchemaMismatchException("Invalid header field type", fieldStack.GetLocation());
        }
    }
    else
    {
        throw SchemaMismatchException("Message has no header", fieldStack.GetLocation());
    }
}

swarmio::data::Map MessageSerializer::DeserializeHeader(ros::serialization::IStream& stream, const FieldStack& fieldStack) const
{
    if (HasHeader())
    {
        auto value = _fields.front()->Deserialize(stream, fieldStack);
        if (value.value_case() == swarmio::data::Variant::ValueCase::kMapValue)
        {
            return value.map_value();
        }
        else
        {
            throw SchemaMismatchException("Invalid header field type", fieldStack.GetLocation());
        }
    }
    else
    {
        throw SchemaMismatchException("Message has no header", fieldStack.GetLocation());
    }
}

uint32_t MessageSerializer::CalculateSerializedContentLength(const swarmio::data::Map& value, const FieldStack& fieldStack) const
{
    if (HasHeader())
    {
        uint32_t length = 0;
        for (auto field = ++_fields.begin(); field != _fields.end(); ++field)
        {
            length += (*field)->CalculateSerializedLength(value.pairs().at((*field)->GetName()), fieldStack);
        }
        return length;
    }
    else
    {
        throw Exception("Message has no header");
    }
}

void MessageSerializer::SerializeContent(ros::serialization::OStream& stream, const swarmio::data::Map& value, const FieldStack& fieldStack) const
{
    if (HasHeader())
    {
        const auto& pairs = value.pairs();
        for (auto field = ++_fields.begin(); field != _fields.end(); ++field)
        {
            auto it = pairs.find((*field)->GetName());
            if (it != pairs.end())
            {
                (*field)->Serialize(stream, it->second, fieldStack);
            }
            else
            {
                (*field)->EmitDefault(stream, fieldStack); 
            }
        }
    }
    else
    {
        throw Exception("Message has no header");
    }
}

swarmio::data::Map MessageSerializer::DeserializeContent(ros::serialization::IStream& stream, const FieldStack& fieldStack) const
{
    if (HasHeader())
    {
        swarmio::data::Map value;
        auto& map = *value.mutable_pairs();
        for (auto field = ++_fields.begin(); field != _fields.end(); ++field)
        {
            map[(*field)->GetName()] = (*field)->Deserialize(stream, fieldStack);
        }
        return value;
    }
    else
    {
        throw Exception("Message has no header");
    }
}

swarmio::data::discovery::Schema MessageSerializer::GetContentSchemaDescriptor() const
{
    if (HasHeader())
    {
        swarmio::data::discovery::Schema schema;
        for (auto field = ++_fields.begin(); field != _fields.end(); ++field)
        {
            (*schema.mutable_fields())[(*field)->GetName()] = (*field)->GetFieldDescriptor();
        }
        return schema;
    }
    else
    {
        throw Exception("Message has no header");
    }
}