#pragma once

#include <swarmio/services/keyvalue/Target.h>
#include <swarmio/data/Variant.pb.h>
#include <swarmio/data/discovery/Type.pb.h>

namespace swarmio::simulator 
{
    /**
     * @brief A strongly typed in-memory parameter
     *        target that can be made read-only.
     * 
     */
    class InMemoryParameter : public services::keyvalue::Target
    {
        private:

            /**
             * @brief Value
             * 
             */
            data::Variant _value;

            /**
             * @brief Type
             * 
             */
            data::discovery::Type _type;

            /**
             * @brief Is read-only?
             * 
             */
            bool _readOnly;

            /**
             * @brief Resource path
             * 
             */
            std::string _path;

        public:
            
            /**
             * @brief Create a new boolean in-memory parameter
             * 
             * @param value Value
             * @param readOnly Is read-only?
             */
            InMemoryParameter(const std::string& path, bool value, bool readOnly = false)
                : _readOnly(readOnly), _path(path)
            {
                _type = data::discovery::Type::BOOL;
                _value.set_bool_value(value);
            }

            /**
             * @brief Create a new string in-memory parameter
             * 
             * @param value Value
             * @param readOnly Is read-only?
             */
            InMemoryParameter(const std::string& path, const std::string& value, bool readOnly = false)
                : _readOnly(readOnly), _path(path)
            {
                _type = data::discovery::Type::STRING;
                _value.set_string_value(value);
            }

            /**
             * @brief Create a new string in-memory parameter
             * 
             * @param value Value
             * @param readOnly Is read-only?
             */
            InMemoryParameter(const std::string& path, const char* value, bool readOnly = false)
                : _readOnly(readOnly), _path(path)
            {
                _type = data::discovery::Type::STRING;
                _value.set_string_value(value);
            }

            /**
             * @brief Create a new integer in-memory parameter
             * 
             * @param value Value
             * @param readOnly Is read-only?
             */
            InMemoryParameter(const std::string& path, int value, bool readOnly = false)
                : _readOnly(readOnly), _path(path)
            {
                _type = data::discovery::Type::INT;
                _value.set_int_value(value);
            }

            /**
             * @brief Create a new integer in-memory parameter
             * 
             * @param value Value
             * @param readOnly Is read-only?
             */
            InMemoryParameter(const std::string& path, double value, bool readOnly = false)
                : _readOnly(readOnly), _path(path)
            {
                _type = data::discovery::Type::DOUBLE;
                _value.set_double_value(value);
            }

            /**
             * @brief Get the current value of the target
             * 
             * @return data::Variant 
             */
            virtual data::Variant Get(const std::string& path) const 
            {
                if (path == _path)
                {
                    return _value;
                }
                else
                {
                    throw Exception("In memory parameter queried for unknown resource path.");
                }
            }

            /**
             * @brief Set the current value of the target
             * 
             * @param value New value
             */
            virtual void Set(const std::string& path, const data::Variant& value)
            {
                if (path == _path)
                {    
                    if (_readOnly)
                    {
                        throw Exception("Parameter is read-only.");
                    }                   
                    else
                    {
                        switch (value.value_case())
                        {
                            case data::Variant::ValueCase::kBoolValue:
                                if (_type == data::discovery::Type::BOOL)
                                {
                                    _value = value;
                                }
                                else
                                {
                                    throw Exception("Invalid type for parameter.");
                                }
                                break;

                            case data::Variant::ValueCase::kDoubleValue:
                                if (_type == data::discovery::Type::DOUBLE)
                                {
                                    _value = value;
                                }
                                else
                                {
                                    throw Exception("Invalid type for parameter.");
                                }
                                break;

                            case data::Variant::ValueCase::kIntValue:
                                if (_type == data::discovery::Type::INT)
                                {
                                    _value = value;
                                }
                                else
                                {
                                    throw Exception("Invalid type for parameter.");
                                }
                                break;

                            case data::Variant::ValueCase::kStringValue:
                                if (_type == data::discovery::Type::STRING)
                                {
                                    _value = value;
                                }
                                else
                                {
                                    throw Exception("Invalid type for parameter.");
                                }
                                break;

                            default:
                                throw Exception("Unknown type for parameter.");
                        }  
                    }
                    
                }
                else
                {
                    throw Exception("In memory parameter queried for unknown resource path.");
                }
            }

            /**
             * @brief Get the data type of the target
             * 
             * @return data::discovery::Type 
             */
            virtual data::discovery::Type GetType(const std::string& path) const override
            {
                if (path == _path)
                {    
                    return _type;  
                }
                else
                {
                    throw Exception("In memory parameter queried for unknown resource path.");
                }
            }

            /**
             * @brief Determines whether the value can be written
             * 
             * @return True if Set operations are allowed
             */
            virtual bool CanWrite(const std::string& path) const noexcept override
            {
                if (path == _path)
                {    
                    return !_readOnly;  
                }
                else
                {
                    throw Exception("In memory parameter queried for unknown resource path.");
                }
            }

            /**
             * @brief Determines whether the value can be read
             * 
             * @return True if Get operations are allowed
             */
            virtual bool CanRead(const std::string& path) const noexcept override
            {
                if (path == _path)
                {    
                    return true;
                }
                else
                {
                    throw Exception("In memory parameter queried for unknown resource path.");
                }
            }

            /**
             * @brief Get the resource path of the parameter
             * 
             * @return const std::string& 
             */
            const std::string& GetPath() const
            {
                return _path;
            }
    };
}

