#pragma once

#include <swarmio/data/Variant.pb.h>
#include <swarmio/data/discovery/Type.pb.h>

namespace swarmio::services::keyvalue 
{
    /**
     * @brief Abstract base class for registered keys
     * 
     */
    class SWARMIO_API Target 
    {
        public:

            /**
             * @brief Get the current value of the target
             * 
             * @return data::Variant 
             */
            virtual data::Variant Get(const std::string& path) const = 0;

            /**
             * @brief Set the current value of the target
             * 
             * @param value New value
             */
            virtual void Set(const std::string& path, const data::Variant& value) = 0;

            /**
             * @brief Get the data type of the target
             * 
             * @return data::discovery::Type 
             */
            virtual data::discovery::Type GetType(const std::string& path) const = 0;

            /**
             * @brief Determines whether the value can be read
             * 
             * @return True if Get operations are allowed
             */
            virtual bool CanRead(const std::string& path) const noexcept
            {
                return true;
            }

            /**
             * @brief Determines whether the value can be written
             * 
             * @return True if Set operations are allowed
             */
            virtual bool CanWrite(const std::string& path) const noexcept
            {
                return false;
            }
    };
}