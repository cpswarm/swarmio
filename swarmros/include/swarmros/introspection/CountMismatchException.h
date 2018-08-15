#pragma once

#include <swarmros/introspection/SchemaMismatchException.h>
#include <swarmio/data/Variant.pb.h>

namespace swarmros::introspection
{
    /**
     * @brief Exception thrown when the count of a fixed
     *        array does not match the count of the variant array.
     * 
     */
    class CountMismatchException : public SchemaMismatchException
    {
        private:

            /**
             * @brief The expected count of the variant array
             * 
             */
            uint32_t _expectedCount;

            /**
             * @brief The actual count of the variant array
             * 
             */
            uint32_t _actualCount;

        public:

            /**
             * @brief Constructor
             * 
             * @param message Message
             * @param location Location
             * @param expectedCount Expected variant array count
             * @param actualCount Actual variant array count
             */
            CountMismatchException(const char* message, const std::string& location, uint32_t expectedCount, uint32_t actualCount) 
                : SchemaMismatchException(message, location), _expectedCount(expectedCount), _actualCount(actualCount) { }       

            /**
             * @brief Get the expected count of the variant array
             * 
             * @return uint32_t
             */
            uint32_t expected_count() const
            {
                return _expectedCount;
            }                        

            /**
             * @brief Get the actual count of the variant array
             * 
             * @return uint32_t
             */
            uint32_t actual_count() const
            {
                return _actualCount;
            }
    };
}
