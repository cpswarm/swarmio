#pragma once

#include <swarmros/introspection/FieldStack.h>

namespace swarmros::introspection 
{
    class KeyedFieldStack : public FieldStack
    {
        private:

            /**
             * @brief Key
             * 
             */
            const std::string& _key;

        public:

            /**
             * @brief Constructor
             * 
             * @param previous Previous stack element
             * @param key Key
             */
            KeyedFieldStack(const FieldStack& previous, const std::string& key)
                : FieldStack(&previous), _key(key) { }

            /**
             * @brief Construct the current location
             * 
             * @return std::string 
             */
            virtual std::string GetLocation() const override
            {
                return GetPrevious()->GetLocation() + "." + _key;
            }
    };
}
 