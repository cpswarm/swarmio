#pragma once

#include <swarmros/introspection/FieldStack.h>

namespace swarmros::introspection 
{
    class RootFieldStack : public FieldStack
    {
        private:

            /**
             * @brief Root name
             * 
             */
            const std::string& _name;

        public:

            /**
             * @brief Construct a root field stack
             * 
             */
            RootFieldStack(const std::string& name)
                : FieldStack(nullptr), _name(name) { }

            /**
             * @brief Construct the current location
             * 
             * @return std::string 
             */
            virtual std::string GetLocation() const
            {
                return _name;
            }
    };
}
 