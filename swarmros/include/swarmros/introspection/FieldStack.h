#pragma once

#include <sstream>

namespace swarmros::introspection 
{
    class FieldStack
    {
        private:

            /**
             * @brief Previous entry in the field stack
             * 
             */
            const FieldStack* _previous;

        protected:

            /**
             * @brief Get the previus element in the stack
             * 
             * @return const FieldStack& 
             */
            const FieldStack* GetPrevious() const
            {
                return _previous;
            }

            /**
             * @brief Construct a child field stack
             * 
             * @param previous Previous stack element
             */
            FieldStack(const FieldStack* previous)
                : _previous(previous) { }

        public:

            /**
             * @brief Construct the current location
             * 
             * @return std::string 
             */
            virtual std::string GetLocation() const = 0;
    };
}
 