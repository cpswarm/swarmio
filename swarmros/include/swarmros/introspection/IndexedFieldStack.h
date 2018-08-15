#pragma once

#include <swarmros/introspection/FieldStack.h>

namespace swarmros::introspection 
{
    class IndexedFieldStack : public FieldStack
    {
        private:

            /**
             * @brief Current index
             * 
             */
            uint64_t _index;

        public:

            /**
             * @brief Constructor
             * 
             * @param previous Previous stack element
             */
            IndexedFieldStack(const FieldStack& previous)
                : FieldStack(&previous), _index(0) { }

            /**
             * @brief Operator overload to increment current index
             * 
             * @return IndexedFieldStack& 
             */
            IndexedFieldStack& operator++()  
            {  
                ++_index;
                return *this;  
            }  

            /**
             * @brief Construct the current location
             * 
             * @return std::string 
             */
            virtual std::string GetLocation() const override
            {
                return GetPrevious()->GetLocation() + "[" + std::to_string(_index) + "]";
            }
    };
}
 