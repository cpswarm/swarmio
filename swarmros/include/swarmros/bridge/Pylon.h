#pragma once

namespace swarmros::bridge
{
    /**
     * @brief Base class for all bridging services
     * 
     */
    class Pylon
    {
        protected:

            /**
             * @brief Mark as abstract
             * 
             */
            Pylon() { }

        public:

            /**
             * @brief Add virtual destructor
             * 
             */
            virtual ~Pylon() { }
    };
}

