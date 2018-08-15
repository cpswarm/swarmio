#pragma once

#include <swarmio/Node.h>
#include <swarmio/data/event/Notification.pb.h>
#include <swarmio/data/discovery/Schema.pb.h>

namespace swarmio::services::event 
{
    /**
     * @brief Abstract base class for Event handlers
     * 
     */
    class SWARMIO_API Handler 
    {
        public:

            /**
             * @brief Handlers are notified using this method
             *        when an event has been triggered.
             * 
             * @param node Source node
             * @param event Event
             * 
             */
            virtual void EventWasTriggered(const Node* node, const data::event::Notification& event) = 0;

            /**
             * @brief Handlers may add parameter descriptors
             *        to the event descriptor when asked to do so
             *        by overriding this method.
             * 
             * @param name Event name 
             * @return data::discovery::Schema 
             */
            virtual data::discovery::Schema DescribeEvent(const std::string& name) = 0;
    };
}