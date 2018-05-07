#pragma once

#include <swarmio/Exception.h>
#include <swarmio/data/discovery/Type.pb.h>
#include <swarmio/services/event/Handler.h>
#include <map>
#include <iostream>

namespace swarmio::simulator 
{
    /**
     * @brief A special event handler that serves discovery 
     *        information and ignores the event itself.
     * 
     */
    class FauxEventHandler final : public services::event::Handler
    {
        private:

            /**
             * @brief Event name
             * 
             */
            std::string _name;

            /**
             * @brief Parameters
             * 
             */
            std::map<std::string, data::discovery::Type> _parameters;

        public:

            /**
             * @brief Create a new fake event handler
             * 
             * @param name Event name
             */
            FauxEventHandler(const std::string& name)
                : _name(name) { }

            /**
             * @brief Add a new parameter to serve for discovery
             * 
             * @param key Parameter name
             * @param type Parameter type
             */
            void AddParameter(const std::string& key, data::discovery::Type type)
            {
                _parameters[key] = type;
            }

            /**
             * @brief Handlers are notified using this method
             *        when an event has been triggered.
             * 
             * @param node Source node
             * @param event Event
             * 
             */
            virtual void EventWasTriggered(const Node* node, const data::event::Notification& event) override {  }

            /**
             * @brief Handlers may add parameter descriptors
             *        to the event descriptor when asked to do so
             *        by overriding this method.
             * 
             * @param descriptor 
             */
            virtual void DescribeEvent(const std::string& name, data::event::Descriptor& descriptor) override
            {
                if (name == _name)
                {
                    for (auto p : _parameters)
                    {
                        (*descriptor.mutable_parameters())[p.first] = p.second;
                    }
                }
                else
                {
                    throw Exception("Fake event handler queried for unknown event.");
                }
            }

            /**
             * @brief Get the name of the event
             * 
             * @return const std::string& 
             */
            const std::string& GetName() const
            {
                return _name;
            }
    };
}

