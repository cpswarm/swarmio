#pragma once

#include <swarmio/simulator/SimulatedLocation.h>
#include <swarmio/services/telemetry/Service.h>
#include <chrono>
#include <memory>
#include <thread>

namespace swarmio::simulator 
{
    /**
     * @brief Simulates linear back-and-forth movement
     *        between two points.
     * 
     */
    class LinearPathTelemetrySimulator final
    {
        private:

            /**
             * @brief Worker thread
             * 
             */
            std::unique_ptr<std::thread> _thread;

            /**
             * @brief Stop marker
             * 
             */
            std::atomic<bool> _shouldStop;

            /**
             * @brief Telemetry service
             * 
             */
            services::telemetry::Service& _telemetryService;

            /**
             * @brief Telemetry key
             * 
             */
            std::string _name;

            /**
             * @brief First point
             * 
             */
            SimulatedLocation _firstPoint;

            /**
             * @brief Second point
             * 
             */
            SimulatedLocation _secondPoint;

            /**
             * @brief Time it takes to complete one trip
             * 
             */
            std::chrono::seconds _duration;

            /**
             * @brief Worker thread
             * 
             */
            void Worker();

        public:

            /**
             * @brief Constructor
             * 
             * @param telemetryService Telemetry service
             * @param name Telemetry key
             * @param firstPoint First point
             * @param secondPoint Second point
             * @param duration Time it takes to complete one trip
             */
            LinearPathTelemetrySimulator(services::telemetry::Service& telemetryService, const std::string& name, SimulatedLocation firstPoint, SimulatedLocation secondPoint, std::chrono::seconds duration);

            /**
             * @brief Stops updating
             * 
             */
            void Stop()
            {
                if (_thread)
                {
                    _shouldStop = true;
                    _thread->join();
                    _thread.reset();
                }
            }

            /**
             * @brief Destructor
             * 
             */
            ~LinearPathTelemetrySimulator()
            {
                Stop();
                _telemetryService.RemoveFieldDefinitionForKey(_name);
                _telemetryService.RemoveValue(_name);
            }
    };
}