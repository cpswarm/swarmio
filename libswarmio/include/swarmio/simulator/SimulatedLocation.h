#pragma once

namespace swarmio::simulator 
{
    /**
     * @brief A location described by a latitude, a longitude
     *        and an altitude.
     * 
     */
    class SimulatedLocation final
    {
        private:

            /**
             * @brief Longitude
             * 
             */
            double _longitude;

            /**
             * @brief Latitude
             * 
             */
            double _latitude;

            /**
             * @brief Altitude
             * 
             */
            double _altitude;

        public:

            /**
             * @brief Construct a new Simulated Location object
             * 
             * @param longitude 
             * @param latitude 
             * @param altitude 
             */
            SimulatedLocation(double longitude, double latitude, double altitude)
                : _longitude(longitude), _latitude(latitude), _altitude(altitude) { }

            /**
             * @brief Get longitude
             * 
             * @return double 
             */
            double GetLongitude()
            {
                return _longitude;
            }

            /**
             * @brief Get latitude
             * 
             * @return double 
             */
            double GetLatitude()
            {
                return _latitude;
            }

            /**
             * @brief Get altitude
             * 
             * @return double 
             */
            double GetAltitude()
            {
                return _altitude;
            }
    };
}