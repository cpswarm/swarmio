#include <swarmio/simulator/LinearPathTelemetrySimulator.h>
#include <swarmio/data/Variant.pb.h>
#include <swarmio/data/discovery/Schema.pb.h>

using namespace swarmio;
using namespace swarmio::simulator;
using namespace std::chrono_literals;

LinearPathTelemetrySimulator::LinearPathTelemetrySimulator(services::telemetry::Service& telemetryService, const std::string& name, SimulatedLocation firstPoint, SimulatedLocation secondPoint, std::chrono::seconds duration)
    : _telemetryService(telemetryService), _firstPoint(firstPoint), _secondPoint(secondPoint), _duration(duration), _shouldStop(false), _name(name)
{
    // Set schema
    data::discovery::Field field;
    auto& subfields = *field.mutable_schema()->mutable_fields();
    subfields["longitude"].set_type(data::discovery::Type::DOUBLE);
    subfields["latitude"].set_type(data::discovery::Type::DOUBLE);
    subfields["altitude"].set_type(data::discovery::Type::DOUBLE);
    telemetryService.SetFieldDefinitionForKey(name, field, false);

    // Start thread
    _thread = std::make_unique<std::thread>(&LinearPathTelemetrySimulator::Worker, this);
}

void LinearPathTelemetrySimulator::Worker()
{
    uint64_t unit = _duration.count() * 20;
    uint64_t cycle = 0;
    while(!_shouldStop)
    {
        // Calculate current stage
        double stage = (cycle % unit) / (double)unit * 2.0;
        if (stage > 1.0)
        {
            stage = 2.0 - stage;
        }

        // Build value
        data::Variant value;
        auto& pairs = *value.mutable_map_value()->mutable_pairs();
        pairs["longitude"].set_double_value(_firstPoint.GetLongitude() + (_secondPoint.GetLongitude() - _firstPoint.GetLongitude()) * stage);
        pairs["latitude"].set_double_value(_firstPoint.GetLatitude() + (_secondPoint.GetLatitude() - _firstPoint.GetLatitude()) * stage);
        pairs["altitude"].set_double_value(_firstPoint.GetAltitude() + (_secondPoint.GetAltitude() - _firstPoint.GetAltitude()) * stage);
        _telemetryService.SetValue(_name, value);

        // Next cycle
        ++cycle;
        std::this_thread::sleep_for(100ms);
    }
}