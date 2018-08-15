#include <swarmio/services/PeriodicService.h>

using namespace swarmio;
using namespace swarmio::services;

void PeriodicService::Worker()
{
    // Create clock
    std::chrono::high_resolution_clock timer;
    auto next = timer.now();

    // Go
    for (;;)
    {
        // Check if we need to stop
        if (_shutdownRequested)
        {
            break;
        }

        // Perform update
        Update();

        // Wait until the next iteration
        next += _period;
        std::this_thread::sleep_until(next);
    }
}

void PeriodicService::MailboxWasConnected() noexcept
{
    // Start worker thread
    _worker = new std::thread(&PeriodicService::Worker, this);
}

void PeriodicService::MailboxWillBeDisconnected() noexcept
{
    // Request shutdown
    _shutdownRequested = true;

    // Wait for worker thread
    _worker->join();

    // Delete worker thread
    delete _worker;
    _worker = nullptr;
}