#pragma once

#include <swarmio/Mailbox.h>
#include <thread>

namespace swarmio::services
{
    /**
     * @brief A service that uses a periodic background
     *        worker to perform its work.
     * 
     */
    class PeriodicService : public Mailbox
    {
        private:

            /**
             * @brief Worker thread
             * 
             */
            std::thread* _worker = nullptr;

            /**
             * @brief Worker thread shutdown requested
             * 
             */
            std::atomic<bool> _shutdownRequested;

            /**
             * @brief Tick period
             * 
             */
            std::chrono::milliseconds _period;

            /**
             * @brief Worker thread entry point
             * 
             */
            void Worker();

        protected:

            /**
             * @brief Construct a new Periodic Service object
             * 
             * @param endpoint Endpoint
             * @param period Background worker period
             */
            PeriodicService(Endpoint* endpoint, std::chrono::milliseconds period)
                : Mailbox(endpoint), _shutdownRequested(false), _period(period), _worker(nullptr) { }

            /**
             * @brief Called when the mailbox is attached to an already running
             *        endpoint or if the attached endpoint has just started.
             * 
             */
            virtual void MailboxWasConnected() noexcept override;

            /**
             * @brief Called right before the mailbox is disconnected from its
             *        endpoint or if the attached endpoint is about to stop.
             * 
             */
            virtual void MailboxWillBeDisconnected() noexcept override;

            /**
             * @brief Called periodically to perform work
             * 
             */
            virtual void Update() = 0;
    };
}