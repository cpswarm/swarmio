#pragma once

#include <swarmio/Mailbox.h>
#include <swarmio/Exception.h>
#include <mutex>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <exception>

namespace swarmio::services 
{
    /**
     * @brief A special mailbox that handles responses to a message.
     * 
     * @tparam T Return value type
     */
    template <class T>
    class Awaiter : public Mailbox 
    {
        private:

            /**
             * @brief Request identifier
             * 
             */
            uint64_t _requestIdentifier;

            /**
             * @brief The value of the response
             * 
             */
            T _response;

            /**
             * @brief Set to true after the initial 
             *        response has been received
             * 
             */
            bool _valid;

            /**
             * @brief The exception that was thrown
             *        while processing the response.
             * 
             */
            std::exception_ptr _exception = nullptr;

            /**
             * @brief Mutex for the condition variable
             * 
             */
            std::mutex _mutex;

            /**
             * @brief Condition variable to signal 
             *        when the response arrives
             * 
             */
            std::condition_variable _conditionVariable;

        protected:

            /**
             * @brief Called when a response for the original message has been received.
             * 
             * @param node Sender node
             * @param message Message
             */
            virtual T ExtractResponse(const Node* node, const data::Message* message) = 0;

            /**
             * @brief Check whether the last message has been received
             * 
             * @return True if no further messages should be processed
             */
            virtual bool IsFinished()
            {
                return _valid;
            }

        public:

            /**
             * @brief Construct a new ResponseAwaiter object
             * 
             * @param endpoint Endpoint
             * @param requestIdentifier Original message identifier
             */
            Awaiter(Endpoint* endpoint, uint64_t requestIdentifier)
                : Mailbox(endpoint), _requestIdentifier(requestIdentifier), _valid(false) { }

            /**
             * @brief Construct an awaiter with a cached value
             * 
             * @param value Value
             */
            Awaiter(const T& value)
                : Mailbox(), _requestIdentifier(0), _valid(true), _response(value) { }

            /**
             * @brief Move an Awaiter object
             * 
             * @param other Other object
             */
            Awaiter(Awaiter&& other)
                : Mailbox(std::move(other))
            {
                // Lock
                std::unique_lock<std::mutex> guard(other._mutex);

                // Pass event handling to this instance
                other.FinishMovingTo(this);

                // Copy over values
                _valid = other._valid;
                _response = other._response;
                _requestIdentifier = other._requestIdentifier;
            }

            /**
             * @brief Get the response value. Will throw an exception 
             *        if called before the response is received.
             *
             * @return const T& 
             */
            const T& GetResponse()
            {
                std::unique_lock<std::mutex> guard(_mutex);
                if (_valid)
                {
                    if (_exception)
                    {
                        std::rethrow_exception(_exception);
                    }
                    else
                    {
                        return _response;
                    }
                }
                else
                {
                    throw Exception("Response is not yet available");
                }
            }

            /**
             * @brief Delivery point of all messages
             * 
             * @param sender The node that has sent the message
             * @param message The message itself
             * @returns True if the message had been processed and should 
             *          not be forwarded to other mailboxes
             */
            virtual bool ReceiveMessage(const Node* sender, const data::Message* message) override
            {
                // Check if we still need the response and then check the reply_to field
                if (!IsFinished() &&
                    message->header().reply_to() == _requestIdentifier)
                {
                    // Acquire lock
                    std::unique_lock<std::mutex> guard(_mutex);

                    // Extract the response
                    try
                    {
                        _response = ExtractResponse(sender, message);
                        _exception = nullptr;
                    }
                    catch (...)
                    {
                        _exception = std::current_exception();
                    }

                    // Mark as valid and notify all clients
                    _valid = true;
                    guard.unlock();
                    _conditionVariable.notify_all();

                    // Handled
                    return true;
                }
                else
                {
                    // Not handled
                    return false;
                }
            }

            /**
             * @brief Wait for the response to become available 
             *        or util the timeout period ellapses.
             * 
             * @param timeout Time to wait
             * @return True if the response has been received.
             */
            bool WaitForResponse(const std::chrono::milliseconds& timeout)
            {
                // Acquire lock
                std::unique_lock<std::mutex> guard(_mutex);

                // Wait
                return _conditionVariable.wait_for(guard, timeout, [this]{ return _valid; });
            }

            /**
             * @brief Checks whether the result of the 
             *        processing has ended with an exception.
             * 
             * @return True if an exception will be thrown by GetResponse().
             */
            bool HasException()
            {
                std::unique_lock<std::mutex> guard(_mutex);
                if (_valid)
                {
                    return (bool)_exception;
                }
                else
                {
                    throw Exception("Response is not yet available");
                }
            }

            /**
             * @brief Get request identifier
             * 
             * @return uint64_t Identifier
             */
            uint64_t GetIdentifier() const
            {
                return _requestIdentifier;
            }
    };
}
