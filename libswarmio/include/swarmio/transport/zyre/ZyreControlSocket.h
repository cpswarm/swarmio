#pragma once

#include <czmq.h>

namespace swarmio::transport::zyre
{
    /**
     * @brief An inproc socket bound to a special 
     *        name generated from an object pointer.
     * 
     */
    class SWARMIO_API ZyreControlSocket final
    {
        private:

            /**
             * @brief Socket reference
             * 
             */
            zsock_t* _socket;

        public:

            /**
             * @brief Construct a new server ZyreControlSocket,
             *        where the endpoint name is determined 
             *        from the in-memory location of the object.
             */
            ZyreControlSocket();

            /**
             * @brief Construct a new client ZyreControlSocket,
             *        where the endpoint name is determined from
             *        the in-memory location of the parent.
             * 
             * @param socket 
             */
            ZyreControlSocket(const ZyreControlSocket* target);

            /**
             * @brief Get the pointer to the socket
             * 
             * @return zsock_t* 
             */
            zsock_t* GetSocket()
            {
                return _socket;
            }

            /**
             * @brief Implicit conversion operator
             * 
             * @return zsock_t* 
             */
            operator zsock_t*() const 
            {
                 return _socket;
            }

            /**
             * @brief Destroy the ZyreControlSocket object
             * 
             */
            virtual ~ZyreControlSocket();
    };
}