#include <swarmio/transport/zyre/ZyreControlSocket.h>
#include <swarmio/Exception.h>

using namespace swarmio;
using namespace swarmio::transport;
using namespace swarmio::transport::zyre;

ZyreControlSocket::ZyreControlSocket()
{
    // Build name
    char name[32];
    snprintf(name, sizeof(name), "@inproc://%p", this);

    // Create socket
    _socket = zsock_new_rep(name);
    if (_socket == NULL)
    {
        throw Exception("Cannot create server inproc socket");
    }
}

ZyreControlSocket::ZyreControlSocket(const ZyreControlSocket* target)
{
    // Build name
    char name[32];
    snprintf(name, sizeof(name), ">inproc://%p", target);

    // Create socket
    _socket = zsock_new_req(name);
    if (_socket == NULL)
    {
        throw Exception("Cannot create client inproc socket");
    }
}

ZyreControlSocket::~ZyreControlSocket()
{
    if (_socket != NULL)
    {
        zsock_destroy(&_socket);
    }
}