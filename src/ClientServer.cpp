/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.1 (2014-07-05)
*/

/** \file Timestamp.cpp
  * \brief This file contains the definitions of the classes
  *        for simple client-server-communication.
  */

#include <cassert>
#include <cstddef>
#include <iostream>

#include "TRTK/ClientServer.hpp"


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    // WINDOWS
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib") // link with Ws2_32.lib
#else
    // Linux
    #include <arpa/inet.h>
    #include <netinet/in.h>
    #include <sys/types.h>
    #include <sys/socket.h> // Sometimes sys/socket.h doesn't include sys/types.h, so don't change the order!
    #include <unistd.h>
#endif


namespace TRTK
{


//////////////////////////////////////////////////////////////////////////////
//                               Exceptions                                 //
//////////////////////////////////////////////////////////////////////////////

NetworkError::NetworkError(const std::string & what) : runtime_error(what)
{
}


ConnectionClosed::ConnectionClosed(const std::string & what) : NetworkError(what)
{
}


//////////////////////////////////////////////////////////////////////////////
//                             Helper Functions                             //
//////////////////////////////////////////////////////////////////////////////

enum {DATA_RECEIVED = 1};


/** \brief Receives a block of data from the given socket.
  *
  * In case of an aborted connection the receiving process is resumed.
  * The returned block of data must be deleted by the user!
  */

void receive_all(int socket_fd, char * & data, size_t & size)
{
    // Each block of data is preceded by a 16 bit word denoting the
    // size of the subsequent block of data.

    char size_LSB = 0; // least significant byte is received first
    char size_MSB = 0; // most significant byte is received second

    int error_code = recv(socket_fd, &size_LSB, 1, 0);

    if (error_code == -1)
    {
        throw NetworkError("Error while receiving data.");
    }
    else if(error_code == 0)
    {
        throw ConnectionClosed("Error: Connection closed."); // EOF can be excluded
    }

    error_code = recv(socket_fd, &size_MSB, 1, 0);

    if (error_code == -1)
    {
        throw NetworkError("Error while receiving data.");
    }
    else if(error_code == 0)
    {
        throw ConnectionClosed("Error: Connection closed."); // EOF can be excluded
    }

    size = ((unsigned short)(reinterpret_cast<unsigned char &>(size_MSB)) << 8) +
            (unsigned short)(reinterpret_cast<unsigned char &>(size_LSB));

    // A transmission can be accidentally disrupted (e.g. due to a kernel
    // interrupt). In that case recv() has to be called several times until
    // the remaining chunks of data are received.

    int bytes_left = size;
    data = new char[size];

    while (bytes_left > 0)
    {
        int received_bytes = recv(socket_fd, &data[size - bytes_left], bytes_left, 0);

        if (received_bytes == 0)
        {
            delete[] data;
            throw ConnectionClosed("Error: Connection closed."); // EOF can be excluded since we do not read more bytes than sent
        }

        if (received_bytes == -1)
        {
            delete[] data;
            throw NetworkError("Error while receiving data.");
        }

        bytes_left -= received_bytes;
    }

    // Acknowledge the reception of data.
    // See send_all() for more details why this step is needed.

    char transmission_confirmation = DATA_RECEIVED;

    if (send(socket_fd, &transmission_confirmation, 1, 0) == -1)
    {
        delete[] data;
        throw NetworkError("Error while sending a transmission confirmation.");
    }
}


/** \brief Sends a block of data over the given socket.
  *
  * In case of an aborted connection the sending process is resumed.
  * At most 65536 bytes can be sent, otherwise sending must be
  * serialized.
  */

void send_all(int socket_fd, const char * data, size_t size)
{
    assert(size >= 0 && size < 65536); // 65536 = 2^16
    if (size > 65536) throw NetworkError("Error: Maximum allowed data size is 65536 bytes.");

    // Each block of data is preceded by a 16 bit word denoting the
    // size of the subsequent block of data.

    char size_LSB = size & 0xFF; // least significant byte is send first
    char size_MSB = (size >> 8) & 0xFF; // most significant byte is send second

    if (send(socket_fd, &size_LSB, 1, 0) == -1 || send(socket_fd, &size_MSB, 1, 0) == -1)
    {
        throw NetworkError("Error while sending data.");
    }

    // A transmission can be accidentally disrupted (e.g. due to a kernel
    // interrupt). In that case send() has to be invoked several times
    // until the remaining chunks of data are fully transmitted.

    int bytes_left = size;

    while (bytes_left > 0)
    {
        int sent_bytes = send(socket_fd, &data[size - bytes_left], bytes_left, 0);
        if (sent_bytes == -1) throw NetworkError("Error while sending data.");
        bytes_left -= sent_bytes;
    }

    // Receive the transmission confirmation.

    // From [http://bert-hubert.blogspot.de/]:
    //
    // > It turns out that in this case, section 4.2.2.13 of RFC 1122 tells us that a
    // > close() with any pending readable data could lead to an immediate reset being sent.
    // >
    // > "A host MAY implement a 'half-duplex' TCP close sequence, so that an application
    // > that has called CLOSE cannot continue to read data from the connection. If such a
    // > host issues a CLOSE call while received data is still pending in TCP, or if new
    // > data is received after CLOSE is called, its TCP SHOULD send a RST to show that
    // > data was lost."
    //
    // Solution (should always work regardless of the TCP stack implementation):
    //
    // To avoid a potential data loss, a sender can send length information and the receiver
    // actively acknowledges that all data was received.

    char transmission_confirmation = 0;

    int error_code = recv(socket_fd, &transmission_confirmation, 1, 0);

    if (error_code == -1)
    {
        throw NetworkError("Error while getting a transmission confirmation.");
    }
    else if (error_code == 0)
    {
        throw ConnectionClosed("Error: Connection closed."); // EOF can be excluded
    }

    if (transmission_confirmation != DATA_RECEIVED)
    {
        throw NetworkError("Transmission failed.");
    }
}


//////////////////////////////////////////////////////////////////////////////
//                                 Client                                   //
//////////////////////////////////////////////////////////////////////////////

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    static int num_winsock_instances = 0;
#endif


/** \brief Constructs a Client object. */

Client::Client() : connected(false)
{
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        // Initialize Winsock

        WSADATA wsaData = {0};
        int retval = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (retval != 0)
        {
            throw(NetworkError("Error: WSAStartup failed."));
        }

        ++num_winsock_instances;
    #endif
}


/** \brief Destroys the Client object. */

Client::~Client()
{
    if (connected)
    {
        disconnect();
    }

    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        if (num_winsock_instances > 0)
        {
            WSACleanup();
            --num_winsock_instances;
        }
    #endif
}


/** \brief Connects to a TCP/IP server.
  *
  * \param [in] host    Must be given in dot-decimal notation.
  * \param [in] port    Port on which the server is running.
  */

void Client::connect(std::string host, short port)
{
    // Set up a socket.

    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_fd == -1)
    {
        throw(NetworkError("Error while setting up the stream socket."));
    }

    // Assign a name to the socket (binding).

    sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(host.c_str());
    address.sin_port = htons(port);

    socklen_t sockaddr_in_size = sizeof(sockaddr_in);

    if (::connect(socket_fd, reinterpret_cast<sockaddr *>(&address), sockaddr_in_size) == -1)
    {
        throw NetworkError("Error while establishing a connection.");
    }

    connected = true;
}


/** \brief Closes the current connection to the TCP/IP server. */

void Client::disconnect()
{
    if (connected)
    {
        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
            closesocket(socket_fd);
        #else
            close(socket_fd);
        #endif

        connected = false;
    }
}


/** \brief Receives a block of data.
  *
  * At most 65536 bytes can be received, otherwise the receiving must be
  * serialized. After having invoked this function \p data points to the
  * received block of data and \p size denotes its size. \p data must be
  * deleted by the user.
  */

void Client::receiveData(char * & data, size_t & size)
{
    receive_all(socket_fd, data, size);
}


/** \brief Carries out a server request.
  *
  * \param [in]  data       Data sent to the server.
  * \param [in]  size       Size of the block of data.
  *
  * This function is used to carry out server requests. The server
  * response is returned in the form of a \c std::pair consisting
  * of a pointer to the block of data (on the heap) and its size.
  * The memory must be freed by the client using \c delete[].
  *
  * Note, at most 65536 bytes can be sent, otherwise sending must
  * be serialized.
  */

std::pair<char *, size_t> Client::request(const char * data, size_t size)
{
    char * received_data = NULL;
    size_t received_data_size = 0;

    sendData(data, size);
    receiveData(received_data, received_data_size);

    return std::pair<char *, size_t>(received_data, received_data_size);
}


/** \brief Sends a given block of data to the server.
  *
  * At most 65536 bytes can be sent, otherwise sending must be serialized.
  */

void Client::sendData(const char * data, size_t size)
{
    send_all(socket_fd, data, size);
}


//////////////////////////////////////////////////////////////////////////////
//                                 Server                                   //
//////////////////////////////////////////////////////////////////////////////

/** \brief Constructs a Server object. */

Server::Server() : close_connection(false), stop_server(false)
{
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        // Initialize Winsock

        WSADATA wsaData = {0};
        int retval = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (retval != 0)
        {
            throw(NetworkError("Error: WSAStartup failed."));
        }

        ++num_winsock_instances;
    #endif
}


/** \brief Destroys a Server object. */

Server::~Server()
{
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        if (num_winsock_instances > 0)
        {
            WSACleanup();
            --num_winsock_instances;
        }
    #endif
}


/** \brief Notify the server about the client's intention to close the connection.
  *
  * Calling this function leads to the termination of the internal event loop
  * regarding incoming client messages. This function could be called from the
  * derived class (see \ref handleRequest()) if the client-server-dialog ends
  * intentionally.
  */

void Server::closeConnection()
{
    close_connection = true;
}


/** \fn std::pair<char *, size_t> Server::handleRequest(const char * received_data, size_t length)
  *
  * \brief Handles incoming requests.
  *
  * This function is called, each time a client sends data to the server.
  * The server responds by returning some data in the form of a \c std::pair
  * consisting of a pointer to the block of data (on the heap) and its size.
  * After having sent the data to the client the memory is freed using
  * \c delete[].
  */


/** \brief Receives a block of data.
  *
  * At most 65536 bytes can be received, otherwise the receiving must be
  * serialized. After having invoked this function \p data points to the
  * received block of data and \p size denotes its size. \p data must be
  * deleted by the user.
  */

void Server::receiveData(char * & data, size_t & size)
{
    receive_all(client_fd, data, size);
}


/** \brief Sends a given block of data to the client.
  *
  * At most 65536 bytes can be sent, otherwise sending must be serialized.
  */

void Server::sendData(const char * data, size_t size)
{
    send_all(client_fd, data, size);
}


/** \brief Starts the server.
  *
  * The server will listen on port \p port and run in an infinite
  * loop. If a client connects to the server the \c virtual method
  * handleRequest() is called and the received data is passed on.
  * The data returned by this method is send to the client.
  */

void Server::startServer(short port)
{
    // Set up a socket.

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (server_fd == -1)
    {
        throw(NetworkError("Error while setting up the stream socket."));
    }

    // Assign a name to the socket (binding).

    sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);

    socklen_t sockaddr_in_size = sizeof(sockaddr_in);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sockaddr_in_size) == -1)
    {
        throw NetworkError("Error while binding the stream socket.");
    }

    while (!stop_server)
    {
        // Listen to a client.

        if (listen(server_fd, 5) == -1)
        {
            throw NetworkError("Error while listening for connections.");
        }

        // Accecpt a connection.

        sockaddr_in client_address;
        client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_address), &sockaddr_in_size);

        if (client_fd == -1)
        {
            throw NetworkError("Error while accepting the connection.");
        }

        close_connection = false;

        try
        {
            while (!close_connection)
            {
                // Read the data/request.

                char * data = NULL;
                size_t data_size = 0;

                try
                {
                    receiveData(data, data_size);
                }
                catch (ConnectionClosed)
                {
                    delete[] data;
                    break;
                }

                // Hand the data on to the derived class.

                std::pair<char *, size_t> reply = handleRequest(data, data_size);

                // Send the server's answer to the client.

                try
                {
                    sendData(reply.first, reply.second);
                }
                catch (ConnectionClosed)
                {
                    delete[] reply.first;
                    delete[] data;
                    break;
                }
                catch(...)
                {
                    delete[] reply.first;
                    delete[] data;
                    throw;
                }

                // Clean up.

                delete[] reply.first;
                delete[] data;
            }
        }
        catch (NetworkError)
        {
            // There seems to be a connection problem so close the
            // current connection and listen to new clients.

            std::cerr << "Server error: Client connection died..." << std::endl;
        }

        // Close the connection.

        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
            closesocket(client_fd);
        #else
            close(client_fd);
        #endif
    }

    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        closesocket(server_fd);
    #else
        close(server_fd);
    #endif

    stop_server = false;
}


/** \brief Stops the server. */

void Server::stopServer()
{
    stop_server = true;
}


} // namespace TRTK
