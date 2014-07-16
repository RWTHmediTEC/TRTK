/*
    Estimation and computation of a multivariate multidimensional polynomial.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.1 (2014-07-05)
*/

/** \file ClientServer.hpp
  * \brief This file contains classes for simple client-server-communication.
  */


#ifndef CLIENT_SERVER_HPP_4749718748
#define CLIENT_SERVER_HPP_4749718748


#include <stdexcept>
#include <utility>


namespace TRTK
{


/** \brief This exception is thrown in case of a network error. */

class NetworkError : public std::runtime_error
{
public:
    explicit NetworkError(const std::string & what);
};


/** \brief This exception is thrown in case of an unexpectedly closed client connection. */

class ConnectionClosed : public NetworkError
{
public:
    explicit ConnectionClosed(const std::string & what);
};


/** \brief Simple TCP/IP client.
  *
  * This class allows creating a simple client by just deriving from it.
  * For examples, please have a look at the detailed documentation
  * section of \ref TRTK::Server.
  *
  * \see TRTK::Server
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2014-07-05
  */

class Client
{
public:
    Client();
    virtual ~Client();

    void connect(std::string host, short port);
    void disconnect();
    std::pair<char *, size_t> request(const char * data, size_t size);

private:
    void receiveData(char * & data, size_t & size);
    void sendData(const char * data, size_t size);

    int socket_fd;
    bool connected;
};


/** \brief Simple TCP/IP server (one client at a time).
  *
  * This class allows creating a simple server by just deriving from it.
  * The server is only able to handle a single client-server-connection
  * at a time (i.e. after accepting a client connection it communicates only
  * with this particular client until this client disconnects).
  *
  * A client-request is handled by the handleRequest() function which
  * is declared \c virtual so that it can be reimplemented by the derived
  * class. It is the only way to communicate with the client. Each client
  * request is followed by the server's answer which is directly sent to
  * the client. It is assumed that the server keeps track of its state.
  *
  * In the case of a network error (transmission error, unexpectedly
  * disconnected client, etc.) an exception of the type \ref NetworkError
  * is thrown.
  *
  * \par Example:
  *
  * Server.cpp
  *
  * \code
  * #include <cstdio>
  * #include <iostream>
  * #include <string>
  *
  * #include <TRTK/ClientServer.hpp>
  *
  * const int PORT = 9734;
  *
  * class Server : public TRTK::Server
  * {
  * public:
  *     std::pair<char *, size_t> handleRequest(const char * data, size_t size)
  *     {
  *         // Put out the received data.
  *         if (size) std::cout << data << std::endl;
  *
  *         // Here the internal state can be changed or
  *         // the server can be shut down.
  *         stopServer();
  *
  *         // Send the answer.
  *         char * str = new char[14];
  *         std::strcpy(str, "Hello Client!");
  *         return std::pair<char *, size_t>(str, 14);
  *     }
  * };
  *
  * int main()
  * {
  *     try
  *     {
  *         Server server;
  *         server.startServer(PORT);
  *     }
  *     catch (std::exception & error)
  *     {
  *         std::cout << error.what() << std::endl;
  *     }
  *
  *     return 0;
  * }
  * \endcode
  *
  * Client.cpp
  *
  * \code
  * #include <iostream>
  * #include <TRTK/ClientServer.hpp>
  *
  * const char HOSTNAME[] = "127.0.0.1"; // "localhost";
  * const int PORT = 9734;
  *
  * class Client : public TRTK::Client
  * {
  * public:
  *     Client() : reply((char *) NULL, 0)
  *     {
  *     }
  *
  *     void run()
  *     {
  *         connect(HOSTNAME, PORT);
  *
  *         char message[] = "Hello server!";
  *         reply = request(&message[0], 14);
  *         if (reply.second) std::cout << reply.first << std::endl;
  *         delete[] reply.first;
  *
  *         disconnect();
  *     }
  *
  * private:
  *     std::pair<char *, size_t> reply;
  * };
  *
  * int main()
  * {
  *     try
  *     {
  *         Client client;
  *         client.run();
  *     }
  *     catch (std::exception & error)
  *     {
  *         std::cout << error.what();
  *     }
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * Server.cpp
  *
  * \verbatim Hello server! \endverbatim
  *
  * Client.cpp
  *
  * \verbatim Hello client! \endverbatim
  *
  * \see TRTK::Client
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2014-07-05
  */

class Server
{
public:
    Server();
    virtual ~Server();

    void startServer(short port);
    void stopServer();

    virtual std::pair<char *, size_t> handleRequest(const char * received_data, size_t length) = 0;

    void closeConnection();

private:
    void receiveData(char * & data, size_t & size);
    void sendData(const char * data, size_t size);

    int client_fd;
    bool close_connection;
    volatile bool stop_server;
};


} // namespace TRTK


#endif // CLIENT_SERVER_HPP_4749718748
