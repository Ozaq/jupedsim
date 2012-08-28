/**
 *
 * Copyright (C) <2009-2010>  <Ulrich Kemloh>
 *
 * @section LICENSE
 * This file is part of JuPedSim.
 *
 * JuPedSim is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * JuPedSim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with JuPedSim. If not, see <http://www.gnu.org/licenses/>.
 *
 * @section DESCRIPTION
 *
 *
 *
 */

#ifndef TRAVISTOCLIENT_H_
#define TRAVISTOCLIENT_H_

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#ifdef _WIN32

#endif

#define QUEUE_LENGTH       5        ///< max queue length of pending connections

/******** macro definitions ******************************************/

#ifdef TRACE_LOGGING

#define dtrace(...)   _printDebugLine(__FILE__, __LINE__, false, __VA_ARGS__)
#define derror(...)   _printDebugLine(__FILE__, __LINE__, true, __VA_ARGS__)

#else

#define dtrace(...)   ((void) 0)
#define derror(...)   (fprintf(stderr, __VA_ARGS__), _printErrorMessage())

#endif /* TRACE_LOGGING */

#ifndef _WIN32
//#define closesocket          close
#define INVALID_SOCKET       (-1)
#define SOCKET_ERROR         (-1)
#endif




#ifdef _WIN32
#include <winsock.h>
#define WS_MAJOR_VERSION   1        ///< major version of Winsock API
#define WS_MINOR_VERSION   1        ///< minor version of Winsock API
#define SHUT_RDWR          SD_BOTH  ///< @c SHUT_RDWR is POSIX standard
typedef SOCKET socket_t;
typedef int socklen_t;
#define startSocketSession() _startWin32SocketSession()
#define stopSocketSession()  _stopWin32SocketSession()

//bool _startWin32SocketSession(void);
//void _stopWin32SocketSession(void);

#else
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>
typedef int socket_t;
#define u_int_32_t unsigned int
#define startSocketSession() (true)
#define stopSocketSession()  ((void) 0)
#endif


#ifndef IPPORT_USERRESERVED
#define IPPORT_USERRESERVED  (5000)  ///< up to this number, ports are reserved and should not be used
#endif

/******** public constants *******************************************/

#define PORT 8989
#define HOST "localhost"

class TraVisToClient {
public:
    /// create a client with the default parameters

    /// create a client with specific parameters
    TraVisToClient(const char* hostname = HOST, unsigned short port = PORT);

    /// destructor
    virtual ~TraVisToClient();

    /// send datablock to the server
    /// this functions is still blocking unfortunalty, so it may
    /// influence the execution time of your program
    void sendData(const char* data);

    /// close the client (end the connection)
    void close();

    /// send a datagram using the unreliable
    /// udp protokoll
    void sendDatagram(char *datagram);

private:
    void createConnection();

    unsigned long lookupHostAddress(const char *hostName);

    socket_t createClientSocket(const char *serverName, unsigned short portNumber);

    socket_t createServerSocket(unsigned short portNumber);

    bool shutdownAndCloseSocket(socket_t sock);

    bool sendMessage(socket_t sock, const void *msg, int msgSize);

    bool receiveMessage(socket_t sock, void *msg, int msgSize);

    void _printErrorMessage(void);

#ifdef _WIN32
    bool _startWin32SocketSession(void);
    void _stopWin32SocketSession(void);
#else
#define closesocket          close
#endif



private:
    bool isConnected;
    //bool dataPending;
    socket_t tcpSocket;
    //FIXME: this could be a security issue. the lenght shuolnd be limited
    char hostname[50];
    unsigned short port;
    std::vector<const char *> msgQueue;

};

#endif /* TRAVISTOCLIENT_H_ */