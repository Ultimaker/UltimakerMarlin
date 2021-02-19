#include <avr/io.h>
#include <string.h>

#include "serial.h"
#include "../frontend/frontend.h"

#ifdef __WIN32__
#include <windows.h>
#define socketCloseFunction closesocket
#else
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#define socketCloseFunction close
#include <string.h>
#endif
#define NO_SOCKET 0xffffffff

extern void USART0_RX_vect();
extern void USART1_RX_vect();
extern void USART2_RX_vect();
extern void USART3_RX_vect();

SerialSim::SerialSim(int port_nr)
{
    this->port_nr = port_nr;
    interrupt = NULL;
    switch(port_nr)
    {
    case 0:
        UCSRxA = &UCSR0A;
        UDRx = &UDR0;
        //interrupt = USART0_RX_vect;
        break;
    case 1:
        UCSRxA = &UCSR1A;
        UDRx = &UDR1;
        //interrupt = USART1_RX_vect;
        break;
    case 2:
        UCSRxA = &UCSR2A;
        UDRx = &UDR2;
        interrupt = USART2_RX_vect;
        break;
    case 3:
        UCSRxA = &UCSR3A;
        UDRx = &UDR3;
        //interrupt = USART3_RX_vect;
        break;
    }

    UCSRxA->setCallback(DELEGATE(registerDelegate, SerialSim, *this, UART_UCSRxA_callback));
    UDRx->setCallback(DELEGATE(registerDelegate, SerialSim, *this, UART_UDRx_callback));
    *UCSRxA = _BV(UDRE0);
    
    recvLine = 0;
    recvPos = 0;
    memset(recvBuffer, '\0', sizeof(recvBuffer));
    
    listen_socket = NO_SOCKET;
    connected_socket = NO_SOCKET;
}

bool SerialSim::listenOnPort(int port_number)
{
    if (listen_socket != NO_SOCKET)
        return false;
    
#ifdef WIN32
    struct WSAData wsaData;
    WSAStartup(MAKEWORD(1, 1), &wsaData);
#endif
    listen_socket = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in sin;
    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(port_number);
    
    if (bind(listen_socket, (struct sockaddr *) &sin, sizeof(sin)) == -1)
    {
        perror("bind");
        return false;
    }
    
    int i = 1;
    setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&i, sizeof(int));
    
    if (listen(listen_socket, 5) == -1)
    {
        perror("listen");
        return false;
    }
    return true;
}

SerialSim::~SerialSim()
{
}

void SerialSim::write(char c)
{
    UDRx->forceValue(c);
    if (interrupt)
        interrupt();
    addToVirtualTerm(c);
}

void SerialSim::write(const char* string)
{
    write(string, strlen(string));
}

void SerialSim::write(const char* buffer, int size)
{
    while(size > 0)
        write(*buffer++);
}

void SerialSim::UART_UCSRxA_callback(uint8_t oldValue, uint8_t& newValue)
{
    //Always mark "write ready" flag, so the serial code never waits.
    newValue |= _BV(UDRE0);
}

void SerialSim::UART_UDRx_callback(uint8_t oldValue, uint8_t& newValue)
{
    addToVirtualTerm(newValue);
    if (connected_socket != NO_SOCKET)
        send(connected_socket, (char*)&newValue, 1, 0);
}

void SerialSim::addToVirtualTerm(char c)
{
    recvBuffer[recvLine][recvPos] = c;
    recvPos++;
    if (recvPos == 80 || c == '\n')
    {
        recvPos = 0;
        recvLine++;
        if (recvLine == SERIAL_LINE_COUNT)
        {
            for(unsigned int n=0; n<SERIAL_LINE_COUNT-1;n++)
                memcpy(recvBuffer[n], recvBuffer[n+1], 80);
            recvLine--;
            memset(recvBuffer[recvLine], '\0', 80);
        }
    }
}

void SerialSim::draw(int x, int y)
{
    for(unsigned int n=0; n<SERIAL_LINE_COUNT;n++)
        Frontend::instance->drawStringSmall(x, y+n*3, recvBuffer[n], 0xFFFFFF);
}

void SerialSim::tick()
{
    if (listen_socket == NO_SOCKET)
        return;
    
    struct timeval tv;
	fd_set   readfds;
	
    tv.tv_sec = 0;
    tv.tv_usec = 1;

    FD_ZERO(&readfds);
    FD_SET(listen_socket, &readfds);
    if (connected_socket != NO_SOCKET)
        FD_SET(connected_socket, &readfds);
    
    unsigned int max_socket_nr = listen_socket + 1;
    if (connected_socket != NO_SOCKET)
        max_socket_nr = std::max(max_socket_nr, connected_socket + 1);
    if (select(max_socket_nr, &readfds, NULL, NULL, &tv) > 0)
    {
        if (FD_ISSET(listen_socket, &readfds))
        {
            if (connected_socket != NO_SOCKET)
                socketCloseFunction(connected_socket);
            connected_socket = accept(listen_socket, NULL, 0);
        }else if (connected_socket != NO_SOCKET && FD_ISSET(connected_socket, &readfds))
        {
            char buffer[1024];
            int read_count = recv(connected_socket, buffer, sizeof(buffer), 0);
            if (read_count <= 0)
            {
                socketCloseFunction(connected_socket);
                connected_socket = NO_SOCKET;
            }else{
                for(int n=0; n<read_count; n++)
                    write(buffer[n]);
            }
        }
    }
}
