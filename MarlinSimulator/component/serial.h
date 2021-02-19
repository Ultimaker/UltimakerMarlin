#ifndef SERIAL_SIM_H
#define SERIAL_SIM_H

#include "base.h"

#define SERIAL_LINE_COUNT 30
class SerialSim : public SimBaseComponent
{
public:
    // @param port_nr: The port index inside the AVR.
    SerialSim(int port_nr);
    virtual ~SerialSim();
    
    // Allow for a socket connection on this serial port. Raw data is passed from the serial port to the socket and back. Accepts a single connection.
    // New connections close the old one to prevent a possible lingering connection blocking connecting from the other side.
    // @param port_number: A TCP port number on which a connection can be made to talk with this virtual serial port trough TCP/IP.
    bool listenOnPort(int port_number);
    
    virtual void draw(int x, int y);
    virtual void tick();

    void write(char c);
    void write(const char* string);
    void write(const char* buffer, int size);
    
    int getPortNr() { return port_nr; }
private:
    typedef void (*InterruptPointer)();

    int port_nr;
    AVRRegistor* UCSRxA;
    AVRRegistor* UDRx;
    InterruptPointer interrupt;
    
    int recvLine, recvPos;
    char recvBuffer[SERIAL_LINE_COUNT][80];
    
    void UART_UCSRxA_callback(uint8_t oldValue, uint8_t& newValue);
    void UART_UDRx_callback(uint8_t oldValue, uint8_t& newValue);
    
    void addToVirtualTerm(char c);
    
    // local members for socket communication.
    unsigned int listen_socket;
    unsigned int connected_socket;
};

#endif//SERIAL_SIM_H
