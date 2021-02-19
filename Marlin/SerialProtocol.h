/*
    Copyright (c) 2017-2021 Ultimaker B.V. All rights reserved.

    Marlin is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Marlin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

/**
 Marlin serial protocol handler.
 This is a new communication protocol implementation which is not backwards compatible
 with the old MarlinV1 protocol used by many repraps and the Ultimaker Original and Ultimaker2.

 The decision to change this protocol was made because the checksum and resend implementation of the MarlinV1 protocol where not robust enough.
*/

/*** Protocol: Description ***
 Host: The system talking to Marlin.
 Slave: Marlin.
 The communication protocol is not mirrored, meaning that the host to slave communication is not the same protocol as the slave to host communication.
 This is because the slave is limited in memory and CPU. And does not have the facilities to keep buffers for re-sending.
 Also the current slave implementation has no concept of a "message" towards the host, but sends data in seperate pieces.
 Changing this would be a serious effort due to the  errors being send asynchronized and thus can be in-between of a normal message.
*/

/*** Protocol: Implementation ***

 *** Host to slave **
 The host to slave* protocol has the following elements:
 * Start of packet
 * Sequence number
 * Data length
 * Payload
 * CRC16

 Start of packet: 0xFF, chosen to fix any possible framing errors.
 Sequence number: 0-254, increased for each message, used for error detection and resend recovery, since 0xFF 0xFF can be received at start of serial comms
 Data length: 1-MAX_CMD_SIZE, size of the payload.
 Payload: squence of bytes, only ASCII. Between 0x1F and 0x7F. The Data length field indicates the amount of payload data.
 CRC16: A CRC16-CCITT calculated over the sequence number, data length and payload data.

 *** Slave to host ***
 ASCII line based protocol for legacy reasons.
 Special commands:
 "\noSSPPCC\n"             - Message received sequence number SS (hexadecimal 0-254), with PP (hexadecimal 0-15) remaining room in the planner buffer for G0/G1 commands and with CC (CRC8) over the data
 "\nrSSPPCC\n"             - Message was not with sequence number SS (hexadecimal 0-254) was not received correctly, with PP (hexadecimal 0-15) remaining room in the planner buffer for G0/G1 commands and with CC (CRC8) over the data
 "\nProtoError: [ascii]\n" - Actual error message to indicate the type of protocol error, for debugging.
 "\nError: [ascii]\n"      - System error.
 "\nLOG: [ascii]\n"        - Log the message in the system log.
 Any other data is part of the reply of a latest command without an "o" until the "o" message is received.
*/

#include "Configuration_adv.h"

#include <inttypes.h>
#include "crc8.h"

class SerialProtocol
{
public:
#define PACKET_HEADER_SIZE 2
    /** Defines the NetworkPacket to receive and possibly send! */
    typedef struct
    {
        uint8_t start_of_packet[PACKET_HEADER_SIZE];
        uint8_t sequence_number;
        uint8_t payload_size;
        uint8_t payload[MAX_CMD_SIZE];
        uint16_t crc16;
    } network_packet_t;

    /** @brief Define function pointer callback as a function taking a network_packet_t and returning a boolean value
     *  std::function from \<functional\> is not available!
     */
    typedef bool (*callback_handle_packet_t)(SerialProtocol::network_packet_t packet);


    /** @brief Read a packet, a character at a time
     *  @param callback When a packet is complete (and without errors), it's passed on to this callback.
     */
    static void readCommand(callback_handle_packet_t callback);

    /** @brief Will put out the "r<receive_packet_number>" message onto the serial line
     * @param plan_buf_free_positions The number of free planner positions.
     */
    static void requestResend(uint8_t plan_buf_free_positions);

    /** @brief Cyclic increase of the sequence number
     */
    static inline void increaseReceivePacketNumber()
    {
        receive_packet_number = (receive_packet_number + 1) % MAX_SEQUENCE_NUMBER;
    }

    /** @brief Sends the acknowledge of the packet identified by the given sequence number and how many buffer space is left
     *  @param receive_packet_number The last correctly received packet
     *  @param plan_buf_free_positions The number of free planner positions.
     */
    static void sendAck(const uint8_t receive_packet_number, uint8_t plan_buf_free_positions);

    /** @brief Creates the initial response to send back to the Host. It will initialize the @ref response_crc8 variable
     *  @param response The response to start with
     *  @param receive_packet_number The sequence number to include in the response
     *  @param plan_buf_free_positions The number of free planner positions.
     */
    static void sendResponse(const char response, uint8_t receive_packet_number, uint8_t plan_buf_free_positions);

private:
    /** @brief This will be updated for the response crc */
    static Crc8 response_crc8;

    /** @brief The sequence number of the packet to be received */
    static uint8_t receive_packet_number;

    /** constexpr is not allowed */
    static const uint8_t MAX_SEQUENCE_NUMBER = 255;
    static const uint8_t HEADER_BYTE         = 0xFF;

    /** Allowable characters (included), starting with space to ~ */
    static const uint8_t MINIMUM_ALLOWED_PAYLOAD_CHARACTER = 0x1F + 1;
    static const uint8_t MAXIMUM_ALLOWED_PAYLOAD_CHARACTER = 0x7F - 1;

    /** Possible states */
    typedef enum {
        WAIT_FOR_HEADER_BYTE,
        WAIT_FOR_SECOND_HEADER_BYTE,
        WAIT_FOR_SEQUENCE_NUMBER,
        WAIT_FOR_PAYLOAD_SIZE,
        WAIT_FOR_PAYLOAD_OR_CRC,
        WAIT_FOR_FINAL_CRC_BYTE
    } reader_state_t;

    /** @brief Sends the rejection of the packet identified by the given sequence number. This means the packet with this sequence number must be resend
     */
    static void sendReject(uint8_t plan_buf_free_positions);

    /** @brief Converts the value to a hex string which is both sent and used to calculate the crc!
     *  @param value The value to add to the response.
     */
    static void addHexNumberToResponse(const uint8_t value);
};

#endif//SERIAL_PROTOCOL_H
