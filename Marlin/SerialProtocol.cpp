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
#include "SerialProtocol.h"
// Required to get the correct MSerial stuff - should be moved to something different, as well as the SERIAL_... macros
#include "Marlin.h"
#include "crc16.h"

const char protocol_error_magic[] PROGMEM = "\nProtoError:";
#define SERIAL_PROTOCOL_ERROR_START serialprintPGM(protocol_error_magic);

/* Converts the sequence number (byte) to int */
#define SERIAL_ECHO_BYTE_AS_NR_LN(x) SERIAL_ECHOLN(int(x))

uint8_t SerialProtocol::receive_packet_number = 0;
Crc8    SerialProtocol::response_crc8;

static const char __PACKET_ACK = 'o';
static const char __PACKET_REJ = 'r';

// To simplify the protocol (no need for separators or keys) having a way to get the free plan buffer space is made accessible, which comes from planner.h
extern uint8_t plan_buf_free_positions();

// Macro and static hexdigits array for quick value to hex conversion
static const char __HEX_DIGITS[] = "0123456789ABCDEF";
#define CONVERT_BYTE_TO_HEX_DIGIT(b) __HEX_DIGITS[0x0F & (b)]

// If a HEADER_BYTE is received, assume it's the start of a new packet.
#define BREAK_ON_HEADER_BYTE(byte) \
    if ((byte) == HEADER_BYTE) \
    {\
        state = WAIT_FOR_SECOND_HEADER_BYTE;\
        break;\
    }

void SerialProtocol::readCommand(callback_handle_packet_t callback)
{
    /* Local static variables to keep track of the reading of bytes to fill the packet */
    static SerialProtocol::network_packet_t packet;
    static SerialProtocol::reader_state_t   state = WAIT_FOR_HEADER_BYTE;
    static uint8_t                          payload_index = 0;

    while (MSerial.available() > 0)
    {
        int serial_input = MSerial.read();
        if (serial_input == -1)
        {
            return;
        }
        unsigned char serial_char = (unsigned char)serial_input;

        /* What byte do we expect? */
        switch (state)
        {
            case WAIT_FOR_HEADER_BYTE:
            {
                if (serial_char == HEADER_BYTE)
                {
                    packet.start_of_packet[0] = serial_char;
                    state = WAIT_FOR_SECOND_HEADER_BYTE;
                }
                break;
            }
            case WAIT_FOR_SECOND_HEADER_BYTE:
            {
                if (serial_char == HEADER_BYTE)
                {
                    packet.start_of_packet[1] = serial_char;
                    state = WAIT_FOR_SEQUENCE_NUMBER;
                }
                else
                {
                    // If it's not a header byte, it was a corrupt byte; wait for start of header again
                    state = WAIT_FOR_HEADER_BYTE;
                }
                break;
            }
            case WAIT_FOR_SEQUENCE_NUMBER:
            {
                if (serial_char == HEADER_BYTE)
                {
                    // Ignore all header bytes after the first 2.
                    // This allows synchronization on any number >= 2 HEADER_BYTES.
                    break;
                }

                if (serial_char != receive_packet_number)
                {
                    // Wrong expected sequence number, ignore complete packet, ask for resend of the expected sequence number
                    SERIAL_PROTOCOL_ERROR_START;
                    SERIAL_ECHOPGM(MSG_ERR_UNEXPECTED_SEQUENCE_NUMBER);
                    SERIAL_ECHO((unsigned int)serial_char);
                    SERIAL_ECHO(",");
                    SERIAL_ECHO_BYTE_AS_NR_LN(receive_packet_number);
                    // An unfortunate dependency on the planner; In the future, we need to push this number into this function, instead of pulling it here
                    requestResend(plan_buf_free_positions());
                    state = WAIT_FOR_HEADER_BYTE;
                }
                else
                {
                    packet.sequence_number = serial_char;
                    state = WAIT_FOR_PAYLOAD_SIZE;
                }
                break;
            }
            case WAIT_FOR_PAYLOAD_SIZE:
            {
                BREAK_ON_HEADER_BYTE(serial_char);

                if ((serial_char == 0) || (serial_char > MAX_CMD_SIZE))
                {
                    // Payload too big too handle
                    SERIAL_PROTOCOL_ERROR_START;
                    SERIAL_ECHOPGM(MSG_ERR_PAYLOAD_SIZE_TOO_BIG);
                    SERIAL_ECHO_BYTE_AS_NR_LN(receive_packet_number);
                    // An unfortunate dependency on the planner; In the future, we need to push this number into this function, instead of pulling it here
                    requestResend(plan_buf_free_positions());
                    state = WAIT_FOR_HEADER_BYTE;
                }
                else
                {
                    packet.payload_size = serial_char;
                    payload_index = 0;
                    state = WAIT_FOR_PAYLOAD_OR_CRC;
                }
                break;
            }
            case WAIT_FOR_PAYLOAD_OR_CRC:
            {
                if (payload_index < packet.payload_size)
                {
                    BREAK_ON_HEADER_BYTE(serial_char);

                    if ((serial_char >= MINIMUM_ALLOWED_PAYLOAD_CHARACTER) && (serial_char <= MAXIMUM_ALLOWED_PAYLOAD_CHARACTER))
                    {
                        packet.payload[payload_index++] = serial_char;
                    }
                    else
                    {
                        SERIAL_PROTOCOL_ERROR_START;
                        SERIAL_ECHOPGM(MSG_ERR_PAYLOAD_ILLEGAL_CHARACTER);
                        SERIAL_ECHO_F(payload_index, DEC);
                        SERIAL_ECHOLN("");
                        // An unfortunate dependency on the planner; In the future, we need to push this number into this function, instead of pulling it here
                        requestResend(plan_buf_free_positions());
                        state = WAIT_FOR_HEADER_BYTE;
                    }
                }
                else
                {
                    packet.crc16 = uint16_t(serial_char) << 8;
                    state = WAIT_FOR_FINAL_CRC_BYTE;
                }
                break;
            }
            // After CRC byte 2, verify packet and pass it on if it's correct.
            case WAIT_FOR_FINAL_CRC_BYTE:
            {
                packet.crc16 += uint16_t(serial_char);

                Crc16 packet_crc16(&packet.sequence_number, 1);
                packet_crc16.update(&packet.payload_size, 1);
                packet_crc16.update(packet.payload, packet.payload_size);

                if (packet.crc16 != packet_crc16.getCrc())
                {
                    // Corrupted packet
                    SERIAL_PROTOCOL_ERROR_START;
                    SERIAL_ECHOPGM(MSG_ERR_CRC_MISMATCH);
                    SERIAL_ECHO_BYTE_AS_NR_LN(receive_packet_number);
                    // An unfortunate dependency on the planner; In the future, we need to push this number into this function, instead of pulling it here
                    requestResend(plan_buf_free_positions());
                }
                else
                {
                    // Hand off packet
                    if (callback(packet))
                    {
                        increaseReceivePacketNumber();
                    }
                    else
                    {
                        SERIAL_PROTOCOL_ERROR_START;
                        SERIAL_ECHOPGM(MSG_ERR_PACKET_UNHANDLED);
                        SERIAL_ECHO_BYTE_AS_NR_LN(receive_packet_number);
                        // An unfortunate dependency on the planner; In the future, we need to push this number into this function, instead of pulling it here
                        requestResend(plan_buf_free_positions());
                    }
                }
                // Yes, fall through, in both cases we expect to get a new packet
            }
            // Unknown case, or finished, wait for new packet in the next loop
            default:
            {
                // Wait for new start of packet
                state = WAIT_FOR_HEADER_BYTE;
                // Even if there is still serial input to process...
                return;
            }
        }
    }
}

void SerialProtocol::requestResend(uint8_t plan_buf_free_positions)
{
    // Flush communications as there is some error
    MSerial.flush();
    sendReject(plan_buf_free_positions);
}


void SerialProtocol::sendAck(const uint8_t sequence_number, uint8_t plan_buf_free_positions)
{
    sendResponse(__PACKET_ACK, sequence_number, plan_buf_free_positions);
}

void SerialProtocol::sendReject(uint8_t plan_buf_free_positions)
{
    sendResponse(__PACKET_REJ, receive_packet_number, plan_buf_free_positions);
}

void SerialProtocol::sendResponse(const char response, uint8_t sequence_number, uint8_t plan_buf_free_positions)
{
    uint8_t  crc8;

    MSerial.println();

    response_crc8.reset();
    response_crc8.update(response);
    MSerial.write(response);
    addHexNumberToResponse(sequence_number);
    addHexNumberToResponse(plan_buf_free_positions);
    crc8 = response_crc8.getCrc();
    MSerial.write(CONVERT_BYTE_TO_HEX_DIGIT(crc8 >> 4));
    MSerial.write(CONVERT_BYTE_TO_HEX_DIGIT(crc8));
    MSerial.println();
}

void SerialProtocol::addHexNumberToResponse(const uint8_t value)
{
    // An uint8_t is 0..255, so 2 bytes for hex representation is enough
    uint8_t temp[2] = { CONVERT_BYTE_TO_HEX_DIGIT(value >> 4), CONVERT_BYTE_TO_HEX_DIGIT(value) };
    response_crc8.update(temp, sizeof(temp));
    MSerial.write(temp[0]);
    MSerial.write(temp[1]);
}
