#!/usr/bin/env python3
import argparse
import socket
import sys
import struct
import binascii
from tqdm import tqdm

# Constants for the protocol
SOH = 0x01  # Start of Header for 128-byte packets
STX = 0x02  # Start of Text for 8190-byte packets
EOT = 0x04  # End of Transmission
ACK = 0x06  # Acknowledge
NAK = 0x15  # Negative Acknowledge
CAN = 0x18  # Cancel
CRC_CHR = ord('C')  # ASCII 'C' indicates use of CRC
TIMEOUT = 10  # seconds

PACKET_SIZE_SOH = 1 + 2 + 1 + 128 + 2
PACKET_SIZE_STX = 1 + 2 + 2 + 8192 + 2

def calculate_crc16(data):
    """Calculate CRC16-CCITT checksum using binascii.crc_hqx."""
    return binascii.crc_hqx(data, 0x0000)

def process_buffer(buffer, file, sock, last_packet_number, pbar):
    """
    Process the receive buffer to extract and handle complete packets.
    
    Returns a tuple of (updated_last_packet_number, eot_received).
    """
    eot_received = False  # Flag to indicate if EOT was received

    while True:
        if len(buffer) < 1:
            # Need at least 1 byte to determine packet type
            return last_packet_number, eot_received

        packet_type = buffer[0]

        if packet_type not in (SOH, STX, EOT, CAN):
            # Unexpected byte, possibly a control character or noise
            buffer.pop(0)  # Remove the unexpected byte
            continue

        length = None

        if packet_type in (SOH, STX):
            total_packet_size = PACKET_SIZE_SOH if packet_type == SOH else PACKET_SIZE_STX

            if len(buffer) < total_packet_size:
                # Wait for more data
                return last_packet_number, eot_received

            # Extract the packet
            packet = buffer[:total_packet_size]
            del buffer[:total_packet_size]  # Remove the packet from the buffer

            # Unpack packet_number and complement
            try:
                header = struct.unpack('!B', packet[:1])[0]
                if header == SOH:
                    packet_number, complement, length = struct.unpack('!BBB', packet[1:4])
                    data = packet[3:-2]
                elif header == STX:
                    packet_number, complement, length = struct.unpack('!BBH', packet[1:5])
                    data = packet[3:-2]
                else:
                    # This should not happen, but added for safety
                    continue
            except struct.error:
                continue

            # Validate packet number
            if packet_number != (~complement & 0xFF):
                continue  # Skip this packet

            # Extract CRC
            crc_received = struct.unpack('!H', packet[-2:])[0]

            if packet_number == 0:
                # Packet 0: Contains MD5 checksum and padding
                md5_received = data[:16].hex()
                # Optionally, you can store or verify the MD5 checksum here

                # Send ACK for packet 0 without CRC validation
                sock.sendall(ACK.to_bytes(1, byteorder='big'))
                last_packet_number = packet_number
                return last_packet_number, eot_received  # No CRC validation for packet 0

            # Validate CRC
            calculated_crc = calculate_crc16(data)

            if crc_received != calculated_crc:
                sock.sendall(NAK.to_bytes(1, byteorder='big'))
                continue  # Optionally, you can implement retry logic here

            # Check for packet sequence
            expected_packet_num = (last_packet_number + 1) % 256
            if packet_number != expected_packet_num:
                continue  # Skip this packet

            # Write data to file
            file.write(data[2:length+2])
            pbar.update(length)
            sock.sendall(ACK.to_bytes(1, byteorder='big'))
            last_packet_number = packet_number

        elif packet_type == EOT:
            # End of Transmission
            sock.sendall(ACK.to_bytes(1, byteorder='big'))
            # Remove EOT byte from buffer
            buffer.pop(0)
            eot_received = True
            return last_packet_number, eot_received

        elif packet_type == CAN:
            # Cancel Transmission
            sock.sendall(ACK.to_bytes(1, byteorder='big'))
            # Remove CAN byte from buffer
            buffer.pop(0)
            return last_packet_number, eot_received

    return last_packet_number, eot_received

def initial_handshake(sock):
    """
    Send two newline characters and wait for an 'ok' response after the second newline.
    """
    try:
        sock.sendall(b'\n')

        response = b''
        while b'ok' not in response.lower():
            chunk = sock.recv(1024)
            if not chunk:
                raise ConnectionError("Connection closed while waiting for 'ok'.")
            response += chunk
    except socket.timeout:
        raise ConnectionError("Socket timeout during initial handshake.")

def download_file(sock, filename, destination_path):
    """Download a file from the server using xmodem protocol."""
    buffer = bytearray()  # Buffer to accumulate data fragments
    last_packet_number = -1
    eot_received = False  # Flag to indicate if EOT was received

    try:
        initial_handshake(sock)
        sock.sendall(f"download {filename}\n".encode())
        sock.sendall(CRC_CHR.to_bytes(1, byteorder='big'))  # Request CRC mode

        with open(destination_path, 'wb') as file, tqdm(
            unit='B',
            unit_scale=True,
            desc=filename,
            dynamic_ncols=True
        ) as pbar:
            while not eot_received:
                try:
                    data = sock.recv(4096)  # Read in chunks
                    if not data:
                        break

                    buffer.extend(data)

                    last_packet_number, eot_received = process_buffer(buffer, file, sock, last_packet_number, pbar)

                except socket.timeout:
                    break

    except Exception as e:
        print(f"Error during file transfer: {e}")
    finally:
        try:
            sock.sendall(EOT.to_bytes(1, byteorder='big'))  # Signal end of transmission
        except Exception:
            pass

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Download a file from a server using xmodem over TCP.')
    parser.add_argument('host', help='Host to connect to')
    parser.add_argument('port', type=int, help='Port to connect to')
    parser.add_argument('file', help='File to download')
    parser.add_argument('destination_path', help='Destination path on server')
    args = parser.parse_args()

    # Establish TCP connection to the server
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(TIMEOUT)
            sock.connect((args.host, args.port))

            # Start the file download process
            download_file(sock, args.file, args.destination_path)

            print("File download completed successfully.")
    except (socket.timeout, socket.error, ConnectionError) as e:
        print(f"Connection error: {e}")
        sys.exit(1)

if __name__ == '__main__':
        main()
