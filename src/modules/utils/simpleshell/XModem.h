#ifndef XMODEM_H
#define XMODEM_H

#include <string>
#include <cstdint>
#include "StreamOutput.h"

#include "quicklz.h"

class XModem {
public:
    bool upload(const std::string& filename, StreamOutput* stream);
    bool download(const std::string& filename, StreamOutput* stream);

private:
    // Buffers

    static const size_t XBUFF_SIZE = COMPRESS_BUFFER_SIZE + BUFFER_PADDING;
    char md5_str[33];
    unsigned char xbuff[XBUFF_SIZE];
    unsigned char lzbuff[DCOMPRESS_BUFFER_SIZE];
    char info_msg[64];

    // Constants
    static const unsigned char SOH = 0x01;
    static const unsigned char STX = 0x02;
    static const unsigned char EOT = 0x04;
    static const unsigned char ACK = 0x06;
    static const unsigned char NAK = 0x15;
    static const unsigned char CAN = 0x16; // FIXME, should be 0x18 but Carvera seems to use 0x16 (SYN) instead
    static const unsigned char CTRLZ = 0x1A;

    // Helper Functions
    int inbyte(unsigned int timeout_ms, StreamOutput* stream);
    int inbytes(char** buf, int size, unsigned int timeout_ms, StreamOutput* stream);
    void flush_input(StreamOutput* stream);
    void cancel_transfer(StreamOutput* stream);
    void set_serial_rx_irq(bool enable);

    // CRC Functions
    static unsigned short crc16_ccitt_update(unsigned short crc, unsigned char* data, unsigned int len);
    int check_crc(int crc, unsigned char* data, unsigned int len);

    // Additional Helpers
    bool decompress(const std::string& sfilename, const std::string& dfilename, uint32_t sfilesize, StreamOutput* stream);
    
    // Constants for transfer
    static const int MAXRETRANS = 10;
    static const int TIMEOUT_MS = 10000;

    class ChunkIterator {
    public:
        ChunkIterator() {};

        ChunkIterator(bool is_crc, bool is_stx, size_t block_size)
        : is_crc(is_crc), is_stx(is_stx), block_size(block_size) {}
        
        void prepare(int packetno, FILE* file) {
            size_t current, end;
            current = ftell(file);
            fseek(file, 0L, SEEK_END);
            end = ftell(file);
            fseek(file, current, SEEK_SET);
            length = end - current;

            if (length > block_size)
                length = block_size;

            this->file = file;

            prepare(packetno);
        }

        size_t next(uint8_t* buffer, size_t size) {
            size_t n = 0;

            while (n < size && state != DONE) {
                switch (state) {
                    case HEADER: {
                        size_t header_size = is_stx ? 5 : 4;

                        memcpy(buffer + n, header, header_size);
                        n += header_size;

                        // Start checksum calculation from byte 3 of the header
                        update_checksum(header + 3, header_size - 3);

                        state = DATA;
                        break;
                    }
                    case DATA: {
                        size_t data_space = block_size - data_in_block;
                        size_t bytes_to_send = size - n;
                        if (bytes_to_send > data_space)
                            bytes_to_send = data_space;

                        size_t bytes_sent = next_file(buffer + n, bytes_to_send);

                        n += bytes_sent;

                        // If we've sent all data in the block, move to CHECKSUM state
                        if (data_in_block >= block_size) {
                            state = CHECKSUM;
                        }

                        // If there's remaining space in the buffer, and we've sent all data, pad with CTRLZ
                        if (n < size && data_index >= length && data_in_block < block_size) {
                            size_t padding = block_size - data_in_block;
                            if (padding > size - n)
                                padding = size - n;

                            memset(buffer + n, XModem::CTRLZ, padding);
                            update_checksum(buffer + n, padding);

                            data_in_block += padding;
                            n += padding;

                            if (data_in_block >= block_size) {
                                state = CHECKSUM;
                            }
                        }
                        break;
                    }
                    case CHECKSUM: {
                        size_t checksum_size = is_crc ? 2 : 1;
                        size_t bytes_to_send = checksum_size - checksum_index;
                        if (bytes_to_send > size - n)
                            bytes_to_send = size - n;

                        if (is_crc) {
                            // Send CRC bytes
                            uint8_t crc_bytes[2] = { static_cast<uint8_t>((checksum >> 8) & 0xFF), static_cast<uint8_t>(checksum & 0xFF) };
                            memcpy(buffer + n, crc_bytes + checksum_index, bytes_to_send);
                        } else {
                            // Send checksum byte
                            buffer[n] = static_cast<uint8_t>(checksum & 0xFF);
                        }

                        checksum_index += bytes_to_send;
                        n += bytes_to_send;

                        if (checksum_index >= checksum_size) {
                            state = DONE;
                        }
                        break;
                    }
                    case DONE:
                        // Packet has been fully sent
                        break;
                }
            }

            i += n;
            return n;
        }

    private:
        void prepare(int packetno) {
            header[0] = is_stx ? XModem::STX : XModem::SOH;
            header[1] = packetno;
            header[2] = ~packetno;
            header[3] = is_stx ? (length >> 8) & 0xFF : length & 0xFF;

            if (is_stx) {
                header[4] = length & 0xFF;
            }

            i = 0;
            checksum = 0;
            state = HEADER;
            data_index = 0;
            data_in_block = 0;
            checksum_index = 0;
            total_data_sent = 0;
            total_packet_size = (is_stx ? 5 : 4) + block_size + (is_crc ? 2 : 1);
        }

        void update_checksum(uint8_t *p, size_t size) {
            if (is_crc) {
				checksum = XModem::crc16_ccitt_update(checksum, p, size);
			} else {
				for (size_t i = 0; i < size; i++) {
					checksum += p[i];
				}
            }
        }

        size_t next_file(uint8_t* buffer, size_t size) {
            size_t bytes_remaining = length - data_index;
            size_t bytes_to_read = size;

            // Calculate how many bytes we can read
            if (bytes_to_read > bytes_remaining)
                bytes_to_read = bytes_remaining;

            // Read data from file
            size_t bytes_read = fread(buffer, 1, bytes_to_read, file);

            // Update indices and checksum
            data_index += bytes_read;
            data_in_block += bytes_read;
            update_checksum(buffer, bytes_read);

            return bytes_read;
        }

        bool is_crc, is_stx;
        size_t block_size, length, i;
        unsigned short checksum;
        uint8_t header[5];
        FILE* file;

        enum State { HEADER, DATA, CHECKSUM, DONE } state;
        size_t data_index;
        size_t data_in_block;
        size_t checksum_index;
        size_t total_data_sent;
        size_t total_packet_size;
    };

    ChunkIterator outputIterator;
};

#endif // XMODEM_H
