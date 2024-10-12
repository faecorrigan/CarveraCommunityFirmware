#include "XModem.h"
#include "libs/Kernel.h"
#include "Conveyor.h"
#include "quicklz.h"
#include "utils.h"
#include "mbed.h"
#include "PublicData.h"
#include "ATCHandlerPublicAccess.h"
#include "Logging.h"
#include "md5.h"
#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

using namespace std;

int XModem::inbyte(unsigned int timeout_ms, StreamOutput* stream) {
    uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->getc();
        vTaskDelay(0);
    }
    return -1;
}

int XModem::inbytes(char **buf, int size, unsigned int timeout_ms, StreamOutput* stream) {
	uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->gets(buf, size);
        vTaskDelay(0);
    }
    return -1;
}

void XModem::flush_input(StreamOutput* stream) {
    while (inbyte(0, stream) >= 0)
        continue;
}

void XModem::cancel_transfer(StreamOutput* stream) {
    stream->putc(CAN);
    flush_input(stream);
}

void XModem::set_serial_rx_irq(bool enable) {
    // Disable serial RX IRQ
    bool enable_irq = enable;
    PublicData::set_value(atc_handler_checksum, set_serial_rx_irq_checksum, &enable_irq);
}

unsigned short XModem::crc16_ccitt_update(unsigned short crc, unsigned char *data, unsigned int len)
{
	static const unsigned short crc_table[] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
		0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
		0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
		0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
		0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
		0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
		0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
		0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
		0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
		0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
		0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
		0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
		0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
		0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
		0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
		0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
		0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
		0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
		0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
		0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
	};

	unsigned char tmp;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]);
	}

	return crc;
}

int XModem::check_crc(int crc, unsigned char *data, unsigned int len) {
    if (crc) {
        unsigned short crc = crc16_ccitt_update(0, data, len);
        unsigned short tcrc = (data[len] << 8) + data[len+1];
        if (crc == tcrc)
            return 1;
    }
    else {
        unsigned char cks = 0;
        for (unsigned int i = 0; i < len; ++i) {
            cks += data[i];
        }
        if (cks == data[len])
        return 1;
    }

    return 0;
}

bool XModem::decompress(const std::string& sfilename, const std::string& dfilename, uint32_t sfilesize, StreamOutput* stream) {
    uint16_t u16Sum = 0;
    uint8_t u8ReadBuffer_hdr[BLOCK_HEADER_SIZE] = { 0 };
    uint32_t u32DcmprsSize = 0, u32BlockSize = 0, u32BlockNum = 0, u32TotalDcmprsSize = 0, i = 0, j = 0, k = 0;
    qlz_state_decompress s_stDecompressState;

    FILE* f_in = fopen(sfilename.c_str(), "rb");
    FILE* f_out = fopen(dfilename.c_str(), "wb");

    if (f_in == NULL || f_out == NULL) {
		stream->printf("Error: Failed to open files for decompression!\r\n");
		goto _exit;
    }

	memset(&s_stDecompressState, 0x00, sizeof(qlz_state_decompress));

    for (i = 0; i < sfilesize - 2; i += BLOCK_HEADER_SIZE + u32BlockSize) {
        fread(u8ReadBuffer_hdr, sizeof(char), BLOCK_HEADER_SIZE, f_in);
        u32BlockSize = u8ReadBuffer_hdr[0] * (1 << 24) + u8ReadBuffer_hdr[1] * (1 << 16) + u8ReadBuffer_hdr[2] * (1 << 8) + u8ReadBuffer_hdr[3];

        if (!u32BlockSize) {
            goto _exit;
        }

        fread(xbuff, sizeof(char), u32BlockSize, f_in);
        u32DcmprsSize = qlz_decompress((const char*)xbuff, lzbuff, &s_stDecompressState);
        if (!u32DcmprsSize) {
            goto _exit;
        }

        for (j = 0; j < u32DcmprsSize; j++) {
            u16Sum += lzbuff[j];
        }

        fwrite(lzbuff, sizeof(char), u32DcmprsSize, f_out);
        u32TotalDcmprsSize += u32DcmprsSize;
        u32BlockNum += 1;
        if (++k > 10) {
            k = 0;
            THEKERNEL.call_event(ON_IDLE);
        }
		stream->printf("#Info: decompart = %lu\r\n", u32BlockNum);
    }

    fread(xbuff, sizeof(char), 2, f_in);

    if (u16Sum != ((xbuff[0] << 8) + xbuff[1])) {
        goto _exit;
    }

	if (f_in != NULL) fclose(f_in);
    if (f_out != NULL) fclose(f_out);

    stream->printf("#Info: decompart = %lu\r\n", u32BlockNum);

	return true;

_exit:
    if (f_in != NULL) fclose(f_in);
    if (f_out != NULL) fclose(f_out);

    return false;
}

bool XModem::upload(const std::string& filename, StreamOutput* stream) {
    char *recv_buff;
    int expected_length, is_stx = 0;
	unsigned short crc = 0;
    int c, length = 0;
    int recv_count = 0;
	bool md5_received = false;
    uint32_t u32filesize = 0;

	memset(info_msg, 0, sizeof(info_msg));

    string md5_filename = change_to_md5_path(filename);
    string lzfilename = change_to_lz_path(filename);
    check_and_make_path(md5_filename);
    check_and_make_path(lzfilename);

	// diasble serial rx irq in case of serial stream, and internal process in case of wifi
    if (stream->type() == 0) {
    	set_serial_rx_irq(false);
    }

    if (!THECONVEYOR.is_idle()) {
        stream->putc(EOT);
        if (stream->type() == 0) {
        	set_serial_rx_irq(true);
        }
        return false;
    }

	THEKERNEL.set_uploading(true);

	//if file is lzCompress file,then need to put .lz dir
	unsigned int start_pos = filename.find(".lz");
	FILE *fd;
	if (start_pos != string::npos) {
		start_pos = lzfilename.rfind(".lz");
		lzfilename=lzfilename.substr(0, start_pos);
    	fd = fopen(lzfilename.c_str(), "wb");
    }
    else {
    	fd = fopen(filename.c_str(), "wb");
    }
		
    FILE *fd_md5 = NULL;
    //if file is lzCompress file,then need to Decompress
	start_pos = md5_filename.find(".lz");
	if (start_pos != string::npos) {
		md5_filename=md5_filename.substr(0, start_pos);
	}
	
    if (filename.find("firmware.bin") == string::npos) {
    	fd_md5 = fopen(md5_filename.c_str(), "wb");
    }

    if (fd == NULL || (filename.find("firmware.bin") == string::npos && fd_md5 == NULL)) {
        stream->putc(EOT);
    	sprintf(info_msg, "Error: failed to open file [%s]!\r\n", fd == NULL ? filename.substr(0, 30).c_str() : md5_filename.substr(0, 30).c_str() );
    	goto upload_error;
    }

	stream->putc('C');
	
    for (;;) {
		int retry = 0;

		if ((c = inbyte(TIMEOUT_MS, stream)) >= 0) {
			switch (c) {
			case SOH:
				expected_length = 128 + 2; // + CRC16
				is_stx = 0;
				goto start_recv;
			case STX:
				expected_length = 8192 + 2; // + CRC16
				is_stx = 1;
				goto start_recv;
			case EOT:
				stream->putc(ACK);
				goto upload_success; /* normal end */
			case CAN:
				stream->putc(ACK);
				sprintf(info_msg, "Info: Upload canceled by remote!\r\n");
				goto upload_error;
				break;
			default:
				break;
			}
		}

        cancel_transfer(stream);
		sprintf(info_msg, "Error: upload sync error! get char [%d]\r\n", c);
        goto upload_error;

    start_recv:
		size_t header_size = is_stx ? 4 : 3;
		size_t file_position;
		int packetno;
		crc = 0;

		retry = 1000;

		do {
			c = inbytes(&recv_buff, header_size, TIMEOUT_MS, stream);
		} while (c == 0 && retry--);

		if (c != header_size) {
			sprintf(info_msg, "Error: header size mismatch: %i != %i\r\n", c, header_size);
			goto upload_error;
		}

		if (recv_buff[0] != (unsigned char)(~recv_buff[1])) {
			sprintf(info_msg, "Error: packet number error\r\n");
			goto upload_error;	
		}

		packetno = recv_buff[0];

		if (is_stx) {
			length = recv_buff[2] * 256 + recv_buff[3];
		} else {
			length = recv_buff[2];
		}

		crc = crc16_ccitt_update(crc, (unsigned char*)recv_buff + 2, is_stx ? 2 : 1);

		// save position in case we need to rewind
		file_position = ftell(fd);

		recv_count = 0;

		while (recv_count < expected_length) {
			retry = 1000;

			do {
				c = inbytes(&recv_buff, expected_length - recv_count, TIMEOUT_MS, stream);
			} while (c == 0 && retry--);

			if (c < 0) {
				sprintf(info_msg, "Error: could not receive data\r\n");
				goto upload_error;
			}

			recv_count += c;

			crc = crc16_ccitt_update(crc, (unsigned char*)recv_buff, c);

			if (packetno == 0 && !md5_received) {
				// packet number 0 contains MD5
				// packet number might wrap around
				if (length != 32 || c < 32) {
					sprintf(info_msg, "Error: could not parse md5 packet\r\n");
					goto upload_error;	
				}

				strncpy(md5_str, recv_buff, 32);
				fwrite(md5_str, sizeof(char), 32, fd_md5);

				md5_received = true;
			} else {
				size_t bytes_to_write = c;

				if (recv_count >= length) {
					size_t excess_data = recv_count - c;
					bytes_to_write = (length > excess_data) ? (length - excess_data) : 0;
				}

				u32filesize += fwrite(recv_buff, sizeof(char), bytes_to_write, fd);
			}
		}

		if (crc == 0) {
			stream->putc(ACK);
		} else {
			stream->putc(NAK);
			fseek(fd, file_position, SEEK_SET);
		}
    }

upload_error:
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
		remove(filename.c_str());
	}
	if (fd_md5 != NULL) {
		fclose(fd_md5);
		fd_md5 = NULL;
		remove(md5_filename.c_str());
	}

	flush_input(stream);

    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }

	THEKERNEL.set_uploading(false);

	stream->printf(info_msg);
	return false;

upload_success:

	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}

	if (fd_md5 != NULL) {
		fclose(fd_md5);
		fd_md5 = NULL;
	}
	
	flush_input(stream);

    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }

	THEKERNEL.set_uploading(false);

	//if file is lzCompress file, then need to decompress
	start_pos = filename.find(".lz");
	string srcfilename = lzfilename;
	string desfilename = filename;

	if (start_pos != string::npos) {
		desfilename=filename.substr(0, start_pos);
		if(!decompress(srcfilename, desfilename, u32filesize, stream))
			return false;
    }

    return true;
}

bool XModem::download(const std::string& filename, StreamOutput* stream) {
	int block_size = 8192;
	int chunk_size = XBUFF_SIZE - 8;
    int crc = 0, is_stx = 1;
    unsigned char packetno = 0;
    int c = 0;

    // open file
	memset(info_msg, 0, sizeof(info_msg));
    string md5_filename = change_to_md5_path(filename);
    string lz_filename = change_to_lz_path(filename);

	// diasble irq
    if (stream->type() == 0) {
    	block_size = 128;
		chunk_size = 128;
    	is_stx = 0;
    	set_serial_rx_irq(false);
    }

    if (!THECONVEYOR.is_idle()) {
        cancel_transfer(stream);
        if (stream->type() == 0) {
        	set_serial_rx_irq(true);
        }
        return false;
    }

	THEKERNEL.set_uploading(true);

    FILE *fd = fopen(md5_filename.c_str(), "rb");
    if (fd != NULL) {
        fgets(md5_str, sizeof(md5_str), fd);
        fclose(fd);
        fd = NULL;
    } else {
		FILE *fd = fopen(filename.c_str(), "rb");
		if (NULL == fd) {
		    cancel_transfer(stream);
			sprintf(info_msg, "Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
			goto download_error;
	    }
		
		MD5 md5;
		do {
			size_t n = fread(xbuff, 1, sizeof(xbuff), fd);
			if (n > 0) md5.update(xbuff, n);
			THEKERNEL.call_event(ON_IDLE);
		} while (!feof(fd));
		strcpy(md5_str, md5.finalize().hexdigest().c_str());
		fclose(fd);
	}
	
	fd = fopen(lz_filename.c_str(), "rb");		//first try to open /.lz/filename
	if (NULL == fd) {	
	    fd = fopen(filename.c_str(), "rb");
	    if (NULL == fd) {
		    cancel_transfer(stream);
			sprintf(info_msg, "Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
			goto download_error;
	    }
	}

	// Wait for C, NAK or CAN
	if ((c = inbyte(TIMEOUT_MS, stream)) >= 0) {
		switch (c) {
			case 'C':
				crc = true;
				break;
			case NAK:
				crc = false;
				break;
			case CAN:
				stream->putc(ACK);
				flush_input(stream);
				sprintf(info_msg, "Info: canceled by remote!\r\n");
				goto download_error;
			default:
				cancel_transfer(stream);
				goto download_error;
			}
	} else {
		cancel_transfer(stream);
		goto download_error;
	}

	{
		ChunkIterator iterator(crc, is_stx, block_size);

		// packet 0 with MD5
		xbuff[0] = SOH;
		xbuff[1] = 0x00;
		xbuff[2] = 0xFF;
		xbuff[3] = strlen(md5_str);
		memcpy(xbuff + 4, md5_str, strlen(md5_str));
		memset(xbuff + 4 + strlen(md5_str), CTRLZ, 128 - 4 - strlen(md5_str));
		unsigned short crc = crc16_ccitt_update(0, xbuff + 3, 128 + 1);
		xbuff[4 + 128] = crc<<8;
		xbuff[4 + 128 + 1] = crc&0xff;

		stream->puts((char *)xbuff, 4 + 128 + 2);

		for (;;) {
			if ((c = inbyte(TIMEOUT_MS, stream)) >= 0) {
				switch (c) {
					case 'C':
					case ACK:
						packetno++;
					case NAK:
						break;
					case CAN:
						stream->putc(ACK);
						sprintf(info_msg, "Info: canceled by remote!\r\n");
						goto download_error;
					default:
						cancel_transfer(stream);
						goto download_error;
				}
			} else {
				cancel_transfer(stream);
				goto download_error;
			}

			size_t position = block_size * (packetno - 1);
			fseek(fd, position, SEEK_SET);

			if (ftell(fd) < position) {
				break;
			}

			iterator.prepare(packetno, fd);

			size_t bytes_read;

			do {
				bytes_read = iterator.next(xbuff, sizeof(xbuff));
				if (bytes_read > 0) {
					stream->puts((char *)xbuff, bytes_read);
				}
			} while (bytes_read > 0);
		}

		// Send End Of Transmission character
	    stream->putc(XModem::EOT);
	}

    // Wait for the final ACK from the receiver
    c = inbyte(TIMEOUT_MS, stream);
    if (c == ACK) {
        goto download_success;
    } else {
        sprintf(info_msg, "Error: No ACK for EOT, received [%02X]!\r\n", c);
        goto download_error;
    }

download_error:
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
	
	flush_input(stream);

    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }

	THEKERNEL.set_uploading(false);

	stream->printf(info_msg);
	return false;

download_success:
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}

    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
	}
	
	THEKERNEL.set_uploading(false);
    
    return true;
}