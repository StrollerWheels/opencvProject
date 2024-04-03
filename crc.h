/**
 * @file Crc16.h
 * @brief
 */

#ifndef CRC16_H__
#define CRC16_H__

unsigned short crc16citt(unsigned char * pcBlock, unsigned short len);
unsigned short crc16citt_32(unsigned char * pcBlock, unsigned int len);
unsigned long crc32(unsigned char * buf, unsigned long len);

#endif /* CRC16_H_ */
