#ifndef CRC16_TABLE_H
#define CRC16_TABLE_H

uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint8_t len);
extern uint16_t const crc_ccitt_table[256];
#endif
