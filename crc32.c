#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>

#include "crc32_table.h"
#include "cmdIndex.h"
#define crc_n4(crc, data, table) crc ^= data; \
crc = (table[(crc & 0xff) + 0x300]) ^        \
      (table[((crc >> 8) & 0xff) + 0x200]) ^    \
      (table[((data >> 16) & 0xff) + 0x100]) ^    \
      (table[data >> 24]);

#define crc_n4d(crc, data, table) crc ^= data; \
crc = (table[(crc & 0xff) + 0x300]) ^    \
      (table[((crc >> 8) & 0xff) + 0x200]) ^    \
      (table[((crc >> 16) & 0xff) + 0x100]) ^    \
      (table[(crc >> 24) & 0xff]);

#define crcsm_n4d(crc, data, table) crc ^= data; \
crc = (crc >> 8) ^ (table[crc & 0xff]); \
    crc = (crc >> 8) ^ (table[crc & 0xff]); \
    crc = (crc >> 8) ^ (table[crc & 0xff]); \
    crc = (crc >> 8) ^ (table[crc & 0xff]);

uint32_t crc32(const uint8_t *data, size_t datalen) {
    uint32_t seed = CRC32_SEED;
    size_t len = datalen;
    uint32_t crc = seed ^ 0xffffffff;

    while (((uintptr_t)data & 3) && len) {
        crc = (crc >> 8) ^ crc_table_crc32_big[(crc & 0xff) ^ *data++];
        len--;
    }

    while (len >= 16) {
        len -= 16;
        crc_n4d(crc, ((uint32_t *)data)[0], crc_table_crc32_big);
        crc_n4d(crc, ((uint32_t *)data)[1], crc_table_crc32_big);
        crc_n4d(crc, ((uint32_t *)data)[2], crc_table_crc32_big);
        crc_n4d(crc, ((uint32_t *)data)[3], crc_table_crc32_big);
        data += 16;
    }

    while (len--) {
        crc = (crc >> 8) ^ crc_table_crc32_big[(crc & 0xff) ^ *data++];
    }

    crc ^= 0xffffffff;

    return crc;
}

uint32_t crc32_check(uint8_t * crc32_buf)
{
	uint32_t ret_value = crc32(crc32_buf,CRC32_LEN_BC);
	return ret_value;
}
//
//int main() {
//    uint8_t data[25] = { 0xAA, 0x01, 0x19, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x58,
//                        0x00, 0x01, 0xc0, 0xa8, 0x01, 0x32, 0x52, 0xc3,
//                        0x51, 0xc3, 0x52, 0xc3 };
//
//    uint8_t data_set2[40] = { 0xaa, 0x01, 0x22, 0x00, 0x02, 0x48, 0x00, 0xff,
//                              0xf6, 0x00, 0x00, 0x31, 0x50, 0x51, 0x44, 0x4c,
//                              0x35, 0x48, 0x30, 0x30, 0x31, 0x55, 0x4c, 0x39,
//                              0x39, 0x31, 0x00, 0x02, 0xe8, 0xfd };
//
//    printf("CRC32 (data set 1): %08" PRIx32 "\n", crc32(data, 21));
//    printf("CRC32 (data set 2): %08" PRIx32 "\n", crc32(data_set2, 30));
//
//    return 0;
//}
