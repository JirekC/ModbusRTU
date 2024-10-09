#ifndef COMMON_CRC_H
#define COMMON_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief CRC: 16 bit used by Modbus RTU, polynom: 0x8005, reversed: 0xA001
 * 
 * @param  data    input data CRC will be calculated from
 * @param  length  ledgth of data
 * @param  crcSeed initial seed or result of previous call if called "per partes"
 * @return calculated CRC
 */
uint16_t CrcModbus (const uint8_t* data, uint8_t length, uint16_t crcSeed);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_CRC_H */