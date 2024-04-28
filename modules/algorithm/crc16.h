#ifndef __CRC16_H
#define __CRC16_H

#include "main.h"

#define CRC_START_16 0xFFFF

uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);

#endif
