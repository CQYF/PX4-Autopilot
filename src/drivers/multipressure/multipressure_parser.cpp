/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file multipressure_parser.cpp
 *
 * Parser for the multipressure sensor protocol
 */

#include "multipressure_parser.h"
#include <string.h>

#define MULTIPRESSURE_FRAME_SIZE 66

int multipressure_parse(char c, char *parserbuf, unsigned *parserbuf_index,
                        MULTIPRESSURE_PARSE_STATE *state, float *pressures)
{
    int ret = -1;

    switch (*state) {
    case MULTIPRESSURE_PARSE_STATE::STATE0_UNSYNC:
        if (c == 0x55) {
            *state = MULTIPRESSURE_PARSE_STATE::STATE1_SYNC_1;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;
        }

        break;

    case MULTIPRESSURE_PARSE_STATE::STATE1_SYNC_1:
        if (c == 0xAA) {
            *state = MULTIPRESSURE_PARSE_STATE::STATE2_GOT_DATA;
            parserbuf[*parserbuf_index] = c;
            (*parserbuf_index)++;

        } else {
            *state = MULTIPRESSURE_PARSE_STATE::STATE0_UNSYNC;
            *parserbuf_index = 0;
        }

        break;

    case MULTIPRESSURE_PARSE_STATE::STATE2_GOT_DATA:
        parserbuf[*parserbuf_index] = c;
        (*parserbuf_index)++;

        if (*parserbuf_index >= MULTIPRESSURE_FRAME_SIZE) {
            *state = MULTIPRESSURE_PARSE_STATE::STATE3_GOT_CHECKSUM;
        }

        break;

    case MULTIPRESSURE_PARSE_STATE::STATE3_GOT_CHECKSUM:
        unsigned char checksum = 0;

        for (int i = 0; i < MULTIPRESSURE_FRAME_SIZE - 1; i++) {
            checksum += parserbuf[i];
        }

        if (c == checksum) {
            for (int i = 0; i < 16; i++) {
                unsigned int val = (unsigned char)parserbuf[2 + i * 4]
                                   | ((unsigned char)parserbuf[3 + i * 4] << 8)
                                   | ((unsigned char)parserbuf[4 + i * 4] << 16)
                                   | ((unsigned char)parserbuf[5 + i * 4] << 24);
                pressures[i] = *(float *)&val;
            }

            ret = 0;
        }

        *state = MULTIPRESSURE_PARSE_STATE::STATE0_UNSYNC;
        *parserbuf_index = 0;

        break;
    }

    return ret;
}