#include "paradox.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_pwm.h"
#include "utils/fskdemod.h"
#include "t55xx.h"

#define PARADOX_FRAME_BITS (96)
#define PARADOX_BUFFER_BITS (104)
#define PARADOX_PREAMBLE_BITS (8)
#define PARADOX_PREAMBLE (0x0F)
#define PARADOX_DATA_BITS (44)
#define PARADOX_PWM_MAX_ENTRIES (PARADOX_FRAME_BITS * 6)

/*
 * Paradox is FSK2a at RF/50. Its 96-bit frame starts with 00001111, then
 * carries 44 data bits as Manchester pairs. A following frame's preamble
 * closes the 104-bit decode window. The last four bits of the six-byte
 * decoded value are padding and are not transmitted.
 *
 * Framing and field positions were cross-checked against Flipper Zero's
 * GPLv3 protocol_paradox.c. This implementation uses local FSK and PWM APIs.
 */

#define PARADOX_FSK_BITRATE (50)
#define PARADOX_FSK0_LOOP (6)
#define PARADOX_FSK0_TOP (8)
#define PARADOX_FSK1_LOOP (5)
#define PARADOX_FSK1_TOP (10)

static nrf_pwm_values_wave_form_t m_paradox_pwm_seq_vals[PARADOX_PWM_MAX_ENTRIES] = {};

static nrf_pwm_sequence_t m_paradox_pwm_seq = {
    .values.p_wave_form = m_paradox_pwm_seq_vals,
    .length = 0,
    .repeats = 0,
    .end_delay = 0,
};

typedef struct {
    uint8_t data[PARADOX_DATA_SIZE];
    uint8_t encoded_bits[PARADOX_BUFFER_BITS];
    uint8_t encoded_bit_count;
    fsk_t *modem;
} paradox_codec;

static void paradox_build_frame(const uint8_t *data, uint8_t *frame) {
    memset(frame, 0, PARADOX_FRAME_BITS);

    for (uint8_t i = 0; i < PARADOX_PREAMBLE_BITS; i++) {
        frame[i] = (uint8_t)((PARADOX_PREAMBLE >> (7 - i)) & 0x01);
    }

    for (uint8_t i = 0; i < PARADOX_DATA_BITS; i++) {
        uint8_t byte_index = i / 8;
        uint8_t bit_index = (uint8_t)(7 - (i % 8));
        uint8_t bit = (uint8_t)((data[byte_index] >> bit_index) & 0x01);
        uint8_t pair_index = (uint8_t)(PARADOX_PREAMBLE_BITS + (i * 2));

        /* Manchester pairs are 01 for zero and 10 for one. */
        frame[pair_index] = bit;
        frame[pair_index + 1] = (uint8_t)!bit;
    }
}

static bool paradox_preamble_valid(const uint8_t *frame) {
    for (uint8_t i = 0; i < PARADOX_PREAMBLE_BITS; i++) {
        uint8_t expected = (uint8_t)((PARADOX_PREAMBLE >> (7 - i)) & 0x01);
        if (frame[i] != expected) {
            return false;
        }
    }
    return true;
}

static bool paradox_decode_frame(paradox_codec *codec) {
    uint8_t decoded[PARADOX_DATA_SIZE] = {0};

    if (codec->encoded_bit_count < PARADOX_BUFFER_BITS ||
        !paradox_preamble_valid(codec->encoded_bits) ||
        !paradox_preamble_valid(codec->encoded_bits + PARADOX_FRAME_BITS)) {
        return false;
    }

    for (uint8_t i = 0; i < PARADOX_DATA_BITS; i++) {
        uint8_t pair_index = (uint8_t)(PARADOX_PREAMBLE_BITS + (i * 2));
        uint8_t pair = (uint8_t)((codec->encoded_bits[pair_index] << 1) |
                                 codec->encoded_bits[pair_index + 1]);
        uint8_t bit;

        if (pair == 0x01) {
            bit = 0;
        } else if (pair == 0x02) {
            bit = 1;
        } else {
            return false;
        }

        decoded[i / 8] |= (uint8_t)(bit << (7 - (i % 8)));
    }

    /* Four low padding bits stay zero, matching the six-byte wire format. */
    memcpy(codec->data, decoded, PARADOX_DATA_SIZE);
    return true;
}

static void paradox_push_bit(paradox_codec *codec, bool bit) {
    if (codec->encoded_bit_count < PARADOX_BUFFER_BITS) {
        codec->encoded_bits[codec->encoded_bit_count++] = (uint8_t)bit;
        return;
    }

    memmove(codec->encoded_bits,
            codec->encoded_bits + 1,
            PARADOX_BUFFER_BITS - 1);
    codec->encoded_bits[PARADOX_BUFFER_BITS - 1] = (uint8_t)bit;
}

static void *paradox_alloc(void) {
    paradox_codec *codec = (paradox_codec *)calloc(1, sizeof(*codec));
    if (codec == NULL) {
        return NULL;
    }

    codec->modem = fsk_alloc(PARADOX_FSK_BITRATE);
    if (codec->modem == NULL) {
        free(codec);
        return NULL;
    }

    return codec;
}

static void paradox_free(void *codec_ptr) {
    paradox_codec *codec = (paradox_codec *)codec_ptr;
    if (codec == NULL) {
        return;
    }

    fsk_free(codec->modem);
    codec->modem = NULL;
    free(codec);
}

static uint8_t *paradox_get_data(void *codec_ptr) {
    paradox_codec *codec = (paradox_codec *)codec_ptr;
    return codec == NULL ? NULL : codec->data;
}

static void paradox_decoder_start(void *codec_ptr, uint8_t format) {
    (void)format;
    paradox_codec *codec = (paradox_codec *)codec_ptr;
    if (codec == NULL) {
        return;
    }

    memset(codec->data, 0, sizeof(codec->data));
    memset(codec->encoded_bits, 0, sizeof(codec->encoded_bits));
    codec->encoded_bit_count = 0;
    if (codec->modem != NULL) {
        codec->modem->c = 0;
        memset(codec->modem->samples, 0, sizeof(codec->modem->samples));
    }
}

static bool paradox_decoder_feed(void *codec_ptr, uint16_t sample) {
    paradox_codec *codec = (paradox_codec *)codec_ptr;
    if (codec == NULL || codec->modem == NULL) {
        return false;
    }

    bool bit = false;
    if (!fsk_feed(codec->modem, sample, &bit)) {
        return false;
    }

    paradox_push_bit(codec, bit);
    return paradox_decode_frame(codec);
}

static void paradox_emit_bit(uint16_t *index, bool bit) {
    uint16_t loop_count = bit ? PARADOX_FSK1_LOOP : PARADOX_FSK0_LOOP;
    uint16_t counter_top = bit ? PARADOX_FSK1_TOP : PARADOX_FSK0_TOP;

    for (uint16_t i = 0; i < loop_count; i++) {
        m_paradox_pwm_seq_vals[*index].channel_0 = counter_top / 2;
        m_paradox_pwm_seq_vals[*index].counter_top = counter_top;
        (*index)++;
    }
}

static nrf_pwm_sequence_t *paradox_modulator(void *codec_ptr, uint8_t *data) {
    (void)codec_ptr;
    uint8_t frame[PARADOX_FRAME_BITS];
    uint16_t index = 0;

    paradox_build_frame(data, frame);
    for (uint8_t i = 0; i < PARADOX_FRAME_BITS; i++) {
        paradox_emit_bit(&index, frame[i] != 0);
    }

    m_paradox_pwm_seq.length = (uint16_t)(index * 4);
    return &m_paradox_pwm_seq;
}

const protocol paradox = {
    .tag_type = TAG_TYPE_PARADOX,
    .data_size = PARADOX_DATA_SIZE,
    .alloc = paradox_alloc,
    .free = paradox_free,
    .get_data = paradox_get_data,
    .modulator = paradox_modulator,
    .decoder = {
        .start = paradox_decoder_start,
        .feed = paradox_decoder_feed,
    },
};

uint8_t paradox_t55xx_writer(uint8_t *data, uint32_t *blks) {
    uint8_t frame[PARADOX_FRAME_BITS];

    paradox_build_frame(data, frame);
    blks[0] = T5577_PARADOX_CONFIG;
    for (uint8_t block = 0; block < PARADOX_T55XX_BLOCK_COUNT - 1; block++) {
        uint32_t word = 0;
        for (uint8_t bit = 0; bit < 32; bit++) {
            word = (word << 1) | frame[(block * 32) + bit];
        }
        blks[block + 1] = word;
    }
    return PARADOX_T55XX_BLOCK_COUNT;
}
