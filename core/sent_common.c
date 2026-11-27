/*
 * OpenSENT - CRCs, fast-channel formats, slow (serial) channel, names.
 * SPDX-License-Identifier: MIT
 */
#include "sent.h"

#include <ctype.h>
#include <string.h>

/* ------------------------------------------------------------------ CRC */

/* c * x^4 mod (x^4 + x^3 + x^2 + 1) */
static uint8_t crc4_mul(uint8_t c)
{
    for (int i = 0; i < 4; i++) {
        c = (uint8_t)(c << 1);
        if (c & 0x10u) {
            c ^= 0x1Du;
        }
    }
    return c & 0x0Fu;
}

/* c * x^6 mod (x^6 + x^4 + x^3 + 1) */
static uint8_t crc6_mul(uint8_t c)
{
    for (int i = 0; i < 6; i++) {
        c = (uint8_t)(c << 1);
        if (c & 0x40u) {
            c ^= 0x59u;
        }
    }
    return c & 0x3Fu;
}

uint8_t sent_crc4(sent_crc_mode_t mode, uint8_t status, const uint8_t *data, unsigned n)
{
    uint8_t c = 5u; /* seed 0101 */
    switch (mode) {
    case SENT_CRC_LEGACY:
        for (unsigned i = 0; i < n; i++) {
            c = crc4_mul(c) ^ (data[i] & 0x0Fu);
        }
        return c;
    case SENT_CRC_STATUS:
        /* status included, input xor-ed before the multiply (Si7215 style) */
        c = crc4_mul(c ^ (status & 0x0Fu));
        for (unsigned i = 0; i < n; i++) {
            c = crc4_mul(c ^ (data[i] & 0x0Fu));
        }
        return c;
    case SENT_CRC_2010:
    case SENT_CRC_AUTO:
    default:
        for (unsigned i = 0; i < n; i++) {
            c = crc4_mul(c) ^ (data[i] & 0x0Fu);
        }
        return crc4_mul(c); /* augmentation with a zero nibble */
    }
}

uint8_t sent_crc6(uint32_t bits24)
{
    uint8_t c = 0x15u; /* seed 010101 */
    for (int i = 3; i >= 0; i--) {
        c = crc6_mul(c) ^ (uint8_t)((bits24 >> (6 * i)) & 0x3Fu);
    }
    return crc6_mul(c); /* augmentation with six zero bits */
}

const char *sent_crc_name(sent_crc_mode_t mode)
{
    static const char *const n[] = {"2010", "LEGACY", "STATUS", "AUTO"};
    return (unsigned)mode < 4u ? n[mode] : "?";
}

/* ------------------------------------------------------- fast channels */

static const struct {
    const char   *name;
    sent_format_t f;
} formats[SENT_FMT_COUNT] = {
    [SENT_FMT_12_12]  = {"12_12", {3, 3, true}},
    [SENT_FMT_12_12N] = {"12_12N", {3, 3, false}},
    [SENT_FMT_16_8]   = {"16_8", {4, 2, false}},
    [SENT_FMT_12]     = {"12", {3, 0, false}},
    [SENT_FMT_16]     = {"16", {4, 0, false}},
    [SENT_FMT_RAW]    = {"RAW", {0, 0, false}},
};

sent_format_t sent_format(sent_format_id_t id)
{
    if ((unsigned)id >= SENT_FMT_COUNT) {
        id = SENT_FMT_RAW;
    }
    return formats[id].f;
}

const char *sent_format_name(sent_format_id_t id)
{
    return (unsigned)id < SENT_FMT_COUNT ? formats[id].name : "?";
}

static bool str_ieq(const char *a, const char *b)
{
    while (*a && *b) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
            return false;
        }
        a++;
        b++;
    }
    return *a == *b;
}

bool sent_format_parse(const char *s, sent_format_id_t *out)
{
    for (unsigned i = 0; i < SENT_FMT_COUNT; i++) {
        if (str_ieq(s, formats[i].name)) {
            *out = (sent_format_id_t)i;
            return true;
        }
    }
    return false;
}

bool sent_unpack(const sent_format_t *f, const uint8_t *d, unsigned n, uint32_t *fc1, uint32_t *fc2)
{
    uint32_t a = 0, b = 0;
    if ((unsigned)f->fc1_nibbles + f->fc2_nibbles > n) {
        *fc1 = *fc2 = 0;
        return false;
    }
    for (unsigned i = 0; i < f->fc1_nibbles; i++) {
        a = (a << 4) | d[i];
    }
    for (unsigned i = 0; i < f->fc2_nibbles; i++) {
        uint8_t v = d[f->fc1_nibbles + i];
        if (f->fc2_reversed) {
            b |= (uint32_t)v << (4 * i);
        } else {
            b = (b << 4) | v;
        }
    }
    *fc1 = a;
    *fc2 = b;
    return true;
}

unsigned sent_pack(const sent_format_t *f, uint32_t fc1, uint32_t fc2, uint8_t *d)
{
    unsigned k = 0;
    for (int i = f->fc1_nibbles - 1; i >= 0; i--) {
        d[k++] = (uint8_t)((fc1 >> (4 * i)) & 0xFu);
    }
    for (unsigned i = 0; i < f->fc2_nibbles; i++) {
        unsigned sh = f->fc2_reversed ? i : (unsigned)(f->fc2_nibbles - 1 - i);
        d[k++] = (uint8_t)((fc2 >> (4 * sh)) & 0xFu);
    }
    return k;
}

/* ----------------------------------------------------- slow channel */

const char *sent_slow_name(sent_slow_fmt_t f)
{
    static const char *const n[] = {"OFF", "SHORT", "ENH12", "ENH16"};
    return (unsigned)f < 4u ? n[f] : "?";
}

static uint32_t enh_crc_input(uint16_t b2_12, uint16_t b3_12)
{
    /* frames 7..18, MSB first: (bit2, bit3) pairs */
    uint32_t x = 0;
    for (int i = 11; i >= 0; i--) {
        x = (x << 2) | (((b2_12 >> i) & 1u) << 1) | ((b3_12 >> i) & 1u);
    }
    return x;
}

unsigned sent_slow_bits(sent_slow_fmt_t fmt, uint8_t id, uint16_t value,
                        unsigned idx, bool bad_crc, uint8_t *b3, uint8_t *b2)
{
    if (fmt == SENT_SLOW_SHORT) {
        uint8_t nib[3] = {(uint8_t)(id & 0xFu), (uint8_t)((value >> 4) & 0xFu), (uint8_t)(value & 0xFu)};
        uint8_t crc = sent_crc4(SENT_CRC_2010, 0, nib, 3);
        if (bad_crc) {
            crc ^= 0x1u;
        }
        uint16_t word = (uint16_t)(((id & 0xFu) << 12) | ((value & 0xFFu) << 4) | crc);
        *b3 = idx == 0;
        *b2 = (word >> (15 - idx)) & 1u;
        return 16;
    }
    if (fmt == SENT_SLOW_ENH12 || fmt == SENT_SLOW_ENH16) {
        /* bit 3 of frames 7..18 (12 bits, frame 7 = MSB) */
        bool     c16  = fmt == SENT_SLOW_ENH16;
        uint16_t hi4  = c16 ? (id & 0xFu) : ((id >> 4) & 0xFu);
        uint16_t lo4  = c16 ? ((value >> 12) & 0xFu) : (id & 0xFu);
        uint16_t b3w  = (uint16_t)((0u << 11) | ((unsigned)c16 << 10) | (hi4 << 6) | (0u << 5) | (lo4 << 1));
        uint16_t b2w  = value & 0x0FFFu;
        uint8_t  crc  = sent_crc6(enh_crc_input(b2w, b3w));
        if (bad_crc) {
            crc ^= 0x1u;
        }
        if (idx < 6) {
            *b3 = 1;
            *b2 = (crc >> (5 - idx)) & 1u;
        } else {
            unsigned k = 17 - idx; /* bit position in the 12-bit words */
            *b3 = (b3w >> k) & 1u;
            *b2 = (b2w >> k) & 1u;
        }
        return 18;
    }
    *b3 = *b2 = 0;
    return 0;
}

void sent_slow_rx_reset(sent_slow_rx_t *s, bool clear_table)
{
    s->sh2 = s->sh3 = 0;
    s->nbits = 0;
    if (clear_table) {
        memset(s->msg, 0, sizeof s->msg);
        s->nmsg = 0;
        s->ok = s->crc_err = 0;
    }
}

static int slow_store(sent_slow_rx_t *s, sent_slow_fmt_t fmt, uint8_t id, uint16_t value, uint32_t ts)
{
    bool enh = fmt != SENT_SLOW_SHORT;
    for (unsigned i = 0; i < s->nmsg; i++) {
        bool menh = s->msg[i].fmt != SENT_SLOW_SHORT;
        if (s->msg[i].id == id && menh == enh) {
            s->msg[i].fmt   = (uint8_t)fmt;
            s->msg[i].value = value;
            s->msg[i].count++;
            s->msg[i].ts = ts;
            return (int)i;
        }
    }
    if (s->nmsg >= SENT_SLOW_IDS) {
        return -1;
    }
    sent_slow_msg_t *m = &s->msg[s->nmsg];
    m->id    = id;
    m->fmt   = (uint8_t)fmt;
    m->value = value;
    m->count = 1;
    m->ts    = ts;
    return (int)s->nmsg++;
}

int sent_slow_rx_feed(sent_slow_rx_t *s, uint8_t status, uint32_t ts)
{
    s->sh2 = (s->sh2 << 1) | ((status >> 2) & 1u);
    s->sh3 = (s->sh3 << 1) | ((status >> 3) & 1u);
    if (s->nbits < 32) {
        s->nbits++;
    }

    /* short serial: bit3 = 1 then fifteen 0 */
    if (s->nbits >= 16 && (s->sh3 & 0xFFFFu) == 0x8000u) {
        uint16_t w    = (uint16_t)(s->sh2 & 0xFFFFu);
        uint8_t  id   = (uint8_t)(w >> 12);
        uint16_t val  = (w >> 4) & 0xFFu;
        uint8_t  crc  = w & 0xFu;
        uint8_t  nib[3] = {id, (uint8_t)(val >> 4), (uint8_t)(val & 0xFu)};
        if (crc == sent_crc4(SENT_CRC_2010, 0, nib, 3) || crc == sent_crc4(SENT_CRC_LEGACY, 0, nib, 3)) {
            s->ok++;
            return slow_store(s, SENT_SLOW_SHORT, id, val, ts);
        }
        s->crc_err++;
        return -1;
    }

    /* enhanced serial: bit3 = 111111 0 C xxxx 0 xxxx 0 */
    if (s->nbits >= 18 && (s->sh3 & 0x3F821u) == 0x3F000u) {
        uint16_t b3w = (uint16_t)(s->sh3 & 0xFFFu);
        uint16_t b2w = (uint16_t)(s->sh2 & 0xFFFu);
        uint8_t  crc = (uint8_t)((s->sh2 >> 12) & 0x3Fu);
        if (crc != sent_crc6(enh_crc_input(b2w, b3w))) {
            s->crc_err++;
            return -1;
        }
        bool    c16 = (b3w >> 10) & 1u;
        uint8_t hi4 = (b3w >> 6) & 0xFu;
        uint8_t lo4 = (b3w >> 1) & 0xFu;
        s->ok++;
        if (c16) {
            return slow_store(s, SENT_SLOW_ENH16, hi4, (uint16_t)((lo4 << 12) | b2w), ts);
        }
        return slow_store(s, SENT_SLOW_ENH12, (uint8_t)((hi4 << 4) | lo4), b2w, ts);
    }
    return -1;
}

/* ------------------------------------------------------------- names */

const char *sent_err_name(sent_err_t e)
{
    static const char *const n[] = {"OK", "CRC", "NIBBLE", "LENGTH", "SHORT", "SYNC"};
    return (unsigned)e < SENT_E_COUNT ? n[e] : "?";
}

static const char *const fault_names[SENT_FAULT_COUNT] = {
    "OFF", "CRC", "SYNC", "NIBBLE", "SHORT", "LONG", "CAL", "SCRC",
};

const char *sent_fault_name(sent_fault_t f)
{
    return (unsigned)f < SENT_FAULT_COUNT ? fault_names[f] : "?";
}

bool sent_fault_parse(const char *s, sent_fault_t *out)
{
    for (unsigned i = 0; i < SENT_FAULT_COUNT; i++) {
        if (str_ieq(s, fault_names[i])) {
            *out = (sent_fault_t)i;
            return true;
        }
    }
    return false;
}

static const char *const wave_names[SENT_WAVE_N] = {
    "CONST", "RAMP", "TRI", "SINE", "SQUARE", "COUNT",
};

const char *sent_wave_name(sent_wave_kind_t k)
{
    return (unsigned)k < SENT_WAVE_N ? wave_names[k] : "?";
}

bool sent_wave_parse(const char *s, sent_wave_kind_t *out)
{
    for (unsigned i = 0; i < SENT_WAVE_N; i++) {
        if (str_ieq(s, wave_names[i])) {
            *out = (sent_wave_kind_t)i;
            return true;
        }
    }
    return false;
}
