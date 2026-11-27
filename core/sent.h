/*
 * OpenSENT - SAE J2716 SENT protocol core (portable C99, no hardware).
 *
 * Time base: everything the decoder sees is an interval between two
 * consecutive falling edges, in "clocks" of a free-running capture timer
 * whose frequency is given in the configuration. The encoder produces
 * pulse lengths in the same unit for the transmit timer.
 *
 * Frame layout (J2716):
 *   sync (56 ticks) | status (12..27) | data 1..N (12..27) | CRC (12..27) | [pause]
 * A nibble of value v lasts 12 + v ticks. Each pulse starts with a low
 * period of >= 4 ticks; only falling edges carry information.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef OPENSENT_SENT_H
#define OPENSENT_SENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_SYNC_TICKS      56u
#define SENT_NIBBLE_OFFSET   12u
#define SENT_MAX_DATA        8u   /* data nibbles per frame (J2716: 1..6, we allow 8) */
#define SENT_PAUSE_MIN       12u
#define SENT_PAUSE_MAX       768u
#define SENT_SLOW_IDS        32u  /* slow-channel message table size */

/* ------------------------------------------------------------------ CRC */

typedef enum {
    SENT_CRC_2010   = 0, /* J2716 2010+ "recommended": data nibbles, seed 5, augmented with 0 */
    SENT_CRC_LEGACY = 1, /* J2716 pre-2010: data nibbles, seed 5, no augmentation */
    SENT_CRC_STATUS = 2, /* vendor variant: status + data nibbles (e.g. Si7215) */
    SENT_CRC_AUTO   = 3, /* receiver only: try all, lock onto the one that matches */
} sent_crc_mode_t;

uint8_t sent_crc4(sent_crc_mode_t mode, uint8_t status, const uint8_t *data, unsigned n);
/* Enhanced serial message CRC over the 24 bits (bit2,bit3 per frame 7..18) */
uint8_t sent_crc6(uint32_t bits24);
const char *sent_crc_name(sent_crc_mode_t mode);

/* ------------------------------------------------------- fast channels */

/* How data nibbles map to the fast-channel signals (J2716 Appendix A). */
typedef struct {
    uint8_t fc1_nibbles;  /* nibbles of fast channel 1, MSN first */
    uint8_t fc2_nibbles;  /* nibbles of fast channel 2 (0 = none) */
    bool    fc2_reversed; /* FC2 sent LSN first (standard for 12+12 bit, A.3) */
} sent_format_t;

typedef enum {
    SENT_FMT_12_12 = 0, /* two 12-bit channels, FC2 reversed (most throttle/pedal/pressure) */
    SENT_FMT_12_12N,    /* two 12-bit channels, FC2 MSN first */
    SENT_FMT_16_8,      /* 16-bit FC1 + 8-bit FC2 */
    SENT_FMT_12,        /* single 12-bit channel (3 nibbles) */
    SENT_FMT_16,        /* single 16-bit channel (4 nibbles) */
    SENT_FMT_RAW,       /* no interpretation */
    SENT_FMT_COUNT
} sent_format_id_t;

sent_format_t sent_format(sent_format_id_t id);
const char   *sent_format_name(sent_format_id_t id);
bool          sent_format_parse(const char *s, sent_format_id_t *out);

/* nibbles -> fc1/fc2 and back. Returns false if the format needs more
 * nibbles than available. */
bool sent_unpack(const sent_format_t *f, const uint8_t *d, unsigned n, uint32_t *fc1, uint32_t *fc2);
unsigned sent_pack(const sent_format_t *f, uint32_t fc1, uint32_t fc2, uint8_t *d);

/* ----------------------------------------------------- slow channel */

typedef enum {
    SENT_SLOW_NONE  = 0,
    SENT_SLOW_SHORT = 1, /* 16 frames: 4-bit ID, 8-bit data, CRC4 */
    SENT_SLOW_ENH12 = 2, /* 18 frames: 8-bit ID, 12-bit data, CRC6 (C=0) */
    SENT_SLOW_ENH16 = 3, /* 18 frames: 4-bit ID, 16-bit data, CRC6 (C=1) */
} sent_slow_fmt_t;

const char *sent_slow_name(sent_slow_fmt_t f);

/* Bits 3 (MSB) and 2 of the status nibble for frame `idx` of a serial
 * message. Returns the number of frames in the message. bad_crc corrupts
 * the message CRC (fault injection). */
unsigned sent_slow_bits(sent_slow_fmt_t fmt, uint8_t id, uint16_t value,
                        unsigned idx, bool bad_crc, uint8_t *b3, uint8_t *b2);

typedef struct {
    uint8_t  id;
    uint8_t  fmt;      /* sent_slow_fmt_t */
    uint16_t value;
    uint32_t count;    /* times received */
    uint32_t ts;       /* timestamp of last reception (clocks) */
} sent_slow_msg_t;

typedef struct {
    uint32_t sh2, sh3;   /* bit2 / bit3 history, newest in bit 0 */
    uint8_t  nbits;      /* valid bits in the history (saturates at 32) */
    sent_slow_msg_t msg[SENT_SLOW_IDS];
    uint8_t  nmsg;
    uint32_t ok, crc_err;
} sent_slow_rx_t;

void sent_slow_rx_reset(sent_slow_rx_t *s, bool clear_table);
/* feed the status nibble of one valid frame; returns message index (>=0)
 * when a message completed with a good CRC, -1 otherwise. */
int  sent_slow_rx_feed(sent_slow_rx_t *s, uint8_t status, uint32_t ts);

/* --------------------------------------------------------- decoder */

typedef enum {
    SENT_OK = 0,
    SENT_E_CRC,      /* CRC mismatch */
    SENT_E_NIBBLE,   /* a nibble pulse outside 12..27 ticks */
    SENT_E_LENGTH,   /* wrong number of nibbles between syncs */
    SENT_E_SHORT,    /* pulse shorter than 12 ticks */
    SENT_E_SYNC,     /* expected a sync/pause, got something else (frame lost) */
    SENT_E_COUNT
} sent_err_t;

const char *sent_err_name(sent_err_t e);

/* frame flags */
#define SENT_F_PAUSE   0x01u  /* frame was followed by a pause pulse */
#define SENT_F_CAL     0x02u  /* successive sync pulses differ by > 1/64 (J2716 option 2) */
#define SENT_F_MARGIN  0x04u  /* a nibble was within 0.15 tick of a rounding boundary */
#define SENT_F_TICK    0x08u  /* tick outside nominal +/- tolerance (only if nominal set) */

typedef struct {
    uint32_t ts;            /* timestamp (clocks) of the falling edge that started the sync */
    uint8_t  status;
    uint8_t  n;             /* data nibbles */
    uint8_t  d[SENT_MAX_DATA];
    uint8_t  crc;           /* received CRC nibble */
    uint8_t  err;           /* sent_err_t */
    uint8_t  crc_mode;      /* CRC mode that matched (when err == OK) */
    uint8_t  flags;
    uint16_t pause_ticks;   /* 0 = no pause */
    uint16_t max_dev_mt;    /* worst nibble deviation from integer, milli-ticks */
    float    tick_ns;       /* tick measured from this frame's sync */
    uint32_t fc1, fc2;      /* according to the configured format */
} sent_frame_t;

typedef struct {
    uint32_t clk_hz;        /* capture timer frequency */
    float    tick_ns;       /* nominal tick; 0 = auto (accept 1.5..100 us) */
    uint8_t  tol_pct;       /* tick tolerance vs nominal, J2716 receiver: 25 */
    uint8_t  nibbles;       /* data nibbles, 0 = learn */
    uint8_t  crc_mode;      /* sent_crc_mode_t */
    uint8_t  fmt;           /* sent_format_id_t */
    bool     no_pause;      /* sensor sends no pause pulse: extra pulses are errors */
} sent_rx_cfg_t;

void sent_rx_cfg_default(sent_rx_cfg_t *c, uint32_t clk_hz);

typedef struct {
    uint32_t pulses;
    uint32_t frames;        /* frames emitted (good + bad) */
    uint32_t ok;
    uint32_t err[SENT_E_COUNT]; /* per sent_err_t */
    uint32_t pauses;
    uint32_t cal_flags;
    uint32_t margin_flags;
    uint32_t resyncs;       /* lost lock */
    uint32_t glitches;      /* pulses < 2 ticks merged into the next one */
} sent_rx_stats_t;

typedef struct {
    sent_rx_cfg_t cfg;
    sent_format_t fmt;
    /* state */
    bool     have_sync;     /* a sync candidate opened the current segment */
    bool     locked;        /* last frame was good */
    uint32_t sync;          /* clocks of the sync that opened the segment */
    uint32_t prev_sync;     /* previous accepted sync (for SENT_F_CAL) */
    uint32_t sync_ts;
    uint32_t ts;            /* running timestamp */
    uint32_t seg[SENT_MAX_DATA + 4];
    uint8_t  nseg;
    uint8_t  learned_n;     /* nibble count learned in auto mode */
    uint8_t  learn_votes;
    uint8_t  crc_lock;      /* matched crc mode in AUTO, 0xff = none yet */
    uint8_t  crc_votes;
    uint32_t carry;         /* glitch time carried into the next pulse */
    sent_rx_stats_t st;
    sent_slow_rx_t  slow;
} sent_rx_t;

void sent_rx_init(sent_rx_t *rx, const sent_rx_cfg_t *cfg);
void sent_rx_reset(sent_rx_t *rx);
/* Feed one falling-edge-to-falling-edge interval. Returns true and fills
 * *out when a frame (good or bad) was completed. */
bool sent_rx_feed(sent_rx_t *rx, uint32_t clocks, sent_frame_t *out);

/* --------------------------------------------------------- encoder */

typedef enum {
    SENT_FAULT_NONE = 0,
    SENT_FAULT_CRC,     /* wrong CRC nibble */
    SENT_FAULT_SYNC,    /* sync pulse 40 ticks instead of 56 */
    SENT_FAULT_NIBBLE,  /* first data nibble 30 ticks (out of range) */
    SENT_FAULT_SHORT,   /* one data nibble missing */
    SENT_FAULT_LONG,    /* one extra data nibble */
    SENT_FAULT_CAL,     /* whole frame 3 % slower: successive-calibration violation */
    SENT_FAULT_SCRC,    /* next slow-channel message with wrong CRC */
    SENT_FAULT_COUNT
} sent_fault_t;

const char *sent_fault_name(sent_fault_t f);
bool        sent_fault_parse(const char *s, sent_fault_t *out);

typedef enum {
    SENT_WAVE_CONST = 0, SENT_WAVE_RAMP, SENT_WAVE_TRI, SENT_WAVE_SINE,
    SENT_WAVE_SQUARE, SENT_WAVE_COUNT_UP, SENT_WAVE_N
} sent_wave_kind_t;

const char *sent_wave_name(sent_wave_kind_t k);
bool        sent_wave_parse(const char *s, sent_wave_kind_t *out);

typedef struct {
    uint8_t  kind;          /* sent_wave_kind_t */
    uint32_t value;         /* CONST value */
    uint32_t min, max;
    float    period_s;
} sent_wave_t;

#define SENT_TX_SLOW_MAX 16u

typedef enum { SENT_PAUSE_OFF = 0, SENT_PAUSE_FIXED, SENT_PAUSE_CONST } sent_pause_mode_t;

typedef struct {
    float    tick_ns;
    uint8_t  low_ticks;     /* low time of each pulse (>= 4) */
    uint8_t  nibbles;       /* data nibbles */
    uint8_t  crc_mode;      /* 2010 / LEGACY / STATUS */
    uint8_t  fmt;           /* sent_format_id_t */
    uint8_t  status_bits;   /* application status bits 0..1 */
    uint8_t  pause_mode;    /* sent_pause_mode_t */
    uint16_t pause_ticks;   /* FIXED: pause length; CONST: total frame length */
    bool     use_raw;       /* send raw[] instead of fc1/fc2 */
    uint8_t  raw[SENT_MAX_DATA];
    sent_wave_t fc[2];
    uint8_t  slow_fmt;      /* sent_slow_fmt_t */
    uint8_t  slow_n;
    uint8_t  slow_id[SENT_TX_SLOW_MAX];
    uint16_t slow_val[SENT_TX_SLOW_MAX];
} sent_tx_cfg_t;

void sent_tx_cfg_default(sent_tx_cfg_t *c);

typedef struct {
    uint32_t clk_hz;
    sent_tx_cfg_t cfg;         /* live config: snapshotted at each frame start */
    /* fault injection, applied at frame start */
    volatile uint8_t  fault;
    volatile uint32_t fault_count; /* frames left to corrupt (0xffffffff = forever) */
    volatile uint32_t fault_every; /* corrupt every Nth frame (1 = consecutive) */
    uint32_t fault_phase;
    /* frame being streamed */
    uint16_t ticks[SENT_MAX_DATA + 5];
    uint8_t  nticks, pos;
    float    scale;            /* tick multiplier for this frame (CAL fault) */
    double   frac;             /* rounding remainder, keeps average exact */
    double   t_s;              /* elapsed time, drives waveforms */
    uint32_t frames;
    uint32_t faults_injected;
    /* slow channel */
    uint8_t  slow_msg, slow_idx;
    bool     slow_bad, slow_cur_bad;
    /* last frame content, for reporting */
    uint32_t last_fc1, last_fc2;
    uint8_t  last_status, last_n, last_d[SENT_MAX_DATA + 1];
} sent_tx_t;

void     sent_tx_init(sent_tx_t *tx, uint32_t clk_hz, const sent_tx_cfg_t *cfg);
/* Next pulse length in clocks; *low gets the low time in clocks. */
uint32_t sent_tx_next(sent_tx_t *tx, uint32_t *low);
/* Build the tick list of one frame (no timing). Exposed for tests. */
unsigned sent_tx_build_frame(sent_tx_t *tx, uint16_t *ticks);

#ifdef __cplusplus
}
#endif
#endif
