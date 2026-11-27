/*
 * sent_replay - decode a logic-analyzer CSV export (time[s],level) with the
 * C core and print one line per frame. Used to cross-check the Python
 * decoder and as a quick offline tool.
 *
 *   sent_replay capture.csv [nibbles] [fmt]
 *
 * SPDX-License-Identifier: MIT
 */
#include "sent.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s capture.csv [nibbles] [fmt]\n", argv[0]);
        return 2;
    }
    FILE *fp = fopen(argv[1], "r");
    if (!fp) {
        perror(argv[1]);
        return 1;
    }
    sent_rx_cfg_t rc;
    sent_rx_cfg_default(&rc, 1000000000u);
    if (argc > 2) {
        rc.nibbles = (uint8_t)atoi(argv[2]);
    }
    if (argc > 3) {
        sent_format_id_t f;
        if (sent_format_parse(argv[3], &f)) {
            rc.fmt = (uint8_t)f;
        }
    }
    static sent_rx_t rx;
    sent_rx_init(&rx, &rc);

    char line[256];
    double t, prev_t = -1;
    int v, prev_v = -1;
    sent_frame_t fr;
    if (!fgets(line, sizeof line, fp)) {
        return 1;
    }
    printf("ts_us,err,status,data,crc,fc1,fc2,tick_ns,flags,pause\n");
    while (fgets(line, sizeof line, fp)) {
        if (sscanf(line, "%lf , %d", &t, &v) != 2) {
            continue;
        }
        if (v == 0 && prev_v == 1) {
            if (prev_t >= 0 && sent_rx_feed(&rx, (uint32_t)llround((t - prev_t) * 1e9), &fr)) {
                printf("%.3f,%s,%X,", fr.ts / 1000.0, sent_err_name((sent_err_t)fr.err), fr.status);
                for (unsigned i = 0; i < fr.n; i++) {
                    printf("%X", fr.d[i]);
                }
                printf(",%X,%u,%u,%.1f,%u,%u\n", fr.crc, fr.fc1, fr.fc2, (double)fr.tick_ns, fr.flags,
                       fr.pause_ticks);
            }
            prev_t = t;
        }
        prev_v = v;
    }
    fclose(fp);
    fprintf(stderr, "frames %u ok %u crc %u nibble %u length %u short %u sync %u pauses %u slow %u/%u\n",
            rx.st.frames, rx.st.ok, rx.st.err[SENT_E_CRC], rx.st.err[SENT_E_NIBBLE], rx.st.err[SENT_E_LENGTH],
            rx.st.err[SENT_E_SHORT], rx.st.err[SENT_E_SYNC], rx.st.pauses, rx.slow.ok, rx.slow.crc_err);
    for (unsigned j = 0; j < rx.slow.nmsg; j++) {
        fprintf(stderr, "slow %s id 0x%02X = 0x%04X x%u\n", sent_slow_name(rx.slow.msg[j].fmt), rx.slow.msg[j].id,
                rx.slow.msg[j].value, rx.slow.msg[j].count);
    }
    return 0;
}
