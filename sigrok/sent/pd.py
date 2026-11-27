##
## OpenSENT - SAE J2716 SENT protocol decoder for libsigrokdecode / PulseView.
##
## SPDX-License-Identifier: MIT
##

import sigrokdecode as srd

from .sentcodec import (CRC_2010, CRC_AUTO, CRC_LEGACY, CRC_NAMES, CRC_STATUS, Decoder as SentDecoder,
                        ERR_NAMES, F_CAL, F_MARGIN, FORMATS, OK, E_CRC)


class SamplerateError(Exception):
    pass


A_SYNC, A_STATUS, A_DATA, A_CRC_OK, A_CRC_ERR, A_PAUSE, A_FC1, A_FC2, A_FRAME, A_SLOW, A_ERROR, A_WARN = range(12)

_CRC = {'auto': CRC_AUTO, '2010': CRC_2010, 'legacy': CRC_LEGACY, 'status': CRC_STATUS}


class Decoder(srd.Decoder):
    api_version = 3
    id = 'sent'
    name = 'SENT'
    longname = 'Single Edge Nibble Transmission (SAE J2716)'
    desc = 'Automotive sensor protocol: fast-channel nibbles and slow serial messages.'
    license = 'mit'
    inputs = ['logic']
    outputs = []
    tags = ['Automotive']
    channels = (
        {'id': 'data', 'name': 'DATA', 'desc': 'SENT line'},
    )
    options = (
        {'id': 'tick', 'desc': 'Nominal tick in us (0 = auto)', 'default': 0.0},
        {'id': 'nibbles', 'desc': 'Data nibbles (0 = learn)', 'default': 0},
        {'id': 'crc', 'desc': 'CRC variant', 'default': 'auto', 'values': ('auto', '2010', 'legacy', 'status')},
        {'id': 'format', 'desc': 'Fast channel layout', 'default': '12_12', 'values': tuple(FORMATS)},
        {'id': 'polarity', 'desc': 'Line polarity', 'default': 'normal', 'values': ('normal', 'inverted')},
    )
    annotations = (
        ('sync', 'Sync / calibration pulse'),
        ('status', 'Status nibble'),
        ('data', 'Data nibble'),
        ('crc-ok', 'CRC OK'),
        ('crc-err', 'CRC error'),
        ('pause', 'Pause pulse'),
        ('fc1', 'Fast channel 1'),
        ('fc2', 'Fast channel 2'),
        ('frame', 'Frame'),
        ('slow', 'Slow serial message'),
        ('error', 'Error'),
        ('warning', 'Warning'),
    )
    annotation_rows = (
        ('pulses', 'Pulses', (A_SYNC, A_STATUS, A_DATA, A_CRC_OK, A_CRC_ERR, A_PAUSE)),
        ('signals', 'Signals', (A_FC1, A_FC2, A_FRAME)),
        ('serial', 'Slow channel', (A_SLOW,)),
        ('errors', 'Errors', (A_ERROR, A_WARN)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.dec = None

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putp(self, p, cls, texts):
        ss, es = int(p[0]), int(p[1])
        self.put(ss, max(es, ss + 1), self.out_ann, [cls, texts])

    def emit(self, f):
        pulses = f.pulses
        if not pulses:
            return
        first, last = pulses[0][0], pulses[-1][1]
        us = 1e6 / self.samplerate
        self.putp(pulses[0], A_SYNC, ['Sync, tick %.3f us' % (f.tick_ns / 1000.0), 'Sync', 'S'])
        if f.err not in (OK, E_CRC) or len(pulses) < 3:
            self.put(int(first), int(last), self.out_ann,
                     [A_ERROR, ['%s error' % ERR_NAMES[f.err], ERR_NAMES[f.err], 'E']])
            return
        body = pulses[1:1 + 1 + len(f.data) + 1]
        self.putp(body[0], A_STATUS, ['Status 0x%X (serial b3=%d b2=%d)' % (f.status, f.status >> 3 & 1,
                                                                            f.status >> 2 & 1),
                                       'St %X' % f.status, '%X' % f.status])
        for i, d in enumerate(f.data):
            self.putp(body[1 + i], A_DATA, ['Data %d: 0x%X' % (i + 1, d), 'D%d %X' % (i + 1, d), '%X' % d])
        crc_p = body[-1]
        if f.err == OK:
            self.putp(crc_p, A_CRC_OK, ['CRC 0x%X OK (%s)' % (f.crc, CRC_NAMES.get(f.crc_mode, '?')),
                                        'CRC OK', 'C'])
        else:
            self.putp(crc_p, A_CRC_ERR, ['CRC 0x%X error' % f.crc, 'CRC ERR', 'E'])
            self.put(int(first), int(crc_p[1]), self.out_ann, [A_ERROR, ['CRC error', 'CRC', 'E']])
        if f.pause_ticks and len(pulses) > len(body) + 1:
            self.putp(pulses[-1], A_PAUSE, ['Pause %d ticks' % f.pause_ticks, 'Pause', 'P'])
        if f.err == OK:
            n1, n2, _ = FORMATS[self.fmt]
            if n1 and len(f.data) >= n1:
                self.put(int(body[1][0]), int(body[n1][1]), self.out_ann,
                         [A_FC1, ['FC1 %d (0x%X)' % (f.fc1, f.fc1), 'FC1 %d' % f.fc1, '%d' % f.fc1]])
            if n2 and len(f.data) >= n1 + n2:
                self.put(int(body[1 + n1][0]), int(body[n1 + n2][1]), self.out_ann,
                         [A_FC2, ['FC2 %d (0x%X)' % (f.fc2, f.fc2), 'FC2 %d' % f.fc2, '%d' % f.fc2]])
            if f.slow:
                m = f.slow
                self.put(int(first), int(last), self.out_ann,
                         [A_SLOW, ['%s serial ID 0x%02X = %d (0x%X)' % (m.fmt_name, m.id, m.value, m.value),
                                   'ID %X = %d' % (m.id, m.value), '%X' % m.id]])
        if f.flags & F_CAL:
            self.put(int(first), int(pulses[0][1]), self.out_ann,
                     [A_WARN, ['Successive calibration pulses differ > 1.5625 %', 'Cal drift', 'W']])
        if f.flags & F_MARGIN:
            self.put(int(first), int(last), self.out_ann,
                     [A_WARN, ['Nibble timing margin low (%.2f tick)' % f.max_dev, 'Margin', 'W']])
        del us

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')
        self.fmt = self.options['format']
        self.dec = SentDecoder(clk_hz=self.samplerate, tick_ns=float(self.options['tick']) * 1000.0,
                               nibbles=int(self.options['nibbles']), crc_mode=_CRC[self.options['crc']],
                               fmt=self.fmt)
        edge = 'f' if self.options['polarity'] == 'normal' else 'r'
        while True:
            self.wait({0: edge})
            for f in self.dec.feed_edge(self.samplenum):
                self.emit(f)
