# OpenSENT protocol reference (v0.1)

The instrument shows up as a USB CDC serial port; the baud rate does not matter. It reports USB product `OpenSENT` (VID:PID 0483:5740). The host simulator (`firmware/sim`) speaks the same protocol on a pseudo terminal.

## Framing

- Commands are ASCII lines that end in `\n` or `\r`. The maximum length is 200 characters, and longer lines get `-363`.
- Every non-empty line gets **exactly one reply line**:
  - `OK` when a set command succeeds;
  - the value when the line is a query;
  - `ERR <code>,"<message>"` on an error.
- Several commands can share a line, separated by `;`. Their replies are joined with `;`, and the first error stops the line.
- Keywords are not case sensitive. Numbers use C syntax (`12.5`, `0x23`). Booleans are `ON`/`OFF`/`1`/`0`.
- **Asynchronous records** start with `#`, so a host can always tell them apart from replies. The only one today is the streamed frame (`#F`).

| Code | Meaning |
|---|---|
| -104 | data type error (not a number) |
| -109 | missing parameter |
| -113 | undefined header |
| -221 | settings conflict (e.g. capture on a disabled channel) |
| -222 | data out of range |
| -224 | illegal parameter value |
| -363 | input buffer overrun |

## Common

| Command | Reply |
|---|---|
| `*IDN?` | `OpenSENT,<board>,<serial>,<fw>` |
| `*RST` | everything off and back to defaults (TX off, receivers off, bridges off, DACs 0 V) |
| `*OPC?` | `1` |
| `HELP?` | comma-separated command list |
| `PINS?` | pin map of this board |
| `SYST:ERR?` | last error (`0,"No error"` when none), then clears it |
| `SYST:BOOT` | reboots into the STM32 ROM DFU bootloader (USB firmware update without ST-Link) |

## Transmitter (sensor emulator)

Configuration takes effect at the next frame boundary, and the generator never produces a torn frame.

| Command | Range / values | Default |
|---|---|---|
| `TX:STATE ON\|OFF` | starts the TX pin; the line idles high when off | OFF |
| `TX:TICK <us>` | 0.5 … 100 | 3 |
| `TX:LOW <ticks>` | low time of every pulse, 4 … 11 | 5 |
| `TX:NIB <n>` | data nibbles 1 … 8 | 6 |
| `TX:CRC 2010\|LEGACY\|STATUS` | CRC variant | 2010 |
| `TX:FMT 12_12\|12_12N\|16_8\|12\|16\|RAW` | fast-channel layout | 12_12 |
| `TX:STATUS <0..3>` | application bits 0–1 of the status nibble | 0 |
| `TX:FC1 <v>` / `TX:FC2 <v>` | constant fast-channel values; the query returns the value last sent | 0x800 / 0x7FF |
| `TX:WAVE <ch>,<kind>,<min>,<max>[,<period_s>]` | kind = `CONST`, `RAMP`, `TRI`, `SINE`, `SQUARE` or `COUNT` (+1 per frame) | |
| `TX:RAW <hex>` | raw data nibbles, e.g. `TX:RAW 5A3C96`; this also sets the nibble count. `TX:RAW OFF` | OFF |
| `TX:PAUSE OFF\|<ticks>\|CONST,<frame_ticks>` | fixed pause of 12–768 ticks, or constant frame length (100–1000) | OFF |
| `TX:SLOW OFF\|SHORT\|ENH12\|ENH16` | serial message format on status bits 2–3 | OFF |
| `TX:SLOW:ADD <id>,<value>` / `TX:SLOW:CLR` | up to 16 messages, sent round robin | |
| `TX:FAULT <type>[,<count>[,<every>]]` | inject `<count>` faults (0 = forever), one every `<every>` frames | |
| `TX:FAULT OFF` / `TX:FAULT?` | `type,left,every,injected=N` | |
| `TX:DRIVE PP\|OD` | push-pull 3.3 V, or open drain for an external 5 V pull-up | PP |
| `TX:INFO?` | `state= frames= faults= t_s= status= data= crc= fc1= fc2= tick_ns=` | |

Fault types:

| Fault | What is sent | What a receiver should report |
|---|---|---|
| `CRC` | CRC nibble + 1 | CRC error |
| `SYNC` | sync pulse of 40 ticks | lost frame (`sync`) |
| `NIBBLE` | first data nibble lasts 30 ticks | nibble out of range |
| `SHORT` | one data nibble missing | length error |
| `LONG` | one extra data nibble | length error (needs `RXn:PAUSE NONE`) or CRC error |
| `CAL` | whole frame 3 % slower | successive-calibration warning on two frames; data still valid |
| `SCRC` | next serial message with a wrong CRC | serial CRC error |

## Receivers

`n` = `0` (PA0), `1` (PA1), `2` (monitor of the TX pin). A bare `RX:` means `RX0:`.

| Command | Description |
|---|---|
| `RXn:STATE ON\|OFF` | enable capture and decoding |
| `RXn:POL NORM\|INV` | inverted input (e.g. behind a transistor stage) |
| `RXn:TICK <us>\|AUTO` | nominal tick; with a value, syncs outside ±25 % are rejected and frames outside ±20 % get the `TICK` flag |
| `RXn:NIB <n>\|AUTO` | data nibbles, or learn them from the stream |
| `RXn:CRC AUTO\|2010\|LEGACY\|STATUS` | AUTO tries all three, then locks after 4 consistent frames |
| `RXn:FMT …` | fast-channel layout used for `fc1` / `fc2` |
| `RXn:PAUSE AUTO\|NONE` | NONE: the sensor never sends a pause, so an extra pulse is an error |
| `RXn:CLR` | reset statistics and the learned state |
| `RXn:LAST?` | last frame: `ch= ts_us= err= status= data= crc= crc_mode= fc1= fc2= tick_ns= pause= flags= dev_mt=` |
| `RXn:VAL?` | last good frame: `fc1,fc2,status,age_ms` |
| `RXn:STAT?` | counters (see below) |
| `RXn:SLOW?` | serial messages received: `ENH12:35=786,ENH12:1=0` or `NONE` |
| `RXn:RAW? [count]` | the last ≤ 64 falling-edge intervals in ns |
| `RXn:CAP <edges>` | arm a raw two-edge capture (2 … 8192 edges); decoding pauses on that channel meanwhile |
| `RXn:CAP?` | `state=IDLE\|RUN\|DONE ch= n= target= level0= clk=` |
| `RXn:CAP:DATA? <offset>,<count>` | up to 100 edge-to-edge intervals in timer clocks; `END` past the end |
| `RXn:PULLTEST <frames>` | RX0/RX1 only, pin left **unconnected**. Bit-bangs frames on the pin through its internal pull-up / pull-down (tick 5 µs, FC1 counting from 0, FC2 = 0x5A5), to test the real capture path without wires |

`RXn:STAT?` fields: `en lock frames ok crc nibble length short sync pauses cal margin glitches resyncs pulses overruns n crc_mode tick_ns slow_ok slow_err`.

Frame `flags` (bit mask): `1` followed by a pause, `2` successive sync pulses differ by more than 1/64, `4` a nibble came within 0.15 tick of a rounding boundary, `8` the tick is more than 20 % from the nominal value. `dev_mt` is the worst nibble deviation from an integer tick count, in milli-ticks.

## Streaming

`STREAM <mask>[,<every>]` sends a record for every decoded frame of the channels in `mask` (bit 0 = RX0, bit 1 = RX1, bit 2 = RX2). With `every`, only every Nth good frame is sent; bad frames are always sent. `STREAM 0` stops it, and `STREAM?` returns `mask,every,dropped=`.

```text
#F <ch>,<ts_us>,<err>,<status hex>,<data hex>,<crc hex>,<fc1>,<fc2>,<tick_ns>,<flags>,<pause_ticks>
#F 2,1003541.9,OK,0,61CFF7,6,1564,2047,3000.0,0,0
```

At a 3 µs tick a sensor sends about 1600 frames/s. That fits in a USB FS link, and `every` keeps the load down for GUIs.

## SENT → analog bridge

`n` = 1 (PA4) or 2 (PA5).

| Command | Description |
|---|---|
| `BRIDGEn:MAP RX<ch>,FC1\|FC2,<in_min>,<in_max>,<v_min>,<v_max>` | linear map with clamping; inverse slopes are allowed |
| `BRIDGEn:FAIL <volts>,<timeout_ms>` | output used when no good frame arrived for `timeout_ms` (default 0 V, 100 ms) |
| `BRIDGEn:STATE ON\|OFF` | |
| `BRIDGEn:OUT?` | `<commanded V>,OK\|FAIL\|OFF,<measured V>` (measured on chip through ADC2) |
| `DACn <volts>` / `DACn?` / `DACn:MEAS?` | manual DAC (refused while the bridge drives it) and its read-back |

The read-back is ratiometric: DAC and ADC share VREF+. It therefore checks the mapping and the output stage, not the absolute accuracy of the 3.3 V reference.
