# SENT (SAE J2716): implementation notes

Practical notes collected while building and validating OpenSENT against real sensor captures. They cover the details that public descriptions of the protocol tend to skip.

## Timing

- **Tick:** 3 µs is the nominal value. J2716 allows 3–90 µs, and the transmitter clock may be off by up to ±20 %, so a receiver measures every frame against its own sync pulse.
- **Sync / calibration pulse:** 56 ticks. **Nibble:** 12 + value ticks (12–27). **Pause:** 12–768 ticks, optional. It is either fixed, or it varies so that every frame has the same total length.
- **Low time:** every pulse starts with a low period of at least 4 ticks (5 is typical). Information is only in the **falling-edge-to-falling-edge** times. A receiver must not depend on the low time.
- **Successive calibration check** (J2716 option 2): if two consecutive sync pulses differ by more than 1/64 (1.5625 %), the frame is suspect. OpenSENT sets flag `CAL` and still decodes the frame, because each frame is self-calibrated.
- **Nibble margin:** after scaling by the frame's own sync, the measured nibble length should be within ±0.5 tick of an integer. OpenSENT reports the worst deviation per frame (`dev_mt`, milli-ticks) and flags frames above 0.35 tick (`MARGIN`). That is a useful health indicator for a sensor or a harness.

## Frame

```
sync | status | D1 .. Dn | CRC | [pause]
```

The **status nibble** carries application bits 0–1, plus bit 2 (serial data) and bit 3 (serial sync) of the slow channel.

**Data layout:** the most common layout is two 12-bit fast channels (throttle, pedal and pressure sensors). FC1 goes in D1–D3, most significant nibble first. **FC2 goes in D4–D6, least significant nibble first.** It is easy to get this wrong and decode FC2 as garbage that happens to look plausible. OpenSENT supports `12_12` (standard), `12_12N` (FC2 sent MSN first), `16_8`, `12`, `16` and `RAW`.

## CRC4: three variants in the wild

All of them use seed `0101` and polynomial x⁴+x³+x²+1, with table `0,13,7,10,14,3,9,4,1,12,6,11,15,2,8,5`.

| Variant | Algorithm | Seen on |
|---|---|---|
| **2010** (recommended) | `c = 5; for d in data: c = T[c] ^ d;` then one more `c = T[c]` (augmentation with a zero nibble) | GM fuel pressure, VAG MAP, Mercedes EGT, Ford throttle |
| **Legacy** (pre-2010) | same, without the final augmentation | Opel / GM throttle bodies |
| **Status included** | `c = 5; for d in [status] + data: c = T[c ^ d]` | some Hall sensors (e.g. Si7215) |

The status-included variant gives exactly the 2010 result when the status nibble is 0. Telling them apart therefore needs frames with a non-zero status nibble. In AUTO mode OpenSENT tries all three, then locks onto the matching variant after 4 consistent frames, so it does not keep a 3/16 false-accept rate.

## Slow channel (serial messages)

**Short serial message**, 16 frames:

- Bit 3 is `1` in the first frame and `0` in the next 15.
- Bit 2 carries 16 bits: a 4-bit ID, 8 bits of data, and a CRC4 over the three nibbles (ID, data high, data low).

**Enhanced serial message**, 18 frames:

```
frame:  1 2 3 4 5 6 | 7 | 8 | 9 10 11 12 | 13 | 14 15 16 17 | 18
bit 3:  1 1 1 1 1 1 | 0 | C |  4 bits    | 0  |   4 bits    | 0
bit 2:  CRC6 (6 bits, MSB first) | 12 data bits (frames 7..18)
```

- `C = 0`: 8-bit ID (frames 9–12 = ID[7:4], 14–17 = ID[3:0]) and 12-bit data.
- `C = 1`: 4-bit ID (frames 9–12) and 16-bit data (frames 14–17 = data[15:12]).
- **CRC6:** polynomial x⁶+x⁴+x³+1 (0x59), seed `010101`. It runs over the **24 interleaved bits (bit 2, bit 3) of frames 7–18**, frame 7 first, then 6 zero bits of augmentation.

The zeros in frames 7, 13 and 18 make the `111111 0` pattern impossible inside the data, so a receiver can synchronise on any frame. Most production sensors use the enhanced format. The IDs 0x90–0x97 seen on the VAG MAP sensor are manufacturer and sensor identification codes; 0x01 is commonly a diagnostic status code.

## Decoding pitfalls handled by OpenSENT

| Situation | Problem | Solution |
|---|---|---|
| Pause of ~56 ticks | looks like a sync | the pulse after it is too long to be a status nibble, so it is the real sync |
| Pause longer than a sync (VAG: 140, Ford: up to 100 ticks, constant frame length) | a naive hunter takes the pause as a sync candidate | while hunting, a long pulse after ≥ 3 valid nibbles first closes the frame as a pause. A short pulse after a wrong candidate replays the buffered pulses |
| Pause of 12–27 ticks | indistinguishable from one more nibble | try both readings with the CRC. With a constant signal both can pass (1 in 16): prefer the J2716-typical 6 nibbles, or configure `RXn:NIB` |
| Edge ringing / glitches (Ford capture) | a spurious edge splits a nibble in two | once locked, any pulse shorter than 11 ticks is merged into the next one |
| Lost sync pulse | the decoder must not swallow two frames | with a known length, the slot after the CRC is the pause. A further pulse emits the complete frame and counts a `SYNC` error |
| Sensor clock off by ±18 % | fixed-tick decoders fail | every frame is scaled by its own sync |

## Interfacing

See [hardware.md](hardware.md). In short:

- The line is 5 V with a pull-up on the receiver side and RC edge shaping at the sensor.
- Divide the line down for a 3.3 V MCU.
- Drive it open drain to emulate a sensor.
