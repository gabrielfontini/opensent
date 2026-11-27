# Hardware

## Board

WeAct Studio **STM32H750VBT6** core board, as sold on AliExpress and elsewhere: 25 MHz crystal, USB-C on OTG FS, SWD header, LED on PE3. The firmware runs the core at 400 MHz (VOS1). TIM2 and TIM5 run at 200 MHz, which gives 5 ns resolution for both generation and capture. Flash usage is about 60 KB of the 128 KB.

Flashing:

- **ST-Link:** `st-flash --reset write build-h750/opensent.bin 0x08000000`
- **USB (no ST-Link):** send `SYST:BOOT` (or hold BOOT0 and press reset) to enter the STM32 ROM DFU
  bootloader, then `dfu-util -a 0 -s 0x08000000 -D build-h750/opensent.bin` and **press RESET**.
  Do not use the `:leave` suffix: on this chip the ROM bootloader bus-faults when it jumps to the
  application after a DFU leave request (observed with a debugger: precise bus fault inside the ROM),
  so the board just stays dark until reset. Only flash alt 0 (internal flash), never option bytes.

## Pins

| Signal | Pin | Notes |
|---|---|---|
| TX | PB10 | 5 V tolerant (FT). Push-pull 3.3 V by default, or `TX:DRIVE OD` for open drain |
| RX0 | PA0 | input capture, internal pull-up, 40 ns digital filter |
| RX1 | PA1 | same |
| DAC1 / DAC2 | PA4 / PA5 | 0–3.3 V, buffered; also read back internally by ADC2 |
| CAN RX / TX | PD0 / PD1 | FDCAN1 to a 3.3 V CAN transceiver (SN65HVD230, TJA1051T/3, ...) |
| GND | | common with the ECU / sensor ground |

Nothing needs to be connected for the self test.

## Talking to real 5 V SENT parts

A SENT line is a 5 V open-collector style signal with RC wave shaping. Recommended interfaces:

### Reading a sensor (sniffing or bridging)

```
 sensor SENT ──┬──────────── (to ECU, if present)
               │
              [R1 10k]
               │
               ├──────────── PA0 (RX0)
               │
              [R2 20k]
               │
 GND ──────────┴──────────── GND
```

The 10 kΩ / 20 kΩ divider maps 5 V to 3.3 V and loads the line with only 30 kΩ. If no ECU is connected, the sensor needs its pull-up: add **10 kΩ to 5 V** on the sensor side. For long or noisy lines, add 100 pF from PA0 to GND, or use a Schmitt buffer (74LVC1G17 powered from 3.3 V with a 5 V tolerant input). When sniffing a live sensor–ECU link, connect only the divider: the instrument does not disturb the line.

### Emulating a sensor towards an ECU

```
 5 V ──[R 10k]──┬──────── ECU SENT input
                │                (many ECUs already have the pull-up)
 PB10 (TX) ─────┘
 set TX:DRIVE OD
```

PB10 is 5 V tolerant in open-drain mode, so it can pull a 5 V line low directly. J2716 transmitters shape the edges with an RC network; a 560 Ω series resistor and 2.2 nF to ground on the ECU side give a realistic waveform, but most receivers do not need it.

### Bridge output to an ECU analog input (0.5–4.5 V sensors)

The DAC delivers 0–3.3 V. Many ECU inputs expect the classic 0.5–4.5 V of a ratiometric sensor. Use a rail-to-rail op-amp powered from 5 V (e.g. MCP6002, TLV9002) as a non-inverting stage with gain 1.5:

```
 PA4 ──── (+) op-amp ──┬──── ECU analog input
          (−) ─────────┤
                  [Rf 5.1k]
          (−) ──[Rg 10k]── GND        gain = 1 + Rf/Rg ≈ 1.5
```

Then map in volts at the DAC, for example `BRIDGE1:MAP RX0,FC1,193,3896,0.333,3.0` for a 0.5–4.5 V output. Choose a fail-safe voltage the ECU treats as a sensor fault, e.g. `BRIDGE1:FAIL 0.1,100`. That way a lost SENT sensor is never seen as a valid position.

> **Safety:** the bridge is a development and bench tool. A throttle or pedal signal is safety relevant: validate the whole chain, including fail-safe behaviour on loss of signal and on CRC errors, before driving a vehicle with it.

## Resource map

| Resource | Use |
|---|---|
| TIM2 (32 bit) | TX PWM; ARR rewritten every period by DMA1 Stream0 (DMAMUX req 22) from a 32-entry ring refilled in the half / complete interrupts |
| TIM5 (32 bit) | free-running capture time base; CH1 / CH2 to DMA1 Stream1 / Stream2 (req 55 / 56), 4096-entry rings |
| EXTI10 | monitor of the TX pin, timestamped from TIM5 at priority 0 |
| DAC1 | bridge outputs, mode "pin + on-chip peripherals" |
| ADC2 | DAC read-back through internal channels 16 and 17, 16 bit, linearity calibrated |
| FDCAN1 | SENT → CAN gateway; message RAM: RX FIFO0 8 elements, TX FIFO 4 elements |
| DWT | cycle-accurate bit-bang for `RXn:PULLTEST` |
| RAM | `.data` / `.bss` / DMA buffers in AXI SRAM (D-cache off); stack in DTCM |
