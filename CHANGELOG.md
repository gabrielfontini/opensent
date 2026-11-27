# Changelog

## 0.1.0 (2026-11-27)

First release. Everything below was verified on a WeAct STM32H750VBT6 by `opensent selftest` (43/43, no external wiring) and on real sensor captures.

- **Sensor emulator:** tick 0.5–100 µs; 1–8 nibbles; CRC 2010 / legacy / status; pauses (fixed and constant frame length); short and enhanced serial messages; waveforms; seven fault types.
- **Receivers:** 2 input-capture channels plus a TX-pin monitor, 5 ns resolution, glitch filter, nibble-count and CRC-variant learning, J2716 successive-calibration check, statistics.
- **Raw two-edge capture** to VCD/CSV, and a **sigrok/PulseView SENT decoder**.
- **SENT → analog bridge** with fail-safe and on-chip read-back (ADC2).
- **SENT → CAN gateway** (FDCAN1) with a valid flag, error count, counter and data age.
- **Standalone startup script** in the on-board QSPI flash; 0.5 s hardware watchdog.
- **16-channel logic analyzer** on a second USB port (SUMP/OLS protocol for sigrok/PulseView), up to 33.3 MS/s, 128 K samples, parallel trigger.
- **MCP server** (`opensent-mcp`): 10 tools for AI agents to drive and verify the instrument.
- **PC tools:** Python codec (frame-identical to the C core), CLI, HTML sensor health report, Web Serial console with an in-page demo, host simulator.
