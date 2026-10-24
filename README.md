# SIDOsc

**The Commodore 64's SID chip as a SuperCollider UGen.**

SIDOsc runs a cycle-accurate emulation of the MOS 6581 / 8580 Sound Interface Device inside `scsynth`, using Dag Lem's [reSID](https://github.com/libsidplayfp/resid). The plugin drives the emulated chip the way a C64 program does, by writing its registers and clocking it at the real PAL rate. The 6581 waveforms and the stepped envelopes come from the emulation.

> **Status:** work in progress.  

### Arguments

| argument | default | meaning |
|---|---|---|
| `freq` | `440` | frequency in Hz; audio rate is read per sample |
| `gain` | `1.0` | output gain |
| `waveform` | `1` | `1` triangle, `2` saw, `4` pulse, `8` noise; add to combine (`3` = triangle + saw) |
| `pulseWidth` | `0.5` | 0–1, the high part of each cycle is `1 − pulseWidth`; pulse waveform only |
| `gate` | `1` | `> 0` starts the envelope, `0` releases it |
| `env` | `Env.adsr(0.002, 0.006, 1.0, 0.006)` | an `Env.adsr`; curve, peak level and bias are ignored |
| `chipModel` | `0` | `0` MOS6581, `1` MOS8580; fixed when the synth starts |


## Roadmap
- [ ] Real chip timing: running at the PAL clock (985,248 Hz) and each audio sample advances it by `clock / sampleRate` ticks
- [ ] The SID filter: cutoff, resonance and low / band / high-pass modes
- [ ] Voices 2 and 3, including hard sync and ring modulation

## Credits

- **reSID** by Dag Lem: the SID emulation at the heart of this plugin (GPL v2 or later).
- **MOS 6581 SID** designed by Bob Yannes, 1982.
- Built on SuperCollider's server plugin API.

## License

SIDOsc is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the [GNU General Public License](https://www.gnu.org/licenses/) for details.
