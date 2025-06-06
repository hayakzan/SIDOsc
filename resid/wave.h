//  ---------------------------------------------------------------------------
//  This file is part of reSID, a MOS6581 SID emulator engine.
//  Copyright (C) 1998 - 2022  Dag Lem <resid@nimrod.no>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//  ---------------------------------------------------------------------------

#ifndef RESID_WAVE_H
#define RESID_WAVE_H

#include "siddefs.h"
#include "dac.h"

namespace reSID
{

// ----------------------------------------------------------------------------
// A 24 bit accumulator is the basis for waveform generation. FREQ is added to
// the lower 16 bits of the accumulator each cycle.
// The accumulator is set to zero when TEST is set, and starts counting
// when TEST is cleared.
// The noise waveform is taken from intermediate bits of a 23 bit shift
// register. This register is clocked by bit 19 of the accumulator.
// ----------------------------------------------------------------------------
class WaveformGenerator
{
public:
  WaveformGenerator();

  void set_sync_source(WaveformGenerator*);
  void set_chip_model(chip_model model);

  void clock();
  void clock(cycle_count delta_t);
  void synchronize();
  void reset();

  void writeFREQ_LO(reg8);
  void writeFREQ_HI(reg8);
  void writePW_LO(reg8);
  void writePW_HI(reg8);
  void writeCONTROL_REG(reg8);
  reg8 readOSC();

  // 12-bit waveform output.
  short output();

  // Calculate and set waveform output value.
#if RESID_FPGA_CODE
  short calculate_waveform_output();
#endif
  void set_waveform_output();
  void set_waveform_output(cycle_count delta_t);

public:
  void clock_shift_register();
  void write_shift_register();
  void reset_shift_register();
  void set_noise_output();

  const WaveformGenerator* sync_source;
  WaveformGenerator* sync_dest;

  reg24 accumulator;

  // Tell whether the accumulator MSB was set high on this cycle.
  bool msb_rising;

  // Fout  = (Fn*Fclk/16777216)Hz
  // reg16 freq;
  reg24 freq;
  // PWout = (PWn/40.95)%
  reg12 pw;

  reg24 shift_register;

  // Remaining time to fully reset shift register.
  cycle_count shift_register_reset;
  // Emulation of pipeline causing bit 19 to clock the shift register.
  cycle_count shift_pipeline;

  // Helper variables for waveform table lookup.
  reg24 ring_msb_mask;
  unsigned short no_noise;
  unsigned short noise_output;
  unsigned short no_noise_or_noise_output;
  unsigned short no_pulse;
  unsigned short pulse_output;

  // The control register right-shifted 4 bits; used for waveform table lookup.
  reg8 waveform;

  // 8580 tri/saw pipeline
  reg12 tri_saw_pipeline;
  reg12 osc3;

  // The remaining control register bits.
  reg8 test;
  reg8 ring_mod;
  reg8 sync;
  // The gate bit is handled by the EnvelopeGenerator.

  // DAC input.
  reg12 waveform_output;
  // Fading time for floating DAC input (waveform 0).
  cycle_count floating_output_ttl;

  chip_model sid_model;

  // Sample data for waveforms, not including noise.
  unsigned short* wave;
  static unsigned short model_wave[2][8][1 << 12];
  // DAC lookup tables.
  static const DAC<12> model_dac[2];

friend class Voice;
friend class SID;
};


// ----------------------------------------------------------------------------
// Inline functions.
// The following functions are defined inline because they are called every
// time a sample is calculated.
// ----------------------------------------------------------------------------

#if RESID_INLINING || defined(RESID_WAVE_CC)

// ----------------------------------------------------------------------------
// SID clocking - 1 cycle.
// ----------------------------------------------------------------------------
RESID_INLINE
void WaveformGenerator::clock()
{
  if (unlikely(test)) {
    // Count down time to fully reset shift register.
    if (unlikely(shift_register_reset) && unlikely(!--shift_register_reset)) {
      reset_shift_register();
    }

    // The test bit sets pulse high.
    pulse_output = 0xfff;
  }
  else {
    // Calculate new accumulator value;
    reg24 accumulator_next = (accumulator + freq) & 0xffffff;
    reg24 accumulator_bits_set = ~accumulator & accumulator_next;
    accumulator = accumulator_next;

    // Check whether the MSB is set high. This is used for synchronization.
    msb_rising = (accumulator_bits_set & 0x800000) ? true : false;

    // Shift noise register once for each time accumulator bit 19 is set high.
    // The shift is delayed 2 cycles.
    if (unlikely(accumulator_bits_set & 0x080000)) {
      // Pipeline: Detect rising bit, shift phase 1, shift phase 2.
      shift_pipeline = 2;
    }
    else if (unlikely(shift_pipeline) && !--shift_pipeline) {
      clock_shift_register();
    }
  }
}

// ----------------------------------------------------------------------------
// SID clocking - delta_t cycles.
// ----------------------------------------------------------------------------
RESID_INLINE
void WaveformGenerator::clock(cycle_count delta_t)
{
  if (unlikely(test)) {
    // Count down time to fully reset shift register.
    if (shift_register_reset) {
      shift_register_reset -= delta_t;
      if (unlikely(shift_register_reset <= 0)) {
        reset_shift_register();
      }
    }

    // The test bit sets pulse high.
    pulse_output = 0xfff;
  }
  else {
    // Calculate new accumulator value;
    reg24 delta_accumulator = delta_t*freq;
    reg24 accumulator_next = (accumulator + delta_accumulator) & 0xffffff;
    reg24 accumulator_bits_set = ~accumulator & accumulator_next;
    accumulator = accumulator_next;

    // Check whether the MSB is set high. This is used for synchronization.
    msb_rising = (accumulator_bits_set & 0x800000) ? true : false;

    // NB! Any pipelined shift register clocking from single cycle clocking
    // will be lost. It is not worth the trouble to flush the pipeline here.

    // Shift noise register once for each time accumulator bit 19 is set high.
    // Bit 19 is set high each time 2^20 (0x100000) is added to the accumulator.
    reg24 shift_period = 0x100000;

    while (delta_accumulator) {
      if (likely(delta_accumulator < shift_period)) {
        shift_period = delta_accumulator;
        // Determine whether bit 19 is set on the last period.
        // NB! Requires two's complement integer.
        if (likely(shift_period <= 0x080000)) {
          // Check for flip from 0 to 1.
          if (((accumulator - shift_period) & 0x080000) || !(accumulator & 0x080000))
            {
              break;
            }
        }
        else {
          // Check for flip from 0 (to 1 or via 1 to 0) or from 1 via 0 to 1.
          if (((accumulator - shift_period) & 0x080000) && !(accumulator & 0x080000))
            {
              break;
            }
        }
      }

      // Shift the noise/random register.
      // NB! The two-cycle pipeline delay is only modeled for 1 cycle clocking.
      clock_shift_register();

      delta_accumulator -= shift_period;
    }

    // Calculate pulse high/low.
    // NB! The one-cycle pipeline delay is only modeled for 1 cycle clocking.
    pulse_output = (accumulator >> 12) >= pw ? 0xfff : 0x000;
  }
}


// ----------------------------------------------------------------------------
// Synchronize oscillators.
// This must be done after all the oscillators have been clock()'ed since the
// oscillators operate in parallel.
// Note that the oscillators must be clocked exactly on the cycle when the
// MSB is set high for hard sync to operate correctly. See SID::clock().
// ----------------------------------------------------------------------------
RESID_INLINE
void WaveformGenerator::synchronize()
{
  // A special case occurs when a sync source is synced itself on the same
  // cycle as when its MSB is set high. In this case the destination will
  // not be synced. This has been verified by sampling OSC3.
  if (unlikely(msb_rising) && sync_dest->sync && !(sync && sync_source->msb_rising)) {
    sync_dest->accumulator = 0;
  }
}


// ----------------------------------------------------------------------------
// Waveform output.
// The output from SID 8580 is delayed one cycle compared to SID 6581;
// this is only modeled for single cycle clocking (see sid.cc).
// ----------------------------------------------------------------------------

// No waveform:
// When no waveform is selected, the DAC input is floating.
//

// Triangle:
// The upper 12 bits of the accumulator are used.
// The MSB is used to create the falling edge of the triangle by inverting
// the lower 11 bits. The MSB is thrown away and the lower 11 bits are
// left-shifted (half the resolution, full amplitude).
// Ring modulation substitutes the MSB with MSB EOR NOT sync_source MSB.
//

// Sawtooth:
// The output is identical to the upper 12 bits of the accumulator.
//

// Pulse:
// The upper 12 bits of the accumulator are used.
// These bits are compared to the pulse width register by a 12 bit digital
// comparator; output is either all one or all zero bits.
// The pulse setting is delayed one cycle after the compare; this is only
// modeled for single cycle clocking.
//
// The test bit, when set to one, holds the pulse waveform output at 0xfff
// regardless of the pulse width setting.
//

// Noise:
// The noise output is taken from intermediate bits of a 23-bit shift register
// which is clocked by bit 19 of the accumulator.
// The shift is delayed 2 cycles after bit 19 is set high; this is only
// modeled for single cycle clocking.
//
// Operation: Calculate EOR result, shift register, set bit 0 = result.
//
//                reset    -------------------------------------------
//                  |     |                                           |
//           test--OR-->EOR<--                                        |
//                  |         |                                       |
//                  2 2 2 1 1 1 1 1 1 1 1 1 1                         |
// Register bits:   2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 <---
//                      |   |       |     |   |       |     |   |
// Waveform bits:       1   1       9     8   7       6     5   4
//                      1   0
//
// The low 4 waveform bits are zero (grounded).
//

RESID_INLINE void WaveformGenerator::clock_shift_register()
{
  // bit0 = (bit22 | test) ^ bit17
  reg24 bit0 = ((shift_register >> 22) ^ (shift_register >> 17)) & 0x1;
  shift_register = ((shift_register << 1) | bit0) & 0x7fffff;

  // New noise waveform output.
  set_noise_output();
}

RESID_INLINE void WaveformGenerator::write_shift_register()
{
  // Write changes to the shift register output caused by combined waveforms
  // back into the shift register.
  // A bit once set to zero cannot be changed, hence the and'ing.
  // FIXME: Write test program to check the effect of 1 bits and whether
  // neighboring bits are affected.

  shift_register &=
    ~((1<<20)|(1<<18)|(1<<14)|(1<<11)|(1<<9)|(1<<5)|(1<<2)|(1<<0)) |
    ((waveform_output & 0x800) << 9) |  // Bit 11 -> bit 20
    ((waveform_output & 0x400) << 8) |  // Bit 10 -> bit 18
    ((waveform_output & 0x200) << 5) |  // Bit  9 -> bit 14
    ((waveform_output & 0x100) << 3) |  // Bit  8 -> bit 11
    ((waveform_output & 0x080) << 2) |  // Bit  7 -> bit  9
    ((waveform_output & 0x040) >> 1) |  // Bit  6 -> bit  5
    ((waveform_output & 0x020) >> 3) |  // Bit  5 -> bit  2
    ((waveform_output & 0x010) >> 4);   // Bit  4 -> bit  0

  noise_output &= waveform_output;
  no_noise_or_noise_output = no_noise | noise_output;
}

RESID_INLINE void WaveformGenerator::reset_shift_register()
{
  shift_register = 0x7fffff;
  shift_register_reset = 0;

  // New noise waveform output.
  set_noise_output();
}

RESID_INLINE void WaveformGenerator::set_noise_output()
{
  noise_output =
    ((shift_register & 0x100000) >> 9) |
    ((shift_register & 0x040000) >> 8) |
    ((shift_register & 0x004000) >> 5) |
    ((shift_register & 0x000800) >> 3) |
    ((shift_register & 0x000200) >> 2) |
    ((shift_register & 0x000020) << 1) |
    ((shift_register & 0x000004) << 3) |
    ((shift_register & 0x000001) << 4);

  no_noise_or_noise_output = no_noise | noise_output;
}

// Combined waveforms:
// By combining waveforms, the bits of each waveform are effectively short
// circuited. A zero bit in one waveform will result in a zero output bit
// (thus the infamous claim that the waveforms are AND'ed).
// However, a zero bit in one waveform may also affect the neighboring bits
// in the output.
//
// Example:
// 
//             1 1
// Bit #       1 0 9 8 7 6 5 4 3 2 1 0
//             -----------------------
// Sawtooth    0 0 0 1 1 1 1 1 1 0 0 0
//
// Triangle    0 0 1 1 1 1 1 1 0 0 0 0
//
// AND         0 0 0 1 1 1 1 1 0 0 0 0
//
// Output      0 0 0 0 1 1 1 0 0 0 0 0
//
//
// Re-vectorized die photographs reveal the mechanism behind this behavior.
// Each waveform selector bit acts as a switch, which directly connects
// internal outputs into the waveform DAC inputs as follows:
//
// * Noise outputs the shift register bits to DAC inputs as described above.
//   Each output is also used as input to the next bit when the shift register
//   is shifted.
// * Pulse connects a single line to all DAC inputs. The line is connected to
//   either 5V (pulse on) or 0V (pulse off) at bit 11, and ends at bit 0.
// * Triangle connects the upper 11 bits of the (MSB EOR'ed) accumulator to the
//   DAC inputs, so that DAC bit 0 = 0, DAC bit n = accumulator bit n - 1.
// * Sawtooth connects the upper 12 bits of the accumulator to the DAC inputs,
//   so that DAC bit n = accumulator bit n. Sawtooth blocks out the MSB from
//   the EOR used to generate the triangle waveform.
//
// We can thus draw the following conclusions:
//
// * The shift register may be written to by combined waveforms.
// * The pulse waveform interconnects all bits in combined waveforms via the
//   pulse line.
// * The combination of triangle and sawtooth interconnects neighboring bits
//   of the sawtooth waveform.
//
// The figure below show how the waveform bits are interconnected:
//
//   Pulse hi/lo  P--...--------------Rl---------------Rl--...
//                            |                |
//   Sawtooth bits            | Sn+1           |  Sn
//                   ...------|--|  -----------|--|  ------...
//   Noise bits            Nn |  |  |     Nn-1 |  |  |
//                         |  |  |  |       |  |  |  |
//   Switch "resistors"    Rn Rp Rs Rt      Rn Rp Rs Rt     N / P / S / T
//                         |  |  |  |       |  |  |  |
//   Waveform switches      \  \  \  \       \  \  \  \     Control reg D7-D4
//                         |  |  |  |       |  |  |  |
//                         ----------       ----------
//                             |                |
//   Waveform output bits /   Wn+1              Wn
//   DAC input bits
//
// Notes:
//   - Triangle output bit 0 is switched to GND via the "resistor" Rtlo.
//     Rtlo has lower resistance than Rt.
//   - The 8580 triangle output bits 11 - 4 are switched via "resistors" Rthi.
//     Rthi have higher resistance than Rt.
//   - Sawtooth output bit 11 is switched via the "resistor" Rshi.
//     Rshi has higher resistance than Rs.
//   - Rshi in the 8580 is different from Rshi in the 6581.
//   - Noise output switch "resistors" Rn11 - Rn4 are all different.
//   - Noise output bits 3 - 0 are switched to GND via the "resistors" Rnlo.
//     Rnlo have lower resistance than Rn11 - Rn4.
//   - The waveform bits are outputs from NMOS inverters. These have
//     comparatively high output impedance, especially for "1" bits. This
//     implies that *all* waveform output bits are in practice interconnected,
//     not only for pulse, but also for the combination of triangle and sawtooth.
//
// This would be computationally expensive to model exactly, since each output
// bit value depends on all inputs, and the output bit values are analog. Tests
// show that 1 bit differences can actually occur in the output from otherwise
// identical samples from OSC3 when waveforms are combined. To further
// complicate the situation the output changes slightly with time (more
// neighboring bits are successively set) when the 12-bit waveform
// registers are kept unchanged.
//
// It is possible to calculate (digital) bit values with reasonable resource
// usage on an FPGA - this is demonstrated in calculate_waveform_output().
// Here, the output is instead approximated by using the upper bits of the
// accumulator as an index to look up the combined output in a table
// containing actual combined waveform samples from OSC3.
// These samples are 8 bit, so 4 bits of waveform resolution is lost.
// All OSC3 samples are taken with FREQ=0x1000, adding a 1 to the upper 12
// bits of the accumulator each cycle for a sample period of 4096 cycles.
//
// Sawtooth+Triangle:
// The accumulator is used to look up an OSC3 sample.
// 
// Pulse+Triangle:
// The accumulator is used to look up an OSC3 sample. When ring modulation is
// selected, the accumulator MSB is substituted with MSB EOR NOT sync_source MSB.
// 
// Pulse+Sawtooth:
// The accumulator is used to look up an OSC3 sample.
// The sample is output if the pulse output is on.
//
// Pulse+Sawtooth+Triangle:
// The accumulator is used to look up an OSC3 sample.
// The sample is output if the pulse output is on.
// 
// Combined waveforms including noise:
// All waveform combinations including noise output zero after a few cycles,
// since the waveform bits are and'ed into the shift register via the shift
// register outputs.

static reg12 noise_pulse6581(reg12 noise)
{
    return (noise < 0xf00) ? 0x000 : noise & (noise<<1) & (noise<<2);
}

static reg12 noise_pulse8580(reg12 noise)
{
    return (noise < 0xfc0) ? noise & (noise << 1) : 0xfc0;
}

#if RESID_FPGA_CODE
RESID_INLINE
short WaveformGenerator::calculate_waveform_output()
{
  int ix = (accumulator ^ (~sync_source->accumulator & ring_msb_mask)) >> 12;
  int x = accumulator;

  // Espresso has been used to simplify sums of products per bit for
  // sawtooth + triangle and pulse + sawtooth + triangle, based on waveform
  // samples.
  // A few manual simplifications have been made for the 8580 waveforms,
  // without introducing any noticeable difference.
  switch (waveform) {
  case 2:
    return accumulator >> 12;
  case 3:
    if (sid_model == 0) {
      return
        ((((x & 0x7fc) == 0x7fc)) << 10) |
        ((((x & 0x7e0) == 0x7e0) | ((x & 0x3fe) == 0x3fe)) << 9) |
        ((((x & 0x7e0) == 0x7e0) | ((x & 0x5ff) == 0x5ff) | ((x & 0x3f0) == 0x3f0)) << 8) |
        ((((x & 0x7e0) == 0x7e0) | ((x & 0x1f8) == 0x1f8) | ((x & 0x3f0) == 0x3f0)) << 7) |
        ((((x & 0x0fc) == 0x0fc) | ((x & 0x1f8) == 0x1f8) | ((x & 0x3f0) == 0x3f0)) << 6) |
        ((((x & 0x07e) == 0x07e) | ((x & 0x1f8) == 0x1f8) | ((x & 0x0fc) == 0x0fc)) << 5) |
        ((((x & 0x13f) == 0x13f) | ((x & 0x07e) == 0x07e) | ((x & 0x7fa) == 0x7fa) | ((x & 0x0bf) == 0x0bf) | ((x & 0x0fc) == 0x0fc)) << 4);
    }
    else {
      return
        ((((x & 0xe7e) == 0xe7e) | ((x & 0xe80) == 0xe80) | ((x & 0xf00) == 0xf00) | ((x & 0xe7d) == 0xe7d)) << 11) |
        ((((x & 0x7f8) == 0x7f8) | ((x & 0xf00) == 0xf00)) << 10) |
        ((((x & 0x7e0) == 0x7e0) | ((x & 0xf0f) == 0xf0f) | ((x & 0xf1b) == 0xf1b) | ((x & 0xbfe) == 0xbfe) | ((x & 0xf1e) == 0xf1e) | ((x & 0xf40) == 0xf40) | ((x & 0xf30) == 0xf30) | ((x & 0xf29) == 0xf29) | ((x & 0xf26) == 0xf26) | ((x & 0xf80) == 0xf80)) << 9) |
        ((((x & 0x7e0) == 0x7e0) | ((x & 0x3f0) == 0x3f0) | ((x & 0xdfe) == 0xdfe) | ((x & 0x5ff) == 0x5ff) | ((x & 0xf80) == 0xf80)) << 8) |
        ((((x & 0x7e0) == 0x7e0) | ((x & 0x3f0) == 0x3f0) | ((x & 0xfc0) == 0xfc0) | ((x & 0x1f8) == 0x1f8) | ((x & 0xeff) == 0xeff)) << 7) |
        ((((x & 0x0fc) == 0x0fc) | ((x & 0x1f8) == 0x1f8) | ((x & 0x3f0) == 0x3f0) | ((x & 0xfe0) == 0xfe0)) << 6) |
        ((((x & 0x07e) == 0x07e) | ((x & 0xff0) == 0xff0) | ((x & 0x7f7) == 0x7f7) | ((x & 0x1f8) == 0x1f8) | ((x & 0x0fc) == 0x0fc)) << 5) |
        ((((x & 0xdbf) == 0xdbf) | ((x & 0x0fc) == 0x0fc) | ((x & 0x3fa) == 0x3fa) | ((x & 0x7f8) == 0x7f8) | ((x & 0x3bf) == 0x3bf) | ((x & 0x07e) == 0x07e)) << 4);
    }
  case 4:
    return pulse_output;
  case 7: {
    if (sid_model == 0) {
      return
        ((((x & 0x7fc) == 0x7fc) | ((x & 0x7fb) == 0x7fb)) << 10) |
        ((((x & 0x7ef) == 0x7ef) | ((x & 0x7f7) == 0x7f7) | ((x & 0x7fc) == 0x7fc) | ((x & 0x7fb) == 0x7fb) | ((x & 0x3ff) == 0x3ff)) << 9) |
        ((((x & 0x7fc) == 0x7fc) | ((x & 0x3ff) == 0x3ff) | ((x & 0x7f7) == 0x7f7) | ((x & 0x7fb) == 0x7fb)) << 8) |
        ((((x & 0x7fc) == 0x7fc) | ((x & 0x3ff) == 0x3ff) | ((x & 0x7fb) == 0x7fb)) << 7) |
        ((((x & 0x7fd) == 0x7fd) | ((x & 0x3ff) == 0x3ff) | ((x & 0x7fe) == 0x7fe)) << 6) |
        ((((x & 0x7fd) == 0x7fd) | ((x & 0x3ff) == 0x3ff) | ((x & 0x7fe) == 0x7fe)) << 5) |
        ((((x & 0x3ff) == 0x3ff) | ((x & 0x7fe) == 0x7fe)) << 4);
    }
    else {
      return
        ((((x & 0xe89) == 0xe89) | ((x & 0xe3e) == 0xe3e) | ((x & 0xec0) == 0xec0) | ((x & 0xe8a) == 0xe8a) | ((x & 0xdf7) == 0xdf7) | ((x & 0xdf8) == 0xdf8) | ((x & 0xe85) == 0xe85) | ((x & 0xe6a) == 0xe6a) | ((x & 0xe90) == 0xe90) | ((x & 0xe83) == 0xe83) | ((x & 0xe67) == 0xe67) | ((x & 0xea0) == 0xea0) | ((x & 0xf00) == 0xf00) | ((x & 0xe5e) == 0xe5e) | ((x & 0xe70) == 0xe70) | ((x & 0xe6c) == 0xe6c)) << 11) |
        ((((x & 0xeee) == 0xeee) | ((x & 0x7ef) == 0x7ef) | ((x & 0x7f2) == 0x7f2) | ((x & 0x7f4) == 0x7f4) | ((x & 0xef0) == 0xef0) | ((x & 0x7f8) == 0x7f8) | ((x & 0xf00) == 0xf00) | ((x & 0x7f1) == 0x7f1)) << 10) |
        ((((x & 0xf78) == 0xf78) | ((x & 0x7f0) == 0x7f0) | ((x & 0x7ee) == 0x7ee) | ((x & 0xf74) == 0xf74) | ((x & 0xf6f) == 0xf6f) | ((x & 0xf80) == 0xf80) | ((x & 0xbff) == 0xbff)) << 9) |
        ((((x & 0xdff) == 0xdff) | ((x & 0xbfe) == 0xbfe) | ((x & 0x7ef) == 0x7ef) | ((x & 0x7f2) == 0x7f2) | ((x & 0x3ff) == 0x3ff) | ((x & 0x7f4) == 0x7f4) | ((x & 0xfc0) == 0xfc0) | ((x & 0xfb8) == 0xfb8) | ((x & 0x7f8) == 0x7f8) | ((x & 0xfb6) == 0xfb6)) << 8) |
        ((((x & 0xbfe) == 0xbfe) | ((x & 0xfdc) == 0xfdc) | ((x & 0xdfe) == 0xdfe) | ((x & 0x7f7) == 0x7f7) | ((x & 0xfda) == 0xfda) | ((x & 0xbfd) == 0xbfd) | ((x & 0x7f8) == 0x7f8) | ((x & 0x3ff) == 0x3ff) | ((x & 0xfe0) == 0xfe0) | ((x & 0xeff) == 0xeff)) << 7) |
        ((((x & 0xfeb) == 0xfeb) | ((x & 0x7fa) == 0x7fa) | ((x & 0xbfe) == 0xbfe) | ((x & 0xdfe) == 0xdfe) | ((x & 0xff0) == 0xff0) | ((x & 0x7fc) == 0x7fc) | ((x & 0x3ff) == 0x3ff) | ((x & 0xfec) == 0xfec) | ((x & 0xeff) == 0xeff)) << 6) |
        ((((x & 0xff6) == 0xff6) | ((x & 0xdff) == 0xdff) | ((x & 0xf7f) == 0xf7f) | ((x & 0xbfe) == 0xbfe) | ((x & 0x7fc) == 0x7fc) | ((x & 0xff5) == 0xff5) | ((x & 0x3ff) == 0x3ff) | ((x & 0xff8) == 0xff8) | ((x & 0xeff) == 0xeff)) << 5) |
        ((((x & 0xdff) == 0xdff) | ((x & 0xf7f) == 0xf7f) | ((x & 0xffa) == 0xffa) | ((x & 0x7fe) == 0x7fe) | ((x & 0xff9) == 0xff9) | ((x & 0xffc) == 0xffc) | ((x & 0x3ff) == 0x3ff) | ((x & 0xeff) == 0xeff)) << 4);
    }
  }
  case 8:
    return no_noise_or_noise_output;
  default:
    return wave[ix] & (no_pulse | pulse_output) & no_noise_or_noise_output;
  }
}
#endif // RESID_FPGA_CODE

RESID_INLINE
void WaveformGenerator::set_waveform_output()
{
  // Set output value.
  if (likely(waveform)) {
    // The bit masks no_pulse and no_noise are used to achieve branch-free
    // calculation of the output value.
    int ix = (accumulator ^ (~sync_source->accumulator & ring_msb_mask)) >> 12;

#if RESID_FPGA_CODE
    waveform_output = calculate_waveform_output();
#else
    waveform_output = wave[ix] & (no_pulse | pulse_output) & no_noise_or_noise_output;
#endif

    if (unlikely((waveform & 0xc) == 0xc))
    {
        waveform_output = (sid_model == MOS6581) ?
            noise_pulse6581(waveform_output) : noise_pulse8580(waveform_output);
    }

    // Triangle/Sawtooth output is delayed half cycle on 8580.
    // This will appear as a one cycle delay on OSC3 as it is
    // latched in the first phase of the clock.
    if ((waveform & 3) && (sid_model == MOS8580))
    {
        osc3 = tri_saw_pipeline & (no_pulse | pulse_output) & no_noise_or_noise_output;
        tri_saw_pipeline = wave[ix];
    }
    else
    {
        osc3 = waveform_output;
    }

    if ((waveform & 0x2) && unlikely(waveform & 0xd) && (sid_model == MOS6581)) {
        // In the 6581 the top bit of the accumulator may be driven low by combined waveforms
        // when the sawtooth is selected
        accumulator &= (waveform_output << 12) | 0x7fffff;
    }

    if (unlikely(waveform > 0x8) && likely(!test) && likely(shift_pipeline != 1)) {
      // Combined waveforms write to the shift register.
      write_shift_register();
    }
  }
  else {
    // Age floating DAC input.
    if (likely(floating_output_ttl) && unlikely(!--floating_output_ttl)) {
      osc3 = waveform_output = 0;
    }
  }

  // The pulse level is defined as (accumulator >> 12) >= pw ? 0xfff : 0x000.
  // The expression -((accumulator >> 12) >= pw) & 0xfff yields the same
  // results without any branching (and thus without any pipeline stalls).
  // NB! This expression relies on that the result of a boolean expression
  // is either 0 or 1, and furthermore requires two's complement integer.
  // A few more cycles may be saved by storing the pulse width left shifted
  // 12 bits, and dropping the and with 0xfff (this is valid since pulse is
  // used as a bit mask on 12 bit values), yielding the expression
  // -(accumulator >= pw24). However this only results in negligible savings.

  // The result of the pulse width compare is delayed one cycle.
  // Push next pulse level into pulse level pipeline.
  pulse_output = -((accumulator >> 12) >= pw) & 0xfff;
}

RESID_INLINE
void WaveformGenerator::set_waveform_output(cycle_count delta_t)
{
  // Set output value.
  if (likely(waveform)) {
    // The bit masks no_pulse and no_noise are used to achieve branch-free
    // calculation of the output value.
    int ix = (accumulator ^ (~sync_source->accumulator & ring_msb_mask)) >> 12;
    waveform_output =
      wave[ix] & (no_pulse | pulse_output) & no_noise_or_noise_output;
    // Triangle/Sawtooth output delay for the 8580 is not modeled
    osc3 = waveform_output;

    if ((waveform & 0x2) && unlikely(waveform & 0xd) && (sid_model == MOS6581)) {
        accumulator &= (waveform_output << 12) | 0x7fffff;
    }

    if (unlikely(waveform > 0x8) && likely(!test)) {
      // Combined waveforms write to the shift register.
      // NB! Since cycles are skipped in delta_t clocking, writes will be
      // missed. Single cycle clocking must be used for 100% correct operation.
      write_shift_register();
    }
  }
  else {
    if (likely(floating_output_ttl)) {
      // Age floating D/A output.
      floating_output_ttl -= delta_t;
      if (unlikely(floating_output_ttl <= 0)) {
        floating_output_ttl = 0;
        osc3 = waveform_output = 0;
      }
    }
  }
}


// ----------------------------------------------------------------------------
// Waveform output (12 bits).
// ----------------------------------------------------------------------------

// The digital waveform output is converted to an analog signal by a 12-bit
// DAC. Re-vectorized die photographs reveal that the DAC is an R-2R ladder
// built up as follows:
// 
//        12V     11  10   9   8   7   6   5   4   3   2   1   0    GND
// Strange  |      |   |   |   |   |   |   |   |   |   |   |   |     |  Missing
// part    2R     2R  2R  2R  2R  2R  2R  2R  2R  2R  2R  2R  2R    2R  term.
// (bias)   |      |   |   |   |   |   |   |   |   |   |   |   |     |
//          --R-   --R---R---R---R---R---R---R---R---R---R---R--   ---
//                 |          _____
//               __|__     __|__   |
//               -----     =====   |
//               |   |     |   |   |
//        12V ---     -----     ------- GND
//                      |
//                     wout
//
// Bit on:  5V
// Bit off: 0V (GND)
//
// As is the case with all MOS 6581 DACs, the termination to (virtual) ground
// at bit 0 is missing. The MOS 8580 has correct termination, and has also
// done away with the bias part on the left hand side of the figure above.
//

RESID_INLINE
short WaveformGenerator::output()
{
  // DAC imperfections are emulated by using waveform_output as an index
  // into a DAC lookup table. readOSC() uses waveform_output directly.
#if RESID_FPGA_CODE
  // The FPGA code calculates the value by bit superpositioning.
  return model_dac[sid_model](waveform_output);
#else
  return model_dac[sid_model][waveform_output];
#endif
}

#endif // RESID_INLINING || defined(RESID_WAVE_CC)

} // namespace reSID

#endif // not RESID_WAVE_H
