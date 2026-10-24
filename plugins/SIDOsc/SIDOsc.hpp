#pragma once

#include "SC_PlugIn.hpp"
#include "sid.h"

namespace SIDOsc {

class SIDOsc : public SCUnit {
public:
    SIDOsc();

    // Input order. Must match the argument order of multiNew() in SIDOsc.sc.
    enum Input {
        Freq = 0,   // Hz, any rate (audio rate = per-sample pitch)
        Gain,       // linear output gain, control rate
        Waveform,   // 1 = triangle, 2 = saw, 4 = pulse, 8 = noise (combinable)
        PulseWidth, // 0..1, used by the pulse waveform
        Gate,       // > 0 = note on (attack/decay/sustain), 0 = release
        Attack,     // seconds, snapped to the nearest SID attack rate
        Decay,      // seconds, snapped to the nearest SID decay rate
        Sustain,    // 0..1, fraction of peak
        Release,    // seconds, snapped to the nearest SID release rate
        ChipModel,  // 0 = MOS6581, 1 = MOS8580 (read once, at synth start)
        NumInputs
    };

private:
    void next(int nSamples);
    void writeFreq(float freq);
//    void writeReg(reSID::reg8 reg, reSID::reg8 value);

    // An instance of the full SID
    reSID::SID sid;
    
    // Persistent previous frequency.
    double mCyclesPerSample; // SID clock ticks per audio sample (~22.34 for 44.1 kHz and PAL clock)
    double mCycleLeftover; // leftover fraction carried between samples

    // Last values written to each register, so we only write on change.
    int mPrevFreqReg;
    int mPrevPulseReg;
    int mPrevControl;
    int mPrevAttackDecay;
    int mPrevSustainRelease;};

} // namespace SIDOsc
