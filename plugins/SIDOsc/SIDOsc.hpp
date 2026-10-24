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
    // Input enum for clarity
    enum Input {Freq = 0, Gain, Waveform, DacType, Gate, Attack, Decay, Sustain, Release};

    // Control-rate gain parameter.
    float mGain;

    // Three voices (to emulate a full SID).
    reSID::Voice voice[3];
    
    //commout
    // Filter and external filter.
//    reSID::Filter filter;
//    reSID::ExternalFilter extFilter;

    // Potentiometer instances.
    reSID::Potentiometer potX, potY;

    // Static DAC models.
    static const reSID::DAC<12> dac6581; // For 6581
    static const reSID::DAC<12> dac8580; // For 8580

    // An instance of the full SID 
    reSID::SID sid;

    // Persistent frequency register value.
    volatile unsigned int freqValue;
    
    // Cache for the previous control register.
    reSID::reg8 mPrevControlReg;
    
    // Persistent previous frequency.
    float mPrevFreq;
    double mCyclesPerSample; // SID clock ticks per audio sample (~22.34 for 44.1 kHz and PAL clock)
    double mCycleLeftover; // leftover fraction carried between samples

    // Cache for the previous gate value and envelope parameter values.
    bool mPrevGate;
    //commout
//    float mPrevAttackInput, mPrevDecayInput, mPrevSustainInput, mPrevReleaseInput;
};

} // namespace SIDOsc
