#define SC_USE_DEPRECATED 1
#include "SIDOsc.hpp"
#include "envelope.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

using namespace reSID;

static InterfaceTable* ft;

namespace SIDOsc {

// Constants
static constexpr double kClockFreq    = 985248.0; // PAL SID clock
//static constexpr double kClockFreq = 1022727.0;  // NTSC SID clock

static constexpr double kAccResolution = 16777216.0; // = 2^24, 24-bit fixed-point phase accumulator
//In the original reSID code, the analog filter (VCF) integrators and feedback loops are implemented in fixed‑point with an 11‑bit fractional resolution. Internally, filter state variables are scaled by 2048 so that the C‑code can stay in integer arithmetic
static constexpr int    kFilterRes    = 2048; // = 2^11
static constexpr int    kDACMaxValue  = 4095; // = 2^12-1, the full‑scale count of a 12‑bit DAC
static constexpr int    kOutNorm      = 32767; // = 2^15-1, the max pos value of a signed 16‑bit sample. After we sum the three voices’ 12‑bit outputs, we divide by kOutNorm to map into the conventional signed‑16 range (−1.0…+1.0 in float)

// First arg: reference voltage
// Second arg: termination‑resistor flag Boolean -- the final termination resistor in the DAC ladder causes a little bump/distortion near full‑scale
const reSID::DAC<12> SIDOsc::dac6581(2.20, false); // that’s the nominal peak amplitude you get out of a real MOS 6581’s resistor ladder.
const reSID::DAC<12> SIDOsc::dac8580(2.00, true); // it's less here because of the addition of the termination-resistor

// Helper: if input index is beyond mNumInputs, return a default.
static inline float getInputDefault(SCUnit* unit, int index, float def) {
    return (index < unit->mNumInputs) ? unit->in0(index) : def;
}

SIDOsc::SIDOsc()
    : mGain(1.0f)
    , freqValue(0)
    , mPrevControlReg(0xFF)
    , mPrevGate(false)
    // Envelope parameters removed:
    // , mPrevAttackInput(-1.0f)
    // , mPrevDecayInput(-1.0f)
    // , mPrevSustainInput(-1.0f)
    // , mPrevReleaseInput(-1.0f)
    , mPrevFreq(-1.0f)
{
    // Initialize three voices.
    for (int v = 0; v < 3; v++) {
        voice[v].set_chip_model(MOS6581);
    }
    // Link voices for sync.
    voice[0].set_sync_source(&voice[2]);
    voice[1].set_sync_source(&voice[0]);
    voice[2].set_sync_source(&voice[1]);

    // Remove filter and envelope initialization.
    // filter.set_chip_model(MOS6581);
    // filter.enable_filter(true);
    // extFilter.enable_filter(true);

    // Potentiometers remain (if used internally by reSID).
    potX.readPOT();
    potY.readPOT();

    // Remove envelope defaults.
    // for (int v = 0; v < 3; v++) {
    //     voice[v].envelope.reset();
    //     voice[v].envelope.writeATTACK_DECAY(0x00);
    //     voice[v].envelope.writeSUSTAIN_RELEASE(0xFF);
    // }
    
    mCalcFunc = make_calc_function<SIDOsc, &SIDOsc::next>();
    next(1);
}

void SIDOsc::next(int nSamples) {
    // --- Read primary parameters (from the input buffers) ---
    const float* freqInput   = in(0);
    const float gainInput    = in0(1);
    const int   waveformType = static_cast<int>(in0(2));
    const int   dacType      = static_cast<int>(in0(3));  // Unused for now.
    const float gateInput    = in0(4);

    // Remove envelope parameters.
    // float attackInput  = (mNumInputs > 7)  ? in0(7)  : 0.0f;
    // float decayInput   = (mNumInputs > 8)  ? in0(8)  : 0.0f;
    // float sustainInput = (mNumInputs > 9)  ? in0(9)  : 1.0f;
    // float releaseInput = (mNumInputs > 10) ? in0(10) : 1.0f;

    mGain = gainInput;

    // --- Build new control register (upper 4 bits: waveformType; bit0: gate) ---
    bool currentGate = (gateInput > 0.5f);
    reSID::reg8 newControlReg = static_cast<reSID::reg8>((waveformType << 4) | (currentGate ? 0x01 : 0x00));

    // Update control registers on gate change.
    if (currentGate != mPrevGate) {
        for (int v = 0; v < 3; v++) {
            voice[v].writeCONTROL_REG(newControlReg);
        }
        mPrevGate = currentGate;
    }
    // (Envelope updates removed.)

    // --- Frequency and oscillator update ---
    const float freqEpsilon = 0.001f;
    float* outputBuffer = this->out(0);
    for (int i = 0; i < nSamples; ++i) {
        float freq = (inRate(0) == calc_FullRate) ? freqInput[i] : in0(0);
        if (freq <= 0.0f) {
            outputBuffer[i] = 0.0f;
            continue;
        }
        if (fabs(freq - mPrevFreq) > freqEpsilon) {
            this->freqValue = static_cast<unsigned int>((freq * kAccResolution) / kClockFreq);
            for (int v = 0; v < 3; v++) {
                voice[v].wave.writeFREQ_LO(static_cast<reSID::reg8>(this->freqValue & 0xFF));
                voice[v].wave.writeFREQ_HI(static_cast<reSID::reg8>((this->freqValue >> 8) & 0xFF));
            }
            mPrevFreq = freq;
        }
        // Clock the oscillators.
        for (int v = 0; v < 3; v++) {
            voice[v].wave.clock();
        }
        for (int v = 0; v < 3; v++) {
            voice[v].wave.synchronize();
        }
        for (int v = 0; v < 3; v++) {
            voice[v].wave.set_waveform_output();
        }
        float sumVoiceOutput = 0.0f;
        for (int v = 0; v < 3; v++) {
            sumVoiceOutput += static_cast<float>(voice[v].wave.output() - voice[v].getWaveZero());
        }
        float mixedOutput = sumVoiceOutput / 3.0f;
        // Remove filter and envelope processing.
        float finalOutput = (mixedOutput / kOutNorm * mGain);
        outputBuffer[i] = finalOutput;
    }
}

} // namespace SIDOsc
PluginLoad(SIDOsc) {
    ft = inTable;
    registerUnit<SIDOsc::SIDOsc>(ft, "SIDOsc", false);
}
