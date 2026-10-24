#include "SIDOsc.hpp"
#include <algorithm>
#include <cmath>

static InterfaceTable* ft;

namespace SIDOsc {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr double kClockFreq = 985248.0;    // PAL SID clock
// static constexpr double kClockFreq = 1022727.0; // NTSC SID clock

static constexpr double kAccResolution = 16777216.0; // = 2^24, for 24-bit fixed-point phase accumulator that stands for one oscillator cycle

// SID register offsets (voice 1 and the global volume register)
// defining only the low bytes: $D417 -> 0x17 because SID actually never sees the high bytes
enum Reg : reSID::reg8 {
    FREQ_LO = 0x00,
    FREQ_HI = 0x01,
    PW_LO = 0x02,
    PW_HI = 0x03,
    CONTROL = 0x04,
    ATTACK_DECAY = 0x05,
    SUSTAIN_RELEASE = 0x06,
    RES_FILT = 0x17,
    MODE_VOL = 0x18,
};

//Envelope generation: The SID doesn’t understand seconds. Its envelope registers accept a 4-bit number per stage, 0 to 15, and each number selects one of 16 fixed speeds built into the chip.
// “the user asked for a 0.4 s attack, which of the 16 numbers is closest?” To answer that, you need to know which time each number stands for, and that’s all kAttackSec is. nearestRateNibble() walks through it and picks the best match.
// this also implies that in SC the user cannot set a 2s attack, it will snap into 3s.

// Attack time in seconds for each nibble value 0..15 (datasheet values).
// Decay and release use the same rates, three times slower.
static constexpr float kAttackSec[16] = { 0.002f, 0.008f, 0.016f, 0.024f, 0.038f, 0.056f, 0.068f, 0.080f,
                                          0.100f, 0.250f, 0.500f, 0.800f, 1.0f,   3.0f,   5.0f,   8.0f };
    
// Nearest nibble for a time in seconds. Compared as ratios (log space),
// because the table is roughly exponential: 0.4 s should snap to 0.5 s.
static int nearestRateNibble(float seconds) {
    const float s = std::max(seconds, 1e-4f);
    int best = 0;
    float bestErr = INFINITY;
    for (int n = 0; n < 16; ++n) {
        const float err = std::fabs(std::log(s / kAttackSec[n]));
        if (err < bestErr) {
            bestErr = err;
            best = n;
        }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

SIDOsc::SIDOsc():
    mCycleLeftover(0.0), //brought this up here
    mPrevFreqReg(-1),
    mPrevPulseReg(-1),
    mPrevControl(-1),
    mPrevAttackDecay(-1),
    mPrevSustainRelease(-1) {
        
    // We don't set sync source here, it's done by reSID.
        //such as voice[0].set_sync_source(&voice[2]); etc.
    
    // Envelope has never been used - it's delegated to reSID here as well.
        //so no env defaults here

    // Chip model is fixed for the synth's life: it changes the chip's analog
    // character (filter curve, combined waveforms, DC offset).
    const bool is8580 = in0(ChipModel) > 0.5f;
    sid.set_chip_model(is8580 ? reSID::MOS8580 : reSID::MOS6581);
    sid.reset();

    // sid.reset() clears the filter registers, including master volume
        // set it, and route voice 1 around the filter.
    // Voice 1 straight to the output, not through the filter, volume at max.
    // TODO: add writeReg later for 8581, 6581n doesn't need it
        // writeReg(RES_FILT, 0x00); etc.
    sid.write(RES_FILT, 0x00); // FILT1 = 0, bypasses the filter
    sid.write(MODE_VOL, 0x0F); // //high nibble 0: no filter mode, low nibble F: master volume 15/
        

    // Advance the SID by clock/sampleRate ticks per audio sample.
    mCyclesPerSample = kClockFreq / sampleRate();

    mCalcFunc = make_calc_function<SIDOsc, &SIDOsc::next>();
    next(1);
}


// ---------------------------------------------------------------------------
// Register write (to be implemented for 8581)
// ---------------------------------------------------------------------------
//
//void SIDOsc::writeReg() {
//}

// ---------------------------------------------------------------------------
// Frequency: Hz -> 16-bit register, written only when it changes
// ---------------------------------------------------------------------------

// the older version is about 22x wrong in terms of freq


// ---------------------------------------------------------------------------
// Audio callback
// ---------------------------------------------------------------------------

void SIDOsc::next(int nSamples) {
    // --- Read primary parameters (from the input buffers) ---
    const float* freqInput     = in(Freq);
    const float gainInput      = in0(Gain); // in0(0) is shorthand for in(0)[0], which is the first sample of the first input
    const int   dacType        = static_cast<int>(std::lround(in0(DacType)));
    const bool freqIsAudioRate = (inRate(Freq) == calc_FullRate);
    const float attackInput    = (mNumInputs > 5) ? in0(5) : 0.0f;
    const float decayInput     = (mNumInputs > 6) ? in0(6) : 0.0f;
    const float sustainInput   = (mNumInputs > 7) ? in0(7) : 1.0f;
    const float releaseInput   = (mNumInputs > 8) ? in0(8) : 1.0f;


    mGain = gainInput;

    // --- Build new control register (upper 4 bits: waveformType; bit0: gate) ---
    // This  will come after the envelope registers, so a new note starts with the
    // current ADSR settings.
    // making sure that we take lower bits from the waveform input
    const int waveformType = static_cast<int>(std::lround(in0(Waveform))) & 0x0F;
    const bool gateInput = in0(Gate) > 0.0f;
    const int controlReg = static_cast<reSID::reg8>((waveformType << 4) | (gate ? 0x01 : 0x00));

    // Update control registers on gate change.
    if (controlReg != mPrevControl) {
        sid.write(CONTROL, static_cast<reSID::reg8>(control));
        mPrevControl = control;
    }

    // --- Frequency and oscillator update ---
    const float freqEpsilon = 0.001f;
    float* outputBuffer = this->out(0);
    for (int i = 0; i < nSamples; ++i) { //nSamples is the block size, not the hardware buffer size
        float freq = freqIsAudioRate ? freqInput[i] : freqInput[0]; // if frequency is audio rate, read a fresh value per sample, 
                                                                    // otherwise use the single block value, which is also refreshed each block    
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
        // Carry fractional SID ticks between samples so the emulated clock (and pitch) stays exact.
        mCycleLeftover += mCyclesPerSample; // 22.34
        const reSID::cycle_count dt = (reSID::cycle_count)mCycleLeftover; // integer part of cycles to clock -- 22
        mCycleLeftover -= dt; // keep the fractional part for the next sample -- 0.34

        for (int v = 0; v < 3; v++) voice[v].wave.clock(dt);
        for (int v = 0; v < 3; v++) voice[v].wave.synchronize();
        for (int v = 0; v < 3; v++) voice[v].wave.set_waveform_output(dt);
    
        float sumVoiceOutput = 0.0f;
        for (int v = 0; v < 3; v++) {
            sumVoiceOutput += static_cast<float>(voice[v].wave.output() - voice[v].getWaveZero());
        }
        float mixedOutput = sumVoiceOutput / 3.0f;
       
        float finalOutput = (mixedOutput / kOutNorm * mGain);
        outputBuffer[i] = finalOutput;
    }
}

} // namespace SIDOsc
PluginLoad(SIDOsc) {
    ft = inTable;
    registerUnit<SIDOsc::SIDOsc>(ft, "SIDOsc", false);
}
