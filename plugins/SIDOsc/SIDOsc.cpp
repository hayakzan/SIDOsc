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

// --- Frequency: Hz -> 16-bit register, written only when it changes ---
void SIDOsc::writeFreq(float freq) {
    // f = reg * clock / 2^24  ->  reg = f * 2^24 / clock
    const double exact = static_cast<double>(freq) * kAccResolution / kClockFreq;
    const int reg = static_cast<int>(std::clamp(std::lround(exact), 0L, 65535L));

    if (reg != mPrevFreqReg) {
        sid.write(FREQ_LO, static_cast<reSID::reg8>(reg & 0xFF));
        sid.write(FREQ_HI, static_cast<reSID::reg8>((reg >> 8) & 0xFF));
        mPrevFreqReg = reg;
    }
}

// ---------------------------------------------------------------------------
// Audio callback
// ---------------------------------------------------------------------------

void SIDOsc::next(int nSamples) {
    // --- Read primary parameters (from the input buffers) ---
    const float* freqInput     = in(Freq);
    const bool freqIsAudioRate = isAudioRateIn(Freq);
    const float gainInput      = in0(Gain); // first sample of the gain input, read once per block (control rate)
    float* outputBuffer        = out(0);

    // --- Frequency (control rate: once per block, audio rate: per sample, in the loop) ---
    if (!freqIsAudioRate) {
        writeFreq(freqInput[0]);
    }

    // --- Pulse width: 0..1 -> 12-bit register, split across PW_LO and PW_HI ---
    const int pulseReg = std::clamp(static_cast<int>(std::lround(in0(PulseWidth) * 4095.0f)), 0, 4095);
    if (pulseReg != mPrevPulseReg) {
        sid.write(PW_LO, static_cast<reSID::reg8>(pulseReg & 0xFF));
        sid.write(PW_HI, static_cast<reSID::reg8>((pulseReg >> 8) & 0x0F));
        mPrevPulseReg = pulseReg;
    }

    // --- Envelope: seconds/level -> four nibbles, two per register ---
    // Times snap to the SID's 16 rates (see kAttackSec); decay and release run ~3x slower.
    const int attack  = nearestRateNibble(in0(Attack));
    const int decay   = nearestRateNibble(in0(Decay) / 3.0f);
    const int release = nearestRateNibble(in0(Release) / 3.0f);
    const int sustain = std::clamp(static_cast<int>(std::lround(in0(Sustain) * 15.0f)), 0, 15);

    const int attackDecay = (attack << 4) | decay;
    if (attackDecay != mPrevAttackDecay) {
        sid.write(ATTACK_DECAY, static_cast<reSID::reg8>(attackDecay));
        mPrevAttackDecay = attackDecay;
    }
    const int sustainRelease = (sustain << 4) | release;
    if (sustainRelease != mPrevSustainRelease) {
        sid.write(SUSTAIN_RELEASE, static_cast<reSID::reg8>(sustainRelease));
        mPrevSustainRelease = sustainRelease;
    }

    // --- Control register (upper 4 bits: waveform; bit 0: gate) ---
    // Written after the envelope registers, so a new note starts with the current ADSR settings.
    // Waveform is clamped to 0..15 
    const int waveformType = std::clamp(static_cast<int>(std::lround(in0(Waveform))), 0, 15);
    const bool gateInput   = in0(Gate) > 0.0f; // opens at > 0, like EnvGen
    const int controlReg   = (waveformType << 4) | (gateInput ? 0x01 : 0x00);

    // Update the control register when waveform or gate changes.
    if (controlReg != mPrevControl) {
        sid.write(CONTROL, static_cast<reSID::reg8>(controlReg));
        mPrevControl = controlReg;
    }
    
    // ---------------------------------------------------------------------------
    // Osc update: Hz -> 16-bit register, written only when it changes
    // ---------------------------------------------------------------------------

    // the older version was about 22x wrong in terms of freq...

    const float outScale = gainInput / 32768.0f; // 16-bit SID output -> -1..1, times gain

    for (int i = 0; i < nSamples; ++i) { // nSamples is the block size, NOT the hardware buffer size
        // ar frequency: read a fresh value per sample.
        if (freqIsAudioRate) {
            writeFreq(freqInput[i]);
        }

        // Carry fractional SID ticks between samples so the emulated clock (and pitch) stays exact.
        mCycleLeftover += mCyclesPerSample;                                             // add ~22.34 ticks
        const reSID::cycle_count dt = static_cast<reSID::cycle_count>(mCycleLeftover); // whole ticks: 22 or 23
        mCycleLeftover -= dt;                                                           // keep the fraction (0–1)

        sid.clock(dt);                              // oscillators, envelope, sync and filters advance by dt ticks
        outputBuffer[i] = sid.output() * outScale;
    }
}
} // namespace SIDOsc
PluginLoad(SIDOsc) {
    ft = inTable;
    registerUnit<SIDOsc::SIDOsc>(ft, "SIDOsc", false);
}
