#pragma once

#include "SC_PlugIn.hpp"
#include "wave.h"
#include "dac.h"
#include "voice.h"
#include "sid.h"
#include "filter.h"
#include "spline.h"
#include "extfilt.h"
#include "pot.h"
#include "envelope.h"
#include <vector>
#include <array>
#include <algorithm>

namespace SIDOsc {

class SIDOsc : public SCUnit {
public:
    SIDOsc();
    ~SIDOsc() = default;

    // SID-specific operations.
    reSID::reg8 readRegister(reSID::reg8 offset);
    void writeRegister(reSID::reg8 offset, reSID::reg8 value);
    //commout
//    void configureFilter(bool enable, double bias);
    void setSamplingParameters(double clockFreq = 985248.0, double sampleFreq = 44100.0);

private:
    void next(int nSamples);

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
    
    // Cache for the previous gate value and envelope parameter values.
    bool mPrevGate;
    //commout
//    float mPrevAttackInput, mPrevDecayInput, mPrevSustainInput, mPrevReleaseInput;
};

} // namespace SIDOsc
