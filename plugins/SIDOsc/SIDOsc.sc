SIDOsc : UGen {
    // waveform: 1 = triangle, 2 = saw, 4 = pulse, 8 = noise (add to combine)
    // env: an Env.adsr; times snap to the SID's 16 rates
    *ar { |freq = 440, gain = 1.0, waveform = 1, pulseWidth = 0.5, gate = 1, env|
        var a, d, s, r;

        env = env ?? { Env.adsr(0.002, 0.006, 1.0, 0.006) };
        if (env.times.size != 3 or: { env.releaseNode != 2 }) {
            Error("SIDOsc: env must be an Env.adsr").throw;
        };
        a = env.times[0];
        d = env.times[1];
        r = env.times[2];
        s = env.levels[2] / env.levels[1].max(1e-6);   // sustain as a fraction of peak

        // Order must match the Input enum in SIDOsc.hpp.
        ^this.multiNew('audio', freq, gain, waveform, pulseWidth, gate, a, d, s, r);
    }
    checkInputs {
        ^this.checkValidInputs;
    }
}
