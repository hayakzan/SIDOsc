SIDOsc : UGen {
    classvar <waveforms;

    *initClass {
        waveforms = (tri: 1, saw: 2, pulse: 4, noise: 8);
    }

    // \tri, \saw, \pulse or \noise -> waveform bits.
    // Arrays expand to multiple channels as usual; numbers and UGens pass through.
    *waveformCode { |wf|
        if (wf.isKindOf(Symbol)) {
            ^waveforms[wf] ?? {
                Error("SIDOsc: unknown waveform '%'. Use \\tri, \\saw, \\pulse or \\noise.".format(wf)).throw
            }
        };
        if (wf.isKindOf(SequenceableCollection)) { ^wf.collect { |w| this.waveformCode(w) } };
        ^wf
    }

    *ar { |freq = 440, gain = 1.0, waveform = \tri, pulseWidth = 0.5, gate = 1, env|
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
        ^this.multiNew('audio', freq, gain, this.waveformCode(waveform), pulseWidth, gate, a, d, s, r);
    }

    checkInputs {
        ^this.checkValidInputs;
    }
}
