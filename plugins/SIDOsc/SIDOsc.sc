SIDOsc : UGen {
    *ar { |freq = 440, gain = 1.0, waveform = 1, dacType = 0, gate = 1, env|
        var a, d, s, r;
		// only accepts Env.adsr()
		env = env ?? { Env.adsr(0.002, 0.006, 1.0, 0.006) };
        if (env.times.size != 3 or: { env.releaseNode != 2 }) {
            Error("SIDOsc: env must be an Env.adsr").throw;
        };
        a = env.times[0];
        d = env.times[1];
        r = env.times[2];
		s = max(env.levels[2] / env.levels[1]);   // sustain as a fraction of peak-(peakLevel*sustainLevel)/peakLevel
        ^this.multiNew('audio', freq, gain, waveform, dacType, gate, a, d, s, r);
    }
    checkInputs {
        // Ensures the inputs are valid (like non-negative freq, etc.)
        ^this.checkValidInputs;
    }
}




