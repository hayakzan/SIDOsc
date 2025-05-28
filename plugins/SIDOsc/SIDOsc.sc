SIDOsc : UGen {
    *ar { |freq = 440, gain = 1.0|
        // Create an audio-rate instance with two parameters:
        // frequency and gain
        ^this.multiNew('audio', freq, gain);
    }
    checkInputs {
        // Ensures the inputs are valid (like non-negative freq, etc.)
        ^this.checkValidInputs;
    }
}
