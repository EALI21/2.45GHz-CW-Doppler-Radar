# 2.45 GHz CW Doppler Radar

An end-to-end 2.45 GHz Continuous-Wave (CW) Doppler radar developed as an Electrical and Electronics Engineering graduation project at Yeditepe University.

The project combines RF modules, directional antennas, an analog baseband circuit, an STM32 microcontroller, and a Python-based signal processing application. The system detects motion, identifies Doppler frequency components, and estimates the magnitude of a target's radial velocity.

## Overview

The radar transmits a continuous 2.45 GHz signal and receives the signal reflected by a moving target. A mixer combines the received echo with a reference signal derived from the same RF source. The resulting low-frequency signal contains the Doppler information used to detect motion and estimate radial velocity.

Unlike FMCW and pulsed radar systems, this fixed-frequency CW radar does not measure absolute target distance.

## System Architecture

### Transmitter and Local Oscillator (LO)

1. **ADF4351 frequency synthesizer** generates the 2.45 GHz continuous-wave carrier.
2. **Broadband RF amplifier** increases the signal power.
3. **Two-way RF power divider** splits the amplified signal into two branches:
   - The transmitting antenna branch.
   - The mixer LO reference branch.
4. **Transmitting Yagi antenna** radiates the signal toward the target.

The transmitter and mixer LO share the same RF source, providing a common frequency reference for Doppler detection.

### Receiver and Baseband

1. **Receiving Yagi antenna** receives the reflected signal.
2. **SPF5189Z low-noise amplifier (LNA)** amplifies the weak received signal.
3. **RF mixer** combines the received signal with the LO reference and produces a low-frequency difference component.
4. **MCP6022 analog circuit** conditions and amplifies the mixer output.
5. **STM32F103C8T6 (Blue Pill)** samples the analog signal and streams the data to a computer.
6. **Python application** processes the samples and displays the results.

## Hardware Components

| Component | Function |
|---|---|
| ADF4351 | Generates the 2.45 GHz carrier |
| Broadband RF amplifier | Increases transmitter and LO signal power |
| Two-way RF power divider | Splits the source signal between TX and LO |
| Two directional Yagi antennas | Transmit the CW signal and receive its reflection |
| SPF5189Z LNA | Amplifies the received RF signal |
| RF mixer | Converts the received RF signal to a low-frequency Doppler component |
| MCP6022 dual op-amp | Provides analog gain, biasing, and baseband filtering |
| STM32F103C8T6 | Acquires and transfers ADC samples |
| Computer running Python | Performs DSP, motion detection, visualization, and data logging |

The analog baseband circuit uses a buffered midpoint reference of approximately 1.65 V. Its input coupling stage has an approximate high-pass corner of 4.8 Hz, while the output low-pass stage has an approximate corner of 312 Hz. The adjustable non-inverting gain stage has a nominal gain of approximately 5.9 V/V.

## Embedded Data Acquisition

The STM32F103C8T6 acquires the baseband signal through ADC1 channel PA0.

Key acquisition parameters:

- **ADC resolution:** 12 bits
- **Nominal sampling rate:** 2,000 samples/s
- **Communication:** Native USB CDC
- **Packet size:** 64 ADC samples per data frame

The firmware sends framed sample data to the host computer, where the Python application reconstructs the sample stream for processing.

## Python Signal Processing

The Python application processes the incoming samples and provides a real-time monitoring interface.

### Motion Detection and Doppler Estimation

The processing pipeline includes:

- USB data-frame validation and sample reconstruction.
- Background estimation and removal of the sample mean before frequency analysis.
- Hann-windowed FFT analysis using a 4,096-sample window.
- A 256-sample hop between successive analysis windows.
- Doppler peak detection in the 5–300 Hz analysis band.
- Adaptive background estimation and rejection of selected stationary interference components.
- Motion-state classification and radial-speed estimation.

The main FFT window contains 2.048 seconds of samples, while successive analysis updates are separated by 128 ms. These values describe the analysis window and update interval, respectively.

The application also includes a short-window detector for brief motion events, a spectrogram, and a speed-history display.

### Live Dashboard and Data Logging

The application provides:

- Live baseband waveform.
- Doppler frequency spectrum.
- Spectrogram with recent signal history.
- Motion state and validated Doppler readings.
- Estimated radial-speed history.
- Diagnostic information.
- CSV recording of raw samples and processed results.

The CSV output includes elapsed time, raw ADC samples, validated Doppler frequency, estimated radial-speed magnitude, motion state, and dropped-packet count.

## Radial Velocity Calculation

For monostatic or approximately quasi-monostatic geometry, the magnitude of radial velocity is estimated from the Doppler frequency:

\[
|v_r| \approx \frac{c|f_D|}{2f_c}
=\frac{\lambda|f_D|}{2}
\]

where:

- \(v_r\) is the radial velocity.
- \(f_D\) is the Doppler frequency.
- \(f_c\) is the carrier frequency.
- \(\lambda\) is the carrier wavelength.
- \(c\) is the speed of light.

At 2.45 GHz, the wavelength is approximately 0.1224 m. Under the monostatic approximation, 1 Hz of Doppler shift corresponds to approximately 0.06118 m/s of radial-speed magnitude.

This conversion is an estimate based on the assumed radar geometry. Because the transmitting and receiving antennas are physically separate, the bistatic geometry can affect the relationship, particularly for nearby targets.

## AWR and LTspice Simulations

Two simulation tools were used for different parts of the system:

- **AWR:** RF carrier distribution, RF power estimates, and mixer frequency conversion.
- **LTspice:** Analog baseband circuit analysis, including gain and filter response.

The simulations support the design analysis but should not be interpreted as direct measurements of the physical radar. Simulated power levels and frequency-conversion results depend on the models and input assumptions used.

## System Capabilities and Limitations

**Motion detection:** The system detects target-related motion through changes in the received Doppler spectrum.

**Radial-speed estimation:** The application estimates radial-speed magnitude from an accepted Doppler frequency component.

**No absolute range measurement:** The radar uses a fixed CW carrier and does not determine the target's absolute distance.

**No approach/recession discrimination:** The current receiver uses a single real-valued baseband channel. It therefore reports speed magnitude and cannot reliably distinguish an approaching target from a receding target. An I/Q receiver and appropriate complex-signal processing would be required to determine the Doppler sign.

**Detection range:** A maximum detection range has not been established through a controlled range-characterization test. Detection depends on the target, antenna geometry, transmitted power, propagation conditions, receiver noise, interference, and detection thresholds.

**Calibration:** The displayed speed is derived from the Doppler conversion model. Independent reference-speed measurements have not established the absolute accuracy of the estimate.

## Installation and Usage

### Requirements

- Python environment compatible with the project's dependencies.
- STM32F103C8T6 development board with the project firmware.
- USB connection between the board and the computer.
- Completed RF and analog hardware assembly.

### Python Setup

Clone the repository:

```bash
git clone <repository-url>
cd <repository-directory>
```

If the repository includes a `requirements.txt` file, install the listed dependencies:

```bash
pip install -r requirements.txt
```

Connect the STM32 board to the computer and make sure the USB serial device is available. Run the receiver application using the actual script filename included in the repository. For example, if the file is named `radar_receiver.py`:

```bash
python radar_receiver.py
```

The serial port configuration may need to be adjusted for the computer's USB device.

### Firmware

Open the STM32 firmware project in STM32CubeIDE, build it, and flash the STM32F103C8T6. The firmware must use the same USB frame format expected by the Python application.

Follow the repository's actual firmware and wiring files when assembling or modifying the system.

## Project Background

This project was developed as an Electrical and Electronics Engineering graduation project at Yeditepe University. The design evolved from an initial FMCW concept into a fixed-frequency CW Doppler radar using commercially available RF modules and a custom analog baseband circuit.

The project combines RF system integration, frequency conversion, analog signal conditioning, embedded data acquisition, and digital signal processing in a working motion-detection system.

## Acknowledgements

The project was inspired in part by MIT Lincoln Laboratory's *Modular System RF Design – Build Your Own Small Radar System* (Williams, 2011), along with relevant radar engineering literature.
