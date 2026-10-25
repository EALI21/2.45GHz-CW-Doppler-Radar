# 2.45 GHz CW Doppler Radar Receiver

This project is a desktop receiver and instrument-style GUI for a 2.45 GHz continuous-wave (CW) Doppler radar.

It communicates with an STM32F103C8T6 over native USB CDC. The program performs local USB acquisition, Doppler analysis, sustained-motion detection, stationary-interference rejection, calibration, visualization, and optional CSV recording.

## Hardware and packet format

- Carrier frequency: **2.45 GHz**
- ADC sample rate: **2000 samples/s**
- Application packet: **64 unsigned 12-bit ADC samples**
- Sustained-motion FFT: **4096 samples** (2.048 s)
- Doppler analysis band: **5 to 300 Hz**

USB packet layout:

```text
AA 55 | sequence uint16 little-endian | count uint16 little-endian (=64)
      | 64 × uint16 little-endian ADC samples
```

The packet parser accepts arbitrary USB read boundaries, validates the sample count and 12-bit ADC range, and resynchronizes after invalid data.

## Speed interpretation and limitations

The receiver displays the radial-speed magnitude:

```text
speed = |Doppler frequency| × 299792458 / (2 × 2.45e9)
```

At 2.45 GHz, one Doppler hertz is approximately **0.0611821 m/s**.

This is a single real baseband channel. It does not determine approaching versus receding direction, absolute target range, full two-dimensional velocity, or total target speed. Motion perpendicular to the instantaneous radar line of sight may produce little Doppler shift.

## Installation

Install dependencies:

```powershell
py -m pip install -r requirements.txt
```

Place the files in a folder of your choice, for example `C:\radar_project`, then run:

```powershell
cd C:\radar_project
py radar_receiver.py --label radial_walk
```

Useful commands:

```powershell
py radar_receiver.py --list-ports
py radar_receiver.py --port COM5 --label radial_walk
py radar_receiver.py --demo --label demo
```

The demo runs without hardware. It performs an empty-scene calibration, then produces a sustained 10 Hz signal, a short 31.25 Hz event, and fixed 82.5/165 Hz interference components. Demo data is not a physical radar measurement.

## Command-line options

```text
--port COMx                 Explicit serial port; otherwise auto-select
--list-ports                List serial ports and exit
--baud BAUD                 USB CDC line coding; default 115200
--buffer-size SAMPLES       Rolling sample buffer; default 8192, minimum 4096
--fft-size SAMPLES          FFT length; default/minimum 4096
--wave-seconds SECONDS      Raw waveform duration displayed in the GUI
--min-peak COUNTS           Motion-entry amplitude above adaptive baseline
--min-snr-db DB             Minimum peak-to-background amplitude ratio
--label TEXT                Optional CSV filename label
--record-dir PATH           CSV output directory
--demo                      Run with synthetic data and no hardware
```

## Keyboard controls

- **C**: Start a new 10-second empty-scene calibration.
- **R**: Start or stop CSV recording.

Recalibration does not stop USB acquisition, the GUI timer, or an active CSV recording. It replaces the no-motion baseline and learned stationary-interference bands using samples received after the key press.

## Dashboard

The interface shows:

- motion status, validated Doppler frequency, radial-speed magnitude, and peak SNR;
- live raw ADC waveform;
- raw Doppler spectrum and no-motion baseline;
- Doppler spectrogram;
- validated radial-speed history;
- USB, parser, detector, and interference diagnostics;
- recording status and CSV file information.

The large frequency card uses the median of the last three new validated FFT results for display only. Detector decisions, CSV output, the FFT peak marker, and speed history retain the raw validated frequency. A HOLD display is never a new measurement.

## Sustained-motion detection

`MotionDetector` uses contiguous 4096-sample windows with DC removal, a Hann window, and a real FFT. A sustained Doppler measurement must pass:

- the 5–300 Hz analysis band;
- adaptive no-motion baseline excess;
- peak SNR and local prominence;
- frequency tracking and entry/exit hysteresis;
- persistence across new FFT frames;
- a recent-window confirmation at the same tone.

The FFT advances by 256 new samples (128 ms). GUI repaints do not count as new detector frames.

The 5–50 Hz walking band normally requires three frames. Frequencies above 50 Hz require six frames and higher local prominence. Peak frequency is refined with a three-bin log-parabolic fit around the Hann-windowed peak. This reduces FFT-bin quantization; it does not calibrate the hardware clock or speed scale.

## Interference rejection

The fixed spurs near 82.5 Hz and 165 Hz are excluded from sustained-motion decisions while remaining visible in the FFT and spectrogram.

During empty-scene calibration, the receiver can learn up to 12 additional stable narrow-band interference lines. Learning requires repeated presence, high SNR and prominence, narrow bandwidth, and stable frequency/amplitude.

A separate monitor evaluates recent time-domain residual energy. It does not remove unknown candidate tones, because that could erase real motion. An unknown stationary line must persist for six seconds before `RECALIBRATION REQUIRED` is displayed. If it disappears for two seconds, the warning clears automatically.

During interference states, speed and frequency are hidden. Suspected results are not recorded as validated measurements.

A single-channel CW system cannot always distinguish a real constant Doppler target from an electrical tone at the same frequency. Keep the scene clear while calibrating and verify settings in the intended environment.

## Brief-motion detection

`BriefMotionDetector` is independent from the sustained-motion detector. It uses 512-sample windows (256 ms) with a 64-sample hop (32 ms). It checks amplitude, SNR, local prominence, baseline excess, time occupancy, tonal energy concentration, and sign changes.

A brief event is displayed as `BRIEF MOTION`, but it does not create a validated frequency or speed measurement and cannot modify the sustained detector.

## Calibration and display behavior

Startup calibration requires seven seconds of contiguous received samples. Missing packets, stale input, or an unprocessable buffer restart an unfinished calibration. The calibration derives the sustained-detector baseline, learned interference bands, and brief-event noise reference.

It is an empty-scene/noise calibration, not a physical speed-scale calibration.

| Setting | Default | Purpose |
| --- | ---: | --- |
| `DISPLAY_MOTION_HOLD_SECONDS` | 2.0 s | Holds the last validated value briefly |
| `DISPLAY_CLEAR_DELAY_SECONDS` | 1.0 s | Delays transition to no-motion status |
| `DISPLAY_SMOOTHING_WINDOW` | 3 frames | Uses a median in large display cards |
| `HISTORY_SECONDS` | 5.0 s | Retains recent speed and spectrogram history |
| `STARTUP_CALIBRATION_SECONDS` | 7.0 s | Required contiguous startup calibration |

Old values are cleared on a data gap, stale stream, USB error, calibration, or interference warning. History does not fabricate zero-speed points or connect across data gaps.

## Spectrogram

`SpectrogramBuffer` uses 1024-sample (512 ms) Hann windows with a 64-sample (32 ms) hop. It removes DC for each window. Frequency-bin spacing is approximately 1.953 Hz.

The display uses a fixed −20 to 50 dB scale relative to one ADC count of sinusoidal peak amplitude. It is not an RF power measurement. Fixed spurs remain visible for engineering inspection.

## CSV recording

Press **R** while the plot window is focused to toggle recording. Files are stored in `recordings` beside the Python program by default; use `--record-dir` to choose another location.

The optional `--label` is sanitized for filenames. Examples include `no_motion`, `radial_walk`, `parallel_walk`, and `hand_motion`.

Each accepted raw ADC sample is written once:

```text
elapsed_time_s,raw_adc_sample,validated_doppler_hz,radial_speed_mps,motion_state,dropped_packet_count
```

Frequency and speed fields contain only the latest causal sustained-detector result. They remain blank during calibration, candidates, no motion, data gaps, stale input, and interference states. Display medians, HOLD values, and brief-motion notices are never written as new measurements.

The CSV queue is bounded so disk delays do not block USB acquisition. If it fills, recording stops and reports an error while the live receiver continues.

## Recommended measurement sequence

1. Secure the radar and verify all connections.
2. Keep the scene still during startup calibration.
3. Confirm `RADAR READY`, a healthy ADC stream, and acceptable packet loss.
4. Record an empty-scene reference with `--label no_motion`.
5. Record motion along the antenna line of sight with `--label radial_walk`.
6. Record approximately parallel motion with `--label parallel_walk`.
7. Record short hand movement with `--label hand_motion`.
8. For speed-accuracy work, use a known distance/time reference and document target geometry and conditions.

Indoor tests demonstrate Doppler response but do not establish absolute speed accuracy. Multipath, target angle, multiple moving body parts, and FFT time averaging affect measurements.

## Tests

Run the automated tests without hardware:

```powershell
py -m unittest -v test_radar_receiver.py
```

The suite contains 103 synthetic tests for packet parsing, stream handling, FFT analysis, calibration, interference rejection, brief events, display state, CSV behavior, and GUI layout. Physical hardware behavior must still be verified in the intended environment.

## Dependencies

```text
pyserial>=3.5,<4
numpy>=1.26,<3
matplotlib>=3.8,<4
```

## References

- [Radial velocity and viewing angle — Embry-Riddle Aeronautical University](https://wx.erau.edu/faculty/mullerb/Wx365/Doppler_velocity/radial_velocity.html)
- [Matplotlib imshow documentation](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.imshow.html)
- [NumPy linear algebra documentation](https://numpy.org/doc/stable/reference/generated/numpy.linalg.lstsq.html)