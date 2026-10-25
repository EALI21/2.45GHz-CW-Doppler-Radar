"""Continuous STM32 radar receiver. Run: py radar_receiver.py --port COM5

Install: py -m pip install -r requirements.txt
Wire format: AA 55, sequence LE16, count LE16 (=64), 64 ADC samples LE16.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import queue
import struct
import sys
import threading
import time

import numpy as np

FS = 2000.0
CARRIER_HZ = 2.45e9
LIGHT_SPEED = 299792458.0
SPEED_PER_HZ = LIGHT_SPEED / (2.0 * CARRIER_HZ)
SYNC = b"\xaa\x55"
PACKET_SAMPLES = 64
PACKET_BYTES = 6 + 2 * PACKET_SAMPLES
HEADER = struct.Struct("<2sHH")
SAMPLES = struct.Struct("<64H")

# Detection defaults: starting values for the reported experiments, not a
# calibration from recorded measurements. ADC amplitude units are counts.
ANALYSIS_BAND_HZ = (5.0, 300.0)
# Empirically observed fixed spurs in the NO-MOTION baseline. Exclude only
# these narrow bands from detection; keep them visible in the raw FFT plot.
SPUR_FREQUENCIES_HZ = (82.5, 165.0)
SPUR_EXCLUSION_HALF_WIDTH_HZ = 2.0
BASELINE_ADAPTATION_RATE = 0.01  # Per new analysis frame; about 13 s time constant.
NOISE_FLOOR_COUNTS = 0.05
MIN_PEAK_SNR_DB = 10.0
MIN_PEAK_PROMINENCE_DB = 6.0
LOCAL_BACKGROUND_RADIUS_HZ = 10.0
PEAK_GUARD_HZ = 2.0
REQUIRED_PERSISTENCE_FRAMES = 3
FREQUENCY_TRACKING_TOLERANCE_HZ = 2.0
FREQUENCY_TRACKING_TOLERANCE_FRACTION = 0.05
MOTION_ENTER_AMPLITUDE = 2.0  # Excess over baseline; retain SNR/prominence/persistence gates.
MOTION_EXIT_AMPLITUDE = 1.1  # Keep an established track through smaller amplitude dips.
MOTION_ENTER_BASELINE_DB = 6.0
MOTION_EXIT_BASELINE_DB = 3.5
WALKING_BAND_HZ = (5.0, 50.0)
WALKING_PEAK_WEIGHT = 2.0  # Soft preference; strong higher-frequency motion can win.
FFT_HOP_SAMPLES = 256  # One decision per 128 ms of NEW data, independent of GUI rate.
RECENT_WINDOW_SAMPLES = 512  # Confirm the tone is still present in the last 256 ms.
MIN_RECENT_AMPLITUDE_FRACTION = 0.25
STALE_SECONDS = 1.0
CSV_QUEUE_PACKETS = 256  # About 8 seconds of disk-stall tolerance.

# Presentation only: none of these values affects detection or CSV semantics.
DISPLAY_MOTION_HOLD_SECONDS = 2.0
DISPLAY_CLEAR_DELAY_SECONDS = 1.0
DISPLAY_SMOOTHING_WINDOW = 3  # Less lag while retaining rejection of a single-frame outlier.
HISTORY_SECONDS = 5.0

STARTUP_CALIBRATION_SECONDS = 7.0
RECALIBRATION_SECONDS = 10.0
HIGH_FREQUENCY_PERSISTENCE_FRAMES = 6
HIGH_FREQUENCY_PROMINENCE_DB = 10.0
INTERFERENCE_MIN_PRESENCE = 0.85
INTERFERENCE_MIN_SNR_DB = 18.0
INTERFERENCE_MIN_PROMINENCE_DB = 10.0
INTERFERENCE_MIN_AMPLITUDE = 2.5  # Independent of the more sensitive motion threshold.
INTERFERENCE_HALF_WIDTH_HZ = 1.0
INTERFERENCE_MAX_HALF_WIDTH_HZ = 2.0
INTERFERENCE_MAX_BANDS = 12
INTERFERENCE_CONFIRM_FRAMES = 3
INTERFERENCE_RECALIBRATE_SECONDS = 6.0  # Continuous stationary evidence, excluding real motion.
INTERFERENCE_CLEAR_SECONDS = 2.0  # Recover automatically when unknown interference disappears.
INTERFERENCE_STATES = ("stationary_interference", "interference_low_confidence", "recalibration_required")
# Independent short-event classifier. These do not retune MotionDetector.
BRIEF_WINDOW_SAMPLES = 512
BRIEF_HOP_SAMPLES = 64
BRIEF_MIN_AMPLITUDE = 5.0  # Keep the independent coherence and impulse-rejection gates.
BRIEF_MIN_SNR_DB = 15.0
BRIEF_MIN_PROMINENCE_DB = 8.0
BRIEF_MIN_BASELINE_DB = 10.0
BRIEF_MIN_EFFECTIVE_SECONDS = 0.080
BRIEF_MAX_50MS_ENERGY_FRACTION = 0.60
BRIEF_MIN_TONE_ENERGY_FRACTION = 0.50
BRIEF_REQUIRED_FRAMES = 2
BRIEF_DISPLAY_SECONDS = 0.6
SPECTROGRAM_WINDOW_SAMPLES = 1024
SPECTROGRAM_HOP_SAMPLES = 64

# Light instrument theme. State text and line styles also carry meaning, so
# the dashboard does not depend on color alone. All panels use a white surface.
DASHBOARD_COLORS = {
    "background": "#eef3f8", "surface": "#ffffff", "ink": "#172b42",
    "muted": "#52667b", "border": "#d5dfe9", "grid": "#e5ecf2",
    "signal": "#216b9d", "baseline": "#7a8da2", "validated": "#00756a",
    "hold": "#946000", "warning": "#a34729", "recording": "#b52e3d",
}


@dataclass(frozen=True)
class Detection:
    state: str = "warming_up"
    validated_hz: float | None = None
    candidate_hz: float | None = None
    persistence: int = 0

    @property
    def speed_mps(self):
        return None if self.validated_hz is None else self.validated_hz * SPEED_PER_HZ


class CsvRecorder:
    """Queue accepted packets to disk without blocking the USB reader on writes."""

    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.worker = None
        self.path = None
        self.message = "Recording off | R: start/stop recording"
        self.rows_written = 0

    def start(self, directory, label, sample_origin):
        if self.worker is not None and self.worker.is_alive():
            raise RuntimeError("Previous recording is still closing")
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        safe_label = "".join(c if c.isalnum() or c in "-_" else "_" for c in label)[:60]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = directory / f"radar_{stamp}{'_' + safe_label if safe_label else ''}.csv"
        handle = self.path.open("x", newline="", encoding="utf-8")
        with self.lock:
            self.pending = queue.Queue(maxsize=CSV_QUEUE_PACKETS)
            self.finish = threading.Event()
            self.origin = sample_origin
            self.rows_written = 0
            self.active = True
            self.message = f"REC: {self.path.name} | R: stop"
            self.worker = threading.Thread(target=self._write, args=(handle,), daemon=True)
            self.worker.start()
        return self.path

    def enqueue(self, start_index, samples, detection, dropped_packets):
        with self.lock:
            if not self.active:
                return
            try:
                self.pending.put_nowait((start_index, samples, detection, dropped_packets))
            except queue.Full:
                self.active = False
                self.finish.set()
                self.message = "CSV recording stopped: disk queue full; queued rows will be saved"

    def _write(self, handle):
        try:
            with handle:
                writer = csv.writer(handle)
                writer.writerow(["elapsed_time_s", "raw_adc_sample", "validated_doppler_hz",
                                 "radial_speed_mps", "motion_state", "dropped_packet_count"])
                last_flush = time.monotonic()
                while not self.finish.is_set() or not self.pending.empty():
                    try:
                        start, samples, detection, dropped = self.pending.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    frequency = "" if detection.validated_hz is None else f"{detection.validated_hz:.6f}"
                    speed = "" if detection.speed_mps is None else f"{detection.speed_mps:.6f}"
                    writer.writerows((f"{(start + i - self.origin) / FS:.6f}", value,
                                      frequency, speed, detection.state, dropped)
                                     for i, value in enumerate(samples))
                    self.rows_written += len(samples)
                    if time.monotonic() - last_flush >= 1.0:
                        handle.flush()
                        last_flush = time.monotonic()
        except (OSError, ValueError) as exc:
            with self.lock:
                self.active = False
                self.finish.set()
                self.message = f"CSV error: {exc}; live acquisition continues"

    def stop(self):
        with self.lock:
            was_active = self.active
            self.active = False
            if self.worker is None:
                return
            self.finish.set()
        self.worker.join(timeout=2.0)
        if was_active:
            if self.worker.is_alive():
                self.message = "CSV is still flushing; wait before starting another recording"
            else:
                self.message = f"Saved {self.rows_written} rows: {self.path.name} | R: record"


@dataclass(frozen=True)
class Packet:
    sequence: int
    samples: tuple[int, ...]


class PacketParser:
    """Incremental byte-stream parser; USB reads need not match packet sizes."""

    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[Packet]:
        self.buffer.extend(data)
        packets = []
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                # A trailing AA might be the first byte of a split header.
                keep = 1 if self.buffer.endswith(b"\xaa") else 0
                remove = len(self.buffer) - keep
                self.discarded_bytes += remove
                del self.buffer[:remove]
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            _, sequence, count = HEADER.unpack_from(self.buffer)
            if count != PACKET_SAMPLES:
                self.invalid_frames += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < PACKET_BYTES:
                break
            values = SAMPLES.unpack_from(self.buffer, HEADER.size)
            if any(value > 4095 for value in values):
                # ADC1 is 12-bit, right aligned. This also helps resynchronize.
                self.invalid_frames += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            packets.append(Packet(sequence, values))
            del self.buffer[:PACKET_BYTES]
        return packets


class StreamState:
    """One reader thread owns writes; the GUI reads snapshots under a lock."""

    def __init__(self, capacity: int = 8192, recorder=None):
        if capacity < 4096:
            raise ValueError("Rolling capacity must be at least 4096 samples")
        self.samples = deque(maxlen=capacity)
        self.expected_sequence = None
        self.received_packets = 0
        self.dropped_packets = 0
        self.duplicate_packets = 0
        self.sequence_restarts = 0
        self.last_packet_time = None
        self.discarded_bytes = 0
        self.invalid_frames = 0
        self.error = ""
        self.sample_index = 0  # Monotonic sample timeline, including known packet gaps.
        self.discontinuity = 0
        self.analysis = Detection()
        self.analysis_sample_index = 0
        self.recorder = recorder
        self.lock = threading.Lock()

    def accept(self, packet: Packet) -> bool:
        """Caller holds lock. Forward gaps below half the sequence range count.

        Backward jumps are treated as a device restart. A 16-bit sequence alone
        cannot unambiguously distinguish restarts from very long outages.
        """
        if self.expected_sequence is not None:
            delta = (packet.sequence - self.expected_sequence) & 0xFFFF
            if delta == 0xFFFF:
                self.duplicate_packets += 1
                return False
            if 0 < delta < 0x8000:
                self.dropped_packets += delta
                self.sample_index += delta * PACKET_SAMPLES
                self.discontinuity += 1
                self.analysis = Detection("data_gap")
                # Preserve the time gap; do not FFT concatenated missing data.
                missing = min(delta * PACKET_SAMPLES, self.samples.maxlen)
                self.samples.extend([math.nan] * missing)
            elif delta >= 0x8000:
                self.sequence_restarts += 1
                self.samples.clear()
                self.discontinuity += 1
                self.analysis = Detection("data_gap")
        self.expected_sequence = (packet.sequence + 1) & 0xFFFF
        self.samples.extend(packet.samples)
        start_index = self.sample_index
        self.sample_index += len(packet.samples)
        self.received_packets += 1
        self.last_packet_time = time.monotonic()
        if self.recorder is not None:
            # Latest causal analysis result; no repeated recording of overlapping
            # FFT windows. Leave frequency/speed blank when not validated.
            detection = self.analysis
            if (detection.validated_hz is not None
                    and start_index - self.analysis_sample_index > STALE_SECONDS * FS):
                detection = Detection("stale")
            self.recorder.enqueue(start_index, packet.samples, detection, self.dropped_packets)
        return True

    def publish_analysis(self, detection, sample_index, discontinuity):
        with self.lock:
            # A packet gap may have arrived while the GUI calculated the FFT.
            if discontinuity == self.discontinuity:
                self.analysis = detection
                self.analysis_sample_index = sample_index

    def feed(self, parser: PacketParser, data: bytes):
        packets = parser.feed(data)
        with self.lock:
            for packet in packets:
                self.accept(packet)
            self.discarded_bytes = parser.discarded_bytes
            self.invalid_frames = parser.invalid_frames

    def snapshot(self):
        with self.lock:
            return np.asarray(self.samples, dtype=np.float64), {
                "packets": self.received_packets,
                "dropped": self.dropped_packets,
                "duplicates": self.duplicate_packets,
                "restarts": self.sequence_restarts,
                "discarded_bytes": self.discarded_bytes,
                "invalid_frames": self.invalid_frames,
                "last_packet_time": self.last_packet_time,
                "error": self.error,
                "sample_index": self.sample_index,
                "discontinuity": self.discontinuity,
            }


@dataclass(frozen=True)
class Spectrum:
    frequencies: np.ndarray
    amplitudes: np.ndarray
    peak_hz: float
    peak_amplitude: float
    peak_to_floor_db: float
    motion_detected: bool

    @property
    def speed_mps(self):
        # A single real ADC channel gives speed magnitude, not direction.
        return self.peak_hz * SPEED_PER_HZ


def analyze(samples, fft_size=4096, min_peak=MOTION_ENTER_AMPLITUDE, min_snr_db=MIN_PEAK_SNR_DB):
    """DC removal -> Hann -> real FFT -> peak restricted to 5..300 Hz.

    Returns None until a full contiguous window is available. Amplitude units
    are ADC counts (approximate sinusoidal peak amplitude, window corrected).
    This stateless function retains the raw FFT/legacy threshold diagnostics.
    Only MotionDetector may declare a VALIDATED Doppler result.
    """
    x = np.asarray(samples, dtype=np.float64)[-fft_size:]
    if len(x) != fft_size or not np.all(np.isfinite(x)):
        return None
    x = x - np.mean(x)  # A bias near 2048 is normal and must not become motion.
    window = np.hanning(fft_size)
    magnitude = 2.0 * np.abs(np.fft.rfft(x * window)) / np.sum(window)
    frequency = np.fft.rfftfreq(fft_size, d=1.0 / FS)
    band = (frequency >= ANALYSIS_BAND_HZ[0]) & (frequency <= ANALYSIS_BAND_HZ[1])
    frequencies, amplitudes = frequency[band], magnitude[band]
    peak_index = int(np.argmax(amplitudes))
    peak_amplitude = float(amplitudes[peak_index])
    floor = max(float(np.median(amplitudes)), 1e-12)
    snr = 20.0 * math.log10(max(peak_amplitude, 1e-12) / floor)
    return Spectrum(
        frequencies, amplitudes, float(frequencies[peak_index]), peak_amplitude,
        snr, peak_amplitude >= min_peak and snr >= min_snr_db,
    )


@dataclass(frozen=True)
class Peak:
    hz: float
    excess: float
    baseline_db: float
    snr_db: float
    prominence_db: float


def refined_peak_frequency(frequencies, amplitudes, index):
    """Three-bin log-parabolic Hann peak estimate; raw FFT arrays stay intact.

    This reduces bin quantization, not the FFT window's temporal averaging.
    Boundary or degenerate peaks retain their measured bin center.
    """
    center = float(frequencies[index])
    if not 0 < index < len(frequencies)-1:
        return center
    values = np.asarray(amplitudes[index-1:index+2], dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values <= 0):
        return center
    if values[1] <= values[0] or values[1] < values[2]:
        return center
    y = np.log(values)
    curvature = y[0]-2*y[1]+y[2]
    if curvature >= -1e-12:
        return center
    offset = float(np.clip(0.5*(y[0]-y[2])/curvature, -0.5, 0.5))
    return center + offset*float(frequencies[index+1]-frequencies[index])


@dataclass(frozen=True)
class InterferenceBand:
    frequency_hz: float
    half_width_hz: float
    amplitude: float
    presence: float


class StationaryInterference:
    """Learn narrow bands only from explicit empty-scene calibration.

    Raw spectra remain intact. Time-domain evidence removes only KNOWN nuisance
    tones: removing unknown candidate tones would also erase real motion.
    """

    def __init__(self):
        self.bands = []
        self.calibrated = False
        self.energy_limit = self.residual_rms = None
        self.recalibration_required = False
        self.reset_watch()

    def reset_watch(self):
        self._reset_stationary_run()
        self.recovery_start = self.recovery_last_index = None

    def _reset_stationary_run(self):
        self.watch_hz = self.watch_start = self.last_index = None
        self.watch_frames = 0

    def _update_recovery(self, sample_index, unknown_stationary):
        # The warning describes current evidence; it must not latch forever.
        # Learned bands/baselines stay intact: this never learns motion away.
        if unknown_stationary or not self.recalibration_required:
            self.recovery_start = self.recovery_last_index = None
            return
        if (self.recovery_last_index is None
                or not 0 < sample_index-self.recovery_last_index <= STALE_SECONDS*FS):
            self.recovery_start = sample_index
        self.recovery_last_index = sample_index
        if sample_index-self.recovery_start >= INTERFERENCE_CLEAR_SECONDS*FS:
            self.recalibration_required = False
            self.recovery_start = self.recovery_last_index = None

    def contains(self, hz):
        return any(abs(hz-b.frequency_hz) <= b.half_width_hz for b in self.bands)

    def allowed(self, frequencies):
        allowed = np.ones(len(frequencies), dtype=bool)
        for band in self.bands:
            allowed &= abs(frequencies-band.frequency_hz) > band.half_width_hz
        return allowed

    @staticmethod
    def lines(f, amplitude):
        """Strong local maxima with narrow half-amplitude width."""
        floor = max(float(np.median(amplitude)), NOISE_FLOOR_COUNTS)
        found = []
        df = float(f[1]-f[0])
        for i in range(len(f)):
            peak = amplitude[i]
            if peak < max(INTERFERENCE_MIN_AMPLITUDE, floor*10**(INTERFERENCE_MIN_SNR_DB/20)):
                continue
            if (i > 0 and peak <= amplitude[i-1]) or (i+1 < len(f) and peak < amplitude[i+1]):
                continue
            distance = abs(f-f[i])
            neighbors = (distance >= max(2.0, 3*df)) & (distance <= 10)
            background = max(float(np.median(amplitude[neighbors])), NOISE_FLOOR_COUNTS)
            if 20*math.log10(peak/background) < INTERFERENCE_MIN_PROMINENCE_DB:
                continue
            left, right = i, i
            while left > 0 and amplitude[left-1] >= peak*0.5:
                left -= 1
            while right+1 < len(f) and amplitude[right+1] >= peak*0.5:
                right += 1
            if (right-left+1)*df > 3.0:
                continue
            offset = 0.0
            if 0 < i < len(f)-1:
                y = np.log(np.maximum(amplitude[i-1:i+2], 1e-12))
                denominator = y[0]-2*y[1]+y[2]
                if abs(denominator) >= 1e-12:
                    offset = np.clip(0.5*(y[0]-y[2])/denominator, -0.5, 0.5)
            found.append((i, float(f[i]+offset*df), float(peak)))
        return found

    def learn(self, frequencies, frames, time_windows):
        stack = np.asarray(frames)
        median = np.median(stack, axis=0)
        observed = [self.lines(frequencies, frame) for frame in stack]
        self.bands = []
        for _, hz, amplitude in sorted(self.lines(frequencies, median), key=lambda line: -line[2]):
            if any(abs(hz-spur) <= SPUR_EXCLUSION_HALF_WIDTH_HZ for spur in SPUR_FREQUENCIES_HZ):
                continue
            hits = []
            for lines in observed:
                nearby = [line for line in lines if abs(line[1]-hz) <= INTERFERENCE_HALF_WIDTH_HZ]
                if nearby:
                    hits.append(min(nearby, key=lambda line: abs(line[1]-hz)))
            presence = len(hits)/len(stack)
            if presence < INTERFERENCE_MIN_PRESENCE:
                continue
            levels = np.array([line[2] for line in hits])
            if np.median(abs(levels-np.median(levels))) > 0.35*np.median(levels):
                continue
            centers = np.array([line[1] for line in hits])
            center = float(np.median(centers))
            width = max(INTERFERENCE_HALF_WIDTH_HZ,
                        float(np.quantile(abs(centers-center), 0.95)) + 2*(frequencies[1]-frequencies[0]))
            if width > INTERFERENCE_MAX_HALF_WIDTH_HZ:
                continue
            if any(abs(center-band.frequency_hz) < width+band.half_width_hz for band in self.bands):
                continue
            self.bands.append(InterferenceBand(center, width, amplitude, presence))
            if len(self.bands) >= INTERFERENCE_MAX_BANDS:
                break
        self.bands.sort(key=lambda band: band.frequency_hz)
        t = np.arange(RECENT_WINDOW_SAMPLES)/FS
        columns = [np.ones(len(t)), np.linspace(-1, 1, len(t))]
        for hz in (*SPUR_FREQUENCIES_HZ, *(band.frequency_hz for band in self.bands)):
            columns.extend((np.sin(2*np.pi*hz*t), np.cos(2*np.pi*hz*t)))
        self.nuisance = np.column_stack(columns)
        self.inverse = np.linalg.pinv(self.nuisance)
        rms = np.array([self.motion_rms(values) for values in time_windows])
        reference = float(np.median(rms))
        scatter = float(1.4826*np.median(abs(rms-reference)))
        self.energy_limit = reference + max(0.25, 4*scatter, 0.15*reference)
        self.calibrated = True
        self.recalibration_required = False
        self.reset_watch()

    def motion_rms(self, samples):
        values = np.asarray(samples[-RECENT_WINDOW_SAMPLES:], dtype=float)
        residual = values - self.nuisance @ (self.inverse @ values)
        return math.sqrt(float(np.mean(residual**2)))

    def review(self, detection, spectrum, samples, sample_index):
        pending_high = detection.candidate_hz is not None and detection.candidate_hz > WALKING_BAND_HZ[1]
        uncertain = Detection("interference_low_confidence", candidate_hz=detection.candidate_hz,
                              persistence=detection.persistence)
        if not self.calibrated:
            return uncertain if pending_high else detection
        self.residual_rms = self.motion_rms(samples)
        near_baseline = self.residual_rms <= self.energy_limit
        lines = [line for line in self.lines(spectrum.frequencies, spectrum.amplitudes)
                 if not any(abs(line[1]-spur) <= SPUR_EXCLUSION_HALF_WIDTH_HZ for spur in SPUR_FREQUENCIES_HZ)]
        unknown_lines = [line for line in lines if not self.contains(line[1])]
        watched = max(unknown_lines or lines, key=lambda line: line[2]) if lines else None
        # Match the existing motion-safety decision before accumulating any
        # stationarity time. A long real movement must not pre-fill this timer.
        suspicious_line = detection.validated_hz is not None and any(
            abs(line[1]-detection.validated_hz) <= INTERFERENCE_HALF_WIDTH_HZ
            for line in unknown_lines)
        confident_motion = detection.validated_hz is not None and (
            not near_baseline or (detection.validated_hz <= WALKING_BAND_HZ[1] and not suspicious_line))
        stationary = near_baseline and watched is not None and not confident_motion
        unknown_stationary = stationary and not self.contains(watched[1])
        self._update_recovery(sample_index, unknown_stationary)
        if not stationary:
            self._reset_stationary_run()
        else:
            hz = watched[1]
            consecutive = (self.last_index is not None and 0 < sample_index-self.last_index <= STALE_SECONDS*FS
                           and self.watch_hz is not None and abs(hz-self.watch_hz) <= INTERFERENCE_HALF_WIDTH_HZ)
            self.watch_frames = self.watch_frames+1 if consecutive else 1
            self.watch_start = self.watch_start if consecutive else sample_index
            # Anchor to the first frequency, not the preceding frame. A moving
            # ridge can otherwise drift indefinitely while looking "stationary".
            if not consecutive:
                self.watch_hz = hz
            self.last_index = sample_index
        if confident_motion:
            return detection
        if self.recalibration_required:
            return Detection("recalibration_required")
        if stationary:
            if self.watch_frames >= INTERFERENCE_CONFIRM_FRAMES:
                if unknown_stationary and sample_index-self.watch_start >= INTERFERENCE_RECALIBRATE_SECONDS*FS:
                    self.recalibration_required = True
                return Detection("recalibration_required" if self.recalibration_required else "stationary_interference")
            return uncertain if pending_high or detection.validated_hz is not None else detection
        if pending_high or (detection.validated_hz is not None and near_baseline):
            return uncertain
        return detection


class MotionDetector:
    """Small stateful layer on the existing mean-removed, Hann-windowed FFT."""

    def __init__(self, fft_size=4096, min_peak=MOTION_ENTER_AMPLITUDE,
                 min_snr_db=MIN_PEAK_SNR_DB):
        self.fft_size = fft_size
        self.enter_amplitude = min_peak
        self.exit_amplitude = min_peak * MOTION_EXIT_AMPLITUDE / MOTION_ENTER_AMPLITUDE
        self.min_snr_db = min_snr_db
        self.baseline = None
        self.spectrum = None
        self.last_frame_index = None
        self.discontinuity = None
        self.track_hz = None
        self.candidate_hz = None
        self.candidate_frames = 0
        self.result = Detection()
        self.interference = StationaryInterference()

    def reset_track(self, state="no_motion"):
        self.track_hz = self.candidate_hz = None
        self.candidate_frames = 0
        self.result = Detection(state)

    @staticmethod
    def close_frequency(a, b):
        tolerance = max(FREQUENCY_TRACKING_TOLERANCE_HZ,
                        FREQUENCY_TRACKING_TOLERANCE_FRACTION * b)
        return abs(a - b) <= tolerance

    def allowed_bins(self, frequencies):
        allowed = np.ones(len(frequencies), dtype=bool)
        for spur in SPUR_FREQUENCIES_HZ:
            allowed &= np.abs(frequencies - spur) > SPUR_EXCLUSION_HALF_WIDTH_HZ
        return allowed & self.interference.allowed(frequencies)

    def peaks(self, spectrum, recent_samples):
        f, raw = spectrum.frequencies, spectrum.amplitudes
        allowed = self.allowed_bins(f)
        floor = max(float(np.median(raw[allowed])), NOISE_FLOOR_COUNTS)
        if self.baseline is None:
            # Start from the robust broadband floor, NOT the first raw spectrum:
            # starting the receiver during strong motion must not learn it away.
            self.baseline = np.full_like(raw, floor)
        excess = np.maximum(raw - self.baseline, 0.0)
        recent = np.asarray(recent_samples[-RECENT_WINDOW_SAMPLES:], dtype=float)
        recent = recent - np.mean(recent)
        window = np.hanning(len(recent))
        weighted = recent * window
        times = np.arange(len(recent)) / FS
        found = []
        for i in range(len(f)):
            # Find maxima in the ORIGINAL spectrum, before excluding bands. This
            # avoids turning the shoulders of a rejected spur into artificial peaks.
            if (not allowed[i] or (i > 0 and raw[i] <= raw[i-1])
                    or (i + 1 < len(f) and raw[i] < raw[i+1])):
                continue
            if excess[i] < self.exit_amplitude:
                continue
            distance = np.abs(f - f[i])
            neighborhood = (distance >= PEAK_GUARD_HZ) & (distance <= LOCAL_BACKGROUND_RADIUS_HZ) & allowed
            background = max(float(np.median(raw[neighborhood])) if np.any(neighborhood) else floor,
                             NOISE_FLOOR_COUNTS)
            snr = 20.0 * math.log10(max(raw[i], NOISE_FLOOR_COUNTS) / floor)
            prominence = 20.0 * math.log10(max(raw[i], NOISE_FLOOR_COUNTS) / background)
            baseline_db = 20.0 * math.log10(max(raw[i], NOISE_FLOOR_COUNTS)
                                          / max(self.baseline[i], NOISE_FLOOR_COUNTS))
            hz = refined_peak_frequency(f, raw, i)
            # Interpolation must not move a neighboring FFT bin into an excluded
            # band and accidentally turn a rejected interference line into speed.
            if not self.allowed_bins(np.array([hz]))[0]:
                continue
            required_prominence = HIGH_FREQUENCY_PROMINENCE_DB if hz > WALKING_BAND_HZ[1] else MIN_PEAK_PROMINENCE_DB
            if snr < self.min_snr_db or prominence < required_prominence:
                continue
            # Overlapping 2.048 s FFTs can retain an old impulse for many updates.
            # A short-window projection checks for current energy at this SAME tone.
            recent_amplitude = 2.0 * abs(np.sum(weighted * np.exp(-2j * np.pi * hz * times))) / np.sum(window)
            if recent_amplitude < max(self.exit_amplitude, MIN_RECENT_AMPLITUDE_FRACTION * raw[i]):
                continue
            found.append(Peak(hz, float(excess[i]), baseline_db, snr, prominence))
        return found

    @staticmethod
    def rank(peak):
        weight = WALKING_PEAK_WEIGHT if WALKING_BAND_HZ[0] <= peak.hz <= WALKING_BAND_HZ[1] else 1.0
        return weight * peak.excess

    def process(self, samples, sample_index, discontinuity=0, stale=False):
        if discontinuity != self.discontinuity:
            self.reset_track("warming_up")
            self.interference.reset_watch()
            self.last_frame_index = None
            self.discontinuity = discontinuity
        if stale:
            self.reset_track("stale")
            self.interference.reset_watch()
            self.last_frame_index = None
            return self.result
        if (self.last_frame_index is not None
                and sample_index - self.last_frame_index < FFT_HOP_SAMPLES):
            return self.result  # Repainting the same/too little new data is not persistence.
        step = FFT_HOP_SAMPLES if self.last_frame_index is None else sample_index - self.last_frame_index
        if step > STALE_SECONDS * FS:
            self.reset_track()  # Do not count widely separated frames as consecutive.
            self.interference.reset_watch()
        self.last_frame_index = sample_index
        self.spectrum = analyze(samples, self.fft_size, self.enter_amplitude, self.min_snr_db)
        if self.spectrum is None:
            self.reset_track("warming_up")
            return self.result
        peaks = self.peaks(self.spectrum, samples)
        stay = [p for p in peaks if p.excess >= self.exit_amplitude
                and p.baseline_db >= MOTION_EXIT_BASELINE_DB]
        enter = [p for p in stay if p.excess >= self.enter_amplitude
                 and p.baseline_db >= MOTION_ENTER_BASELINE_DB]
        tracked = [p for p in stay if self.track_hz is not None and self.close_frequency(p.hz, self.track_hz)
                   and (p.hz <= WALKING_BAND_HZ[1] or self.track_hz > WALKING_BAND_HZ[1])]
        if tracked:
            self.track_hz = max(tracked, key=self.rank).hz
            self.candidate_hz, self.candidate_frames = None, 0
            self.result = Detection("validated_motion", validated_hz=self.track_hz)
        elif enter:
            peak = max(enter, key=self.rank)
            if (self.candidate_hz is not None and self.close_frequency(peak.hz, self.candidate_hz)
                    and (peak.hz > WALKING_BAND_HZ[1]) == (self.candidate_hz > WALKING_BAND_HZ[1])):
                self.candidate_frames += 1
            else:
                self.candidate_frames = 1
            self.candidate_hz = peak.hz
            required_frames = HIGH_FREQUENCY_PERSISTENCE_FRAMES if peak.hz > WALKING_BAND_HZ[1] else REQUIRED_PERSISTENCE_FRAMES
            if self.candidate_frames >= required_frames:
                self.track_hz = peak.hz
                self.candidate_hz, self.candidate_frames = None, 0
                self.result = Detection("validated_motion", validated_hz=self.track_hz)
            else:
                # A jump from a low-frequency track to a distant peak is only a
                # candidate; never display the new speed before it persists.
                self.result = Detection("candidate_motion", candidate_hz=peak.hz,
                                        persistence=self.candidate_frames)
        else:
            self.reset_track()
            if not stay:
                # Learn ONLY no-motion frames. Freeze during candidates/motion;
                # cap upward changes to resist a transient rejected by other gates.
                alpha = 1.0 - (1.0 - BASELINE_ADAPTATION_RATE) ** (step / FFT_HOP_SAMPLES)
                target = np.minimum(self.spectrum.amplitudes,
                                    self.baseline * 2.0 + NOISE_FLOOR_COUNTS)
                self.baseline += alpha * (target - self.baseline)
                self.baseline = np.maximum(self.baseline, NOISE_FLOOR_COUNTS)
        was_validated = self.result.validated_hz is not None
        self.result = self.interference.review(self.result, self.spectrum, samples, sample_index)
        if self.result.state in INTERFERENCE_STATES and (was_validated or self.result.state != "interference_low_confidence"):
            self.track_hz = None
        return self.result


def new_windows(samples, sample_index, last_end, size, hop):
    """Bounded catch-up on an absolute sample grid; never replay GUI frames."""
    first = sample_index - len(samples)
    start = max(first + size, size if last_end is None else last_end + hop)
    start = ((start + hop - 1) // hop) * hop
    emitted = False
    for end in range(start, sample_index + 1, hop):
        emitted = True
        yield end, samples[end - size - first:end - first]
    # With a minimum-size rolling buffer there may be only one available
    # window and its end need not align with the absolute hop grid (e.g. C).
    if not emitted and len(samples) >= size and (last_end is None or sample_index-last_end >= hop):
        yield sample_index, samples[-size:]


@dataclass(frozen=True)
class BriefDetection:
    detected: bool = False
    sample_index: int | None = None
    snr_db: float | None = None
    # Deliberately no speed/frequency output: a short event is not a sustained
    # velocity measurement, even when its short FFT contains a strong tone.


class BriefMotionDetector:
    """Independent, conservative short-event classifier, outside the USB lock.

    A 256 ms Hann spectrum must contain concentrated tone energy. Time-domain
    occupancy rejects 1-sample and 5..50 ms impulses even when overlapping FFTs
    would make them persist. Wider spur guards account for short-FFT resolution;
    the sustained detector's narrow exclusions and thresholds are unchanged.
    """

    def __init__(self):
        n = BRIEF_WINDOW_SAMPLES
        t = np.arange(n) / FS
        # Least-squares removal of DC/trend and the two known nuisance tones.
        # This is local to brief detection; raw FFT/spectrogram remain visible.
        columns = [np.ones(n), np.linspace(-1, 1, n)]
        for frequency in SPUR_FREQUENCIES_HZ:
            columns.extend((np.sin(2*np.pi*frequency*t), np.cos(2*np.pi*frequency*t)))
        self.nuisance = np.column_stack(columns)
        self.inverse = np.linalg.pinv(self.nuisance)
        self.window = np.hanning(n)
        f = np.fft.rfftfreq(n, 1/FS)
        self.band = (f >= ANALYSIS_BAND_HZ[0]) & (f <= ANALYSIS_BAND_HZ[1])
        self.frequencies = f[self.band]
        self.allowed = np.ones(len(self.frequencies), dtype=bool)
        for spur in SPUR_FREQUENCIES_HZ:
            self.allowed &= abs(self.frequencies - spur) > SPUR_EXCLUSION_HALF_WIDTH_HZ + 2*FS/n
        self.baseline = np.full(len(self.frequencies), NOISE_FLOOR_COUNTS)
        self.noise_rms = 1.0
        self.calibration_spectra, self.calibration_noise = [], []
        self.discontinuity = None
        self.last_end = None
        self.valid_from = 0
        self.reset_event()

    def reset_event(self):
        self.track_hz = None
        self.frames = 0
        self.result = BriefDetection()

    def set_interference(self, bands):
        # Rebuild the nuisance projection before deriving the short-window noise
        # reference. Otherwise a strong learned line would inflate that reference
        # and hide genuine brief hand movements elsewhere in the band.
        t = np.arange(BRIEF_WINDOW_SAMPLES)/FS
        columns = [np.ones(len(t)), np.linspace(-1, 1, len(t))]
        for hz in (*SPUR_FREQUENCIES_HZ, *(band.frequency_hz for band in bands)):
            columns.extend((np.sin(2*np.pi*hz*t), np.cos(2*np.pi*hz*t)))
        self.nuisance = np.column_stack(columns)
        self.inverse = np.linalg.pinv(self.nuisance)
        self.allowed[:] = True
        for hz, width in ([(hz, SPUR_EXCLUSION_HALF_WIDTH_HZ) for hz in SPUR_FREQUENCIES_HZ]
                          + [(band.frequency_hz, band.half_width_hz) for band in bands]):
            self.allowed &= abs(self.frequencies-hz) > width+2*FS/BRIEF_WINDOW_SAMPLES

    def spectrum(self, values):
        centered = values - np.mean(values)
        residual = centered - self.nuisance @ (self.inverse @ centered)
        amplitude = (2*np.abs(np.fft.rfft(residual*self.window))/self.window.sum())[self.band]
        return residual, amplitude

    def observe_calibration(self, values):
        residual, amplitude = self.spectrum(values[-BRIEF_WINDOW_SAMPLES:])
        self.calibration_spectra.append(amplitude)
        self.calibration_noise.append(float(1.4826*np.median(abs(residual - np.median(residual)))))

    def finish_calibration(self, sample_index, discontinuity):
        if self.calibration_spectra:
            self.baseline = np.maximum(np.median(self.calibration_spectra, axis=0), NOISE_FLOOR_COUNTS)
            self.noise_rms = max(float(np.median(self.calibration_noise)), 0.25)
        self.calibration_spectra.clear()
        self.calibration_noise.clear()
        self.last_end = sample_index // BRIEF_HOP_SAMPLES * BRIEF_HOP_SAMPLES
        self.valid_from = sample_index
        self.discontinuity = discontinuity
        self.reset_event()

    def classify_window(self, values):
        if not np.any(self.allowed):
            return None
        residual, raw = self.spectrum(values)
        i = int(np.argmax(np.where(self.allowed, raw, -np.inf)))
        floor = max(float(np.median(raw[self.allowed])), NOISE_FLOOR_COUNTS)
        peak = float(raw[i])
        snr = 20*math.log10(max(peak, NOISE_FLOOR_COUNTS)/floor)
        distance = abs(self.frequencies - self.frequencies[i])
        neighbors = self.allowed & (distance >= 2*FS/BRIEF_WINDOW_SAMPLES) & (distance <= 30)
        background = max(float(np.median(raw[neighbors])), NOISE_FLOOR_COUNTS)
        prominence = 20*math.log10(max(peak, NOISE_FLOOR_COUNTS)/background)
        baseline_db = 20*math.log10(max(peak, NOISE_FLOOR_COUNTS)/self.baseline[i])
        # Do not turn shoulders of excluded peaks into new local maxima.
        local_maximum = ((i == 0 or raw[i] > raw[i-1]) and (i+1 == len(raw) or raw[i] >= raw[i+1]))
        energy = residual**2
        total = float(energy.sum())
        if total <= 0:
            return None
        effective_seconds = total**2 / max(float(np.sum(energy**2)), 1e-12) / FS
        cumulative = np.concatenate(([0.0], np.cumsum(energy)))
        width = round(0.050*FS)
        concentrated = float(np.max(cumulative[width:] - cumulative[:-width])) / total
        tone_fraction = float(np.sum(raw[distance <= FS/BRIEF_WINDOW_SAMPLES]**2)) / max(float(np.sum(raw**2)), 1e-12)
        # Require several appreciable sign excursions; step-like pulses fail.
        threshold = max(3*self.noise_rms, 0.18*float(np.max(abs(residual))))
        signs = np.sign(residual[abs(residual) >= threshold])
        crossings = int(np.count_nonzero(np.diff(signs)))
        if (local_maximum and peak >= BRIEF_MIN_AMPLITUDE
                and snr >= BRIEF_MIN_SNR_DB and prominence >= BRIEF_MIN_PROMINENCE_DB
                and baseline_db >= BRIEF_MIN_BASELINE_DB
                and math.sqrt(total / len(values)) >= 3*self.noise_rms
                and effective_seconds >= BRIEF_MIN_EFFECTIVE_SECONDS
                and concentrated <= BRIEF_MAX_50MS_ENERGY_FRACTION
                and tone_fraction >= BRIEF_MIN_TONE_ENERGY_FRACTION and crossings >= 4):
            return float(self.frequencies[i]), snr
        return None

    def process(self, samples, sample_index, discontinuity=0, stale=False, sustained=False):
        if discontinuity != self.discontinuity or stale:
            self.reset_event()
            # Discard pre-gap history, even when the buffer includes older data.
            self.valid_from = sample_index if self.discontinuity is not None or stale else sample_index-len(samples)
            self.last_end = sample_index if self.discontinuity is not None else None
            self.discontinuity = discontinuity
        if stale:
            return self.result
        if sustained:
            self.reset_event()
            self.last_end = sample_index // BRIEF_HOP_SAMPLES * BRIEF_HOP_SAMPLES
            return self.result
        for end, values in new_windows(samples, sample_index, self.last_end,
                                       BRIEF_WINDOW_SAMPLES, BRIEF_HOP_SAMPLES):
            if self.last_end is not None and end - self.last_end > BRIEF_HOP_SAMPLES:
                self.reset_event()
            self.last_end = end
            if end - BRIEF_WINDOW_SAMPLES < self.valid_from or not np.all(np.isfinite(values)):
                self.reset_event()
                continue
            peak = self.classify_window(values)
            if peak is None:
                self.track_hz, self.frames = None, 0
            else:
                hz, snr = peak
                self.frames = self.frames + 1 if self.track_hz is not None and abs(hz-self.track_hz) <= FS/BRIEF_WINDOW_SAMPLES else 1
                self.track_hz = hz
                if self.frames >= BRIEF_REQUIRED_FRAMES:
                    self.result = BriefDetection(True, end, snr)
        if self.result.sample_index is not None and sample_index - self.result.sample_index > BRIEF_DISPLAY_SECONDS*FS:
            self.result = BriefDetection()
        return self.result


class StartupCalibration:
    """Seven seconds of contiguous received data, not seven GUI/wall seconds.

    Seed the original slow adaptive baseline with median no-motion spectra.
    Acquisition/recording continue. A gap/stall restarts unfinished calibration.
    No sustained decisions run until ready; the detector algorithm stays intact.
    """

    def __init__(self, seconds=STARTUP_CALIBRATION_SECONDS, minimum_index=None):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError("Calibration duration must be positive")
        self.seconds = seconds
        self.minimum_index = minimum_index
        self.done = False
        self.start_index = self.last_end = self.last_seen = None
        self.discontinuity = None
        self.elapsed = 0.0
        self.frames = []
        self.time_windows = []
        self.spectrum = None

    @property
    def remaining(self):
        return max(0.0, self.seconds - self.elapsed)

    def process(self, samples, sample_index, detector, brief, discontinuity=0, stale=False):
        if self.done:
            return True
        missed = self.last_seen is not None and sample_index - self.last_seen > len(samples)
        if discontinuity != self.discontinuity or stale or missed:
            self.start_index = sample_index if self.discontinuity is not None or stale else self.minimum_index
            self.last_end = None
            self.frames.clear()
            self.time_windows.clear()
            brief.calibration_spectra.clear()
            brief.calibration_noise.clear()
            self.elapsed = 0.0
            self.spectrum = None
        self.discontinuity, self.last_seen = discontinuity, sample_index
        if stale or not len(samples):
            return False
        invalid = np.flatnonzero(~np.isfinite(samples))
        usable = samples[invalid[-1]+1:] if len(invalid) else samples
        if self.start_index is None:
            self.start_index = sample_index - len(usable)
        usable = usable[max(0, self.start_index - (sample_index-len(usable))):]
        self.elapsed = (sample_index - self.start_index) / FS
        for end, values in new_windows(usable, sample_index, self.last_end, detector.fft_size, FFT_HOP_SAMPLES):
            self.last_end = end
            spectrum = analyze(values, detector.fft_size)
            if spectrum is not None:
                self.spectrum = spectrum
                self.frames.append(spectrum.amplitudes)
                self.time_windows.append(values[-RECENT_WINDOW_SAMPLES:].copy())
        if self.elapsed >= self.seconds and self.frames:
            detector.baseline = np.maximum(np.median(self.frames, axis=0), NOISE_FLOOR_COUNTS)
            detector.spectrum = self.spectrum
            detector.reset_track()
            detector.last_frame_index = None
            detector.discontinuity = discontinuity
            detector.interference.learn(self.spectrum.frequencies, self.frames, self.time_windows)
            brief.set_interference(detector.interference.bands)
            brief.calibration_spectra.clear()
            brief.calibration_noise.clear()
            for values in self.time_windows:
                brief.observe_calibration(values)
            brief.finish_calibration(sample_index, discontinuity)
            self.frames.clear()
            self.time_windows.clear()
            self.done = True
        return self.done


class SpectrogramBuffer:
    """Incremental DC-removed/Hann STFT; NaN columns preserve missing time.

    Raw spectra include fixed spurs for engineering inspection. Fixed dB scale
    is relative to 1 ADC count of sinusoidal peak amplitude (not RF power).
    """

    def __init__(self, seconds=HISTORY_SECONDS):
        self.seconds = seconds
        self.size, self.hop = SPECTROGRAM_WINDOW_SAMPLES, SPECTROGRAM_HOP_SAMPLES
        self.window = np.hanning(self.size)
        f = np.fft.rfftfreq(self.size, 1/FS)
        self.band = (f >= ANALYSIS_BAND_HZ[0]) & (f <= ANALYSIS_BAND_HZ[1])
        self.frequencies = f[self.band]
        self.columns = deque(maxlen=math.ceil(seconds*FS/self.hop)+2)
        self.last_end = self.discontinuity = None
        self.valid_from = 0

    def update(self, samples, sample_index, discontinuity=0, stale=False):
        if discontinuity != self.discontinuity:
            self.columns.clear()
            self.valid_from = sample_index if self.discontinuity is not None else sample_index-len(samples)
            self.last_end = sample_index if self.discontinuity is not None else None
            self.discontinuity = discontinuity
        if stale:
            self.valid_from = sample_index
            self.last_end = sample_index
            self.columns.clear()
            return
        for end, values in new_windows(samples, sample_index, self.last_end, self.size, self.hop):
            self.last_end = end
            if end - self.size >= self.valid_from and np.all(np.isfinite(values)):
                values = values - np.mean(values)
                amplitudes = (2*np.abs(np.fft.rfft(values*self.window))/self.window.sum())[self.band]
                db = 20*np.log10(np.maximum(amplitudes, 0.01))
            else:
                db = np.full(len(self.frequencies), np.nan)
            self.columns.append((end, db))
        while self.columns and self.columns[0][0] < sample_index - self.seconds*FS:
            self.columns.popleft()

    def image(self, sample_index, age_seconds=0.0):
        width = math.ceil(self.seconds*FS/self.hop)
        data = np.full((len(self.frequencies), width), np.nan)
        for end, db in self.columns:
            age = (sample_index-end)/FS + max(0.0, age_seconds)
            column = width - 1 - int(math.floor(age*FS/self.hop + 1e-9))
            if 0 <= column < width:
                data[:, column] = db
        return np.ma.masked_invalid(data)


@dataclass(frozen=True)
class DisplayReading:
    state: str
    frequency_hz: float | None = None
    raw_frequency_hz: float | None = None
    quality_db: float | None = None
    age_seconds: float = 0.0

    @property
    def speed_mps(self):
        return None if self.frequency_hz is None else self.frequency_hz * SPEED_PER_HZ

    @property
    def is_held(self):
        return self.state == "hold"


@dataclass(frozen=True)
class HistoryPoint:
    timestamp: float
    speed_mps: float | None
    kind: str  # "validated", "held", or "gap"; never fabricate no-motion speeds.


class DisplayState:
    """Read-only presentation of detector output. Never publish this into CSV.

    Fresh FFT frame IDs, not GUI repaints, feed the median and refresh the hold
    timer. Monotonic wall time makes the hold/debounce independent of frame rate.
    """

    def __init__(self, hold_seconds=DISPLAY_MOTION_HOLD_SECONDS,
                 clear_seconds=DISPLAY_CLEAR_DELAY_SECONDS,
                 smoothing_window=DISPLAY_SMOOTHING_WINDOW, history_seconds=HISTORY_SECONDS):
        self.hold_seconds, self.clear_seconds = hold_seconds, clear_seconds
        self.history_seconds = history_seconds
        self.frequencies = deque(maxlen=smoothing_window)
        self.qualities = deque(maxlen=smoothing_window)
        self.history = deque()
        self.discontinuity = None
        self.last_update = None
        self.reading = DisplayReading("warming_up")
        self._reset_values()

    def _reset_values(self):
        self.frequencies.clear()
        self.qualities.clear()
        self.last_frame_id = None
        self.last_valid_time = None
        self.last_raw_hz = None
        self.shown_hz = self.shown_quality = None
        self.quiet_since = self.last_candidate_time = None

    def _gap(self, now):
        if self.history and self.history[-1].kind != "gap":
            self.history.append(HistoryPoint(now, None, "gap"))

    def _trim_history(self, now):
        while self.history and self.history[0].timestamp < now - self.history_seconds:
            self.history.popleft()

    def update(self, detection, frame_id, now=None, discontinuity=0, quality_db=None):
        now = time.monotonic() if now is None else now
        # Gaps, resets and stale data override the visual hold immediately.
        if (discontinuity != self.discontinuity
                or (self.last_update is not None
                    and now - self.last_update > max(STALE_SECONDS, self.hold_seconds))):
            self._gap(now)
            self._reset_values()
        self.discontinuity = discontinuity
        self.last_update = now
        self._trim_history(now)
        if detection.state in ("stale", "data_gap", "warming_up", "baseline_calibration", *INTERFERENCE_STATES):
            self._reset_values()
            self._gap(now)
            state = "warming_up" if detection.state == "data_gap" else detection.state
            self.reading = DisplayReading(state)
            return self.reading

        new_frame = frame_id is not None and frame_id != self.last_frame_id
        if new_frame:
            self.last_frame_id = frame_id
            if detection.state == "validated_motion" and detection.validated_hz is not None:
                raw_hz = detection.validated_hz
                new_event = (self.last_valid_time is None
                             or now - self.last_valid_time > self.hold_seconds
                             or not MotionDetector.close_frequency(raw_hz, self.last_raw_hz))
                if new_event:
                    self.frequencies.clear()
                    self.qualities.clear()
                    self._gap(now)  # Do not draw a ramp between unrelated events.
                self.frequencies.append(raw_hz)
                if quality_db is not None and math.isfinite(quality_db):
                    self.qualities.append(quality_db)
                self.shown_hz = float(np.median(self.frequencies))
                self.shown_quality = float(np.median(self.qualities)) if self.qualities else None
                self.last_raw_hz, self.last_valid_time = raw_hz, now
                self.quiet_since = self.last_candidate_time = None
            elif detection.state == "candidate_motion":
                self.last_candidate_time, self.quiet_since = now, None

        if detection.state == "no_motion" and self.quiet_since is None:
            self.quiet_since = now
        age = math.inf if self.last_valid_time is None else now - self.last_valid_time
        if age <= self.hold_seconds:
            live = detection.state == "validated_motion" and age <= STALE_SECONDS
            self.reading = DisplayReading("validated_motion" if live else "hold",
                                          self.shown_hz, self.last_raw_hz, self.shown_quality, age)
        else:
            # Numeric values expire on the hold deadline even if status debounce
            # has not finished. A later event starts a new smoothing window.
            self.frequencies.clear()
            self.qualities.clear()
            quiet_ready = self.quiet_since is not None and now - self.quiet_since >= self.clear_seconds
            if (detection.state == "candidate_motion"
                    or (self.last_candidate_time is not None and not quiet_ready)):
                state = "candidate_motion"
            elif detection.state == "validated_motion":
                state = "stale"  # A repeated old frame is not a new measurement.
            elif self.last_valid_time is not None and not quiet_ready:
                state = "hold"  # Status settling only; expired numbers stay blank.
            else:
                state = "no_motion"
            self.reading = DisplayReading(state)

        if self.reading.state == "validated_motion" and new_frame:
            self.history.append(HistoryPoint(now, detection.speed_mps, "validated"))
        elif self.reading.is_held and self.reading.speed_mps is not None:
            self.history.append(HistoryPoint(now, self.reading.speed_mps, "held"))
        elif self.reading.state != "validated_motion":
            self._gap(now)
        return self.reading

    def history_arrays(self, now):
        self._trim_history(now)
        times = np.array([p.timestamp - now for p in self.history])
        valid = np.array([p.speed_mps if p.kind == "validated" else np.nan for p in self.history])
        held = np.array([p.speed_mps if p.kind == "held" else np.nan for p in self.history])
        return times, valid, held


def instrument_reading(reading, brief, calibrating=False, ready=False, error=False):
    """Choose card status without changing detections, history, or CSV output."""
    if error:
        return DisplayReading("data_error")
    if reading.state == "stale":
        return reading
    if calibrating:
        return DisplayReading("baseline_calibration")
    if reading.state in INTERFERENCE_STATES:
        # Even a previous HOLD must not leak a speed into a safety warning.
        return DisplayReading(reading.state)
    if reading.state == "validated_motion":
        return reading
    # A brief-event notification must not blank a still-valid LAST VALIDATED
    # card. The brief result remains visible in diagnostics. Expired holds with
    # no numeric value do not suppress a new brief-event notification.
    if reading.is_held and reading.frequency_hz is not None:
        return reading
    if brief.detected:
        return DisplayReading("brief_motion")
    # Retain the existing rule: candidates never replace a held validation.
    if ready and reading.state == "no_motion":
        return DisplayReading("radar_ready")
    return reading


def validated_peak_snr(detector, detection):
    """Read-only copy of the existing Peak.snr_db calculation for the selected
    VALIDATED frequency. Do not call peaks(), adapt the baseline, or retune it.
    """
    spectrum = detector.spectrum
    if spectrum is None or detection.validated_hz is None:
        return None
    index = int(np.argmin(np.abs(spectrum.frequencies - detection.validated_hz)))
    allowed = detector.allowed_bins(spectrum.frequencies)
    floor = max(float(np.median(spectrum.amplitudes[allowed])), NOISE_FLOOR_COUNTS)
    return 20.0 * math.log10(max(spectrum.amplitudes[index], NOISE_FLOOR_COUNTS) / floor)


def available_ports():
    from serial.tools import list_ports
    return sorted(list_ports.comports(), key=lambda p: p.device)


def choose_port(requested=None):
    if requested:
        return requested
    ports = available_ports()
    if not ports:
        raise RuntimeError("No COM ports found. Connect the Blue Pill or pass --port COM5.")
    # Standard STM32 virtual COM VID/PID. Still allow an explicit port override.
    stm = [p for p in ports if p.vid == 0x0483 and p.pid == 0x5740]
    if len(stm) == 1:
        return stm[0].device
    if len(ports) == 1:
        return ports[0].device
    for i, port in enumerate(ports, 1):
        print(f"{i}: {port.device} - {port.description}")
    if not sys.stdin.isatty():
        raise RuntimeError("Multiple COM ports found; pass --port COMx.")
    answer = input("Select a port number or enter COMx: ").strip()
    if answer.isdecimal() and 1 <= int(answer) <= len(ports):
        return ports[int(answer) - 1].device
    if answer.upper().startswith("COM") and answer[3:].isdecimal():
        return answer.upper()
    raise RuntimeError("Invalid COM port selection")


def serial_worker(connection, state, stop):
    parser = PacketParser()
    try:
        while not stop.is_set():
            data = connection.read(min(max(connection.in_waiting, 1), 65536))
            if data:
                state.feed(parser, data)
    except Exception as exc:
        if not stop.is_set():
            with state.lock:
                state.error = f"Serial error: {exc}. Reconnect and restart the receiver."


def demo_worker(state, stop):
    """Clear-scene calibration, then walking and brief-burst synthetic examples."""
    parser = PacketParser()
    sample_number = 0
    sequence = 0
    deadline = time.monotonic()
    rng = np.random.default_rng(2026)
    while not stop.is_set():
        t = (sample_number + np.arange(PACKET_SAMPLES)) / FS
        phase = np.mod(np.maximum(0, t - STARTUP_CALIBRATION_SECONDS - 1), 12.0)
        enabled = t >= STARTUP_CALIBRATION_SECONDS + 1
        walk = enabled & (phase < 5)
        hand = enabled & (phase >= 8) & (phase < 8.24)
        values = (2048 + 100*np.sin(2*np.pi*82.5*t) + 60*np.sin(2*np.pi*165*t)
                  + walk*35*np.sin(2*np.pi*10*t) + hand*80*np.sin(2*np.pi*31.25*t)
                  + rng.normal(0, 1, PACKET_SAMPLES))
        x = np.clip(np.rint(values), 0, 4095).astype(int)
        packet = HEADER.pack(SYNC, sequence, PACKET_SAMPLES) + SAMPLES.pack(*x)
        state.feed(parser, packet[:17])
        state.feed(parser, packet[17:])
        sample_number += PACKET_SAMPLES
        sequence = (sequence + 1) & 0xFFFF
        deadline += PACKET_SAMPLES / FS
        stop.wait(max(0.0, deadline - time.monotonic()))


class RadarPlot:
    """Matplotlib instrument dashboard; display state never feeds the detector."""

    def __init__(self, state, args, label):
        import matplotlib.pyplot as plt
        from matplotlib.ticker import MaxNLocator

        self.state, self.args, self.connection_label = state, args, label
        self.detector = MotionDetector(args.fft_size, args.min_peak, args.min_snr_db)
        self.brief_detector = BriefMotionDetector()
        self.calibration = StartupCalibration()
        self.spectrogram = SpectrogramBuffer()
        self.brief_result = BriefDetection()
        self._ready_at = None
        self._learned_artists = []
        self.display = DisplayState()
        self.colors = DASHBOARD_COLORS
        self._texts = []
        self._wave_bounds = None
        self._fft_ceiling = 8.0
        self._history_ceiling = 1.0
        self._record_started = None
        self._record_duration = 0.0
        self._was_recording = False
        self.test_label = " ".join(args.label.replace("_", " ").split()).upper() or "UNLABELED TEST"

        with plt.rc_context({"font.family": "DejaVu Sans", "font.size": 10,
                             "axes.spines.top": False, "axes.spines.right": False,
                             "figure.facecolor": self.colors["background"],
                             "axes.facecolor": self.colors["surface"],
                             "text.color": self.colors["ink"],
                             "axes.labelcolor": self.colors["muted"],
                             "xtick.color": self.colors["muted"],
                             "ytick.color": self.colors["muted"]}):
            self.figure = plt.figure(figsize=(15, 10.5), dpi=100)
            layout = self.figure.add_gridspec(
                6, 12, height_ratios=[0.65, 1.7, 2.8, 2.4, 0.65, 0.9],
                left=0.065, right=0.975, top=0.97, bottom=0.035, hspace=0.82, wspace=0.95)
            header = self.figure.add_subplot(layout[0, :])
            header.set_axis_off()
            self._text(header, 0, 0.74, "2.45 GHz CW Doppler Radar", 23, weight="bold")
            self.header_line = self._text(header, 0, 0.09, "Connecting...", 9.8, color=self.colors["muted"])
            self.test_tag = self._text(header, 1, 0.8, self.test_label, 10.5, ha="right", weight="bold")
            self.health_tag = self._text(header, 1, 0.10, "WAITING FOR DATA", 9.5, ha="right", weight="bold")

            self.cards = {}
            for i, (key, title) in enumerate((
                    ("motion", "MOTION STATUS"), ("frequency", "VALIDATED DOPPLER"),
                    ("speed", "RADIAL SPEED MAGNITUDE"), ("quality", "SIGNAL QUALITY"))):
                card = self.figure.add_subplot(layout[1, 3*i:3*i+3])
                card.set(xlim=(0, 1), ylim=(0, 1), xticks=[], yticks=[])
                for spine in card.spines.values():
                    spine.set_visible(True)
                    spine.set_color(self.colors["border"])
                accent = card.axhline(1, color=self.colors["border"], lw=3, clip_on=False)
                self._text(card, 0.07, 0.82, title, 9, color=self.colors["muted"], weight="bold")
                value = self._text(card, 0.07, 0.48 if key == "motion" else 0.45, "—", 16 if key == "motion" else 25,
                                   weight="bold", va="center", linespacing=1.03)
                note = self._text(card, 0.07, 0.10, "Awaiting data", 8.2, color=self.colors["muted"])
                self.cards[key] = {"axis": card, "value": value, "note": note, "accent": accent}

            self.wave_ax = self.figure.add_subplot(layout[2, :5])
            self.fft_ax = self.figure.add_subplot(layout[2, 5:])
            self.spectrogram_ax = self.figure.add_subplot(layout[3, :7])
            self.history_ax = self.figure.add_subplot(layout[3, 7:])
            for axis in (self.wave_ax, self.fft_ax, self.spectrogram_ax, self.history_ax):
                axis.set_axisbelow(True)
                axis.grid(True, color=self.colors["grid"], lw=0.7)
                for spine in axis.spines.values():
                    spine.set_color(self.colors["border"])
                axis.yaxis.set_major_locator(MaxNLocator(nbins=4))

            self.wave_ax.set_title("Live Baseband Signal", loc="left", fontsize=12, fontweight="bold", pad=20)
            self.wave_ax.set(xlabel="Recent time (s)", ylabel="ADC counts",
                             xlim=(-args.wave_seconds, 0), ylim=(0, 4095))
            self._text(self.wave_ax, 1, 1.035, "Raw signal · DC bias retained", 8.2,
                       color=self.colors["muted"], ha="right")
            (self.wave_line,) = self.wave_ax.plot([], [], color=self.colors["signal"], lw=1.0)

            self.fft_ax.set_title("Doppler Spectrum", loc="left", fontsize=12, fontweight="bold", pad=20)
            self.fft_ax.set(xlabel="Doppler frequency (Hz)", ylabel="Amplitude (ADC counts)",
                            xlim=ANALYSIS_BAND_HZ, ylim=(0, 8))
            self.fft_ax.set_xticks([5, 50, 100, 150, 200, 250, 300])
            self.raw_readout = self._text(self.fft_ax, 1, 1.035, "", 8.2,
                                          color=self.colors["muted"], ha="right")
            (self.fft_line,) = self.fft_ax.plot([], [], color=self.colors["signal"], lw=1.15, label="Raw FFT")
            (self.baseline_line,) = self.fft_ax.plot([], [], color=self.colors["baseline"],
                                                   lw=1.0, label="No-motion baseline")
            self.peak_line = self.fft_ax.axvline(5, color=self.colors["validated"], lw=1.5, label="Validated peak")
            self.held_peak_line = self.fft_ax.axvline(5, color=self.colors["hold"], lw=1.3,
                                                     ls="--", label="Last validated (hold)")
            for i, spur in enumerate(SPUR_FREQUENCIES_HZ):
                self.fft_ax.axvspan(spur - SPUR_EXCLUSION_HALF_WIDTH_HZ,
                                   spur + SPUR_EXCLUSION_HALF_WIDTH_HZ,
                                   color=self.colors["hold"], alpha=0.10,
                                   label="Excluded spurs" if i == 0 else None)
            self.fft_legend = self.fft_ax.legend(loc="upper right", fontsize=7.7, frameon=False)
            self.peak_line.set_visible(False)
            self.held_peak_line.set_visible(False)

            self.spectrogram_ax.set_title("Doppler Spectrogram", loc="left", fontsize=12, fontweight="bold", pad=8)
            self.spectrogram_ax.set(xlabel="Recent time (s) · window end", ylabel="Doppler frequency (Hz)",
                                    xlim=(-HISTORY_SECONDS, 0), ylim=ANALYSIS_BAND_HZ)
            self.spectrogram_ax.grid(False)
            cmap = plt.get_cmap("cividis").with_extremes(bad=self.colors["grid"])
            frequencies = self.spectrogram.frequencies
            df = FS / SPECTROGRAM_WINDOW_SAMPLES
            self.spectrogram_image = self.spectrogram_ax.imshow(
                self.spectrogram.image(0), origin="lower", aspect="auto", interpolation="nearest",
                extent=(-HISTORY_SECONDS, 0, frequencies[0]-df/2, frequencies[-1]+df/2),
                cmap=cmap, vmin=-20, vmax=50)
            self.spectrogram_ax.set_ylim(*ANALYSIS_BAND_HZ)
            for spur in SPUR_FREQUENCIES_HZ:
                self.spectrogram_ax.axhline(spur, color="white", ls=":", lw=0.65, alpha=0.5)
            self._text(self.spectrogram_ax, 1, 1.035, "Raw STFT · −20 to 50 dB re 1 ADC count", 8,
                       color=self.colors["muted"], ha="right")
            self.history_ax.set_title("Radial Speed History", loc="left", fontsize=11, fontweight="bold", pad=8)
            self.history_ax.set(xlabel="Time relative to now (s)", ylabel="Speed (m/s)",
                                xlim=(-HISTORY_SECONDS, 0), ylim=(0, 1))
            (self.history_line,) = self.history_ax.plot([], [], color=self.colors["validated"],
                                                       lw=1.4, marker="o", ms=2.5, label="Validated sustained measurements")
            (self.history_hold_line,) = self.history_ax.plot([], [], color=self.colors["hold"], lw=1.2,
                                                            ls="--", drawstyle="steps-post", label="Hold · last validated")
            self.history_hold_line.set_visible(False)  # Held cards are never new history measurements.
            self.history_legend = self.history_ax.legend(handles=[self.history_line], loc="upper right", fontsize=7.7, frameon=False)

            diagnostics = self.figure.add_subplot(layout[4, :])
            diagnostics.set_axis_off()
            self.status = self._text(diagnostics, 0, 1.0, "", 8.1,
                                     color=self.colors["muted"], va="top", linespacing=1.5)
            self.interference_note = self._text(diagnostics, 0, -0.30,
                       "Learned interference: awaiting calibration | C: recalibrate (10 s)",
                       8.1, color=self.colors["muted"])
            self.velocity_note = self._text(diagnostics, 0, -0.90,
                       "Radial speed only | Sideways motion may read near zero | Direction / range unavailable",
                       8.4, color=self.colors["ink"])

            recording = self.figure.add_subplot(layout[5, :])
            recording.set(xlim=(0, 1), ylim=(0, 1), xticks=[], yticks=[])
            for spine in recording.spines.values():
                spine.set_visible(True)
                spine.set_color(self.colors["border"])
            self.record_badge = self._text(recording, 0.018, 0.72, "RECORDING OFF", 11, weight="bold")
            self.record_test = self._text(recording, 0.24, 0.72, "TEST · " + self.test_label, 9.8, weight="bold")
            self.record_time = self._text(recording, 0.98, 0.72, "00:00  |  R: START / STOP", 9.2, ha="right")
            self.record_file = self._text(recording, 0.018, 0.37, "No CSV file selected", 9, color=self.colors["muted"])
            self.record_status = self._text(recording, 0.018, 0.10, "Focus this window and press R to record.",
                                            8, color=self.colors["muted"])

        # Preserve useful old attribute names for callers inspecting the plot.
        self.readout = self.cards["motion"]["value"]
        self.figure.canvas.manager.set_window_title("2.45 GHz CW Doppler Radar")
        self.figure.canvas.mpl_connect("key_press_event", self.on_key)
        self.figure.canvas.mpl_connect("resize_event", self._resize)
        self._resize(None)
        self.timer = self.figure.canvas.new_timer(interval=50)
        self.timer.add_callback(self.update)
        self.timer.start()

    def _text(self, axis, x, y, text, size, **kwargs):
        artist = axis.text(x, y, text, transform=axis.transAxes, fontsize=size,
                           color=kwargs.pop("color", self.colors["ink"]), **kwargs)
        self._texts.append((artist, size))
        return artist

    @staticmethod
    def _shorten(text, limit):
        return text if len(text) <= limit else text[:limit-1] + "…"

    def _resize(self, event):
        width, height = self.figure.get_size_inches()
        self.font_scale = max(0.68, min(1.15, width / 15.0, height / 9.5))
        for artist, size in self._texts:
            artist.set_fontsize(size * self.font_scale)
        for axis in (self.wave_ax, self.fft_ax, self.spectrogram_ax, self.history_ax):
            axis.set_title(axis.get_title(loc="left"), loc="left",
                           fontsize=(11 if axis == self.history_ax else 12) * self.font_scale,
                           fontweight="bold",
                           pad=(8 if axis in (self.spectrogram_ax, self.history_ax) else 20) * self.font_scale)
            axis.xaxis.label.set_size(9 * self.font_scale)
            axis.yaxis.label.set_size(9 * self.font_scale)
            axis.tick_params(labelsize=8.5 * self.font_scale)
        for legend in (self.fft_legend, self.history_legend):
            for text in legend.get_texts():
                text.set_fontsize(7.7 * self.font_scale)
        self.test_tag.set_text(self._shorten(self.test_label, 36))
        self.record_test.set_text("TEST · " + self._shorten(self.test_label, 42))

    def on_key(self, event):
        if event.key is not None and event.key.lower() == "c":
            self.recalibrate()
            self.update()
            return
        if event.key is None or event.key.lower() != "r" or self.state.recorder is None:
            return
        recorder = self.state.recorder
        try:
            if recorder.active:
                recorder.stop()
            else:
                _, stats = self.state.snapshot()
                path = recorder.start(self.args.record_dir, self.args.label, stats["sample_index"])
                self._record_started = time.monotonic()
                self._record_duration = 0.0
                print(f"Recording CSV: {path}")
        except (OSError, RuntimeError) as exc:
            recorder.message = f"Cannot start CSV recording: {exc}"
            print(recorder.message)
        self._recording(time.monotonic())
        self.figure.canvas.draw_idle()

    def recalibrate(self):
        _, stats = self.state.snapshot()
        self.detector = MotionDetector(self.args.fft_size, self.args.min_peak, self.args.min_snr_db)
        self.brief_detector = BriefMotionDetector()
        self.brief_result = BriefDetection()
        # Exclude every pre-key sample from the new ten-second calibration.
        self.calibration = StartupCalibration(RECALIBRATION_SECONDS, minimum_index=stats["sample_index"])
        self.display = DisplayState()
        self._ready_at = None
        self.state.publish_analysis(Detection("baseline_calibration"), stats["sample_index"], stats["discontinuity"])
        self.peak_line.set_visible(False)
        self.held_peak_line.set_visible(False)
        self.history_line.set_data([], [])
        self.history_hold_line.set_data([], [])
        self._refresh_interference_bands()
        # Do not stop the serial reader, the CSV writer, or the plot timer.

    def _refresh_interference_bands(self):
        for artist in self._learned_artists:
            artist.remove()
        self._learned_artists.clear()
        for i, band in enumerate(self.detector.interference.bands):
            self._learned_artists.append(self.fft_ax.axvspan(
                band.frequency_hz-band.half_width_hz, band.frequency_hz+band.half_width_hz,
                color=self.colors["warning"], alpha=0.13,
                label="Learned interference" if i == 0 else None))
        self.fft_legend = self.fft_ax.legend(loc="upper right", fontsize=7.7*self.font_scale, frameon=False)
        for line in self.fft_legend.get_lines():
            line.set_visible(True)

    def _recording(self, now):
        recorder = self.state.recorder
        if recorder is None:
            return
        if recorder.active:
            if self._record_started is None:
                self._record_started = now
            self._record_duration = now - self._record_started
        elif self._was_recording and self._record_started is not None:
            self._record_duration = now - self._record_started
            self._record_started = None
        self._was_recording = recorder.active
        failure = any(word in recorder.message.lower() for word in ("error", "queue full", "cannot"))
        badge = "REC · RECORDING" if recorder.active else ("RECORDING ERROR" if failure else "RECORDING OFF")
        self.record_badge.set_text(badge)
        self.record_badge.set_color(self.colors["recording"] if recorder.active or failure else self.colors["muted"])
        seconds = int(self._record_duration)
        self.record_time.set_text(f"{seconds // 60:02d}:{seconds % 60:02d}  |  R: START / STOP")
        self.record_file.set_text("CSV · " + (recorder.path.name if recorder.path is not None else "No file yet"))
        note = recorder.message if failure or (not recorder.active and recorder.path is not None) else (
            "Raw samples and detector decisions are recorded; display holds/medians are never written to CSV.")
        self.record_status.set_text(self._shorten(note, 145))

    def _cards(self, reading):
        if reading.state in INTERFERENCE_STATES:
            reading = DisplayReading(reading.state)
        held = reading.is_held
        states = {
            "stationary_interference": ("INTERFERENCE\nDETECTED", "STATIONARY INTERFERENCE", "warning"),
            "interference_low_confidence": ("INTERFERENCE /\nLOW CONFIDENCE", "Evidence insufficient · speed hidden", "hold"),
            "recalibration_required": ("RECALIBRATION\nREQUIRED", "Press C · clear scene for 10 s", "warning"),
            "baseline_calibration": ("BASELINE\nCALIBRATION", f"Keep area clear · {math.ceil(self.calibration.remaining)} s", "hold"),
            "radar_ready": ("RADAR READY", "Baseline initialized · area clear", "validated"),
            "brief_motion": ("BRIEF MOTION", "DETECTED · no sustained speed", "hold"),
            "data_error": ("USB / DATA\nERROR", "Disconnected · restart receiver", "warning"),
            "no_motion": ("NO MOTION", "No significant validated motion", "muted"),
            "candidate_motion": ("CANDIDATE\nMOTION", "Awaiting detector confirmation", "hold"),
            "validated_motion": ("VALIDATED\nMOTION", "Current validated detector result", "validated"),
            "hold": ("VALIDATED\nMOTION", "HOLD / LAST VALIDATED", "hold"),
            "stale": ("NO RECENT\nDATA", "Measurements cleared", "warning"),
            "warming_up": ("WARMING UP", "Collecting a contiguous FFT window", "muted"),
        }
        title, note, color = states[reading.state]
        if held:
            note += f" · {reading.age_seconds:.1f} s" if reading.frequency_hz is not None else " · expired"
        self.cards["motion"]["value"].set_text(title)
        self.cards["motion"]["note"].set_text(note)
        self.cards["frequency"]["value"].set_text("—" if reading.frequency_hz is None else f"{reading.frequency_hz:.2f} Hz")
        self.cards["speed"]["value"].set_text("—" if reading.speed_mps is None else f"{reading.speed_mps:.3f} m/s")
        measurement_note = "LAST VALIDATED · display median" if held else f"Display median · {DISPLAY_SMOOTHING_WINDOW} FFT results"
        if reading.frequency_hz is None:
            measurement_note = "No active validated value"
        self.cards["frequency"]["note"].set_text(measurement_note)
        self.cards["speed"]["note"].set_text("Magnitude only · " + ("LAST VALIDATED" if held and reading.speed_mps is not None else "radial component"))
        quality = reading.quality_db
        self.cards["quality"]["value"].set_text("—" if quality is None else f"{quality:.1f} dB")
        strength = "Strong" if quality is not None and quality >= 30 else "Good" if quality is not None and quality >= 20 else "Fair" if quality is not None and quality >= 12 else "Poor"
        self.cards["quality"]["note"].set_text(
            "Awaiting validated peak" if quality is None else ("LAST VALIDATED · " if held else strength + " · ") + "peak SNR")
        if reading.state == "brief_motion":
            self.cards["frequency"]["note"].set_text("Brief event · Doppler not validated")
            self.cards["speed"]["note"].set_text("No sustained speed measurement")
        for key, card in self.cards.items():
            card["accent"].set_color(self.colors[color])
            card["value"].set_color(self.colors[color] if reading.frequency_hz is not None or key == "motion" else self.colors["ink"])

    def update(self):
        now = time.monotonic()
        samples, stats = self.state.snapshot()
        waveform = samples[-int(self.args.wave_seconds * FS):]
        if len(waveform):
            self.wave_line.set_data((np.arange(len(waveform)) - len(waveform) + 1) / FS, waveform)
            valid = waveform[np.isfinite(waveform)]
            if len(valid):
                low, high = float(np.min(valid)), float(np.max(valid))
                margin = max(50.0, 0.1 * (high - low))
                bounds = (max(0, math.floor((low - margin) / 50) * 50),
                          min(4095, math.ceil((high + margin) / 50) * 50))
                if self._wave_bounds is not None:
                    bounds = (min(bounds[0], self._wave_bounds[0]), max(bounds[1], self._wave_bounds[1]))
                self._wave_bounds = bounds
                self.wave_ax.set_ylim(*bounds)  # Expand as needed; no frame-by-frame scale flicker.

        age = now - stats["last_packet_time"] if stats["last_packet_time"] is not None else math.inf
        stale = age > STALE_SECONDS or bool(stats["error"])
        was_ready = self.calibration.done
        ready = self.calibration.process(samples, stats["sample_index"], self.detector,
                                         self.brief_detector, stats["discontinuity"], stale)
        if ready and not was_ready:
            self._ready_at = now
            self._refresh_interference_bands()
        if ready:
            detection = self.detector.process(samples, stats["sample_index"], stats["discontinuity"], stale)
            self.brief_result = self.brief_detector.process(
                samples, stats["sample_index"], stats["discontinuity"], stale,
                sustained=detection.state == "validated_motion")
        else:
            detection = Detection("stale" if stale and stats["last_packet_time"] is not None else "baseline_calibration")
            self.brief_result = BriefDetection()
        analysis_index = self.detector.last_frame_index
        # Interference safety is applied before publication. Suspicious speed
        # never enters CSV; brief notifications and UI holds remain separate.
        self.state.publish_analysis(detection, stats["sample_index"] if analysis_index is None else analysis_index,
                                    stats["discontinuity"])
        reading = self.display.update(detection, analysis_index, now, stats["discontinuity"],
                                      validated_peak_snr(self.detector, detection))
        reading = instrument_reading(reading, self.brief_result, not ready,
                                     ready=self._ready_at is not None and now-self._ready_at < 1.5,
                                     error=bool(stats["error"]))
        self._cards(reading)

        spectrum = self.detector.spectrum if ready else self.calibration.spectrum
        if spectrum is not None:
            self.fft_line.set_data(spectrum.frequencies, spectrum.amplitudes)
            if self.detector.baseline is not None:
                self.baseline_line.set_data(spectrum.frequencies, self.detector.baseline)
            else:
                self.baseline_line.set_data([], [])
            self._fft_ceiling = max(self._fft_ceiling, 2.0 ** math.ceil(math.log2(max(1.0, float(np.max(spectrum.amplitudes)) * 1.15))))
            self.fft_ax.set_ylim(0, self._fft_ceiling)
            excluded = not self.detector.allowed_bins(np.array([spectrum.peak_hz]))[0]
            explanation = ("learned stationary band" if self.detector.interference.contains(spectrum.peak_hz)
                           else "excluded fixed spur" if excluded else "not a speed measurement")
            self.raw_readout.set_text(f"Raw maximum: {spectrum.peak_hz:.2f} Hz · " +
                                      explanation)
        else:
            self.fft_line.set_data([], [])
            self.baseline_line.set_data([], [])
            self.raw_readout.set_text("Waiting for a complete FFT window")
        self.peak_line.set_visible(reading.state == "validated_motion")
        self.held_peak_line.set_visible(reading.is_held and reading.raw_frequency_hz is not None)
        if reading.raw_frequency_hz is not None:
            self.peak_line.set_xdata([reading.raw_frequency_hz] * 2)
            self.held_peak_line.set_xdata([reading.raw_frequency_hz] * 2)

        times, valid_speeds, held_speeds = self.display.history_arrays(now)
        self.history_line.set_data(times, valid_speeds)
        self.history_hold_line.set_data(times, held_speeds)
        visible_speeds = valid_speeds
        visible_speeds = visible_speeds[np.isfinite(visible_speeds)]
        if len(visible_speeds):
            self._history_ceiling = max(self._history_ceiling, math.ceil(float(np.max(visible_speeds)) * 1.2 * 2) / 2)
        self.history_ax.set_ylim(0, self._history_ceiling)

        self.spectrogram.update(samples, stats["sample_index"], stats["discontinuity"], stale)
        self.spectrogram_image.set_data(self.spectrogram.image(stats["sample_index"], age if math.isfinite(age) else 0))

        health = "USB DISCONNECTED / DATA ERROR" if stats["error"] else "WAITING FOR DATA" if stats["last_packet_time"] is None else "NO RECENT DATA" if stale else "RADAR READY · USB CONNECTED" if ready else "USB CONNECTED · CALIBRATING"
        if self.args.demo and not stale:
            health = "RADAR READY · DEMO STREAM" if ready else "DEMO STREAM · CALIBRATING"
        if ready and not stale and detection.state in INTERFERENCE_STATES:
            health = "RECALIBRATION REQUIRED" if detection.state == "recalibration_required" else "INTERFERENCE / LOW CONFIDENCE" if detection.state == "interference_low_confidence" else "INTERFERENCE DETECTED"
        port = "DEMO · SYNTHETIC DATA" if self.args.demo else self.connection_label
        self.header_line.set_text(f"{port}  |  Sampling: {FS:.0f} Hz  |  Carrier: {CARRIER_HZ / 1e9:.2f} GHz  |  Dropped: {stats['dropped']}")
        self.health_tag.set_text(health)
        self.health_tag.set_color(self.colors["warning"] if stale or detection.state in INTERFERENCE_STATES else self.colors["validated"])
        raw_validated = "—" if detection.validated_hz is None else f"{detection.validated_hz:.2f} Hz"
        self.status.set_text(
            f"DIAGNOSTICS   Packets {stats['packets']:,}   ·   Dropped {stats['dropped']}   ·   Duplicates {stats['duplicates']}   ·   Restarts {stats['restarts']}   ·   Invalid frames {stats['invalid_frames']}   ·   Resync bytes {stats['discarded_bytes']}\n"
            + (self._shorten(stats["error"], 150) if stats["error"] else
               f"ADC stream: {'STALE' if stale else 'OK'}   ·   FFT {self.args.fft_size} / {FS / self.args.fft_size:.3f} Hz   ·   Hann / DC removed   ·   Sustained: {detection.state}   ·   Raw validated: {raw_validated}   ·   Brief: {'DETECTED' if self.brief_result.detected else '—'}"))
        guard = self.detector.interference
        learned = ", ".join(f"{band.frequency_hz:.1f}±{band.half_width_hz:.1f}" for band in guard.bands)
        energy = "" if guard.residual_rms is None else f" | Motion RMS {guard.residual_rms:.2f} / quiet limit {guard.energy_limit:.2f}"
        self.interference_note.set_text(self._shorten(
            f"Learned stationary bands (Hz): {learned or 'none'}{energy} | C: recalibrate (10 s)", 165))
        self.interference_note.set_color(self.colors["warning"] if detection.state in INTERFERENCE_STATES else self.colors["muted"])
        self._recording(now)
        self.figure.canvas.draw_idle()
        return True


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="COM port, for example COM5; omitted = auto/select")
    parser.add_argument("--list-ports", action="store_true", help="List COM ports and exit")
    parser.add_argument("--baud", type=int, default=115200,
                        help="CDC line coding (does not set ADC Fs or USB throughput)")
    parser.add_argument("--buffer-size", type=int, default=8192, help="Rolling samples, minimum 4096")
    parser.add_argument("--fft-size", type=int, default=4096, help="FFT samples, minimum 4096")
    parser.add_argument("--wave-seconds", type=float, default=1.0, help="Waveform display duration")
    parser.add_argument("--min-peak", type=float, default=MOTION_ENTER_AMPLITUDE,
                        help="Enter threshold above the adaptive baseline, ADC counts")
    parser.add_argument("--min-snr-db", type=float, default=MIN_PEAK_SNR_DB,
                        help="Minimum peak/background amplitude ratio, dB")
    parser.add_argument("--label", default="", help="Optional CSV filename label, e.g. radial_walk")
    parser.add_argument("--record-dir", type=Path, default=Path(__file__).resolve().parent / "recordings",
                        help="CSV folder; default: recordings beside this script; R toggles recording")
    parser.add_argument("--demo", action="store_true", help="Synthetic clear scene, walking and brief motion")
    args = parser.parse_args(argv)
    if args.fft_size < 4096 or args.buffer_size < args.fft_size:
        parser.error("Require buffer-size >= fft-size >= 4096")
    if not math.isfinite(args.wave_seconds) or not 0 < args.wave_seconds <= args.buffer_size / FS:
        parser.error("wave-seconds must be positive and fit inside buffer-size / 2000")
    if (not math.isfinite(args.min_peak) or not math.isfinite(args.min_snr_db)
            or args.min_peak <= 0 or args.min_snr_db < 0):
        parser.error("min-peak must be positive and min-snr-db nonnegative")
    if args.baud <= 0:
        parser.error("baud must be positive")
    return args


def main(argv=None):
    args = arguments(argv)
    if args.list_ports:
        ports = available_ports()
        for port in ports:
            print(f"{port.device}: {port.description} [{port.hwid}]")
        if not ports:
            print("No COM ports found.")
        return
    import matplotlib.pyplot as plt
    recorder = CsvRecorder()
    state, stop = StreamState(args.buffer_size, recorder), threading.Event()
    connection = None
    worker = None
    try:
        if args.demo:
            label = "DEMO: synthetic walking / brief motion"
            worker = threading.Thread(target=demo_worker, args=(state, stop), daemon=True)
        else:
            import serial
            port = choose_port(args.port)
            connection = serial.Serial(port, baudrate=args.baud, timeout=0.05)
            label = port
            worker = threading.Thread(target=serial_worker, args=(connection, state, stop), daemon=True)
        print(f"Receiving {label}. Close the plot window or press Ctrl+C to stop.")
        print("1 Doppler Hz = %.9f m/s; direction is not available from one ADC channel." % SPEED_PER_HZ)
        print("Focus the plot and press R to start/stop CSV recording. Label:", args.label or "(none)")
        # Matplotlib normally assigns R to reset/home; reserve it for recording.
        plt.rcParams["keymap.home"] = [key for key in plt.rcParams["keymap.home"] if key.lower() != "r"]
        plt.rcParams["keymap.back"] = [key for key in plt.rcParams["keymap.back"] if key.lower() != "c"]
        worker.start()
        plot = RadarPlot(state, args, label)
        plot.figure.canvas.mpl_connect("close_event", lambda event: stop.set())
        plt.show()
    finally:
        stop.set()
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)
        if connection is not None:
            connection.close()
        recorder.stop()
        if recorder.path is not None:
            print(recorder.message)
        plt.close("all")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except ImportError as exc:
        raise SystemExit(f"Missing dependency: {exc}. Run: py -m pip install -r requirements.txt")
    except Exception as exc:
        raise SystemExit(str(exc))
