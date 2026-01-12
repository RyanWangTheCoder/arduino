
#!/usr/bin/env python3
"""
MyoWare BLE Binary Stream Client (EMG + IMU + Vitals)
------------------------------------------------------
- Connects to Nordic UART Service (NUS)
- Parses binary frames with TLV records
- Logs EMG + IMU to rotating CSV files
- Logs SpO₂ + HR to separate CSV file
- Real-time plots: EMG raw + envelope, IMU magnitude
"""

import asyncio
import struct
import time
import csv
import os
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from bleak import BleakScanner, BleakClient

# UUIDs
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

MAGIC = b"\xA5\x5A"
HEADER_LEN = 14
CRC_LEN = 2
TLV_EMG_BATCH = 0x01
TLV_IMU_BATCH = 0x02
TLV_VITALS    = 0x03

# CSV rotation settings for EMG/IMU
MAX_ROWS = 2000000
file_idx = 0
row_count = 0

def open_csv():
    global file_idx, row_count
    fname = f"emg_imu_log_{file_idx:03d}.csv"
    f = open(fname, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["ts_ms","env","raw","ref","ax","ay","az","gx","gy","gz"])
    print(f"[CSV] Logging EMG/IMU to {fname}")
    row_count = 0
    return f, w

csv_file, writer = open_csv()

# Vitals CSV
vitals_file = open("vitals_log.csv", "w", newline="", encoding="utf-8")
vitals_writer = csv.writer(vitals_file)
vitals_writer.writerow(["ts_ms","spo2","hr_bpm","age_ms","flags","brightness"])
print("[CSV] Logging Vitals to vitals_log.csv")

# Plot buffers
EMG_RING = deque(maxlen=5000)
ENV_RING = deque(maxlen=5000)
IMU_RING = deque(maxlen=5000)

# Filters for EMG envelope
class ButterLPF:
    def __init__(self, fs, cutoff=3.6):
        from scipy.signal import butter, lfilter_zi
        self.b, self.a = butter(2, cutoff / (fs / 2.0), btype='low')
        self.zi = lfilter_zi(self.b, self.a) * 0.0
    def update(self, x):
        from scipy.signal import lfilter
        y, self.zi = lfilter(self.b, self.a, [x], zi=self.zi)
        return float(max(y[0], 0.0))

# CRC16
def crc16_ccitt(data, crc=0xFFFF):
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF

# Frame parser
class FrameParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < HEADER_LEN:
                break
            # Peek header
            magic0, magic1, ver, flags, frame_seq, t_us, payload_len = struct.unpack_from("<BBBBIIH", self.buf, 0)
            total_len = HEADER_LEN + payload_len + CRC_LEN
            if len(self.buf) < total_len:
                break
            frame_bytes = bytes(self.buf[:total_len])
            del self.buf[:total_len]
            if flags & 0x01:
                pre_crc = frame_bytes[:-2]
                rx_crc = struct.unpack_from("<H", frame_bytes, len(frame_bytes) - 2)[0]
                calc_crc = crc16_ccitt(pre_crc)
                if calc_crc != rx_crc:
                    print(f"[WARN] CRC error (calc={calc_crc:04X}, rx={rx_crc:04X}), skipping frame")
                    continue
            payload = frame_bytes[HEADER_LEN:HEADER_LEN + payload_len]
            tlvs = []
            off = 0
            while off + 3 <= len(payload):
                tlv_type = payload[off]
                tlv_len = struct.unpack_from("<H", payload, off + 1)[0]
                off += 3
                if off + tlv_len > len(payload):
                    break
                tlv_val = payload[off:off + tlv_len]
                off += tlv_len
                tlvs.append((tlv_type, tlv_val))
            frames.append((frame_seq, t_us, tlvs))
        return frames

def decode_emg(val):
    emg_seq0, t0_off, sr_hz = struct.unpack_from("<IhH", val, 0)
    channels = val[8]
    count = struct.unpack_from("<H", val, 11)[0]
    samples = np.frombuffer(val[13:], dtype="<u2").reshape(count, channels)
    return emg_seq0, sr_hz, samples

def decode_imu(val):
    imu_seq0, t0_off, sr_hz = struct.unpack_from("<IhH", val, 0)
    count = val[8]
    samples = np.frombuffer(val[12:], dtype="<i2").reshape(count, 6)
    return imu_seq0, sr_hz, samples

def decode_vitals(val):
    # Layout: spo2(u16), hr_x10(u16), age_ms(u16), flags(u8), brightness(u8)
    spo2, hr_x10, age_ms, flags, bright = struct.unpack_from("<HHHBB", val, 0)
    spo2_val = spo2 if spo2 != 0xFFFF else None
    hr_val = (hr_x10 / 10.0) if hr_x10 != 0xFFFF else None
    return {
        "spo2": spo2_val,
        "hr": hr_val,
        "age_ms": age_ms,
        "flags": flags,
        "brightness": bright
    }

async def main():
    KNOWN = {TLV_EMG_BATCH, TLV_IMU_BATCH, TLV_VITALS}
    device = await BleakScanner.find_device_by_filter(lambda d, adv: NUS_SERVICE_UUID.lower() in (adv.service_uuids or []))
    if not device:
        print("Device not found.")
        return
    print(f"Connecting to {device.address}...")
    async with BleakClient(device) as client:
        try:
            await client.request_mtu(247)
        except Exception:
            print("[INFO] MTU request not supported on this platform.")
        print("Connected. Subscribing...")
        parser = FrameParser()
        fs_emg = 2000
        env_filter = ButterLPF(fs_emg)

        # Plot setup
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
        emg_line, = ax1.plot([], [], lw=1.0, label="EMG RAW")
        env_line, = ax1.plot([], [], lw=1.5, label="Envelope")
        imu_line, = ax2.plot([], [], lw=1.2, label="IMU |acc|")
        ax1.legend(); ax2.legend()

        def update_plot(_):
            if len(EMG_RING) > 0:
                t = np.arange(len(EMG_RING)) / fs_emg
                emg_line.set_data(t, list(EMG_RING))
                env_line.set_data(t, list(ENV_RING))
                ax1.set_xlim(max(0, t[-1]-2), t[-1])
                ax1.set_ylim(0, max(1, max(EMG_RING)*1.1))
            if len(IMU_RING) > 0:
                imu_line.set_data(range(len(IMU_RING)), IMU_RING)
                ax2.set_xlim(0, len(IMU_RING))
                ax2.set_ylim(0, max(1, max(IMU_RING)*1.1))
            return emg_line, env_line, imu_line

        ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)
        plt.tight_layout()
        plt.show(block=False)

        def handle(_, data):
            global csv_file, writer, row_count, file_idx
            frames = parser.feed(data)
            for _, _, tlvs in frames:
                emg_samples = None
                imu_samples = None
                for t, v in tlvs:
                    if t not in KNOWN:
                        print(f"[DEBUG] Unknown TLV type={t:#x}")
                    if t == TLV_EMG_BATCH:
                        _, _, emg_samples = decode_emg(v)
                        for s in emg_samples:
                            raw = int(s[1])
                            EMG_RING.append(raw)
                            ENV_RING.append(env_filter.update(abs(raw)))
                    elif t == TLV_IMU_BATCH:
                        _, _, imu_samples = decode_imu(v)
                        for s in imu_samples:
                            mag = np.sqrt(s[0]**2 + s[1]**2 + s[2]**2)
                            IMU_RING.append(mag)
                    elif t == TLV_VITALS:
                        vitals = decode_vitals(v)
                        ts_ms = int(time.time()*1000)
                        print(f"[Vitals] SpO₂={vitals['spo2'] if vitals['spo2'] else 'NA'}% "
                              f"HR={vitals['hr'] if vitals['hr'] else 'NA'} bpm "
                              f"(age={vitals['age_ms']}ms, bright=0x{vitals['brightness']:02X})")
                        vitals_writer.writerow([
                            ts_ms,
                            vitals['spo2'] if vitals['spo2'] else "",
                            vitals['hr'] if vitals['hr'] else "",
                            vitals['age_ms'],
                            vitals['flags'],
                            vitals['brightness']
                        ])
                        vitals_file.flush()
                if emg_samples is not None or imu_samples is not None:
                    ts_ms = int(time.time()*1000)
                    max_len = max(len(emg_samples) if emg_samples is not None else 0,
                                  len(imu_samples) if imu_samples is not None else 0)
                    for i in range(max_len):
                        env = emg_samples[i][0] if emg_samples is not None and i < len(emg_samples) else ""
                        raw = emg_samples[i][1] if emg_samples is not None and i < len(emg_samples) else ""
                        ref = emg_samples[i][2] if emg_samples is not None and i < len(emg_samples) else ""
                        ax = imu_samples[i][0] if imu_samples is not None and i < len(imu_samples) else ""
                        ay = imu_samples[i][1] if imu_samples is not None and i < len(imu_samples) else ""
                        az = imu_samples[i][2] if imu_samples is not None and i < len(imu_samples) else ""
                        gx = imu_samples[i][3] if imu_samples is not None and i < len(imu_samples) else ""
                        gy = imu_samples[i][4] if imu_samples is not None and i < len(imu_samples) else ""
                        gz = imu_samples[i][5] if imu_samples is not None and i < len(imu_samples) else ""
                        writer.writerow([ts_ms, env, raw, ref, ax, ay, az, gx, gy, gz])
                        row_count += 1
                        if row_count >= MAX_ROWS:
                            csv_file.close()
                            file_idx += 1
                            csv_file, writer = open_csv()
                    csv_file.flush()

        await client.start_notify(NUS_TX_UUID, handle)
        print("Streaming... Press Ctrl+C to stop.")
        try:
            while True:
                await asyncio.sleep(0.1)
                plt.pause(0.001)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(NUS_TX_UUID)
        csv_file.close()
        vitals_file.close()

if __name__ == "__main__":
    asyncio.run(main())
