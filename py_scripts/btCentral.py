
#!/usr/bin/env python3
"""
MyoWare BLE Binary Stream Client (EMG + IMU)
--------------------------------------------
- Connects to Nordic UART Service (NUS)
- Parses binary frames with TLV records
- Logs EMG + IMU to CSV
- Real-time plots: EMG raw + envelope, IMU magnitude
"""

import asyncio
import struct
import time
import csv
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

CSV_FILE = "emg_imu_log.csv"

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
                # Wait for more data
                break
            # Now we have full frame
            frame_bytes = bytes(self.buf[:total_len])
            del self.buf[:total_len]
            if flags & 0x01:
                pre_crc = frame_bytes[:-2]
                rx_crc = struct.unpack_from("<H", frame_bytes, len(frame_bytes) - 2)[0]
                calc_crc = crc16_ccitt(pre_crc)
                if calc_crc != rx_crc:
                    print(f"CRC error (calc={calc_crc:04X}, rx={rx_crc:04X}), skipping frame")
                    continue
            # Parse TLVs
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

async def main():
    KNOWN = {TLV_EMG_BATCH, TLV_IMU_BATCH}
    device = await BleakScanner.find_device_by_filter(lambda d, adv: NUS_SERVICE_UUID.lower() in (adv.service_uuids or []))
    if not device:
        print("Device not found.")
        return
    print(f"Connecting to {device.address}...")
    async with BleakClient(device) as client:
        try:
            await client.request_mtu(247)
        except Exception:
            print("MTU request not supported on this platform.")
        print("Connected. Subscribing...")
        parser = FrameParser()
        csv_file = open(CSV_FILE, "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(["ts_ms","env","raw","ref","ax","ay","az","gx","gy","gz"])
        fs_emg = 2000
        env_filter = ButterLPF(fs_emg)
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
                #env_line.set_data(t, [env_filter.update(abs(x)) for x in EMG_RING])
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
            print(f"Received {len(data)} bytes")
            frames = parser.feed(data)
            print(f"Parsed {len(frames)} frames")
            for _, _, tlvs in frames:
                emg_samples = None
                imu_samples = None
                for t, v in tlvs:
                    if t not in KNOWN:
                        # print once per minute to avoid spam
                        print(f"Unknown TLV type={t:#x}")
                    if t == TLV_EMG_BATCH:
                        _, _, emg_samples = decode_emg(v)
                        for s in emg_samples:
                            raw = int(s[1])
                            EMG_RING.append(raw)  # RAW channel
                            ENV_RING.append(env_filter.update(abs(raw)))
                    elif t == TLV_IMU_BATCH:
                        _, _, imu_samples = decode_imu(v)
                        for s in imu_samples:
                            mag = np.sqrt(s[0]**2 + s[1]**2 + s[2]**2)
                            IMU_RING.append(mag)
                if emg_samples is not None or imu_samples is not None:
                    ts_ms = int(time.time()*1000)
                    for i in range(max(len(emg_samples) if emg_samples is not None else 0,
                                       len(imu_samples) if imu_samples is not None else 0)):
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
                    csv_file.flush()
        await client.start_notify(NUS_TX_UUID, handle)
        print("Streaming... Press Ctrl+C to stop.")
        try:
            while True:
                await asyncio.sleep(0.1)
                plt.pause(0.001)  # allow GUI to process events
        except KeyboardInterrupt:
            pass

        except KeyboardInterrupt:
            pass
        await client.stop_notify(NUS_TX_UUID)
        csv_file.close()

if __name__ == "__main__":
    asyncio.run(main())
