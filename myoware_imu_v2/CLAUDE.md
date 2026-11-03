# CLAUDE.md — myoware_imu_v2 (ESP32 firmware + hardware)

This is the **firmware/hardware** repo for the wearable: ESP32 + MyoWare 2.0 EMG + IMU + MAX30105 vitals,
streaming binary TLV frames over Nordic UART Service (BLE). Entry point `myoware_imu_v2.ino`;
design in `HLD_FW.md`; pin map / electrode placement / RAW pogo-pin fix in `HARDWARE.md`.
This device's IMU FSR settings (`IMU_ACC_FSR_G`, `IMU_GYRO_FSR_DPS`) and EMG rate determine how the
captured logs must be scaled downstream — keep them in sync with the analysis configs.

Shared system context, data formats, terminology, and working rules are in the emg_util analysis repo:

@../../emg_util/CLAUDE.md
