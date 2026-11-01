"""Wi-Fi/UDP API for the Edugate TurtleBot Burger controller."""

import math
import socket
import threading
import time


class EdugateRobotAPI:
    """Thread-safe wrapper around the robot UDP protocol."""

    def __init__(self, ip="192.168.4.1", local_port=9999,
                 remote_port=8888, cpr=360.0):
        self.ip = ip
        self.local_port = local_port
        self.remote_port = remote_port
        self.cpr = float(cpr)

        self._running = True
        self._motor_cmd = "0,0,0,0"
        self._speed_a, self._dir_a = 0, 0
        self._speed_b, self._dir_b = 0, 0

        self._encoder_a = 0
        self._encoder_b = 0
        self._rpm_a = 0.0
        self._rpm_b = 0.0
        self._last_enc_a = 0
        self._last_enc_b = 0

        self._accel = (0.0, 0.0, 9.81)
        self._gyro = (0.0, 0.0, 0.0)
        self.steering_yaw = 0.0
        self._gyro_z_offset = 0.0
        self._last_imu_time = None
        self._roll_offset = 0.0
        self._pitch_offset = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.local_port))

        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()

        print("[Edugate] API init. Calibrating sensors...")
        self._calibrate_sensors()

    def _calibrate_sensors(self):
        time.sleep(0.5)
        samples_gz, samples_roll, samples_pitch = [], [], []
        for _ in range(25):
            ax, ay, az = self._accel
            gz = self._gyro[2]
            samples_gz.append(gz)
            samples_roll.append(math.atan2(ay, az) * 180.0 / math.pi)
            samples_pitch.append(
                math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180.0 / math.pi
            )
            time.sleep(0.04)
        self._gyro_z_offset = math.degrees(sum(samples_gz) / len(samples_gz))
        self._roll_offset = sum(samples_roll) / len(samples_roll)
        self._pitch_offset = sum(samples_pitch) / len(samples_pitch)
        print("[Edugate] Calibration done. Roll, Pitch, Yaw = 0.0°")

    def _tx_loop(self):
        while self._running:
            try:
                self.sock.sendto(
                    self._motor_cmd.encode(), (self.ip, self.remote_port)
                )
            except OSError:
                pass
            time.sleep(0.05)

    def _rx_loop(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(1024)
                parts = data.decode().strip().split(',')
                if len(parts) != 8:
                    continue
                enc_a, enc_b, ax, ay, az, gx, gy, gz = map(float, parts)
                self._encoder_a = int(enc_a)
                self._encoder_b = int(enc_b)
                self._accel = (ax, ay, az)
                self._gyro = (gx, gy, gz)

                now = time.time()
                if self._last_imu_time is not None:
                    dt = now - self._last_imu_time
                    unbiased_gz = math.degrees(gz) - self._gyro_z_offset
                    if abs(unbiased_gz) > 0.05:
                        self.steering_yaw += unbiased_gz * dt
                        if self.steering_yaw > 180:
                            self.steering_yaw -= 360
                        elif self.steering_yaw < -180:
                            self.steering_yaw += 360
                    if dt > 0:
                        self._rpm_a = (
                            (self._encoder_a - self._last_enc_a) / self.cpr
                        ) / (dt / 60.0)
                        self._rpm_b = (
                            (self._encoder_b - self._last_enc_b) / self.cpr
                        ) / (dt / 60.0)
                self._last_imu_time = now
                self._last_enc_a = self._encoder_a
                self._last_enc_b = self._encoder_b
            except (OSError, ValueError, UnicodeDecodeError):
                pass

    def set_motor_speed(self, motor_id: int, speed: int):
        direction = 1 if speed > 0 else (-1 if speed < 0 else 0)
        abs_speed = min(abs(speed), 255)
        if motor_id == 1:
            self._speed_a = abs_speed
            self._dir_a = direction
        elif motor_id == 2:
            self._speed_b = abs_speed
            self._dir_b = direction
        self._motor_cmd = (
            f"{self._speed_a},{self._dir_a},{self._speed_b},{self._dir_b}"
        )

    def get_motor_speed(self, motor_id: int):
        if motor_id == 1:
            return round(self._rpm_a, 1)
        if motor_id == 2:
            return round(self._rpm_b, 1)
        return 0.0

    def get_encoder_counts(self):
        return self._encoder_a, self._encoder_b

    def get_IMU(self):
        ax, ay, az = self._accel
        roll = math.atan2(ay, az) * 180.0 / math.pi - self._roll_offset
        pitch = (
            math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180.0 / math.pi
            - self._pitch_offset
        )
        return {
            "roll": round(roll, 1),
            "pitch": round(pitch, 1),
            "yaw": round(self.steering_yaw, 1),
        }

    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def close(self):
        self.stop()
        for _ in range(5):
            try:
                self.sock.sendto(
                    b"0,0,0,0", (self.ip, self.remote_port)
                )
            except OSError:
                break
            time.sleep(0.01)
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass
