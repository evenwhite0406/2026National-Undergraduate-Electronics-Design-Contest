#!/usr/bin/env python3
"""通过K230 UART3自动测试并整定独立双轮速度PID。"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少pyserial，请先执行: python -m pip install pyserial")
    raise SystemExit(2)


@dataclass
class Sample:
    timestamp: float
    target: int
    left: int
    right: int
    left_pwm: int
    right_pwm: int


class SpeedPidTuner:
    def __init__(self, port: str, baud: int, target: int, csv_path: Path):
        self.ser = serial.Serial(port, baud, timeout=0.15)
        self.target = target
        self.csv_path = csv_path
        self.all_samples: list[Sample] = []

    def close(self) -> None:
        self.ser.close()

    def send(self, command: str) -> None:
        self.ser.write((command.strip() + "\n").encode("ascii"))
        self.ser.flush()

    def read_sample(self) -> Sample | None:
        raw = self.ser.readline().decode("ascii", errors="ignore").strip()
        if not raw:
            return None
        if not raw.startswith("DATA,"):
            print("固件:", raw)
            return None
        fields = raw.split(",")
        if len(fields) != 6:
            return None
        try:
            sample = Sample(time.time(), *(int(value) for value in fields[1:]))
        except ValueError:
            return None
        self.all_samples.append(sample)
        return sample

    def wait_ready(self, timeout: float = 8.0) -> None:
        self.ser.reset_input_buffer()
        self.send("GET")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.ser.readline().decode("ascii", errors="ignore").strip()
            if raw:
                print("固件:", raw)
                if raw.startswith(("PARAM,", "READY,")):
                    return
            self.send("GET")
            time.sleep(0.25)
        raise RuntimeError(
            "未收到固件响应：请确认已运行SPEED PID TUNE、串口为UART3且波特率正确"
        )

    def run_case(self, kp: int, ki: int, kd: int,
                 duration: float = 4.5, settle: float = 1.5) -> tuple[float, list[Sample]]:
        print(f"\n测试 Kp={kp/1000:.3f} Ki={ki/1000:.3f} Kd={kd/1000:.3f}")
        self.send("TARGET,0")
        time.sleep(0.7)
        self.send(f"PID,{kp},{ki},{kd}")
        self.send(f"TARGET,{self.target}")
        started = time.monotonic()
        samples: list[Sample] = []
        overload_count = 0
        while time.monotonic() - started < duration:
            sample = self.read_sample()
            if sample is None:
                continue
            elapsed = time.monotonic() - started
            print(
                f"\rT={sample.target:3d} L={sample.left:3d} R={sample.right:3d} "
                f"PWM={sample.left_pwm:3d}/{sample.right_pwm:3d}",
                end="", flush=True,
            )
            if max(sample.left, sample.right) > min(100, self.target + 25):
                overload_count += 1
            else:
                overload_count = 0
            if overload_count >= 5:
                self.send("TARGET,0")
                raise RuntimeError("速度连续严重超调，已将目标置0并终止自动调参")
            if elapsed >= settle:
                samples.append(sample)
        print()
        self.send("TARGET,0")
        if len(samples) < 10:
            return 1e9, samples

        averages = [(s.left + s.right) / 2.0 for s in samples]
        differences = [abs(s.left - s.right) for s in samples]
        mae = statistics.mean(abs(value - self.target) for value in averages)
        ripple = statistics.pstdev(averages)
        mismatch = statistics.mean(differences)
        overshoot = max(0.0, max(averages) - self.target)
        saturation = statistics.mean(
            1.0 if max(s.left_pwm, s.right_pwm) >= 98 else 0.0 for s in samples
        )
        score = mae + 0.55 * ripple + 0.35 * mismatch + 1.5 * overshoot + 10.0 * saturation
        print(
            f"评分={score:.3f} 误差={mae:.2f} 波动={ripple:.2f} "
            f"左右差={mismatch:.2f} 超调={overshoot:.2f}"
        )
        return score, samples

    def auto_tune(self) -> tuple[int, int, int]:
        best_kp, best_ki, best_kd = 1500, 0, 0
        best_score = 1e9

        print("\n阶段1：Ki=Kd=0，搜索Kp")
        for kp in (600, 900, 1200, 1500, 1800, 2200, 2800):
            score, _ = self.run_case(kp, 0, 0)
            if score < best_score:
                best_score, best_kp = score, kp

        print(f"\n阶段2：固定Kp={best_kp}，搜索Ki")
        best_score = 1e9
        for ki in (100, 250, 400, 550, 700, 850, 1000):
            score, _ = self.run_case(best_kp, ki, 0)
            if score < best_score:
                best_score, best_ki = score, ki

        print(f"\n阶段3：固定Kp={best_kp}, Ki={best_ki}，小范围搜索Kd")
        best_score = 1e9
        for kd in (0, 2, 5, 10, 20):
            score, _ = self.run_case(best_kp, best_ki, kd)
            if score < best_score:
                best_score, best_kd = score, kd

        self.send(f"PID,{best_kp},{best_ki},{best_kd}")
        self.send(f"TARGET,{self.target}")
        print(
            f"\n推荐并已应用: SPD_PID_KP_X1000={best_kp}, "
            f"SPD_PID_KI_X1000={best_ki}, SPD_PID_KD_X1000={best_kd}"
        )
        return best_kp, best_ki, best_kd

    def save_csv(self) -> None:
        with self.csv_path.open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.writer(file)
            writer.writerow(("timestamp", "target", "left", "right", "left_pwm", "right_pwm"))
            for sample in self.all_samples:
                writer.writerow((sample.timestamp, sample.target, sample.left, sample.right,
                                 sample.left_pwm, sample.right_pwm))
        print("数据已保存:", self.csv_path.resolve())


def choose_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    if len(ports) == 1:
        print("自动选择串口:", ports[0].device, ports[0].description)
        return ports[0].device
    print("检测到的串口：")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    raise RuntimeError("请使用 --port COMx 指定USB转串口模块")


def main() -> int:
    parser = argparse.ArgumentParser(description="MSPM0双轮速度PID自动调参")
    parser.add_argument("--port", help="串口，例如COM6；只有一个串口时可省略")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--target", type=int, default=15, choices=range(5, 36), metavar="5..35")
    parser.add_argument("--csv", type=Path, default=Path("speed_pid_tuning.csv"))
    args = parser.parse_args()

    port = choose_port(args.port)
    print("安全提示：必须架空车轮，保持电源稳定，确认按键4可随时停机。")
    input("运行固件中的SPEED PID TUNE任务后，按回车开始连接……")
    try:
        tuner = SpeedPidTuner(port, args.baud, args.target, args.csv)
    except serial.SerialException as error:
        print(f"无法打开串口 {port}: {error}")
        print("请关闭串口助手/CCS终端，重新插拔USB转串口，并确认设备管理器中的COM号。")
        print("当前检测到的串口：")
        for item in list_ports.comports():
            print(f"  {item.device}: {item.description} [{item.hwid}]")
        return 2
    try:
        tuner.wait_ready()
        tuner.auto_tune()
        print("最终参数正在持续运行，观察确认后按Ctrl+C结束；固件按键4负责停机。")
        while True:
            tuner.read_sample()
    except KeyboardInterrupt:
        print("\n用户中止，发送TARGET,0。")
    finally:
        try:
            tuner.send("TARGET,0")
        except serial.SerialException:
            pass
        tuner.save_csv()
        tuner.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
