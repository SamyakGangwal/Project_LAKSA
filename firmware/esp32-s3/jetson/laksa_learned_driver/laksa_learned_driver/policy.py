"""Dependency-free (NumPy) inference for the LAKSA learned LiDAR driver.

The network is a TinyLidarNet-style 1-D CNN trained by imitation learning in
simulation.  Training exports every weight plus the full input/output contract
into one ``.npz`` file so the Jetson needs neither PyTorch nor ONNX Runtime.

Inputs:  ``bins`` normalized LiDAR ranges and the requested speed cap.
Outputs: road-wheel steering angle (rad, positive left) and target speed (m/s).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from .scan_features import ScanContract, bin_scan, normalize

MODEL_FORMAT = "laksa-learned-driver-v1"


@dataclass(frozen=True)
class OutputContract:
    steering_left_max_rad: float
    steering_right_max_rad: float
    speed_cap_norm_mps: float
    wheelbase_m: float

    @classmethod
    def from_dict(cls, data: dict) -> "OutputContract":
        return cls(
            steering_left_max_rad=float(data["steering_left_max_rad"]),
            steering_right_max_rad=float(data["steering_right_max_rad"]),
            speed_cap_norm_mps=float(data["speed_cap_norm_mps"]),
            wheelbase_m=float(data["wheelbase_m"]),
        )

    def to_dict(self) -> dict:
        return {
            "steering_left_max_rad": self.steering_left_max_rad,
            "steering_right_max_rad": self.steering_right_max_rad,
            "speed_cap_norm_mps": self.speed_cap_norm_mps,
            "wheelbase_m": self.wheelbase_m,
        }

    def encode_steering(self, steering_rad):
        steering = np.asarray(steering_rad, dtype=np.float64)
        return np.where(
            steering >= 0.0,
            steering / self.steering_left_max_rad,
            steering / self.steering_right_max_rad,
        )

    def decode_steering(self, normalized):
        value = np.clip(np.asarray(normalized, dtype=np.float64), -1.0, 1.0)
        return np.where(
            value >= 0.0,
            value * self.steering_left_max_rad,
            value * self.steering_right_max_rad,
        )


def _conv1d(x: np.ndarray, weight: np.ndarray, bias: np.ndarray, stride: int) -> np.ndarray:
    windows = sliding_window_view(x, weight.shape[2], axis=1)[:, ::stride, :]
    return np.einsum("ilk,oik->ol", windows, weight, optimize=True) + bias[:, None]


class LearnedDriverPolicy:
    def __init__(self, path: str | Path) -> None:
        with np.load(Path(path), allow_pickle=False) as data:
            meta = json.loads(str(data["metadata_json"]))
            if meta.get("format") != MODEL_FORMAT:
                raise ValueError(f"unsupported model format: {meta.get('format')}")
            self.metadata = meta
            self.scan = ScanContract.from_dict(meta["scan_contract"])
            self.output = OutputContract.from_dict(meta["output_contract"])
            self._convs = [
                (
                    data[f"conv{i}_weight"].astype(np.float32),
                    data[f"conv{i}_bias"].astype(np.float32),
                    int(stride),
                )
                for i, stride in enumerate(meta["conv_strides"])
            ]
            self._fcs = [
                (data[f"fc{i}_weight"].astype(np.float32), data[f"fc{i}_bias"].astype(np.float32))
                for i in range(int(meta["fc_layers"]))
            ]
        if self._convs[0][0].shape[1] != 1:
            raise ValueError("first convolution must take one LiDAR channel")

    def forward(self, features: np.ndarray, speed_cap_mps: float) -> tuple[float, float]:
        """Return raw network outputs (normalized steering, speed fraction)."""
        x = np.asarray(features, dtype=np.float32).reshape(1, -1)
        if x.shape[1] != self.scan.bins:
            raise ValueError(f"expected {self.scan.bins} LiDAR features, got {x.shape[1]}")
        for weight, bias, stride in self._convs:
            x = np.maximum(_conv1d(x, weight, bias, stride), 0.0)
        cap = np.float32(speed_cap_mps / self.output.speed_cap_norm_mps)
        h = np.concatenate([x.reshape(-1), np.array([cap], dtype=np.float32)])
        for index, (weight, bias) in enumerate(self._fcs):
            h = weight @ h + bias
            if index < len(self._fcs) - 1:
                h = np.maximum(h, 0.0)
        steering_norm = float(np.tanh(h[0]))
        speed_fraction = float(1.0 / (1.0 + np.exp(-h[1])))
        return steering_norm, speed_fraction

    def act_on_binned(self, binned_m: np.ndarray, speed_cap_mps: float) -> tuple[float, float]:
        steering_norm, speed_fraction = self.forward(normalize(binned_m, self.scan), speed_cap_mps)
        steering = float(self.output.decode_steering(steering_norm))
        return steering, speed_fraction * speed_cap_mps

    def act(self, ranges, angles, speed_cap_mps: float) -> tuple[float, float]:
        """Map a vehicle-frame scan to (steering_rad, speed_mps)."""
        return self.act_on_binned(bin_scan(ranges, angles, self.scan), speed_cap_mps)
