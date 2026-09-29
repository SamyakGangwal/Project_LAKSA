"""TinyLidarNet-style 1-D CNN and its export to the Jetson NumPy format."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from laksa_learned_driver.policy import MODEL_FORMAT, OutputContract  # noqa: E402
from laksa_learned_driver.scan_features import ScanContract  # noqa: E402

CONVS = ((1, 24, 5, 2), (24, 36, 5, 2), (36, 48, 3, 2), (48, 64, 3, 1), (64, 64, 3, 1))
HIDDEN = (100, 50, 10)


class TinyLidarNet(nn.Module):
    def __init__(self, bins: int) -> None:
        super().__init__()
        self.convs = nn.ModuleList(nn.Conv1d(i, o, k, stride=s) for i, o, k, s in CONVS)
        with torch.no_grad():
            flat = self._features(torch.zeros(1, 1, bins)).shape[1]
        sizes = (flat + 1, *HIDDEN, 2)
        self.fcs = nn.ModuleList(nn.Linear(a, b) for a, b in zip(sizes[:-1], sizes[1:]))

    def _features(self, scan: torch.Tensor) -> torch.Tensor:
        x = scan
        for conv in self.convs:
            x = torch.relu(conv(x))
        return x.flatten(1)

    def forward(self, scan: torch.Tensor, cap_norm: torch.Tensor) -> torch.Tensor:
        h = torch.cat([self._features(scan.unsqueeze(1)), cap_norm.unsqueeze(1)], dim=1)
        for index, fc in enumerate(self.fcs):
            h = fc(h)
            if index < len(self.fcs) - 1:
                h = torch.relu(h)
        return h  # raw logits: [steering (tanh), speed fraction (sigmoid)]


def export_npz(model: TinyLidarNet, path: Path, scan: ScanContract, output: OutputContract,
               training_summary: dict) -> None:
    arrays = {}
    for index, conv in enumerate(model.convs):
        arrays[f"conv{index}_weight"] = conv.weight.detach().cpu().numpy().astype(np.float32)
        arrays[f"conv{index}_bias"] = conv.bias.detach().cpu().numpy().astype(np.float32)
    for index, fc in enumerate(model.fcs):
        arrays[f"fc{index}_weight"] = fc.weight.detach().cpu().numpy().astype(np.float32)
        arrays[f"fc{index}_bias"] = fc.bias.detach().cpu().numpy().astype(np.float32)
    metadata = {
        "format": MODEL_FORMAT,
        "architecture": "TinyLidarNet-1D",
        "conv_strides": [s for _, _, _, s in CONVS],
        "fc_layers": len(model.fcs),
        "scan_contract": scan.to_dict(),
        "output_contract": output.to_dict(),
        "frames": {
            "input_angles": "vehicle frame, 0 = forward, positive = left, origin at LiDAR",
            "steering": "road-wheel angle, positive = left (REP-103)",
        },
        "training": training_summary,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, metadata_json=np.array(json.dumps(metadata, sort_keys=True)), **arrays)
