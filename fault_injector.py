"""
ADS Geometric Fault Injector
============================
Systematic perturbation of road waypoint data to evaluate ADS resilience.

Canonical fault taxonomy:
    F1 — Waypoint Displacement (GPS spoofing)
    F2 — Curvature Injection (sharp curve insertion)
    F3 — Waypoint Dropout (map data loss)
    F4 — Noise Injection (Gaussian waypoint noise)

Legacy fault names (F1_gaussian_noise, F2_displacement, etc.) remain
supported for backward compatibility.
"""

import numpy as np
import copy
import json
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum


# ---------------------------------------------------------------------------
# Fault type enum and configuration
# ---------------------------------------------------------------------------

class FaultType(Enum):
    # Canonical taxonomy (preferred)
    WAYPOINT_DISPLACEMENT = "F1_waypoint_displacement"
    CURVATURE_INJECTION   = "F2_curvature_injection"
    WAYPOINT_DROPOUT      = "F3_waypoint_dropout"
    NOISE_INJECTION       = "F4_noise_injection"

    # Legacy names (backward compatibility)
    LEGACY_GAUSSIAN_NOISE = "F1_gaussian_noise"
    LEGACY_DISPLACEMENT = "F2_displacement"
    LEGACY_CURVATURE_INJECTION = "F3_curvature_injection"
    LEGACY_DROPOUT = "F4_dropout"


@dataclass
class FaultConfig:
    """
    Fully describes one fault injection configuration.
    Used as the key for experimental bookkeeping.
    """
    fault_type: FaultType
    severity:   float          # primary severity parameter (see per-fault docs)
    seed:       int  = 42      # random seed for reproducibility
    # F1-specific
    affected_fraction: float = 0.20   # fraction of waypoints displaced
    # F2-specific
    window_size:       int   = 12     # number of consecutive waypoints in bump
    inject_at:         str   = "random" # "peak", "entry", "random"
    # F4-specific
    contiguous:        bool  = False  # True = contiguous block dropout
    dropout_blocks:    int   = 1      # number of dropout blocks for F4 contiguous mode

    def __post_init__(self) -> None:
        # Keep config values in safe ranges for reproducible behavior.
        self.severity = float(self.severity)
        self.affected_fraction = min(max(float(self.affected_fraction), 0.0), 1.0)
        self.window_size = max(2, int(self.window_size))
        self.inject_at = str(self.inject_at).lower()
        if self.inject_at not in ("peak", "entry", "random"):
            self.inject_at = "random"
        self.dropout_blocks = max(1, int(self.dropout_blocks))

    def label(self) -> str:
        return f"{self.fault_type.value}_s{self.severity:.3f}_seed{self.seed}"


# ---------------------------------------------------------------------------
# Core injector
# ---------------------------------------------------------------------------

class FaultInjector:
    """
    Applies geometric faults to road waypoint lists.

    Usage:
        cfg = FaultConfig(FaultType.NOISE_INJECTION, severity=0.15, seed=42)
        injector = FaultInjector(cfg)
        perturbed_road = injector.inject(original_road)

    Each inject() call is deterministic given the same seed.
    Original road data is never modified (deep copy).
    """

    def __init__(self, config: FaultConfig):
        self.config = config
        self._call_index = 0

    def inject(self, road: List[List[float]]) -> List[List[float]]:
        """
        Apply the configured fault to road waypoints.

        Args:
            road: list of [x, y, z, width] waypoints

        Returns:
            Perturbed road (new list, original unchanged)
        """
        road = copy.deepcopy(road)
        # Keep runs reproducible while avoiding identical RNG streams for every road.
        rng = np.random.default_rng(self.config.seed + self._call_index)
        self._call_index += 1

        ft = self.config.fault_type
        if ft in (FaultType.NOISE_INJECTION, FaultType.LEGACY_GAUSSIAN_NOISE):
            return self._gaussian_noise(road, rng)
        elif ft in (FaultType.WAYPOINT_DISPLACEMENT, FaultType.LEGACY_DISPLACEMENT):
            return self._displacement(road, rng)
        elif ft in (FaultType.CURVATURE_INJECTION, FaultType.LEGACY_CURVATURE_INJECTION):
            return self._curvature_injection(road, rng)
        elif ft in (FaultType.WAYPOINT_DROPOUT, FaultType.LEGACY_DROPOUT):
            return self._dropout(road, rng)
        else:
            raise ValueError(f"Unknown fault type: {ft}")

    def inject_with_width(self, road: List[List[float]], road_width: float) -> Tuple[List[List[float]], float]:
        """
        Apply fault injection to the geometry and pass through the road width.

        Returns:
            (perturbed_road, perturbed_road_width)
        """
        perturbed_road = self.inject(road)
        return perturbed_road, road_width

    
    # ------------------------------------------------------------------
    # F1 — Targeted waypoint displacement
    # ------------------------------------------------------------------
    def _displacement(self, road, rng):
        """
        Displace a random fraction of waypoints by a fixed magnitude
        in a random direction.

        Severity = displacement magnitude in metres.
        affected_fraction = fraction of waypoints displaced (default 20%).
        Calibrated range: 0.2m (mild) to 2.5m (extreme, ~30% of road width).

        Threat model: GPS spoofing — adversary injects false coordinates
        for a subset of reference points in the HD map.
        """
        n         = len(road)
        if n == 0:
            return road
        delta     = max(0.0, self.config.severity)
        affected_fraction = min(max(self.config.affected_fraction, 0.0), 1.0)
        n_affected = max(1, int(n * affected_fraction))
        indices   = rng.choice(n, size=n_affected, replace=False)

        for idx in indices:
            angle    = float(rng.uniform(0, 2 * np.pi))
            road[idx][0] += delta * np.cos(angle)
            road[idx][1] += delta * np.sin(angle)
        return road

    # ------------------------------------------------------------------
    # F2 — Curvature injection (lateral bump)
    # ------------------------------------------------------------------
    def _curvature_injection(self, road, rng):
        """
        Inject an artificial lateral deviation over a contiguous window
        of waypoints using a smooth sinusoidal bump profile.

        The bump is applied perpendicular to the local road direction,
        so it creates a genuine curve rather than a random offset.

        Severity = maximum lateral displacement (bump amplitude) in metres.
        window_size = number of waypoints affected (default 12 ≈ 4.3m).
        inject_at:
            "peak"   — inject at the road's highest-curvature region
            "entry"  — inject at the straight entry section
            "random" — random location

        Calibrated range: 0.3m (mild) to 2.5m (extreme).

        Threat model: HD map corruption — adversary modifies road geometry
        data to introduce a sharp turn the agent is unprepared for.
        This is the fault type most analogous to ResDrive's attack scenarios.
        """
        n          = len(road)
        if n < 3:
            return road
        amplitude  = max(0.0, self.config.severity)
        window     = min(self.config.window_size, n - 2)
        if window <= 1:
            return road
        pts        = np.array([[wp[0], wp[1]] for wp in road])

        # Determine injection location
        if self.config.inject_at == "peak":
            start = self._find_peak_curvature_region(pts, window)
        elif self.config.inject_at == "entry":
            start = max(1, n // 6)  # first sixth of road
        else:  # random
            start = int(rng.integers(1, max(2, n - window - 1)))

        # Keep injected windows away from extreme road boundaries so the attack
        # is encountered during normal driving rather than at initialization/termination.
        edge_margin = max(1, window // 2)
        max_start = max(1, n - window - edge_margin)
        start = min(max(start, edge_margin), max_start)

        end = min(start + window, n - 1)

        # Smooth sinusoidal bump profile over the window
        window_actual = end - start
        bump_profile  = np.sin(np.linspace(0, np.pi, window_actual))

        for i, wp_idx in enumerate(range(start, end)):
            # Local road direction at this waypoint
            if wp_idx > 0 and wp_idx < n - 1:
                tangent = pts[wp_idx + 1] - pts[wp_idx - 1]
            elif wp_idx == 0:
                tangent = pts[1] - pts[0]
            else:
                tangent = pts[-1] - pts[-2]

            tang_norm = np.linalg.norm(tangent)
            if tang_norm < 1e-10:
                continue

            # Perpendicular (normal) to road direction
            normal = np.array([-tangent[1], tangent[0]]) / tang_norm

            # Apply bump
            offset = amplitude * bump_profile[i] * normal
            road[wp_idx][0] += float(offset[0])
            road[wp_idx][1] += float(offset[1])

        return road

    def _find_peak_curvature_region(self, pts: np.ndarray, window: int) -> int:
        """Find the start index of the highest-curvature window."""
        n = len(pts)
        if n < 3:
            return 1

        curvatures = []
        for i in range(1, n - 1):
            p1, p2, p3 = pts[i-1], pts[i], pts[i+1]
            a = np.linalg.norm(p2 - p1)
            b = np.linalg.norm(p3 - p2)
            c = np.linalg.norm(p3 - p1)
            area = abs(np.cross(p2 - p1, p3 - p1)) / 2
            denom = a * b * c
            curvatures.append((4 * area / denom) if denom > 1e-10 else 0.0)

        curv = np.array(curvatures)
        # Find window with highest mean curvature
        best_start, best_mean = 1, 0.0
        for s in range(1, max(2, n - window - 1)):
            e = min(s + window, n - 1)
            mean_k = curv[s-1:e-1].mean() if e > s else 0.0
            if mean_k > best_mean:
                best_mean, best_start = mean_k, s

        return best_start

    # ------------------------------------------------------------------
    # F3 — Waypoint dropout
    # ------------------------------------------------------------------
    def _dropout(self, road, rng):
        """
        Remove a fraction of waypoints from the road.

        Severity = dropout rate (fraction of waypoints removed).
        contiguous=False: randomly distributed dropouts
        contiguous=True:  one or more contiguous blocks removed (harder fault)

        Always preserves first and last waypoints.
        Calibrated range: 0.05 (mild, 1-in-20) to 0.40 (extreme, 2-in-5).

        Threat model: partial map data loss — segment of HD map data
        corrupted or missing during transmission, forcing spline
        interpolation to bridge larger gaps than it was designed for.
        """
        rate = min(max(self.config.severity, 0.0), 1.0)
        n    = len(road)
        if n < 3:
            return road

        # Always keep first and last
        interior = list(range(1, n - 1))
        n_drop   = max(0, int(len(interior) * rate))

        if n_drop == 0:
            return road

        if self.config.contiguous:
            # Remove one or more contiguous blocks.
            blocks = max(1, int(self.config.dropout_blocks))
            block_size = max(1, n_drop // blocks)
            remainder = n_drop - (block_size * blocks)

            edge_margin = max(1, block_size)
            taken = set()
            pts = np.array([[wp[0], wp[1]] for wp in road], dtype=float)
            local_curv = self._point_curvatures(pts)

            for b in range(blocks):
                curr_size = block_size + (1 if b < remainder else 0)
                if curr_size <= 0:
                    continue

                # Candidate start range keeps blocks away from edges.
                min_start = edge_margin
                max_start = len(interior) - curr_size - edge_margin
                if max_start < min_start:
                    min_start = 0
                    max_start = max(0, len(interior) - curr_size)

                # Prefer high-curvature windows to make dropout faults harder.
                best_start_idx = None
                best_score = -1.0
                for start_idx in range(min_start, max_start + 1):
                    candidate = set(interior[start_idx: start_idx + curr_size])
                    if not candidate.isdisjoint(taken):
                        continue
                    score = float(local_curv[list(candidate)].mean())
                    score += float(rng.uniform(0.0, 1e-6))
                    if score > best_score:
                        best_score = score
                        best_start_idx = start_idx

                if best_start_idx is not None:
                    candidate = set(interior[best_start_idx: best_start_idx + curr_size])
                    taken.update(candidate)
                    placed = True
                else:
                    # Fallback: place anywhere non-overlapping if possible.
                    placed = False
                    for start_idx in range(min_start, max_start + 1):
                        candidate = set(interior[start_idx: start_idx + curr_size])
                        if candidate.isdisjoint(taken):
                            taken.update(candidate)
                            placed = True
                            break

                if not placed:
                    # If we cannot place more blocks, stop gracefully.
                    break

            drop_set = taken

            # Add local boundary kinks around removed contiguous runs to make
            # the resulting track disturbance more pronounced without exceeding
            # the global validation constraints.
            self._apply_dropout_boundary_kinks(road, drop_set, rate)
        else:
            # Random distributed dropouts
            drop_indices = rng.choice(interior, size=n_drop, replace=False)
            drop_set     = set(drop_indices)

        # Keep dropout disruptive but avoid producing roads that are rejected
        # solely due to oversized interpolation gaps.
        drop_set = self._repair_drop_set_for_gap(road, drop_set, max_gap=9.9)

        return [wp for i, wp in enumerate(road) if i not in drop_set]
    
    def _point_curvatures(self, pts: np.ndarray) -> np.ndarray:
        """Estimate pointwise curvature; endpoints are assigned zero."""
        n = len(pts)
        if n < 3:
            return np.zeros(n, dtype=float)

        curv = np.zeros(n, dtype=float)
        for i in range(1, n - 1):
            p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
            a = np.linalg.norm(p2 - p1)
            b = np.linalg.norm(p3 - p2)
            c = np.linalg.norm(p3 - p1)
            area = abs(np.cross(p2 - p1, p3 - p1)) / 2
            denom = a * b * c
            curv[i] = (4 * area / denom) if denom > 1e-10 else 0.0
        return curv

    def _apply_dropout_boundary_kinks(self, road, drop_set: set, rate: float) -> None:
        """Introduce a lateral mismatch at dropout boundaries to increase impact."""
        if not drop_set:
            return

        sorted_idx = sorted(drop_set)
        runs = []
        start = prev = sorted_idx[0]
        for idx in sorted_idx[1:]:
            if idx == prev + 1:
                prev = idx
                continue
            runs.append((start, prev))
            start = prev = idx
        runs.append((start, prev))

        pts = np.array([[wp[0], wp[1]] for wp in road], dtype=float)
        kink = min(1.8, max(0.35, rate * 6.0))

        for run_start, run_end in runs:
            left = run_start - 1
            right = run_end + 1
            if left <= 0 or right >= len(road) - 1:
                continue

            tangent = pts[right] - pts[left]
            tang_norm = np.linalg.norm(tangent)
            if tang_norm < 1e-10:
                continue

            normal = np.array([-tangent[1], tangent[0]]) / tang_norm
            left_off = -0.5 * kink * normal
            right_off = 0.5 * kink * normal

            road[left][0] += float(left_off[0])
            road[left][1] += float(left_off[1])
            road[right][0] += float(right_off[0])
            road[right][1] += float(right_off[1])

    def _repair_drop_set_for_gap(self, road, drop_set: set, max_gap: float = 9.9) -> set:
        """Reinsert points when dropout creates too-large waypoint gaps."""
        if not drop_set:
            return drop_set

        pts = np.array([[wp[0], wp[1]] for wp in road], dtype=float)
        drop_set = set(drop_set)

        while True:
            keep = [i for i in range(len(road)) if i not in drop_set]
            if len(keep) < 2:
                break

            worst_dist = -1.0
            worst_pair = None
            for li, ri in zip(keep[:-1], keep[1:]):
                d = float(np.linalg.norm(pts[ri] - pts[li]))
                if d > worst_dist:
                    worst_dist = d
                    worst_pair = (li, ri)

            if worst_pair is None or worst_dist <= max_gap:
                break

            li, ri = worst_pair
            candidates = [idx for idx in drop_set if li < idx < ri]
            if not candidates:
                # No dropped point exists between the large-gap pair.
                break

            # Reinsert the midpoint index first to split the largest gap quickly.
            candidates.sort()
            restore_idx = candidates[len(candidates) // 2]
            drop_set.remove(restore_idx)

        return drop_set

    # ------------------------------------------------------------------
    # F4 — Gaussian noise on all x,y coordinates
    # ------------------------------------------------------------------
    def _gaussian_noise(self, road, rng):
        """
        Add zero-mean Gaussian noise to every waypoint's x and y.

        Severity = noise standard deviation in metres.
        Calibrated range: 0.05m (mild) to 0.80m (extreme, ~2x segment spacing).

        Threat model: accumulated sensor noise in mapping pipeline,
        or low-amplitude GPS multipath interference.
        """
        sigma = max(0.0, self.config.severity)
        for wp in road:
            wp[0] += float(rng.normal(0, sigma))  # x
            wp[1] += float(rng.normal(0, sigma))  # y
        return road

# ---------------------------------------------------------------------------
# Preset severity sweeps
# ---------------------------------------------------------------------------

SEVERITY_SWEEPS = {
    FaultType.NOISE_INJECTION: [0.05, 0.15, 0.30, 0.50, 0.80],
    FaultType.WAYPOINT_DISPLACEMENT:   [0.20, 0.50, 1.00, 1.50, 2.50],
    FaultType.CURVATURE_INJECTION: [0.30, 0.70, 1.20, 1.80, 2.50],
    FaultType.WAYPOINT_DROPOUT:        [0.05, 0.10, 0.20, 0.30, 0.40],
}

SEVERITY_LABELS = ["mild", "moderate", "strong", "severe", "extreme"]


def make_sweep_configs(
    fault_type: FaultType,
    seeds: List[int] = None,
    **kwargs
) -> List[FaultConfig]:
    """
    Generate a list of FaultConfigs sweeping all severity levels
    for a given fault type.

    Args:
        fault_type: which fault to sweep
        seeds: list of random seeds (one config per seed per severity)
        **kwargs: additional FaultConfig fields (e.g. inject_at="entry")

    Returns:
        List of FaultConfig objects ordered mild → extreme
    """
    if seeds is None:
        seeds = [42]
    severities = SEVERITY_SWEEPS[fault_type]
    configs = []
    for sev in severities:
        for seed in seeds:
            configs.append(FaultConfig(
                fault_type=fault_type,
                severity=sev,
                seed=seed,
                **kwargs
            ))
    return configs


# ---------------------------------------------------------------------------
# Utility: validate injected road
# ---------------------------------------------------------------------------

def validate_road(road: List[List[float]], original_len: int = None) -> Tuple[bool, str]:
    """
    Basic sanity checks on injected road data.
    Returns (is_valid, reason_if_invalid).
    """
    if not road or len(road) < 3:
        return False, f"Too few waypoints: {len(road)}"

    if original_len and len(road) < original_len * 0.5:
        return False, f"Too many dropouts: {len(road)}/{original_len} remain"

    pts = np.array([[wp[0], wp[1]] for wp in road])
    diffs = np.diff(pts, axis=0)
    dists = np.linalg.norm(diffs, axis=1)

    if dists.max() > 10.0:
        return False, f"Waypoint gap too large: {dists.max():.2f}m (max allowed 10m)"

    if dists.min() < 0.01:
        return False, f"Duplicate/near-duplicate waypoints: min dist {dists.min():.4f}m"

    return True, "OK"


# ---------------------------------------------------------------------------
# Quick test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys, os
    sys.path.insert(0, os.path.dirname(__file__))

    # Load the failed road
    road_path = os.path.join(os.path.dirname(__file__), "test_road.json")
    if os.path.exists(road_path):
        with open(road_path) as f:
            road = json.load(f)
    else:
        # minimal synthetic road
        road = [[100.0 + i*0.5, 100.0 - i*0.4, 0.0, 3.0] for i in range(50)]

    print(f"Original road: {len(road)} waypoints")
    print()

    # Recommended demo set: single-attack style defaults with safer severities.
    tests = [
        FaultConfig(FaultType.WAYPOINT_DISPLACEMENT, severity=0.50, seed=42),
        FaultConfig(FaultType.CURVATURE_INJECTION, severity=1.20, seed=42, inject_at="peak"),
        FaultConfig(FaultType.WAYPOINT_DROPOUT, severity=0.05, seed=42, contiguous=True),
        FaultConfig(FaultType.NOISE_INJECTION, severity=0.15, seed=42),
    ]

    for cfg in tests:
        injector = FaultInjector(cfg)
        perturbed = injector.inject(road)
        valid, reason = validate_road(perturbed, len(road))
        print(f"  {cfg.fault_type.value:<30} severity={cfg.severity:.2f}  "
              f"waypoints: {len(road)}->{len(perturbed)}  valid={valid}  {reason}")
