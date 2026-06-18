"""Public API for the fault injection module."""

from .fault_injector import (
    FaultConfig,
    FaultInjector,
    FaultType,
    SEVERITY_LABELS,
    SEVERITY_SWEEPS,
    make_sweep_configs,
    validate_road,
)

__all__ = [
    "FaultConfig",
    "FaultInjector",
    "FaultType",
    "SEVERITY_LABELS",
    "SEVERITY_SWEEPS",
    "make_sweep_configs",
    "validate_road",
]
