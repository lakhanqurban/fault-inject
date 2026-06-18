"""
Integration patch for RandomTestGenerator
==========================================
Shows exactly how to plug the FaultInjector into your existing pipeline.
This is the ONLY change needed to RandomTestGenerator.generate().

Replace this block in generate():

    with open(road_data_file, 'r') as f:
        road_data = json.load(f)

With the patched version below. Everything else stays identical.
"""

# In RandomTestGenerator.__init__, add:
#
#   from fault_injection.fault_injector import FaultInjector, FaultConfig, FaultType
#   self.fault_injector = None  # None = no fault injection (default, baseline behaviour)
#
#
# In RandomTestGenerator.generate(), replace the json.load block with:
#
#   with open(road_data_file, 'r') as f:
#       road_data = json.load(f)
#
#   # --- FAULT INJECTION (one line) ---
#   if self.fault_injector is not None:
#       road_data = self.fault_injector.inject(road_data)
#   # --- END FAULT INJECTION ---
#
#   control_points = road_data
#   spline_points  = road_data
#   ...rest unchanged...


# -------------------------------------------------------------------------
# Example: running a single fault configuration
# -------------------------------------------------------------------------

EXAMPLE = """
from driving.random_test_generator import RandomTestGenerator
from fault_injection.fault_injector import FaultInjector, FaultConfig, FaultType

# Normal run (no fault)
gen = RandomTestGenerator(map_size=250)
roads = gen.generate()

# Run with Gaussian noise at moderate severity
cfg = FaultConfig(FaultType.GAUSSIAN_NOISE, severity=0.30, seed=42)
gen.fault_injector = FaultInjector(cfg)
roads = gen.generate()

# Run with curvature injection at severe level, targeting peak curvature region
cfg = FaultConfig(
    FaultType.CURVATURE_INJECTION,
    severity=1.80,
    seed=42,
    inject_at="peak"
)
gen.fault_injector = FaultInjector(cfg)
roads = gen.generate()
"""

# -------------------------------------------------------------------------
# Campaign loop example
# -------------------------------------------------------------------------

CAMPAIGN_LOOP = """
from fault_injection.fault_injector import FaultType, SEVERITY_SWEEPS, SEVERITY_LABELS

# Run all severity levels for all fault types on your seed roads
for fault_type in FaultType:
    for sev_idx, severity in enumerate(SEVERITY_SWEEPS[fault_type]):
        cfg = FaultConfig(fault_type=fault_type, severity=severity, seed=42)
        gen.fault_injector = FaultInjector(cfg)
        roads = gen.generate()     # ← runs the simulator
        # ... collect signal CSVs, run STL monitor, record results
        print(f"{fault_type.value} {SEVERITY_LABELS[sev_idx]}: done")
"""

if __name__ == "__main__":
    print("Integration guide:")
    print("="*60)
    print("1. In RandomTestGenerator.__init__:")
    print("     self.fault_injector = None")
    print()
    print("2. In RandomTestGenerator.generate(), after json.load():")
    print("     if self.fault_injector is not None:")
    print("         road_data = self.fault_injector.inject(road_data)")
    print()
    print("3. Usage example:")
    print(EXAMPLE)
    print()
    print("4. Campaign loop:")
    print(CAMPAIGN_LOOP)
