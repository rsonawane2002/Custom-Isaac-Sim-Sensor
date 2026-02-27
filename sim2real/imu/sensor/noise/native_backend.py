import sys

# Path to your compiled C++ .so file
COMPILED_CODE_PATH = "/home/sameer/Downloads/"

if COMPILED_CODE_PATH not in sys.path:
    sys.path.append(COMPILED_CODE_PATH)

try:
    import sim2real_native_v0_1 as sim2real_native
    _NATIVE_AVAILABLE = True
    print("[Sim2Real IMU] Native C++ backend loaded successfully.")
except ImportError as e:
    _NATIVE_AVAILABLE = False
    print(f"[Sim2Real IMU] WARNING: Could not load C++ backend: {e}")
    print("[Sim2Real IMU] IMU prims will be registered but noise will not be applied.")


class NativeBackend:
    """
    Wraps the C++ sim2real_native pybind module.
    One Sim2RealCore instance is created per IMU prim path,
    so each sensor has independent bias/filter state.
    """

    def __init__(self):
        # Maps prim_path -> Sim2RealCore instance
        self._engines = {}

    def register(self, prim_path: str, config: dict, seed: int = 123):
        """Create a C++ engine instance for a given IMU prim."""
        if not _NATIVE_AVAILABLE:
            return
        engine = sim2real_native.Sim2RealCore(seed)
        if hasattr(engine, "update_configuration"):
            engine.update_configuration(config)
        self._engines[prim_path] = engine
        print(f"[Sim2Real IMU] Registered C++ engine for {prim_path} | config={config}")

    def unregister(self, prim_path: str):
        """Remove the engine for a prim (e.g. if it's deleted from stage)."""
        self._engines.pop(prim_path, None)

    def step(self, prim_path: str, sim_time: float, truth: dict | None):
        """
        Run one noise step for this prim.
        truth: dict with keys 'lin_acc' and 'ang_vel' (numpy arrays, shape [3])
               If None (truth not yet available), returns None.
        Returns dict with 'lin_acc' and 'ang_vel', or None.
        """
        if not _NATIVE_AVAILABLE:
            return truth

        engine = self._engines.get(prim_path)
        if engine is None:
            return truth

        if truth is None:
            return None

        import numpy as np
        lin_acc = truth.get("lin_acc", [0.0, 0.0, 0.0])
        ang_vel = truth.get("ang_vel", [0.0, 0.0, 0.0])

        result = engine.process(
            np.array(lin_acc, dtype=float),
            np.array(ang_vel, dtype=float),
            sim_time
        )
        return result

    def is_registered(self, prim_path: str) -> bool:
        return prim_path in self._engines
