import omni.physx
import omni.timeline
import omni.usd


class Sim2RealRuntime:
    """
    Subscribes to Isaac Sim physics step events and ticks every
    IMU prim tagged with sim2real:enabled=True at its configured ODR.

    Lifecycle:
        runtime = Sim2RealRuntime(backend)
        runtime.start()   # call in extension on_startup
        runtime.stop()    # call in extension on_shutdown

    Data flow per tick:
        Native Isaac IMUSensor (clean physics truth)
                    ↓
          C++ noise engine (NativeBackend)
                    ↓
          Custom sim2real prim (stores noisy result as custom data)
    """

    def __init__(self, backend):
        self._backend = backend
        self._physx_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

        # prim_path -> accumulated dt waiting to fire next sensor tick
        self._accum = {}

        # prim_path -> sim_time of last tick (for passing to C++ engine)
        self._last_sim_time = {}

        # prim_path -> config dict
        self._imu_registry = {}

        # prim_path -> cached Isaac IMUSensor instance (initialized once at registration)
        # These read from the native Isaac IMU prim on the attach link, which provides
        # clean physics truth (gravity-inclusive specific force + angular velocity).
        self._isaac_sensors = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        if self._physx_sub is not None:
            return
        physx = omni.physx.get_physx_interface()
        self._physx_sub = physx.subscribe_physics_step_events(self._on_physics_step)
        print("[Sim2Real Runtime] Physics step subscription active.")

    def stop(self):
        self._physx_sub = None
        self._accum.clear()
        self._last_sim_time.clear()
        self._imu_registry.clear()
        self._isaac_sensors.clear()
        print("[Sim2Real Runtime] Stopped.")

    def register_imu(self, prim_path: str, config: dict, seed: int = 123):
        """
        Explicitly register an IMU prim so the runtime tracks it.
        Called by extension._spawn_imu() right after creating the prim.

        prim_path: path to the custom sim2real prim
                   e.g. /World/franka/panda_hand/ASM330LHH
        config:    noise config dict, must contain 'attachPrimPath' key
                   e.g. {"attachPrimPath": "/World/franka/panda_hand", "odr_hz": 104.0, ...}
        """
        self._imu_registry[prim_path] = config
        self._accum[prim_path] = 0.0
        self._backend.register(prim_path, config, seed=seed)

        # Cache the native Isaac IMUSensor at registration time.
        # This avoids creating a new object every physics step.
        attach_path = config.get("attachPrimPath", "")
        if attach_path:
            self._init_isaac_sensor(prim_path, attach_path)
        else:
            print(f"[Sim2Real Runtime] WARNING: No attachPrimPath in config for {prim_path}. "
                  f"Truth kinematics will be unavailable.")

        print(f"[Sim2Real Runtime] Registered IMU: {prim_path}")

    def unregister_imu(self, prim_path: str):
        self._imu_registry.pop(prim_path, None)
        self._accum.pop(prim_path, None)
        self._last_sim_time.pop(prim_path, None)
        self._isaac_sensors.pop(prim_path, None)
        self._backend.unregister(prim_path)
        print(f"[Sim2Real Runtime] Unregistered IMU: {prim_path}")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _init_isaac_sensor(self, prim_path: str, attach_path: str):
        """
        Create and initialize a native Isaac IMUSensor for the given attach link.
        The native sensor is assumed to live at attach_path/Imu_Sensor.
        This is called once at registration — NOT every physics step.

        attach_path: the robot link the sensor is attached to
                     e.g. /World/franka/panda_hand
        """
        try:
            from omni.isaac.sensor import IMUSensor

            # The native Isaac IMU prim sits on the attach link.
            # This is the clean physics truth source — gravity-inclusive specific force.
            isaac_sensor_path = attach_path + "/Imu_Sensor"

            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(isaac_sensor_path)
            if not prim.IsValid():
                print(f"[Sim2Real Runtime] WARNING: Native Isaac IMU prim not found at "
                      f"{isaac_sensor_path}. Check the prim name in your stage.")
                print(f"[Sim2Real Runtime] Hint: Run this in Script Editor to find it:")
                print(f"    for p in stage.Traverse():")
                print(f"        if 'mu' in str(p.GetPath()).lower(): print(p.GetPath())")
                return

            sensor = IMUSensor(prim_path=isaac_sensor_path)
            sensor.initialize()
            self._isaac_sensors[prim_path] = sensor
            print(f"[Sim2Real Runtime] Cached native Isaac IMUSensor at {isaac_sensor_path}")

        except Exception as e:
            print(f"[Sim2Real Runtime] ERROR: Could not init native Isaac sensor for "
                  f"{prim_path}: {e}")

    def _on_physics_step(self, dt: float):
        if not self._timeline.is_playing():
            return

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return

        # Catch any IMU prims placed before runtime started (e.g. loaded from saved USD)
        self._sync_registry_from_stage(stage)

        sim_time = self._timeline.get_current_time()

        for prim_path, config in list(self._imu_registry.items()):
            odr = float(config.get("odr_hz", 100.0))
            sensor_dt = 1.0 / odr

            accum = self._accum.get(prim_path, 0.0) + dt

            while accum >= sensor_dt:
                accum -= sensor_dt
                self._tick_imu(stage, prim_path, config, sensor_dt, sim_time)

            self._accum[prim_path] = accum

    def _tick_imu(self, stage, prim_path: str, config: dict, sensor_dt: float, sim_time: float):
        """
        Fire one IMU sample for this prim.
        Reads clean truth from the native Isaac IMUSensor, passes through C++ noise engine,
        and stores the result as custom data on the sim2real prim.
        """
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            self.unregister_imu(prim_path)
            return

        # Read clean physics truth from native Isaac IMUSensor
        truth = self._get_truth_kinematics(prim_path)

        # Pass through C++ noise engine
        result = self._backend.step(prim_path, sim_time, truth)

        # Store noisy result as custom data on the sim2real prim for downstream use
        # (verification scripts, logging, socket publishing, etc.)
        if result is not None:
            lin_acc = result.get("lin_acc")
            ang_vel = result.get("ang_vel")
            if lin_acc is not None:
                prim.SetCustomDataByKey("sim2real:last_lin_acc", list(lin_acc))
            if ang_vel is not None:
                prim.SetCustomDataByKey("sim2real:last_ang_vel", list(ang_vel))

        self._last_sim_time[prim_path] = sim_time

    def _get_truth_kinematics(self, prim_path: str) -> dict | None:
        """
        Read one frame from the cached native Isaac IMUSensor for this prim.
        Returns dict with 'lin_acc' and 'ang_vel' (gravity-inclusive, body frame),
        or None if the sensor is unavailable.

        This is called at ODR rate (e.g. 104Hz) — the sensor object was created
        once at registration so there is no per-step construction overhead.
        """
        sensor = self._isaac_sensors.get(prim_path)
        if sensor is None:
            return None

        try:
            import numpy as np
            # read_gravity=True: lin_acc includes gravitational specific force,
            # which is exactly what a real IMU measures and what our C++ engine expects.
            raw = sensor.get_current_frame(read_gravity=True)
            if raw is None:
                return None

            return {
                "lin_acc": np.array(raw["lin_acc"], dtype=float),
                "ang_vel": np.array(raw["ang_vel"], dtype=float)
            }

        except Exception as e:
            print(f"[Sim2Real Runtime] _get_truth_kinematics error for {prim_path}: {e}")
            return None

    def _sync_registry_from_stage(self, stage):
        """
        Scan the stage for any sim2real:enabled prims not yet in the registry.
        Catches prims loaded from saved USD files.
        """
        for prim in stage.Traverse():
            cd = prim.GetCustomData()
            if not cd or not cd.get("sim2real:enabled", False):
                continue
            prim_path = str(prim.GetPath())
            if prim_path not in self._imu_registry:
                # Rebuild config from custom data keys
                config = {
                    k.replace("sim2real:", ""): v
                    for k, v in cd.items()
                    if k.startswith("sim2real:")
                    and k not in ("sim2real:enabled", "sim2real:model",
                                  "sim2real:last_lin_acc", "sim2real:last_ang_vel")
                }
                print(f"[Sim2Real Runtime] Auto-discovered IMU from stage: {prim_path}")
                self.register_imu(prim_path, config)
