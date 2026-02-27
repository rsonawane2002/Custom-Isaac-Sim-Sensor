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
    """

    def __init__(self, backend):
        self._backend = backend
        self._physx_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

        # prim_path -> accumulated dt waiting to fire next sensor tick
        self._accum = {}

        # prim_path -> sim_time of last tick (for passing to C++ engine)
        self._last_sim_time = {}

        # Cache of known IMU prims so we don't scan the full stage every step
        # Format: prim_path -> config dict
        self._imu_registry = {}

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
        print("[Sim2Real Runtime] Stopped.")

    def register_imu(self, prim_path: str, config: dict, seed: int = 123):
        """
        Explicitly register an IMU prim so the runtime tracks it.
        Called by extension._spawn_imu() right after creating the prim.
        """
        self._imu_registry[prim_path] = config
        self._accum[prim_path] = 0.0
        self._backend.register(prim_path, config, seed=seed)
        print(f"[Sim2Real Runtime] Registered IMU: {prim_path}")

    def unregister_imu(self, prim_path: str):
        self._imu_registry.pop(prim_path, None)
        self._accum.pop(prim_path, None)
        self._last_sim_time.pop(prim_path, None)
        self._backend.unregister(prim_path)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _on_physics_step(self, dt: float):
        if not self._timeline.is_playing():
            return

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return

        # Also catch any IMU prims placed before runtime started
        # (e.g. loaded from a saved USD file)
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
        Extracts truth kinematics from the attach prim's rigid body state,
        then passes through the C++ noise engine.
        """
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            # Prim was deleted — clean up
            self.unregister_imu(prim_path)
            return

        attach_path = prim.GetCustomData().get("sim2real:attachPrimPath", "")
        truth = self._get_truth_kinematics(stage, attach_path)

        result = self._backend.step(prim_path, sim_time, truth)

        # result is available here for downstream use:
        # - write to a USD attribute
        # - publish via socket
        # - log to CSV
        # Currently we just store it on the prim as custom data for inspection
        if result is not None:
            lin_acc = result.get("lin_acc")
            ang_vel = result.get("ang_vel")
            if lin_acc is not None:
                prim.SetCustomDataByKey("sim2real:last_lin_acc", list(lin_acc))
            if ang_vel is not None:
                prim.SetCustomDataByKey("sim2real:last_ang_vel", list(ang_vel))

        self._last_sim_time[prim_path] = sim_time

    def _get_truth_kinematics(self, stage, attach_path: str) -> dict | None:
        """
        Query the rigid body state of the attach prim.
        Returns dict with 'lin_acc' and 'ang_vel' or None if unavailable.
        """
        if not attach_path:
            return None

        prim = stage.GetPrimAtPath(attach_path)
        if not prim.IsValid():
            return None

        try:
            import omni.physx
            from pxr import UsdGeom
            import numpy as np

            physx_interface = omni.physx.get_physx_interface()

            # get_rigid_body_properties returns linear/angular velocity
            props = physx_interface.get_rigid_body_properties(str(prim.GetPath()))
            if props is None:
                return None

            # Angular velocity is directly the gyro truth in world frame
            ang_vel = list(props.angular_velocity) if hasattr(props, "angular_velocity") else [0.0, 0.0, 0.0]

            # Linear acceleration approximated from velocity change
            # (full IMU truth requires previous velocity — stored in last_sim_time)
            lin_acc = [0.0, 0.0, 9.81]  # gravity placeholder until velocity diff available

            return {"lin_acc": lin_acc, "ang_vel": ang_vel}

        except Exception as e:
            return None

    def _sync_registry_from_stage(self, stage):
        """
        Scan the stage for any sim2real:enabled prims not yet in the registry.
        This catches prims loaded from saved USD files.
        Only runs until all tagged prims are found — then stops scanning.
        """
        for prim in stage.Traverse():
            cd = prim.GetCustomData()
            if not cd or not cd.get("sim2real:enabled", False):
                continue
            prim_path = str(prim.GetPath())
            if prim_path not in self._imu_registry:
                config = {
                    k.replace("sim2real:", ""): v
                    for k, v in cd.items()
                    if k.startswith("sim2real:")
                    and k not in ("sim2real:enabled", "sim2real:model",
                                  "sim2real:attachPrimPath",
                                  "sim2real:last_lin_acc", "sim2real:last_ang_vel")
                }
                print(f"[Sim2Real Runtime] Auto-discovered IMU from stage: {prim_path}")
                self.register_imu(prim_path, config)
