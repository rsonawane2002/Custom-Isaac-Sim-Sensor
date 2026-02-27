import omni.ext
import omni.kit.menu.utils as menu_utils
from omni.kit.menu.utils import MenuItemDescription
import omni.usd
import omni.kit.commands
from .config import get_ext_root, load_model_config
from .runtime import Sim2RealRuntime
from .noise import NativeBackend

class Sim2RealIMUSensorExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[Sim2Real IMU] Extension starting up")
        self._ext_id = ext_id
        self._ext_root = get_ext_root(ext_id)
        print(f"[Sim2Real IMU] Extension root: {self._ext_root}")

        # Start the physics-driven runtime
        self._backend = NativeBackend()
        self._runtime = Sim2RealRuntime(self._backend)
        self._runtime.start()

        # Build the Create menu
        self._menu_items = [
            MenuItemDescription(
                name="Sensors",
                sub_menu=[
                    MenuItemDescription(
                        name="STMicroelectronics IMU",
                        sub_menu=[
                            MenuItemDescription(
                                name="ASM330LHH",
                                onclick_fn=lambda: self._spawn_imu(model="ASM330LHH"),
                            ),
                            MenuItemDescription(
                                name="LSM6DSV",
                                onclick_fn=lambda: self._spawn_imu(model="LSM6DSV"),
                            ),
                        ]
                    )
                ]
            )
        ]
        menu_utils.add_menu_items(self._menu_items, "Create")
        menu_utils.rebuild_menus()

    def on_shutdown(self):
        self._runtime.stop()
        menu_utils.remove_menu_items(self._menu_items, "Create")
        menu_utils.rebuild_menus()
        print("[Sim2Real IMU] Extension shut down")

    def _spawn_imu(self, model="ASM330LHH"):
        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("[Sim2Real IMU] ERROR: No stage open.")
            return

        # Load config from JSON
        try:
            config = load_model_config(self._ext_root, model)
        except FileNotFoundError as e:
            print(str(e))
            return

        # Auto-attach: use selected prim as parent, else /World
        selection = omni.usd.get_context().get_selection().get_selected_prim_paths()
        parent_path = selection[0] if selection else "/World"

        # Build a unique prim path under the parent
        base_path = f"{parent_path}/{model}"
        prim_path = base_path
        i = 0
        while stage.GetPrimAtPath(prim_path).IsValid():
            i += 1
            prim_path = f"{base_path}_{i}"

        # Create the IMU Xform prim
        from pxr import UsdGeom, Gf, Vt
        UsdGeom.Xform.Define(stage, prim_path)
        imu_prim = stage.GetPrimAtPath(prim_path)

        # Write Sim2Real metadata
        imu_prim.SetCustomDataByKey("sim2real:enabled", True)
        imu_prim.SetCustomDataByKey("sim2real:model", model)
        imu_prim.SetCustomDataByKey("sim2real:attachPrimPath", parent_path)
        for k, v in config.items():
            imu_prim.SetCustomDataByKey(f"sim2real:{k}", v)

        # Add visible dark navy cube marker as child prim
        visual_path = f"{prim_path}/visual"
        cube = UsdGeom.Cube.Define(stage, visual_path)
        cube.GetSizeAttr().Set(0.05)  # 5cm
        cube.GetDisplayColorAttr().Set(
            Vt.Vec3fArray([Gf.Vec3f(0.03, 0.07, 0.18)])  # Dark navy
        )

        # Shift the cube down 5cm (cosmetic only, logical sensor stays at origin)
        cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.03))

        visual_prim = stage.GetPrimAtPath(visual_path)
        visual_prim.SetCustomDataByKey("physics:collisionEnabled", False)

        # Register with runtime so it starts ticking immediately
        self._runtime.register_imu(prim_path, config, seed=123)

        print(f"[Sim2Real IMU] Spawned {model} at {prim_path}")
        print(f"[Sim2Real IMU] Attached to: {parent_path}")
        print(f"[Sim2Real IMU] Config: {config}")
