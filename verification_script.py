import omni.timeline
import omni.usd
import csv
import os
import sys
import math
import numpy as np
import random
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction
from omni.kit.app import get_app
from omni.isaac.sensor import IMUSensor

# --- Load NativeBackend from our extension ---
from sim2real.imu.sensor.noise.native_backend import NativeBackend

# --- Wrapper Class (uses NativeBackend instead of raw C++ import) ---
class Sim2RealIMUSensor:
    def __init__(self, prim_path, name="imu", seed=123, config=None):
        self._sensor = IMUSensor(prim_path=prim_path, name=name)
        self._timeline = omni.timeline.get_timeline_interface()
        self._prim_path = prim_path

        self._backend = NativeBackend()
        if config:
            self._backend.register(prim_path, config, seed=seed)
            print(f"C++ Config Applied via NativeBackend: {config}")
        else:
            self._backend.register(prim_path, {}, seed=seed)

    def initialize(self, physics_sim_view=None):
        self._sensor.initialize(physics_sim_view)

    def get_current_frame(self, read_gravity=True):
        raw = self._sensor.get_current_frame(read_gravity=read_gravity)
        current_time = self._timeline.get_current_time()

        result = self._backend.step(
            self._prim_path,
            sim_time=current_time,
            truth={
                "lin_acc": raw["lin_acc"],
                "ang_vel": raw["ang_vel"]
            }
        )

        if result is not None:
            return {"lin_acc": result["lin_acc"], "ang_vel": result["ang_vel"]}
        else:
            return {"lin_acc": raw["lin_acc"], "ang_vel": raw["ang_vel"]}

    def __getattr__(self, name):
        return getattr(self._sensor, name)


# --- Global Config ---
ROBOT_PATH = "/World/franka"

# Both sensors read from the same real Isaac physics IMU prim.
# The NativeBackend adds noise on top of the truth data from this prim.
IMU_PATH_CLEAN = "/World/franka/panda_hand/Imu_Sensor"
IMU_PATH_GUI   = "/World/franka/panda_hand/Imu_Sensor"

OUTPUT_DIR = os.path.expanduser("~/Documents/trajectories_verification")

# Simulation Params
STARTING_TRAJ_INDEX = 0
NUM_TRAJECTORIES = 5
TRAJ_DURATION = 4.0
SETTLE_TIME = 1.0
RESET_TIME = 2.0
STARTUP_FRAMES = 60

HOME = np.array([0.000167, -0.786, 4.01e-5, -2.35502, 9.29e-5, 1.571, 0.786, 0.04, 0.04])

# Noise config — matches ASM330LHH.json
NOISE_CONFIG = {
    "accel_fs_g": 8.0,
    "gyro_fs_dps": 2000.0,
    "odr_hz": 104.0,
    "vibration": True
}

# --- State ---
robot      = None
imu_clean  = None   # Standard Isaac IMUSensor — raw physics, no noise
imu_gui    = None   # Same prim, wrapped with NativeBackend noise engine
timeline   = omni.timeline.get_timeline_interface()

frame         = 0
phase         = 'startup'
traj_idx      = STARTING_TRAJ_INDEX
time_in_phase = 0.0
targets       = []

# Log Handles
file_clean  = None;  writer_clean = None
file_gui    = None;  writer_gui   = None

CSV_HEADER = ['time', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7',
              'gripper1', 'gripper2',
              'imu_ax', 'imu_ay', 'imu_az',
              'imu_gx', 'imu_gy', 'imu_gz']

def smooth_interp(t):
    return 10*t**3 - 15*t**4 + 6*t**5

def gen_target():
    t = HOME.copy()
    MAX_OFFSET = math.pi / 5
    for i in range(7):
        t[i] = HOME[i] + random.uniform(-MAX_OFFSET, MAX_OFFSET)
    return t

def start_log():
    global file_clean, writer_clean, file_gui, writer_gui

    traj_folder = os.path.join(OUTPUT_DIR, "traj_" + str(traj_idx))
    os.makedirs(traj_folder, exist_ok=True)

    # 1. Clean log
    path_clean = os.path.join(traj_folder, "clean_imu_" + str(traj_idx) + ".csv")
    file_clean = open(path_clean, 'w', newline='')
    writer_clean = csv.writer(file_clean)
    writer_clean.writerow(CSV_HEADER)

    # 2. GUI sensor log
    path_gui = os.path.join(traj_folder, "gui_imu_" + str(traj_idx) + ".csv")
    file_gui = open(path_gui, 'w', newline='')
    writer_gui = csv.writer(file_gui)
    writer_gui.writerow(CSV_HEADER)

def log(pos):
    global writer_clean, writer_gui
    if not (imu_clean and imu_gui and writer_clean):
        return

    current_time = timeline.get_current_time()

    # A. Clean data — raw Isaac physics
    raw = imu_clean.get_current_frame(read_gravity=True)
    acc_c = raw['lin_acc']
    vel_c = raw['ang_vel']

    # B. GUI sensor data — same truth, C++ noise applied on top
    noisy = imu_gui.get_current_frame(read_gravity=True)
    acc_n = noisy['lin_acc']
    vel_n = noisy['ang_vel']

    # Write clean
    writer_clean.writerow([current_time] + list(pos[:9]) +
                          [acc_c[2], -1 * acc_c[1], acc_c[0],
                           vel_c[2], -1 * vel_c[1], vel_c[0]])

    # Write noisy
    writer_gui.writerow([current_time] + list(pos[:9]) +
                        [acc_n[2], -1 * acc_n[1], acc_n[0],
                         vel_n[2], -1 * vel_n[1], vel_n[0]])

    file_clean.flush()
    file_gui.flush()

def close_log():
    global file_clean, writer_clean, file_gui, writer_gui
    if file_clean: file_clean.close()
    if file_gui:   file_gui.close()
    file_clean = None;  writer_clean = None
    file_gui   = None;  writer_gui   = None

# --- Main Loop ---
def update(e):
    global robot, imu_clean, imu_gui, frame, phase, traj_idx, time_in_phase, targets

    if not timeline.is_playing():
        return

    frame += 1
    dt = e.payload["dt"]

    if robot is None:
        robot = Franka(prim_path=ROBOT_PATH, name="franka")
        robot.initialize()
        robot.set_joints_default_state(positions=HOME, velocities=np.zeros(9))
        robot.post_reset()

        # 1. Clean sensor — raw Isaac physics, no noise
        imu_clean = IMUSensor(prim_path=IMU_PATH_CLEAN, name="imu_clean")
        imu_clean.initialize()

        # 2. GUI sensor — same prim, NativeBackend adds C++ noise on top
        imu_gui = Sim2RealIMUSensor(
            prim_path=IMU_PATH_GUI,
            name="imu_gui",
            seed=123,
            config=NOISE_CONFIG
        )
        imu_gui.initialize()

        targets = [gen_target() for _ in range(NUM_TRAJECTORIES)]
        print(f"VERIFICATION MODE: Logging to {OUTPUT_DIR}")
        print(f"  Clean sensor:  {IMU_PATH_CLEAN}")
        print(f"  GUI sensor:    {IMU_PATH_GUI} (+ C++ noise)")
        return

    if phase == 'startup':
        if frame > STARTUP_FRAMES:
            phase = 'moving'
            time_in_phase = 0.0
            start_log()
            print("Trajectory " + str(traj_idx))
        return

    time_in_phase += dt
    pos = robot.get_joint_positions()

    if phase == 'moving':
        progress = min(time_in_phase / TRAJ_DURATION, 1.0)
        t = smooth_interp(progress)
        target_pos = HOME + t * (targets[traj_idx - STARTING_TRAJ_INDEX] - HOME)
        robot.apply_action(ArticulationAction(joint_positions=target_pos))
        log(pos)
        if progress >= 1.0:
            phase = 'settling'
            time_in_phase = 0.0

    elif phase == 'settling':
        log(pos)
        if time_in_phase >= SETTLE_TIME:
            close_log()
            traj_idx += 1
            if traj_idx >= STARTING_TRAJ_INDEX + NUM_TRAJECTORIES:
                print("Done: Verification Data Collected")
                timeline.stop()
                return
            targets.append(gen_target())
            phase = 'resetting'
            time_in_phase = 0.0

    elif phase == 'resetting':
        progress = min(time_in_phase / RESET_TIME, 1.0)
        t = smooth_interp(progress)
        reset_pos = pos + t * (HOME - pos)
        robot.apply_action(ArticulationAction(joint_positions=reset_pos))
        if progress >= 1.0:
            robot.set_joint_positions(HOME)
            robot.set_joint_velocities(np.zeros(9))
            phase = 'moving'
            time_in_phase = 0.0
            start_log()
            print("Trajectory " + str(traj_idx))

# --- Start ---
stream = get_app().get_update_event_stream()
sub = stream.create_subscription_to_pop(update)
print("Collecting " + str(NUM_TRAJECTORIES) + " trajectories -> " + OUTPUT_DIR)
print("Press PLAY to begin.")
