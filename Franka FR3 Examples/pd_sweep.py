import mujoco
import mujoco.viewer
import numpy as np
import time
import csv
import os

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

# target
target_qpos = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
target_qvel = np.zeros(7)

# combos to try (kp, kd)
pairs = [
    (20,  2 * np.sqrt(20) * 0.5),
    (20,  2 * np.sqrt(20)),
    (20,  2 * np.sqrt(20) * 2),
    (100, 2 * np.sqrt(100) * 0.5),
    (100, 2 * np.sqrt(100)),
    (100, 2 * np.sqrt(100) * 2),
    (400, 2 * np.sqrt(400) * 0.5),
    (400, 2 * np.sqrt(400)),
    (400, 2 * np.sqrt(400) * 2),
]

out_dir = "pd_sweep_results"
os.makedirs(out_dir, exist_ok=True)

def compute_metrics(qtrace, ctrltrace, t, target=target_qpos):
    # qtrace shape: (steps, 7)
    # ctrltrace shape: (steps, 7)
    # t: timestep (float)
    qtrace = np.array(qtrace)
    ctrltrace = np.array(ctrltrace)
    steps = qtrace.shape[0]
    time_vec = np.arange(steps) * t

    # For simplicity evaluate metric on joint 1 (index 0) and also average across joints
    err = qtrace - target[np.newaxis, :]
    abs_err = np.abs(err)
    overshoot = np.max(qtrace - target[np.newaxis, :], axis=0)  # per joint
    overshoot_max = np.max(overshoot)

    # settling time: time when abs error falls below 2% of final (use first time after 0.1s)
    settle_threshold = 0.02 * np.abs(target) + 1e-3
    settling_time = np.full(7, np.nan)
    for j in range(7):
        idx = np.where(np.abs(err[:, j]) <= settle_threshold[j])[0]
        if idx.size > 0:
            settling_time[j] = time_vec[idx[0]]

    peak_torque = np.max(np.abs(ctrltrace), axis=0)  # per joint

    return {
        "overshoot_max": float(overshoot_max),
        "settling_time_mean": float(np.nanmean(settling_time)),
        "peak_torque_mean": float(np.mean(peak_torque)),
        "settling_time_per_joint": settling_time.tolist(),
        "peak_torque_per_joint": peak_torque.tolist()
    }

# simulation parameters
sim_time = 6.0        # seconds per run
dt = model.opt.timestep
max_steps = int(sim_time / dt)

results = []

for idx, (kp, kd) in enumerate(pairs):
    print(f"Running {idx+1}/{len(pairs)}: Kp={kp:.1f}, Kd={kd:.3f}")
    # reset sim
    mujoco.mj_resetData(model, data)

    qtrace = []
    ctrltrace = []
    t0 = time.time()

    # simple loop without opening viewer (headless). If you want to watch, set show_viewer=True below.
    show_viewer = False
    if show_viewer:
        viewer = mujoco.viewer.launch_passive(model, data)

    for step in range(max_steps):
        # controller
        position_error = target_qpos - data.qpos[:7]
        velocity_error = target_qvel - data.qvel[:7]
        torque = (kp * position_error) + (kd * velocity_error)
        data.ctrl[:7] = torque

        mujoco.mj_step(model, data)

        qtrace.append(np.copy(data.qpos[:7]))
        ctrltrace.append(np.copy(data.ctrl[:7]))

        # optional: small sleep to run real-time, comment out for faster runs
        # time.sleep(dt)

    if show_viewer:
        viewer.close()

    metrics = compute_metrics(qtrace, ctrltrace, dt)
    metrics["kp"] = kp
    metrics["kd"] = kd
    results.append(metrics)

    # save traces to csv for this run
    csv_path = os.path.join(out_dir, f"trace_kp{int(kp)}_kd{int(round(kd))}.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time"] + [f"q{j}" for j in range(7)] + [f"u{j}" for j in range(7)])
        for i, (qrow, urow) in enumerate(zip(qtrace, ctrltrace)):
            writer.writerow([i*dt] + qrow.tolist() + urow.tolist())

# write summary results
summary_path = os.path.join(out_dir, "summary.csv")
with open(summary_path, "w", newline="") as f:
    keys = ["kp","kd","overshoot_max","settling_time_mean","peak_torque_mean"]
    writer = csv.DictWriter(f, fieldnames=keys)
    writer.writeheader()
    for r in results:
        writer.writerow({k: r.get(k, "") for k in keys})

print("Sweep finished. Results saved to:", out_dir)
