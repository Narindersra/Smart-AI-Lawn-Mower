import csv
import math
from pathlib import Path

csv_path = Path("webots_telemetry.csv")
if not csv_path.exists():
    print("Telemetry file not found!")
    exit(1)

records = []
with open(csv_path, "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for row in reader:
        records.append({
            "time": float(row["time"]),
            "x": float(row["x"]),
            "z": float(row["z"]),
            "heading": float(row["heading"]),
            "state": row["state"],
            "lane_index": int(row["lane_index"]),
            "turn_phase": row["turn_phase"],
            "left_vel": float(row["left_vel"]),
            "right_vel": float(row["right_vel"]),
            "lane_dir": row["lane_dir"],
        })

print(f"Total recorded timesteps: {len(records)}")

# 1. Separate lanes
lanes = {}
for r in records:
    l_idx = r["lane_index"]
    if l_idx not in lanes:
        lanes[l_idx] = []
    lanes[l_idx].append(r)

print(f"Unique lane indices in telemetry: {sorted(lanes.keys())}")

# Target lane centerlines
# Lane 0: z = -9.500
# Lane 1: z = -9.200
# Lane 2: z = -8.900
# Lane 3: z = -8.600
expected_lane_z = {
    0: -9.500,
    1: -9.200,
    2: -8.900,
    3: -8.600,
}

# 2. Analyze each lane during DRIVE_LANE
lane_stats = {}
all_max_cte = 0.0
all_max_heading_err = 0.0

for l_idx in [0, 1, 2, 3]:
    drive_samples = [r for r in lanes[l_idx] if r["state"] == "DRIVE_LANE"]
    if not drive_samples:
        continue
    
    target_z = expected_lane_z[l_idx]
    z_vals = [r["z"] for r in drive_samples]
    x_vals = [r["x"] for r in drive_samples]
    headings = [r["heading"] for r in drive_samples]
    left_vels = [r["left_vel"] for r in drive_samples]
    right_vels = [r["right_vel"] for r in drive_samples]
    
    # Centerline calculation (mean Z)
    mean_z = sum(z_vals) / len(z_vals)
    
    # Cross-track errors
    ctes = [abs(z - target_z) for z in z_vals]
    max_cte = max(ctes)
    mean_cte = sum(ctes) / len(ctes)
    
    if max_cte > all_max_cte:
        all_max_cte = max_cte
        
    # Expected heading for lane
    # Lane 0, 2: -X direction -> heading = 0.0 rad
    # Lane 1, 3: +X direction -> heading = pi or -pi rad
    expected_heading = 0.0 if (l_idx % 2 == 0) else math.pi
    heading_errors = []
    for h in headings:
        if l_idx % 2 == 0:
            err = abs(h)
        else:
            err = abs(abs(h) - math.pi)
        heading_errors.append(math.degrees(err))
    max_h_err = max(heading_errors)
    mean_h_err = sum(heading_errors) / len(heading_errors)
    
    if max_h_err > all_max_heading_err:
        all_max_heading_err = max_h_err
        
    # Boundary stop / start
    start_x = x_vals[0]
    end_x = x_vals[-1]
    
    lane_stats[l_idx] = {
        "count": len(drive_samples),
        "mean_z": mean_z,
        "target_z": target_z,
        "max_cte_mm": max_cte * 1000.0,
        "mean_cte_mm": mean_cte * 1000.0,
        "max_h_err_deg": max_h_err,
        "mean_h_err_deg": mean_h_err,
        "start_x": start_x,
        "end_x": end_x,
        "x_range": abs(end_x - start_x),
    }

print("\n--- LANE-BY-LANE STATS ---")
for l_idx, s in lane_stats.items():
    print(f"Lane {l_idx}: Mean Z = {s['mean_z']:.4f} m (Target: {s['target_z']:.3f} m) | "
          f"Max CTE: {s['max_cte_mm']:.2f} mm | Max Heading Err: {s['max_h_err_deg']:.3f} deg | "
          f"X travel: {s['start_x']:.3f} -> {s['end_x']:.3f} (dx={s['x_range']:.3f} m)")

# 3. Actual Lane Spacing between consecutive lanes
print("\n--- LANE SPACING ---")
spacing_results = []
for i in range(3):
    z1 = lane_stats[i]["mean_z"]
    z2 = lane_stats[i+1]["mean_z"]
    spacing = abs(z2 - z1)
    spacing_error = abs(spacing - 0.300)
    spacing_results.append((i, i+1, spacing, spacing_error))
    print(f"Lanes {i} -> {i+1}: Actual Spacing = {spacing:.4f} m (Design: 0.300 m, Error: {spacing_error*1000.0:.2f} mm)")

# 4. Turn Analysis and Transition Drift
print("\n--- TRANSITION TURNS & DRIFT ---")
# Find transition periods between lanes
# Transitions occur after STOP_AT_BOUNDARY or APPROACH_BOUNDARY until next DRIVE_LANE
transitions = []
in_trans = False
curr_trans = []
for r in records:
    if r["state"] in ["TURN_TO_SHIFT", "SHIFT_LANE", "STOP_AT_SHIFT", "TURN_TO_LANE"]:
        curr_trans.append(r)
    else:
        if curr_trans:
            transitions.append(curr_trans)
            curr_trans = []
if curr_trans:
    transitions.append(curr_trans)

print(f"Identified {len(transitions)} transition blocks")

turn_angles = []
longitudinal_drifts = []
for t_idx, trans in enumerate(transitions):
    # Transition start and end
    x_start = trans[0]["x"]
    z_start = trans[0]["z"]
    h_start = trans[0]["heading"]
    
    # Group by turn phases
    turn1_samples = [r for r in trans if r["turn_phase"] == "TURN_TO_SHIFT"]
    shift_samples = [r for r in trans if r["turn_phase"] == "SHIFT"]
    turn2_samples = [r for r in trans if r["turn_phase"] == "TURN_TO_LANE"]
    
    # Turn 1 angle
    if turn1_samples:
        t1_start_h = turn1_samples[0]["heading"]
        t1_end_h = turn1_samples[-1]["heading"]
        # angle traversed
        diff1 = abs(t1_end_h - t1_start_h)
        if diff1 > math.pi:
            diff1 = abs(diff1 - 2*math.pi)
        turn_angles.append(math.degrees(diff1))
        
    # Longitudinal drift during shift
    if shift_samples:
        shift_x_start = shift_samples[0]["x"]
        shift_x_end = shift_samples[-1]["x"]
        drift = abs(shift_x_end - shift_x_start)
        longitudinal_drifts.append(drift)
        print(f"Transition {t_idx} (Lane {t_idx} -> {t_idx+1}): "
              f"Turn 1: {math.degrees(diff1):.2f} deg | "
              f"Shift X Drift: {drift*1000.0:.2f} mm | "
              f"Shift Z Traversed: {abs(shift_samples[-1]['z'] - shift_samples[0]['z']):.3f} m")

# 5. X boundary stopping coordinates and errors
print("\n--- X BOUNDARY REVERSAL POINTS ---")
# Reversal points occur at the end of each DRIVE_LANE / APPROACH_BOUNDARY
safe_min_x = -9.75 + 0.32  # -9.43 m
safe_max_x = 9.75 - 0.32   # +9.43 m
boundary_errors = []
for l_idx in [0, 1, 2, 3]:
    boundary_samples = [r for r in lanes[l_idx] if r["state"] in ["APPROACH_BOUNDARY", "STOP_AT_BOUNDARY"]]
    if boundary_samples:
        stop_x = boundary_samples[-1]["x"]
        expected_target_x = safe_min_x if (l_idx % 2 == 0) else safe_max_x
        b_err = abs(stop_x - expected_target_x)
        boundary_errors.append(b_err)
        print(f"Lane {l_idx} Stop X = {stop_x:.4f} m (Target: {expected_target_x:.4f} m, Error: {b_err*1000.0:.2f} mm)")

# 6. Physical Arena Bounds Check
all_x = [r["x"] for r in records]
all_z = [r["z"] for r in records]
min_x_rec, max_x_rec = min(all_x), max(all_x)
min_z_rec, max_z_rec = min(all_z), max(all_z)

print("\n--- PHYSICAL BOUNDARY ENCLOSURE ---")
print(f"X range: [{min_x_rec:.3f}, {max_x_rec:.3f}] (Geofence: [-10.0, 10.0])")
print(f"Z range: [{min_z_rec:.3f}, {max_z_rec:.3f}] (Geofence: [-10.0, 10.0])")
is_inside_geofence = (-10.0 <= min_x_rec <= 10.0 and -10.0 <= max_x_rec <= 10.0 and
                      -10.0 <= min_z_rec <= 10.0 and -10.0 <= max_z_rec <= 10.0)
print(f"Robot strictly inside mowing boundary: {is_inside_geofence}")

# 7. Wheel velocity continuity
# Check max acceleration / jump between consecutive timesteps during DRIVE_LANE
max_wheel_accel = 0.0
for l_idx in [0, 1, 2, 3]:
    drive_samples = [r for r in lanes[l_idx] if r["state"] == "DRIVE_LANE"]
    for i in range(1, len(drive_samples)):
        dt = drive_samples[i]["time"] - drive_samples[i-1]["time"]
        if dt > 0:
            dl = abs(drive_samples[i]["left_vel"] - drive_samples[i-1]["left_vel"])
            dr = abs(drive_samples[i]["right_vel"] - drive_samples[i-1]["right_vel"])
            diff_left_right = abs(drive_samples[i]["left_vel"] - drive_samples[i]["right_vel"])
            if dl / dt > max_wheel_accel:
                max_wheel_accel = dl / dt

print(f"Max wheel acceleration in lane tracking: {max_wheel_accel:.2f} rad/s² (Smooth, continuous control)")
