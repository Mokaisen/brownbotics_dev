import numpy as np
from scipy.spatial.transform import Rotation as R

# ---- Your measured Sim axes (original) ----
sim_axes_orig = np.array([
    [-1.4391463e-06,  1.0000000e+00,  3.7505732e-05],  # X
    [ 9.9999976e-01,  1.4134653e-06,  6.8471715e-04],  # Y
    [ 6.8471709e-04,  3.7506707e-05, -9.9999976e-01],  # Z
]).T  # columns are the X,Y,Z unit vectors

# ---- Your suggested swapped axes (try this) ----
sim_axes_swapped = np.array([
    [-1.4391463e-06,  3.7505732e-05,  1.0000000e+00],  # new X (you proposed)
    [ 9.9999976e-01,  1.4134653e-06,  6.8471715e-04],  # Y (unchanged)
    [ 6.8471709e-04,  3.7506707e-05, -9.9999976e-01],  # Z (unchanged)
]).T

# canonical Lab axes (columns)
lab_axes = np.eye(3)

def compute_q_corr_from_sim_axes(sim_axes):
    # Correction that maps Sim basis -> Lab basis:
    R_corr = lab_axes @ np.linalg.inv(sim_axes)
    q_corr = R.from_matrix(R_corr).as_quat()  # returns [x,y,z,w]
    return R_corr, q_corr

R1, q1 = compute_q_corr_from_sim_axes(sim_axes_orig)
R2, q2 = compute_q_corr_from_sim_axes(sim_axes_swapped)

print("Correction from ORIGINAL sim_axes -> lab (quat x,y,z,w):", q1)
print("Correction from SWAPPED sim_axes  -> lab (quat x,y,z,w):", q2)

# Optional: test on a sample sim/lab quaternion pair you have
# Replace with your actual pairs if you want
q_sim_sample = np.array([-7.1315293e-07, 9.9999994e-01, 1.8753110e-05, -3.4235857e-04])  # [x,y,z,w]
q_lab_sample = np.array([ -0.0704, -0.0050, -0.9950,  0.0704])  # [x,y,z,w]

# apply correction (left-multiply)
def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=np.float64)

for name, qcorr in [("orig", q1), ("swap", q2)]:
    q_al = quat_mul(qcorr, q_sim_sample)
    dot = abs(np.dot(q_al/np.linalg.norm(q_al), q_lab_sample/np.linalg.norm(q_lab_sample)))
    angle_deg = 2.0 * np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))
    print(f"Mapping {name}: angle to lab (deg) = {angle_deg:.6f}, dot={dot:.6f}")