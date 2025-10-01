import numpy as np

# replace these with your exact values
# q_sim = np.array([-1.8753110e-05, -3.4235857e-04, -7.1315293e-07, -9.9999994e-01], dtype=np.float64)  # [x,y,z,w]
# q_lab = np.array([ 1.979e-03,  9.6628e-05, -9.9753e-01,  7.0216e-02], dtype=np.float64)                 # [x,y,z,w]

q_sim = np.array([-7.1315293e-07, 9.9999994e-01, 1.8753110e-05, -3.4235857e-04], dtype=np.float64)  # [x,y,z,w]
q_lab = np.array([-0.0704, -0.0050, -0.9950,  0.0704], dtype=np.float64)    

def quat_inv(q):
    x,y,z,w = q
    return np.array([-x, -y, -z, w], dtype=q.dtype)

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    x =  w1*x2 + x1*w2 + y1*z2 - z1*y2
    y =  w1*y2 - x1*z2 + y1*w2 + z1*x2
    z =  w1*z2 + x1*y2 - y1*x2 + z1*w2
    w =  w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x,y,z,w], dtype=q1.dtype)

def normalize(q):
    return q / np.linalg.norm(q)

# compute correction quaternion that maps sim -> lab
q_corr = quat_mul(q_lab, quat_inv(q_sim))
q_corr = normalize(q_corr)

# apply correction (left-multiply)
q_sim_aligned = quat_mul(q_corr, q_sim)
q_sim_aligned = normalize(q_sim_aligned)

# diagnostics
dot = abs(np.dot(normalize(q_sim_aligned), normalize(q_lab)))
angle_deg = 2.0 * np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))

print("q_corr (x,y,z,w):", q_corr)
print("q_sim_aligned (x,y,z,w):", q_sim_aligned)
print("q_lab (x,y,z,w):", q_lab)
print("abs(dot) between aligned and lab:", dot)
print("angle between aligned and lab (deg):", angle_deg)