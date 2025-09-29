import numpy as np

def normalize(q):
    q = np.asarray(q, dtype=np.float64)
    return q / np.linalg.norm(q)

def quat_dot(q1, q2):
    return float(np.dot(normalize(q1), normalize(q2)))

def quat_angle_deg_between(q1, q2):
    # angle between rotations represented by q1 and q2
    d = abs(quat_dot(q1, q2))            # abs because q and -q represent same rotation
    d = np.clip(d, -1.0, 1.0)
    return 2.0 * np.degrees(np.arccos(d))  # in degrees

# your quaternions: replace with exact values you printed
#q_sim = np.array([-3.4235857e-04,  1.8753110e-05, -9.9999994e-01,  7.1315293e-07]) # corrected in x rot
#q_sim = np.array([0.70513, 0.08271163, -0.70228183, 0.05244716]) # new position q_flip_x
#q_sim = np.array([0.64402395, 0.08724395, -0.75863653, 0.04574079]) # new position q_flip_x test 2
#q_sim = np.array([-0.6458307, -0.13177024, 0.7519254, -0.01214766]) # new position q_flip_x correction
#q_sim = np.array([9.9999994e-01, 7.1315293e-07, -3.4235857e-04, -1.8753110e-05]) # init sim flip x and z
#q_sim = np.array([-0.22480203, -0.1255056, 0.9650348, 0.04919611]) # -135 pos
#q_sim = np.array([0.5173309, 0.39389768, 0.56867856, -0.5038036]) # -180 pos test 2
# q_sim = np.array([0.01050854, -0.12682873, 0.6796725, -0.7223914])

#q_sim = np.array([-6.8471709e-04, -4.8696286e-10, 9.9999976e-01, -1.2840591e-08])
#q_sim = np.array([0.64388096, -0.08725526, 0.7587555, 0.04575904]) #sim axes -180

#q_sim = np.array([0.30076018, -0.10071149, 0.94832385, 0.00908318]) #sim axes -135
#q_sim = np.array([-7.1315293e-07,  9.9999994e-01,  1.8753110e-05, -3.4235857e-04]) # no corrected
#q_lab = np.array([ 1.979e-03,  9.6628e-05, -9.9753e-01,  7.0216e-02])  # example lab quat
#q_lab = np.array([ -0.0704, -0.0050, -0.9950,  0.0704])  # init quat in lab
#q_lab = np.array([ -0.6555, -0.0463, -0.7519,  0.0532])  # init quat in lab -180
#q_lab = np.array([ -0.3123, -0.0220, -0.9474,  0.0667])  # quat in lab -135 pos
#ee_quat:  tensor([[-0.0704, -0.0050, -0.9950,  0.0704]], device='cuda:0')

# several samples: 

# #isaac sim
#  0.03017137  0.99143517  0.09714099 -0.08191201 # 0 deg
# -0.00886605  0.948527    0.10068633  0.30013362 # 135 deg
# -0.04575205  0.7586048   0.08724737  0.6440601  # 180 deg 
# -0.49433035  0.48017326  0.6067491   0.39613968 # 180 & 0 degrees 

# #isaac lab
# 1.8150e-03,  7.4397e-05, -9.9753e-01,  7.0215e-02 # 0 deg
# -0.3123, -0.0220, -0.9474,  0.0667 # 135 deg
# -0.6553, -0.0463, -0.7521,  0.0532 # 180 deg
# -0.4685,  0.4682, -0.5299, -0.5296 # 180 & 0 degrees


q_sim = np.array([-0.02055616, -0.02986415, 0.99016607, -0.13511783])


q_lab = np.array([ 1.8150e-03,  7.4397e-05, -9.9753e-01,  7.0215e-02])

#isaac sim
# EE pos: [0.71203215 0.16027084 0.55721275]
# EE quat: [ 0.70513     0.08271163 -0.70228183  0.05244716]

#isaac lab
# ee_pos:  tensor([[0.6573, 0.0664, 0.5208]], device='cuda:0')
# ee_quat:  tensor([[-0.6555, -0.0463, -0.7519,  0.0532]], device='cuda:0')

print("dot (abs):", abs(quat_dot(q_sim, q_lab)))
print("angle between (deg):", quat_angle_deg_between(q_sim, q_lab))