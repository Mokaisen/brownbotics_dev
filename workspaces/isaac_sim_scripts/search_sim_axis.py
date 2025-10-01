import numpy as np
from itertools import permutations, product
from scipy.spatial.transform import Rotation as R

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

def best_axis_mapping_try(sim_axes, lab_axes=np.eye(3), sim_quats=None, lab_quats=None):
    """
    Try permutations and sign flips on sim_axes columns to find the mapping that best
    matches lab_axes. If sim_quats/lab_quats provided, uses them to score mapping;
    otherwise scores by closeness of R_corr to a rotation (small residual).
    """
    best = None
    best_score = 1e9

    idxs = [0,1,2]
    for perm in permutations(idxs):
        for signs in product([1,-1], repeat=3):
            # build candidate sim_axes_candidate: take sim_axes columns permuted and signed
            cols = []
            for i, s in zip(perm, signs):
                cols.append(sim_axes[:, i] * s)
            sim_candidate = np.column_stack(cols)  # 3x3

            # compute correction
            R_corr = lab_axes @ np.linalg.inv(sim_candidate)
            # ensure it's a rotation (orthonormal) by projecting via SVD (optional)
            U, _, Vt = np.linalg.svd(R_corr)
            R_corr_proj = U @ Vt
            det = np.linalg.det(R_corr_proj)
            if det < 0:
                # skip improper reflection unless absolutely needed
                continue

            # Score: if quaternions provided, use mean angle after correction
            if sim_quats is not None and lab_quats is not None:
                total_angle = 0.0
                for qs, ql in zip(sim_quats, lab_quats):
                    q_corr = R.from_matrix(R_corr_proj).as_quat()
                    # apply correction as left-mul
                    # q_al = q_corr * qs (use same quat_mul function as above)
                    q_al = quat_mul(q_corr, qs)
                    q_al = q_al / np.linalg.norm(q_al)
                    # angle to ql
                    dot = abs(np.dot(q_al, ql) / (np.linalg.norm(q_al)*np.linalg.norm(ql)))
                    dot = np.clip(dot, -1.0, 1.0)
                    angle = 2.0 * np.degrees(np.arccos(dot))
                    total_angle += angle
                score = total_angle / len(sim_quats)
            else:
                # fallback score: deviation of R_corr from being orthonormal (small is good)
                diff = R_corr - R_corr_proj
                score = np.linalg.norm(diff)

            if score < best_score:
                best_score = score
                best = {
                    'perm': perm,
                    'signs': signs,
                    'sim_candidate': sim_candidate,
                    'R_corr': R_corr_proj,
                    'q_corr': R.from_matrix(R_corr_proj).as_quat(),
                    'score': score
                }
    return best

# Example usage:
# Provide sim_quats and lab_quats arrays (N x 4), if you have them, otherwise leave None.
# best = best_axis_mapping_try(sim_axes_orig, np.eye(3), sim_quats=[q_sim_sample, ...], lab_quats=[q_lab_sample, ...])
# print(best['perm'], best['signs'], best['q_corr'], best['score'])

# ---- Your measured Sim axes (original) ----
sim_axes_orig = np.array([
    [-1.4391463e-06,  1.0000000e+00,  3.7505732e-05],  # X
    [ 9.9999976e-01,  1.4134653e-06,  6.8471715e-04],  # Y
    [ 6.8471709e-04,  3.7506707e-05, -9.9999976e-01],  # Z
]).T  # columns are the X,Y,Z unit vectors

# Paste your sim/lab quats here (use a list if you have multiple samples)
sim_quats = [np.array([-7.1315293e-07, 9.9999994e-01, 1.8753110e-05, -3.4235857e-04])]
lab_quats = [np.array([-0.0704, -0.0050, -0.9950, 0.0704])]

best = best_axis_mapping_try(sim_axes_orig, np.eye(3), sim_quats=sim_quats, lab_quats=lab_quats)

print("Best permutation:", best['perm'])
print("Best signs:", best['signs'])
print("Best correction quaternion (x,y,z,w):", best['q_corr'])
print("Score (mean angle deg):", best['score'])