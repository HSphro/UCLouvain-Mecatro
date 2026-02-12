import numpy as np
import math

# ==========================================================
# KNOWN BEACON POSITIONS (in mm)
# ==========================================================
B1_REF = np.array([0.0,    0.0])
B2_REF = np.array([0.0, 2000.0])
B3_REF = np.array([3000.0, 1000.0])

# For continuity over time:
Previous_B1 = B1_REF
Previous_B2 = B2_REF
Previous_B3 = B3_REF


# ==========================================================
# 1) Resolve ambiguity of beacon identity from 3 clusters
# ==========================================================
def assign_clusters_to_beacons(cluster_list, prev_B1, prev_B2, prev_B3):
    """
    cluster_list: 3 cluster centroids [ [x,y], [x,y], [x,y] ]
    """

    # convert to numpy arrays
    C = [np.array(c) for c in cluster_list]

    # pairwise distances
    d01 = np.linalg.norm(C[0] - C[1])
    d02 = np.linalg.norm(C[0] - C[2])
    d12 = np.linalg.norm(C[1] - C[2])

    distances = [
        (d01, (0, 1, 2)),
        (d02, (0, 2, 1)),
        (d12, (1, 2, 0)),
    ]

    # the smallest distance corresponds to the "base" of the isosceles triangle
    distances.sort(key=lambda x: x[0])
    _, (i1, i2, i3) = distances[0]

    # two possible symmetric assignments
    SolA_B1 = C[i1]
    SolA_B2 = C[i2]
    SolA_B3 = C[i3]

    SolB_B1 = C[i2]
    SolB_B2 = C[i1]
    SolB_B3 = C[i3]

    # temporal consistency check
    errA = (np.linalg.norm(SolA_B1 - prev_B1) +
            np.linalg.norm(SolA_B2 - prev_B2) +
            np.linalg.norm(SolA_B3 - prev_B3))

    errB = (np.linalg.norm(SolB_B1 - prev_B1) +
            np.linalg.norm(SolB_B2 - prev_B2) +
            np.linalg.norm(SolB_B3 - prev_B3))

    if errA < errB:
        return SolA_B1, SolA_B2, SolA_B3
    else:
        return SolB_B1, SolB_B2, SolB_B3


# ==========================================================
# 2) Compute robot position using trilateration
#    Given 3 known beacons and 3 measured distances
# ==========================================================
def trilaterate(B1, B2, B3, d1, d2, d3):
    """
    B1, B2, B3: known beacon coordinates (numpy arrays)
    d1, d2, d3: measured robot-to-beacon distances
    """

    # Following standard trilateration formulas
    # Step 1: Create the orthonormal basis ex, ey
    ex = (B2 - B1)
    d = np.linalg.norm(ex)
    ex = ex / d

    i = np.dot(ex, B3 - B1)
    temp = B3 - B1 - i * ex
    ey = temp / np.linalg.norm(temp)

    j = np.dot(ey, B3 - B1)

    # Step 2: Solve for coordinates
    x = (d1**2 - d2**2 + d**2) / (2 * d)
    y = (d1**2 - d3**2 + i**2 + j**2 - 2*i*x) / (2 * j)

    # Step 3: Convert to global coordinate system
    robot_pos = B1 + ex * x + ey * y
    return robot_pos


# ==========================================================
# 3) Compute robot orientation
#    Using the angle between (B2 - B1) in world frame and
#    (assigned B2 - assigned B1) in robot frame
# ==========================================================
def compute_robot_orientation(B1, B2):
    """
    Orientation is the rotation needed to align the real beacons
    with the reference geometry.
    """

    # Reference direction (global, fixed)
    ref_vec = B2_REF - B1_REF
    angle_ref = math.atan2(ref_vec[1], ref_vec[0])

    # Measured direction (robot sees B1,B2 here)
    meas_vec = B2 - B1
    angle_meas = math.atan2(meas_vec[1], meas_vec[0])

    # Robot heading is difference
    theta = angle_meas - angle_ref

    # normalize to [-pi, pi]
    while theta > math.pi:  theta -= 2 * math.pi
    while theta < -math.pi: theta += 2 * math.pi

    return theta



# ==========================================================
# EXAMPLE USAGE
# ==========================================================
if __name__ == "__main__":

    # Example cluster detections (in robot frame)
    cluster_list = [
        [100.0, -20.0],
        [80.0, 1980.0],
        [3980.0, 980.0]
    ]

    # Step 1 — Assign clusters to B1,B2,B3
    B1, B2, B3 = assign_clusters_to_beacons(
        cluster_list, Previous_B1, Previous_B2, Previous_B3
    )

    print("Resolved Beacon Assignments:")
    print("B1 =", B1)
    print("B2 =", B2)
    print("B3 =", B3)

    # Step 2 — Compute distances to robot
    d1 = np.linalg.norm(B1)
    d2 = np.linalg.norm(B2)
    d3 = np.linalg.norm(B3)

    # Step 3 — Trilaterate robot position
    robot_pos = trilaterate(B1_REF, B2_REF, B3_REF, d1, d2, d3)

    print("\nRobot Position (global frame):")
    print("x = %.2f mm" % robot_pos[0])
    print("y = %.2f mm" % robot_pos[1])

    # Step 4 — Robot orientation
    theta = compute_robot_orientation(B1, B2)

    print("\nRobot Orientation:")
    print("theta = %.2f degrees" % (theta * 180 / math.pi))