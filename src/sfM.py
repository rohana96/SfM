import numpy as np
from collections import defaultdict
from .triangulate import triangulate
from utils import plot3d_points, plot3d_animation
import cv2


def findMatches(points1, points2):
    p1_dict = {tuple(p1): i for (i, p1) in enumerate(points1)}
    matches = {j: p1_dict[tuple(p2)] for j, p2 in enumerate(points2) if tuple(p2) in p1_dict}
    extra_ids = [j for j, p2 in enumerate(points2) if tuple(p2) not in p1_dict]
    return matches, np.array(extra_ids)


def getCamera(correspondences, K, cam_id):
    points_2d, points_3d = zip(*correspondences.items())
    points_2d = np.array(points_2d).astype(np.float32)
    points_3d = np.array(points_3d).astype(np.float32)
    C = estimateP(points_2d, points_3d)

    _, rod, T = cv2.solvePnP(points_3d, points_2d, K, np.array([0, 0, 0, 0]))
    R, _ = cv2.Rodrigues(rod)
    print(f"Rotation and Translation matrices for Camera {cam_id} \nR:{R} \nT:{T}")
    return C


def estimateP(pts2d, pts3d):
    A = []
    for i in range(len(pts3d)):
        x, y = pts2d[i]
        X, Y, Z = pts3d[i]
        curr_entry = [X, Y, Z, 1, 0, 0, 0, 0, -1 * x * X, -1 * x * Y, -1 * x * Z, -1 * x]
        A.append(curr_entry)
        curr_entry = [0, 0, 0, 0, X, Y, Z, 1, -1 * y * X, -1 * y * Y, -1 * y * Z, -1 * y]
        A.append(curr_entry)

    A = np.array(A)
    U, S, Vt = np.linalg.svd(A)
    P = Vt[-1]
    P = P.reshape(3, 4)
    return P


def triangulate_and_add(point_cloud, Cs, correspondences, corresp_2d_to_3d, extra_ids, id_pairs):
    for i, j in id_pairs:
        if len(extra_ids[(i, j)]) != 0:
            point_cloud_ij = add_points3d(i, j, Cs[i], Cs[j], correspondences, corresp_2d_to_3d, select_ids=extra_ids[(i, j)])
            point_cloud = np.concatenate([point_cloud, point_cloud_ij], axis=0)
    return point_cloud


def add_points3d(id1, id2, C1, C2, correspondences, corresp_2d_to_3d, select_ids=None):
    if select_ids is not None:
        points1, points2 = correspondences[(id1, id2)]['cam1'][select_ids], correspondences[(id1, id2)]['cam2'][select_ids]
    else:
        points1, points2 = correspondences[(id1, id2)]['cam1'], correspondences[(id1, id2)]['cam2']

    points3d, _ = triangulate(points1, points2, C1, C2)
    corresp_2d_to_3d[id1].update({tuple(pt_2d): pt_3d for (pt_2d, pt_3d) in zip(points1, points3d)})
    corresp_2d_to_3d[id2].update({tuple(pt_2d): pt_3d for (pt_2d, pt_3d) in zip(points2, points3d)})
    return points3d


def load_data(path='data/data_cow'):
    correspondences = defaultdict(dict)

    for i in range(4):
        for j in range(i + 1, 4):
            id_pair = (i + 1, j + 1)
            correspondences[id_pair]['cam1'] = np.load(f'{path}/correspondences/pairs_{i + 1}_{j + 1}/cam1_corresp.npy')
            correspondences[id_pair]['cam2'] = np.load(f'{path}/correspondences/pairs_{i + 1}_{j + 1}/cam2_corresp.npy')

    C1 = np.load(f'{path}/cameras/cam1.npz')
    C2 = np.load(f'{path}/cameras/cam2.npz')
    K1, R1, t1 = C1['K'], C1['R'], C1['T']
    K2, R2, t2 = C2['K'], C2['R'], C2['T']

    return correspondences, (K1, R1, t1), (K2, R2, t2)


def sfm(path='data/data_cow', plot=True):
    correspondences, (K1, R1, t1), (K2, R2, t2) = load_data(path)
    extrinsics1 = np.concatenate([R1, t1[:, np.newaxis]], axis=1)
    extrinsics2 = np.concatenate([R2, t2[:, np.newaxis]], axis=1)
    Cs = {}
    Cs[1] = K1 @ extrinsics1
    Cs[2] = K2 @ extrinsics2
    corresp_2d_to_3d = {i: {} for i in range(1, 5)}
    extra_ids = {}
    matches = {}

    # Step 1: Triangulation between C1 and C2
    point_cloud = add_points3d(1, 2, Cs[1], Cs[2], correspondences, corresp_2d_to_3d)
    if plot:
        plot3d_animation(point_cloud, "sfm_cam_1_2", 'out/sfm')  # Plotting first phase of point cloud after triangulation

    # Step 2: Find correspondences for C3 pose estimation
    matches[(1, 3)], extra_ids[(1, 3)] = findMatches(correspondences[(1, 2)]['cam1'], correspondences[(1, 3)]['cam1'])
    for j, i in matches[(1, 3)].items():
        corresp_2d_to_3d[3][tuple(correspondences[(1, 3)]['cam2'][j])] = corresp_2d_to_3d[1][tuple(correspondences[(1, 2)]['cam1'][i])]

    matches[(2, 3)], extra_ids[(2, 3)] = findMatches(correspondences[(1, 2)]['cam2'], correspondences[(2, 3)]['cam1'])
    for j, i in matches[(2, 3)].items():
        corresp_2d_to_3d[3][tuple(correspondences[(2, 3)]['cam2'][j])] = corresp_2d_to_3d[2][tuple(correspondences[(1, 2)]['cam2'][i])]

    # Step 3: Triangulation for C3
    Cs[3] = getCamera(corresp_2d_to_3d[3], K1, cam_id=3)
    point_cloud = triangulate_and_add(point_cloud, Cs, correspondences, corresp_2d_to_3d, extra_ids, [(1, 3), (2, 3)])
    if plot:
        plot3d_animation(point_cloud, "sfm_cam_1_2_3", 'out/sfm')

    # Step 4: Find correspondences for C4 pose estimation
    matches[(1, 4)], extra_ids[(1, 4)] = findMatches(correspondences[(1, 2)]['cam1'], correspondences[(1, 4)]['cam1'])
    for j, i in matches[(1, 4)].items():
        corresp_2d_to_3d[4][tuple(correspondences[(1, 4)]['cam2'][j])] = corresp_2d_to_3d[1][tuple(correspondences[(1, 2)]['cam1'][i])]

    matches[(2, 4)], extra_ids[(2, 4)] = findMatches(correspondences[(1, 2)]['cam2'], correspondences[(2, 4)]['cam1'])
    for j, i in matches[(2, 4)].items():
        corresp_2d_to_3d[4][tuple(correspondences[(2, 4)]['cam2'][j])] = corresp_2d_to_3d[2][tuple(correspondences[(1, 2)]['cam2'][i])]

    matches[(3, 4)], extra_ids[(3, 4)] = findMatches(correspondences[(1, 3)]['cam2'], correspondences[(3, 4)]['cam1'])
    for j, i in matches[(3, 4)].items():
        corresp_2d_to_3d[4][tuple(correspondences[(3, 4)]['cam2'][j])] = corresp_2d_to_3d[3][tuple(correspondences[(1, 3)]['cam2'][i])]

    # Step 5: Triangulation -- C4
    Cs[4] = getCamera(corresp_2d_to_3d[4], K1, cam_id=4)
    point_cloud = triangulate_and_add(point_cloud, Cs, correspondences, corresp_2d_to_3d, extra_ids, [(1, 4), (2, 4), (3, 4)])
    if plot:
        plot3d_animation(point_cloud, "sfm_cam_1_2_3_4", 'out/sfm')
