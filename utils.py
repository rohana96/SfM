import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def toHomogenous(pts):
    return np.vstack([pts[:, 0], pts[:, 1], np.ones(pts.shape[0])]).T.copy()


def toUnhomogenize(pts):
    return pts[:, :-1] / pts[:, None, -1]


def singularize(F):
    U, S, V = np.linalg.svd(F)
    S[-1] = 0
    F = U.dot(np.diag(S).dot(V))
    return F


def get_RT(E):
    U, S, V = np.linalg.svd(E)
    m = S[:2].mean()
    E = U.dot(np.array([[m, 0, 0], [0, m, 0], [0, 0, 0]])).dot(V)
    U, S, V = np.linalg.svd(E)
    W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    # if np.linalg.det(U.dot(W).dot(V)) < 0:
    #     W = -W

    RTs = np.zeros([3, 4, 4])
    RTs[:, :, 0] = np.concatenate([U.dot(W).dot(V), U[:, 2].reshape([-1, 1]) / abs(U[:, 2]).max()], axis=1)
    RTs[:, :, 1] = np.concatenate([U.dot(W).dot(V), -U[:, 2].reshape([-1, 1]) / abs(U[:, 2]).max()], axis=1)
    RTs[:, :, 2] = np.concatenate([U.dot(W.T).dot(V), U[:, 2].reshape([-1, 1]) / abs(U[:, 2]).max()], axis=1)
    RTs[:, :, 3] = np.concatenate([U.dot(W.T).dot(V), -U[:, 2].reshape([-1, 1]) / abs(U[:, 2]).max()], axis=1)
    return RTs


def normalize(points):
    x_mean = np.mean(points[:, 0])
    y_mean = np.mean(points[:, 1])
    sigma = np.mean(np.sqrt((points[:, 0] - x_mean) ** 2 + (points[:, 1] - y_mean) ** 2))
    M = np.sqrt(2) / sigma
    T = np.array([
        [M, 0, -M * x_mean],
        [0, M, -M * y_mean],
        [0, 0, 1]
    ])
    return T


def load_correspondences_and_intrinsics(correspondences_file, intrinstic_file):
    correspondences = np.load(correspondences_file)
    points1 = correspondences["pts1"]
    points2 = correspondences["pts2"]
    K = np.load(intrinstic_file)
    K1 = K["K1"]
    K2 = K["K2"]
    return points1, points2, K1, K2


def plot_points(img, points, path):
    img = img.copy()
    for point in points:
        x, y = point
        img = cv2.circle(img, (int(x), int(y)), radius=15, color=(0, 0, 255), thickness=-1)
    cv2.imwrite(path, img)


def plot3d_points(points3d):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    x, y, z = points3d[:, 0], points3d[:, 1], points3d[:, 2]
    ax.scatter(x, y, z, c=z, cmap='rainbow', s=1, alpha=1)
    plt.show()
    plt.close()


# Code snippet credits: Piazza
def plot3d_animation(points3d, tag, savepath):
    def rotate(angle):
        ax.view_init(azim=angle)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    x, y, z = points3d[:, 0], points3d[:, 1], points3d[:, 2]
    ax.scatter(x, y, z, c=z, cmap='rainbow', s=1, alpha=1)

    if not os.path.exists(savepath):
        os.makedirs(savepath, exist_ok=True)
    rot_animation = animation.FuncAnimation(fig, rotate, frames=np.arange(0, 361, 2), interval=50)
    rot_animation.save(f'{savepath}/rotation_{tag}.gif', dpi=80, writer='imagemagick')


def plot_epipolar_line(img, line, file):
    img = img.copy()
    H, W = img.shape[:2]
    x1 = 0
    y1 = int(-line[2] / line[1])
    x2 = int(H - 1)
    y2 = int((-line[2] - line[0] * (H - 1)) / line[1])
    img = cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 10)
    cv2.imwrite(file, img)


def epipolar_distance(pts1, pts2, F):
    epipolar_line1 = (F.T @ pts2.T).T
    epipolar_line2 = (F @ pts1.T).T
    d1 = np.abs(np.sum(epipolar_line2 * pts2, axis=1)) / np.sqrt(epipolar_line2[:, 0] ** 2 + epipolar_line2[:, 1] ** 2)
    d2 = np.abs(np.sum(epipolar_line1 * pts1, axis=1)) / np.sqrt(epipolar_line1[:, 0] ** 2 + epipolar_line1[:, 1] ** 2)
    return d1 + d2
