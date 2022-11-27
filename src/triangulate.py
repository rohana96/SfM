import numpy as np
from utils import toUnhomogenize


def triangulate(pts1, pts2, C1, C2):

    p1_1 = C1[0, :]
    p1_2 = C1[1, :]
    p1_3 = C1[2, :]

    p2_1 = C2[0, :]
    p2_2 = C2[1, :]
    p2_3 = C2[2, :]

    N = min(pts1.shape[0], pts2.shape[0])
    X = np.zeros((N, 4))

    for i in range(N):
        x1 = pts1[i, 0]
        y1 = pts1[i, 1]
        x2 = pts2[i, 0]
        y2 = pts2[i, 1]
        A = np.vstack((y1 * p1_3 - p1_2,
                       p1_1 - x1 * p1_3,
                       y2 * p2_3 - p2_2,
                       p2_1 - x2 * p2_3))

        U, D, VT = np.linalg.svd(A)
        X[i, :] = VT[-1, :]

    X = X.T  # N x 4 --> 4 x N
    X /= X[-1, :]  # 4 x N


    pts1_reprojected = C1.dot(X)
    pts2_reprojected = C2.dot(X)

    pts1_reprojected /= pts1_reprojected[-1, :]
    pts2_reprojected /= pts2_reprojected[-1, :]

    pts1_homo = np.concatenate((pts1.T, np.ones((1, pts1.shape[0]))), axis=0)
    pts2_homo = np.concatenate((pts2.T, np.ones((1, pts2.shape[0]))), axis=0)

    err1 = np.sum(np.square(pts1_reprojected - pts1_homo))
    err2 = np.sum(np.square(pts2_reprojected - pts2_homo))

    err = err1 + err2
    X = toUnhomogenize(X.T)  # 4 x N --> N x 3
    return X, err

