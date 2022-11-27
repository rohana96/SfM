from .triangulate import triangulate
from utils import get_RT
import numpy as np


def find_M2(F, pts1, pts2, intrinsics):
    K1 = intrinsics["K1"]
    K2 = intrinsics["K2"]
    E = K2.T @ F @ K1
    M1 = np.hstack((np.identity(3), np.zeros(3)[:, np.newaxis]))
    C1 = K1.dot(M1)
    C2 = np.zeros((3, 4))
    M2s = get_RT(E)

    N = min(pts1.shape[0], pts2.shape[0])
    P = np.zeros((N, 4))

    best_M2 = np.zeros((3, 4))
    best_error = np.finfo('float').max
    for i in range(M2s.shape[2]):
        M2 = M2s[:, :, i]
        _C2 = np.dot(K2, M2)
        X, err = triangulate(pts1, pts2, C1, _C2)
        if err < best_error and np.all(X[:, -1] >= 0):
            best_error = err
            P = X
            C2 = _C2
            best_M2 = M2

    print(f"Best Error {best_error}")
    print("Rotation: ", best_M2[:, :3], " Translation: ", best_M2[:, -1])
    return P
