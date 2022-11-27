from utils import toHomogenous, singularize, normalize
import numpy as np


def eightpoint(pts1, pts2):
    pts1_homogenous, pts2_homogenous = toHomogenous(pts1), toHomogenous(pts2)
    T = normalize(pts1_homogenous)
    T_prime = normalize(pts2_homogenous)

    pts1_homogenous = (T @ pts1_homogenous.T).T
    pts2_homogenous = (T_prime @ pts2_homogenous.T).T

    x1, y1 = pts1_homogenous[:, 0], pts1_homogenous[:, 1]
    x2, y2 = pts2_homogenous[:, 0], pts2_homogenous[:, 1]

    A = np.vstack((x2 * x1, x2 * y1, x2, y2 * x1, y2 * y1, y2, x1, y1, np.ones(x1.shape))).T
    U, D, VT = np.linalg.svd(A)
    F = VT[-1, :]
    F = F.reshape((3, 3))
    F = singularize(F)
    F = T_prime.T @ F @ T

    return F


def sevenpoint(pts1, pts2):
    pts1_homogenous, pts2_homogenous = toHomogenous(pts1), toHomogenous(pts2)
    T = normalize(pts1_homogenous)
    T_prime = normalize(pts2_homogenous)

    pts1_homogenous = (T @ pts1_homogenous.T).T
    pts2_homogenous = (T_prime @ pts2_homogenous.T).T

    x1, y1 = pts1_homogenous[:, 0], pts1_homogenous[:, 1]
    x2, y2 = pts2_homogenous[:, 0], pts2_homogenous[:, 1]

    A = np.vstack((x2 * x1, x2 * y1, x2, y2 * x1, y2 * y1, y2, x1, y1, np.ones(x1.shape))).T
    U, D, VT = np.linalg.svd(A)
    F1 = VT[-1, :]
    F1 = F1.reshape((3, 3)).T
    F2 = VT[-2, :]
    F2 = F2.reshape((3, 3)).T
    eigenvals, _ = np.linalg.eig(np.dot(np.linalg.inv(F2), F1))
    eigenvals = [eigenval.real for eigenval in eigenvals if eigenval.imag == 0]
    Farray = [(F1 - val * F2) for val in eigenvals]
    Farray = [singularize(F) for F in Farray]
    Farray = [T_prime.T @ F @ T for F in Farray]
    Farray = [F / F[2, 2] for F in Farray]
    return Farray
