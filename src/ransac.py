import numpy as np
from tqdm import tqdm

from .compute_fundamental import (
    eightpoint,
    sevenpoint
)
from utils import (
    toHomogenous,
    epipolar_distance
)


def ransacF(pts1, pts2, threshold=3e-4, max_iteration=1000, method='seven'):
    N = pts1.shape[0]
    pts1_homo, pts2_homo = toHomogenous(pts1), toHomogenous(pts2)
    max_inliers = 0

    bestF = None
    inlier_cnt_arr = []
    cnt_inlier = 0

    for i in tqdm(range(max_iteration)):
        Farray = None
        if method == 'seven':
            idxs = np.random.choice(N, 7, replace=False)
            Farray = sevenpoint(pts1[idxs], pts2[idxs])
        elif method == 'eight':
            idxs = np.random.choice(N, 8, replace=False)
            Farray = [eightpoint(pts1[idxs], pts2[idxs])]

        for F in Farray:
            distance = epipolar_distance(pts1_homo, pts2_homo, F)
            inlier_mask = distance < threshold
            cnt_inlier = inlier_mask.sum()

            if cnt_inlier > max_inliers:
                bestF, max_inliers = F, cnt_inlier

        inlier_cnt_arr.append(max_inliers / N)

    return bestF, inlier_cnt_arr
