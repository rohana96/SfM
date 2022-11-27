import argparse
import matplotlib.pyplot as plt

import cv2
import numpy as np

from src import (
    ransacF,
    sfm,
    run_colmap,
    find_M2
)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", choices=["fundamental", "ransac", "triangulate", "sfm", "colmap"], type=str)
    parser.add_argument("--image", "-i", type=str)
    parser.add_argument("--algorithm", "-a", choices=["eight", "seven"], default="eight")
    parser.add_argument("--num_iters", type=int, default=10000)
    parser.add_argument("--threshold", type=float, default=3)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    if args.run == 'triangulate':
        # load data
        intrinsics = np.load('data/monument/intrinsics.npy', allow_pickle=True).item()
        points = np.load('data/monument/some_corresp_noisy.npz', allow_pickle=True)
        points1 = points['pts1'].astype(np.int)
        points2 = points['pts2'].astype(np.int)

        img1_file = 'data/monument/im1.jpg'
        img2_file = 'data/monument/im2.jpg'

        img1 = cv2.imread(img1_file)
        img2 = cv2.imread(img2_file)

        output_dir = 'out/triangulate'

        bestF, best_inlier_percentage = ransacF(points1, points2, max_iteration=args.num_iters, method=arg.algorithm, threshold=args.threshold)
        best_M2, C1, C2, points3d = find_M2(bestF, points1, points2, intrinsics)

        colours = img1[points1[:, 1], points1[:, 0]] / 255.
        colours = colours[:, ::-1]
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        x, y, z = points3d[:, 0], points3d[:, 1], points3d[:, 2]
        ax.scatter(x, y, z, c=colours)
        plt.show()
        plt.close()

    if args.run == 'sfm':
        sfm(plot=False)

    if args.run == 'colmap':
        run_colmap()
