'''
Reference: https://github.com/colmap/pycolmap
'''

import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

import pycolmap
import numpy as np
from utils import plot3d_points
from pathlib import Path


# def run_colmap(image_dir="data/phototourism/sacre_coeur/set_100/images", output_path="./out/sacre_coeur"):
def run_colmap(image_dir="data/bunny_data_18_uniform/images", output_path="./out/stress_test/num_views/bunny_data_18_uniform"):
# def run_colmap(image_dir="data/paper_roll/images", output_path="./out/stress_test/surface/data/paper_roll"):
    # image_dir = Path(image_dir)
    output_path = Path(output_path)

    if not os.path.exists(output_path):
        os.makedirs(output_path, exist_ok=True)
    database_path = output_path / "database.db"

    pycolmap.extract_features(database_path, image_dir)
    pycolmap.match_exhaustive(database_path)
    maps = pycolmap.incremental_mapping(database_path, image_dir, output_path)
    maps[0].write(output_path)

    point_cloud = []
    points_3d = maps[0].points3D.items()
    for i in range(len(points_3d)):
        xyz = list(points_3d)[i][1].xyz
        point_cloud.append(xyz)

    point_cloud = np.array(point_cloud)
    plot3d_points(point_cloud)
