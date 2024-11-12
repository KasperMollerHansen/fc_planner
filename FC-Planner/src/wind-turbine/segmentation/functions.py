#%%
import os
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
# %%
def translate_pcd(pcd, rotation_center):
    points = np.asarray(pcd.points)
    points += rotation_center
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def copy_pcd(pcd):
    pcd_copy = o3d.geometry.PointCloud()
    pcd_copy.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
    return pcd_copy
# %%
class LoadData:
    FILE_PATH = os.path.abspath(__file__)

    def _get_path_to_src(self):
        path = self.FILE_PATH
        while not path.endswith("src"):
            path = os.path.dirname(path)
        return path
    
    def _read_point_cloud(self, path, file_name):
        pcd = o3d.io.read_point_cloud(path + file_name)
        return pcd
    
    def load_point_cloud(self, path_from_src):
        src_path = self._get_path_to_src()
        pcd = self._read_point_cloud(src_path, path_from_src)
        return pcd
    
    def save_point_cloud(self, path_from_src, pcd):
        src_path = self._get_path_to_src()
        o3d.io.write_point_cloud(src_path + path_from_src, pcd, write_ascii=True)
    
# %%
class CenterWings:
    @staticmethod
    def _get_points_outside_radius(pcd, center, radius):
        pcd = copy_pcd(pcd)
        points = np.asarray(pcd.points)
        distances = np.linalg.norm(points - center, axis=1)
        outside_points = points[distances > radius]

        return outside_points
    @staticmethod
    def _get_centroids_of_groups_within_radius(center, points, radius):
        # Cluster points using DBSCAN
        try:
            clustering = DBSCAN(eps=radius, min_samples=1).fit(points)
        except ValueError:
            return []

        # Get labels for each point
        labels = clustering.labels_

        # Find unique clusters
        unique_labels = np.unique(labels)

        centroids = []

        # Iterate over each cluster
        for label in unique_labels:
            # Get the points belonging to this cluster
            cluster_points = points[labels == label]
            # Calculate the centroid of the cluster
            centroid = np.mean(cluster_points, axis=0)
            # Set x-coordinate to the center x-coordinate
            centroid[0] = center[0]
            centroids.append(centroid)

        return centroids
    @staticmethod
    def _adjust_center(centroids, center, radius):
        center_new = center.copy()
        for centroid in centroids:
            unit_vec = (centroid - center) / np.linalg.norm(centroid - center)
            center_new += ((centroid - center) - unit_vec * radius)
        return center_new
    
    def determine_center(self, center, pcd, radius):
        center = center.copy()
        outside_points = self._get_points_outside_radius(pcd, center, radius)
        centroids = self._get_centroids_of_groups_within_radius(center, outside_points, radius*0.1)
        new_center = self._adjust_center(centroids, center, radius)
        return new_center

# %%
class RotatePointCloud:
    @staticmethod
    def _get_rotation_matrix_from_axis_angle(axis_angle):
        angle = np.linalg.norm(axis_angle)
        if angle == 0:
            return np.eye(3)

        # Normalize the axis of rotation
        axis = axis_angle / angle
        x, y, z = axis

        # Compute the components of the rotation matrix
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        one_minus_cos = 1 - cos_theta

        # Rodrigues' rotation formula
        rotation_matrix = np.array([
            [cos_theta + x * x * one_minus_cos, x * y * one_minus_cos - z * sin_theta, x * z * one_minus_cos + y * sin_theta],
            [y * x * one_minus_cos + z * sin_theta, cos_theta + y * y * one_minus_cos, y * z * one_minus_cos - x * sin_theta],
            [z * x * one_minus_cos - y * sin_theta, z * y * one_minus_cos + x * sin_theta, cos_theta + z * z * one_minus_cos]
        ])

        return rotation_matrix

    def rotate(self, center ,pcd_init, axis_angle):
        # Create a new point cloud to store the rotated points
        pcd = copy_pcd(pcd_init)

        rotation_center = center
        rotation_matrix = self._get_rotation_matrix_from_axis_angle(axis_angle)
        
        # 1. Translate point cloud so that rotation center is at the origin
        pcd = translate_pcd(pcd,-rotation_center)
        
        # 2. Rotate the point cloud
        rotation_matrix = self._get_rotation_matrix_from_axis_angle(axis_angle)
        rotated_points = np.dot(np.asarray(pcd.points), rotation_matrix.T)
        pcd.points = o3d.utility.Vector3dVector(rotated_points)

       # 3. Translate the point cloud back to its original position
        pcd = translate_pcd(pcd,rotation_center)
        return pcd



# %%
