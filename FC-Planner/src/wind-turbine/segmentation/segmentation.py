#%%
import os
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN

try:
    import rospy
except ModuleNotFoundError:
    print("Not running on the robot")


#%%
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
    
    def clear_rotated_folder(self):
        src_path = self._get_path_to_src()
        path = src_path + "/wind-turbine/data/rotated/"
        for file in os.listdir(path):
            os.remove(path + file)
# %%
class CenterWings:
    @staticmethod
    def _get_points_from_radius(pcd, center, radius, outside=True):
        pcd = copy_pcd(pcd)
        points = np.asarray(pcd.points)
        distances = np.linalg.norm(points - center, axis=1)
        if outside:
            radius_points = points[distances > radius]
        else:
            radius_points = points[distances < radius]
        return radius_points
    
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
        outside_points = self._get_points_from_radius(pcd, center, radius, outside=True)
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
    
#%%    
class pcd_segmentation:
    CENTER_WINGS = np.array([2.08266810e01, 6.90885454e-02, 1.40179431e02])
    CENTER_WINGS_MODEL = np.array([0, 0, 140.15])   

    def __init__(self):
        self.load_data = LoadData()
        self.rotate_pcd = RotatePointCloud()
        self.center_wings = CenterWings()
        self._load_point_clouds()
        self.CENTER_WINGS = self._determine_wing_center(self.wings)
        self.CENTER_TOWER = self._determine_tower_center(self.tower)
        self._make_wings_even() # Save in wings_new.pcd

    def _load_point_clouds(self):
        self.wings_old = self.load_data.load_point_cloud("/wind-turbine/data/wings_gt_dsx5.pcd")
        self.wings = self.load_data.load_point_cloud("/wind-turbine/data/wings_new.pcd")
        self.tower = self.load_data.load_point_cloud("/wind-turbine/data/tower_gt_ds.pcd")

    def _determine_wing_center(self, pcd):
        pcd = copy_pcd(pcd)
        center = self.CENTER_WINGS  #center = self.wings_old.get_center()
        for i in range(3):
            for _ in range(10):
                old_center = center
                pcd_rot = self.rotate_pcd.rotate(center, pcd, np.array([np.deg2rad(i*120),0,0]))
                center = self.center_wings.determine_center(center, pcd_rot, 65)
                dist = np.linalg.norm(old_center - center)
                if dist < 1e-9:
                    break
        print(center)
        return center
    
    def _determine_tower_center(self, pcd_init):
        # Extract points from the point cloud
        points = np.asarray(pcd_init.points)

        # Sort the points by the z-values (3rd column in the array)
        sorted_points = points[points[:, 2].argsort()]

        # Calculate the index for the 90th percentile
        cut_off_index = int(0.6 * len(sorted_points))

        # Select points that are below the 90th percentile of z-values (top 10% removed)
        filtered_points = sorted_points[:cut_off_index]

        # Create a new point cloud with the filtered points
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        
        center = pcd.get_center()
        # Set the z-value of the center to the z-value of the lowest point
        center[2] = np.mean(sorted_points[0:10, 2])
        return center
    
    def _make_wings_even(self):
        wings_old = copy_pcd(self.wings_old)
        wings_tips = self.center_wings._get_points_from_radius(self.wings_old, self.CENTER_WINGS, 55, outside=True)
        wings_tips_pc = o3d.geometry.PointCloud()
        wings_tips_pc.points = o3d.utility.Vector3dVector(wings_tips)

        wings_new = wings_old.uniform_down_sample(7)
        wings_added = wings_new + wings_tips_pc
        # Remove duplicates
        self.load_data.save_point_cloud("/wind-turbine/data/wings_new.pcd",wings_added)
       


    def main(self, axis_angle_tower, axis_angle_blades):
        wings_rot = self.rotate_pcd.rotate(self.CENTER_WINGS, self.wings, np.array(axis_angle_blades))
        wings_rot = self.rotate_pcd.rotate(self.CENTER_TOWER, wings_rot, np.array(axis_angle_tower))
        tower_rot = self.rotate_pcd.rotate(self.CENTER_TOWER, self.tower, np.array(axis_angle_tower))

        wind_turbine_rotated = wings_rot + tower_rot

        self.load_data.clear_rotated_folder()
        # Save the rotated point clouds
        self.load_data.save_point_cloud("/wind-turbine/data/rotated/tower.pcd", tower_rot)
        self.load_data.save_point_cloud("/wind-turbine/data/rotated/wings.pcd", wings_rot)
        self.load_data.save_point_cloud("/wind-turbine/data/rotated/wind_turbine.pcd", wind_turbine_rotated)


        # Add the tower 
# %%
if __name__ == "__main__":
    try:
        rospy.init_node("pcd_segmentation_node", anonymous=True)
        
        # Retrieve parameters from ROS
        axis_angle_tower = rospy.get_param("~axis_angle_tower", [0, 0, 0])
        axis_angle_blades = rospy.get_param("~axis_angle_blades", [0, 0, 0])

        # Print debug info
        rospy.loginfo(f"Axis angle tower: {axis_angle_tower}")
        rospy.loginfo(f"Axis angle blades: {axis_angle_blades}")


    except NameError:
        axis_angle_tower = [0,0,180]
        axis_angle_blades = [60,0,0]
    
    # Convert angles to radians
    axis_angle_tower = [np.deg2rad(a) for a in axis_angle_tower]
    axis_angle_blades = [np.deg2rad(a) for a in axis_angle_blades]
    seg = pcd_segmentation()
    seg.main(axis_angle_tower=axis_angle_tower, axis_angle_blades=axis_angle_blades)


# %%
