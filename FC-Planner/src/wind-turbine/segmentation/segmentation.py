#%%
import open3d as o3d
import numpy as np
import functions as f

try:
    import rospy
except ModuleNotFoundError:
    print("Not running on the robot")


#%%
class pcd_segmentation:
    CENTER_WINGS = np.array([20.82506782,0.24930474,140.12672987])
        
    def __init__(self):
        self.load_data = f.LoadData()
        self.rotate_pcd = f.RotatePointCloud()
        self.center_wings = f.CenterWings()
        self._load_point_clouds()
        self.CENTER_WINGS = self._determine_wing_center(self.wings)
        self.CENTER_TOWER = self._determine_tower_center(self.tower)

    def _load_point_clouds(self):
        self.wings_old = self.load_data.load_point_cloud("/wind-turbine/data/wings_gt_ds.pcd")
        self.wings = self.load_data.load_point_cloud("/wind-turbine/data/wings_new.pcd")
        self.tower = self.load_data.load_point_cloud("/wind-turbine/data/tower_gt_ds.pcd")

    def _determine_wing_center(self, pcd):
        #center = self.wings.get_center()
        center = self.CENTER_WINGS
        for _ in range(50):
            old_center = center
            center = self.center_wings.determine_center(old_center, pcd, 65)
            if np.linalg.norm(old_center - center) < 1e-12:
                print("Converged")
                print(center)
                break
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
        wings = []
        for i in range(3):
            wings_rot = self.rotate_pcd.rotate(self.CENTER_WINGS, self.wings, [np.deg2rad(120*i),0,0])
            wings.append(wings_rot)
        wings_new = wings[0] + wings[1] + wings[2]
        wings_new = wings_new.uniform_down_sample(3)
        self.load_data.save_point_cloud("/wind-turbine/data/wings_new.pcd",wings_new)


    def main(self, axis_angle_tower, axis_angle_blades):
        wings_rot = self.rotate_pcd.rotate(self.CENTER_WINGS, self.wings, np.array(axis_angle_blades))
        wings_rot = self.rotate_pcd.rotate(self.CENTER_TOWER, wings_rot, np.array(axis_angle_tower))
        tower_rot = self.rotate_pcd.rotate(self.CENTER_TOWER, self.tower, np.array(axis_angle_tower))

        wind_turbine_rotated = wings_rot + tower_rot
        self.load_data.save_point_cloud("/wind-turbine/data/rotated/tower.pcd", tower_rot)
        self.load_data.save_point_cloud("/wind-turbine/data/rotated/wings.pcd", wings_rot)
        self.load_data.save_point_cloud("/wind-turbine/data/rotated/wind_turbine.pcd", wind_turbine_rotated)

        # Add the tower 
# %%
if __name__ == "__main__":
    rospy.loginfo(f"TEST")
    try:
        rospy.init_node("pcd_segmentation_node", anonymous=True)
        
        # Retrieve parameters from ROS
        axis_angle_tower = rospy.get_param("~axis_angle_tower", [0, 0, 0])
        axis_angle_blades = rospy.get_param("~axis_angle_blades", [0, 0, 0])

    except NameError:
        axis_angle_tower = [0,0,180]
        axis_angle_blades = [60,0,0]
    
    # Print debug info
    rospy.loginfo(f"Axis angle tower: {axis_angle_tower}")
    rospy.loginfo(f"Axis angle blades: {axis_angle_blades}")

    # Convert angles to radians
    axis_angle_tower = [np.deg2rad(a) for a in axis_angle_tower]
    axis_angle_blades = [np.deg2rad(a) for a in axis_angle_blades]
    seg = pcd_segmentation()
    seg.main(axis_angle_tower=axis_angle_tower, axis_angle_blades=axis_angle_blades)



# %%
