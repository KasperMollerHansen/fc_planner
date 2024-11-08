#%%
import open3d as o3d
import os
# %%
class Segmentation:
    FILE_PATH = os.path.abspath(__file__)

    def __init__(self):
        pass

    def _get_path_to_src(self):
        path = self.FILE_PATH
        while not path.endswith("src"):
            path = os.path.dirname(path)
        return path
    
    def _read_point_cloud(self, path, file_name):
        pcd = o3d.io.read_point_cloud(path + file_name)
        return pcd
    
    def main(self):
        src_path = self._get_path_to_src()
        pcd = self._read_point_cloud(src_path + "/wind-turbine/data/", "windmill.pcd")
        o3d.visualization.draw_geometries([pcd])


#%%
if __name__ == "__main__":
    seg = Segmentation()
    seg.main()
# %%

