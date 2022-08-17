<!-- Floor Segmentation Demonstration ROS Package -->
This is a demo package for floor segmentation for AIR Lab. ROS Guide.

<!-- GETTING STARTED -->

Before running the node, publish sample point cloud in cloud folder.

```sh
cd cloud
rosrun pcl_ros pcd_to_pointcloud lab_cloud.pcd 1.0 _frame_id:=map /cloud_pcd:=/sample_pcd
```

Run floor_segmentation node

```sh
rosrun plane_segmentation plane_segmentation
```
