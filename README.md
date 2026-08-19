# YOLO and Tag

独立的 YOLO + 小蓝 AprilTag 视觉算法仓库。

本仓库与 `Grasp_hexapod` 完全独立，不修改、不依赖其控制代码。这里包含 YOLO
RGB-D 位姿源码和小蓝 Tag 节点源码；模型权重不提交，部署时通过参数指定。

源码位置：

- `ros/hexapod_perception/`：YOLO + RGB-D 位姿算法
- `ros/xiaolan_tag/`：小蓝三 Tag 检测与事件节点

## 输入

- RGB：`/camera/color/image_raw` (`sensor_msgs/Image`)
- D2C 对齐深度：`/camera/depth/image_raw` (`sensor_msgs/Image`)
- 彩色内参：`/camera/color/camera_info` (`sensor_msgs/CameraInfo`)
- TF：`base_link <- camera_color_optical_frame`

## YOLO 输出

- `/hexapod_perception/board_pose` (`geometry_msgs/PoseStamped`)
- `/hexapod_perception/board_position` (`geometry_msgs/PointStamped`)
- `/hexapod_perception/xiaolan_pose` (`geometry_msgs/PoseStamped`)
- `/hexapod_perception/xiaolan_position` (`geometry_msgs/PointStamped`)
- `/hexapod_perception/status` (`std_msgs/String` JSON)

位置单位为 m，四元数顺序为 `x,y,z,w`，默认输出坐标系为 `base_link`。消费完整
位姿前必须检查 `status` 的 `*_valid` 字段和消息时间戳。

## 小蓝 Tag 输出

Tag family 为 `tag36h11`。小蓝三枚 Tag 配置如下：

| 角色 | ID | 码边框边长 |
|---|---:|---:|
| left | 0 | 0.060 m |
| right | 5 | 0.060 m |
| rear | 6 | 0.060 m |

- `/tag_detections`：`apriltag_ros/AprilTagDetectionArray`
- `/yolo_and_tag/left_angle_deg`、`/yolo_and_tag/right_angle_deg`：滤波角度
- `/yolo_and_tag/side_angle`：带 ID、角色、位姿的 JSON
- `/yolo_and_tag/tag_event`：后 Tag 确认/丢失事件 JSON
- `/yolo_and_tag/side_scan_request`：请求规划侧扫位姿

侧面角度定义为相机光学坐标系 `+Z` 到 Tag `+X` 的夹角，范围 `[0,180]` 度，
不是机器人坐标系中的带符号 yaw。Tag 节点只发布感知结果和事件，不直接控制
速度、步态、关节或舵机。

## 联调

```bash
rostopic echo -n 1 /hexapod_perception/status
rostopic echo -n 1 /hexapod_perception/xiaolan_pose
rostopic echo -n 1 /tag_detections
rostopic echo -n 1 /yolo_and_tag/side_angle
```

遮挡或超时后，调用方必须安全降级，不能无限期使用缓存位姿。算法权重不随仓库
提交，部署时通过参数指定绝对路径。
