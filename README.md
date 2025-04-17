# xarm6_octomap_avoidance

本パッケージは，Intel RealSense D435i を使用して取得したRGB-D点群からOctomapを構築し，それをMoveIt!に取り込むことで，xArm6ロボットがリアルタイムに障害物を避けながら軌道を生成・計画できるシステムです．  
論文「Dynamic Collision Avoidance System for a Manipulator Based on RGB-D Sensing」の再現を目的としています．

---

## ✅ 構成要素

- xArm6ロボット（`train_data_smoothing`など外部パッケージで起動）
- Intel RealSense D435i
- octomap_server（PointCloud → Occupancy Grid）
- MoveIt!（軌道生成）
- TF構成（`map`を基準座標とし，センサとロボットを配置）
- Octomapの手動注入ノード（`publish_octomap_to_moveit.py`）

---

## 📦 パッケージ構成
```bash
.
├── CMakeLists.txt
├── README.md
├── launch
│   ├── full_system.launch
│   ├── octomap_mapping.launch
│   └── xarm6_bringup.launch
├── package.xml
├── rviz
│   └── xarm6_octomap_avoidance.rviz
├── scripts
│   └── publish_octomap_to_moveit.py
└── src
```


---

## 🔧 依存パッケージ

- [xarm_ros](https://github.com/xArm-Developer/xarm_ros)
- [realsense2_camera](https://github.com/IntelRealSense/realsense-ros)
- [octomap_server](http://wiki.ros.org/octomap_server)
- moveit_commander, moveit_ros
- tf, rospy, std_msgs, geometry_msgs, octomap_msgs

---

## Install
```bash
git clone https://github.com/Amos2610/xarm6_octomap_avoidance.git
```


## 🚀 実行手順

### ① ワークスペースのビルド

```bash
cd ~/<your_workspace>
catkin build
source devel/setup.bash
```

### ② xArm6の起動

```bash
roslaunch xarm6_octomap_avoidance xarm6_bringup.launch
```

### ③ systemの起動

```bash
roslaunch xarm6_octomap_avoidance full_system.launch
```

### ④ 障害物判定をつけるスクリプトの起動
```bash
rosrun xarm6_octomap_avoidance publish_octomap_to_moveit.py
```