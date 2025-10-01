#!/usr/bin/env python3
import rospy
import roslib
import yaml
from moveit_commander import RobotCommander, MoveGroupCommander


class MoveItMapping:
    def __init__(self):
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")
        self.joints_list = []

        # ファイルパスの取得
        # xarm6_octomap_avoidanceパッケージを探す(roslibパッケージが必要)
        self.joint_path = roslib.packages.get_pkg_dir("xarm6_octomap_avoidance") + "/io/param.yaml"


    def load_joint_values(self, map_name="mapping_semi_static"):
        """
        YAMLファイルからジョイント値リストを読み込む
        Args:
            map_name (str): 読み込むジョイント値リストのキー名
        """
        with open(self.joint_path, "r") as file:
            data = yaml.safe_load(file)
        self.joints_list = data["Joint"][map_name]
        print(f"Loaded {len(self.joints_list)} joint configurations.")

    def execute_joint_trajectories(self):
        """
        全ジョイント構成に対してMoveItでプラン・実行
        """
        self.xarm.set_max_velocity_scaling_factor(0.3)  # 速度を30%に制限
        self.xarm.set_max_acceleration_scaling_factor(0.15)  # 加速度を15%に制限
        for idx, joint_values in enumerate(self.joints_list):
            if rospy.is_shutdown():
                print("ROS is shutting down. Exiting...")
                break
            print(f"\n[ {idx+1} / {len(self.joints_list)} ] Joint values: {joint_values}")
            try:
                self.xarm.set_start_state_to_current_state()
                self.xarm.set_joint_value_target(joint_values)

                print("Planning...")
                plan = self.xarm.plan()

                if plan and plan[0]:  # (success_flag, plan, _, _) の形式に対応
                    print("Executing plan...")
                    self.xarm.execute(plan[1], wait=True)
                    rospy.sleep(2.0)  # 軽く待機して安定
                else:
                    print("Planning failed. Skipping this configuration.")
                    continue
            except Exception as e:
                print(f"Execution error: {e}")
                continue

        print("Finished all joint movements.")


if __name__ == "__main__":
    try:
        moveit_mapping = MoveItMapping()
        moveit_mapping.load_joint_values()
        moveit_mapping.execute_joint_trajectories()
    except rospy.ROSInterruptException:
        pass
