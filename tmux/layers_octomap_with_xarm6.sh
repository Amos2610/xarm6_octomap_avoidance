#!/usr/bin/env bash
set -e

SESSION="layers_octomap_with_xarm6"
if [ "$HOME" = "/home/calpis-sour" ]; then
  WORKDIR="$HOME/vision_ws"
  RVIZ_CFG="$WORKDIR/src/xarm6_octomap_avoidance/rviz/xarm6_octomap_avoidance.rviz"
else
  WORKDIR="$HOME/nishidalab_ws"
  RVIZ_CFG="$WORKDIR/src/5_skills/xarm6_octomap_avoidance/rviz/xarm6_octomap_avoidance.rviz"
fi
# 既存セッションがあればアタッチ
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[tmux] attach to existing session: $SESSION"
  exec tmux attach -t "$SESSION"
fi

# 新規セッション作成
tmux new-session -d -s "$SESSION" -c "$WORKDIR"
    # ペイン1：bringup
    tmux send-keys -t $SESSION "cd $WORKDIR" C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION  C-l
    if [ "$HOME" = "/home/calpis-sour" ]; then
      tmux send-keys -t $SESSION 'roslaunch xarm6_octomap_avoidance xarm6_bringup.launch sim:=true'
    else
      tmux send-keys -t $SESSION 'roslaunch xarm6_octomap_avoidance xarm6_bringup.launch'
    fi

    # ペイン3: full_system
    # 垂直分割してペインを作成
    tmux select-pane -t $SESSION:0.0
    tmux split-window -h -t $SESSION:0.0
    tmux send-keys -t $SESSION "cd $WORKDIR" C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION  C-l
    tmux send-keys -t $SESSION 'rosrun xarm6_octomap_avoidance yolo_to_collision_object.py'    

    # ペイン2: yolo_to_collision_object
    # ペイン2を選択してから、さらに垂直分割
    tmux select-pane -t $SESSION:0.0
    tmux split-window -h -t $SESSION:0.0
    tmux send-keys -t $SESSION "cd $WORKDIR" C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION C-l
    tmux send-keys -t $SESSION 'rosrun xarm6_octomap_avoidance autonomous_workspace_recognition.py'

    # ペイン3: 動的障害物検出
    # ペイン1を選択してから、さらに水平分割
    tmux select-pane -t $SESSION:0.0
    tmux split-window -v -t $SESSION:0.0
    tmux send-keys -t $SESSION "cd $WORKDIR" C-m
    tmux send-keys -t $SESSION 'source devel/setup.bash' C-m
    tmux send-keys -t $SESSION 'sleep 2' C-m
    tmux send-keys -t $SESSION C-l
    tmux send-keys -t $SESSION 'roslaunch xarm6_octomap_avoidance full_system.launch'

# 最初のペインをアクティブにしてセッションにアタッチ
tmux select-pane -t $SESSION:0.0
tmux attach -t "$SESSION"