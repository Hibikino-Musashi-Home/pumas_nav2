# 付録C: 検証手順書

[← 設計文書 本体](README.md)

---

## C.0 安全

- 初回の実走行は **Go2 を吊るす**か、**周囲 2 m 以上を空けて**実施する
- ジョイスティックを接続しておく。`twist_mux` で joy（優先度 10）がナビゲーション（優先度 5）
  より優先されるため、**手動で割り込んで停止できる**
- `/stop`、`/navigation/stop`、`/simple_move/stop`（いずれも `std_msgs/Empty`）で
  停止できることを走行前に確認する

---

## C.0.1 起動の構成

### 層の分担

Go2 のセンサと駆動を ROS 2 に載せる層（以下 bringup 層）と、本スタックの launch は
どちらも `slam_toolbox` と `rviz2` を起動しうる。**重複するものは bringup 層側で切る。**

| 役割 | 担当 |
|---|---|
| ドライバ、`robot_state_publisher`、`/scan` 生成、`twist_mux`、joy | bringup 層 |
| `slam_toolbox`、pumas 一式、RViz | 本スタックの launch |

SLAM と自己位置推定を本スタック側が持つのは、保存済み地図 + AMCL/EMCL に
切り替える判断を「どう動くか」を決める側に残すためである。

### 起動順

**bringup 層が先。** `simple_move` は `/cmd_vel` を `transient_local` で publish するため、
本スタックを先に立てると、あとから現れた `twist_mux` が
**キャッシュされた最後の速度指令をいきなり受け取る**。ロボットが意図せず歩き出す。

### 段階

| 段階 | `cmd_vel_topic` | 目的 |
|---|---|---|
| 1 | ダミー（例 `/pumas/cmd_vel`） | 全ノードの起動確認、検知ボックスの目視。**ロボットは動かない** |
| 2 | `/cmd_vel`（`twist_mux` の navigation 入力） | 実走行 |

```bash
ros2 launch navigation_start go2_navigation_slam.launch.xml \
    cmd_vel_topic:=/pumas/cmd_vel \
    base_link_name:=base_ground \
    max_lateral_speed:=0.0
```

保存済み地図で走らせる場合は `go2_navigation_localization.launch.xml` を使い、
`map_dir` と `map_name` を渡す。

`base_link_name:=base_ground` は必須である（付録A の A.1）。

### 停止

| 手段 | 内容 |
|---|---|
| ジョイスティック | `twist_mux` で joy（優先度 10）がナビ（優先度 5）を上書きする。**最優先の手段** |
| `/stop` | `ros2 topic pub --once /stop std_msgs/msg/Empty '{}'` |
| launch の停止 | `Ctrl-C` |

ジョイスティックを無効にすると手動割り込みが効かなくなる。実走行では有効にしておく。

### 実行中のパラメータ変更

再起動せずに詰められる。

```bash
ros2 param set /simple_move max_linear_speed 0.4
ros2 param set /simple_move max_lateral_speed 0.15
ros2 param set /map_augmenter inflation_radius 0.30
ros2 param set /potential_fields laser_min_z -0.20
```

`laser_max_x` のように `potential_fields` と `map_augmenter` の両方が持つ値は、
**両方に設定する**必要がある。

---

## C.1 共通の準備

ROS 2 環境と本ワークスペースの `install/setup.bash` を読み込み、
ロボットとの接続モードをサイトの手順に従って設定しておく。

---

## C.2 Phase 1: ビルド

```bash
colcon build --packages-up-to pumas_nav2 --symlink-install
ros2 pkg list | grep -E 'mvn_pln|simple_move|map_augmenter|path_planner|potential_fields'
```

**完了条件**: 上記 5 パッケージが列挙される。

**実績（2026-08-31）**: 13 パッケージがエラーなくビルドされた（27.8 秒）。
出力された警告はいずれも既存のもの（未使用引数、符号比較）である。

`rtabmap` を使う launch（`rtabmap_localization.launch.xml`, `rtabmap_slam.launch.xml`）は
コンテナに `rtabmap` が無いため動かないが、本設計では使わないので問題ない。

---

## C.3 Phase 2: 起動確認（実機は動かさない）

`cmd_vel_topic` をダミー（例: `/pumas/cmd_vel`）に向けて起動し、
**実機に速度指令が届かない状態**で全ノードが立ち上がることを確認する。

```bash
ros2 launch navigation_start go2_navigation_slam.launch.xml \
    cmd_vel_topic:=/pumas/cmd_vel
```

### 確認項目

| # | 確認 | コマンド | 対応する問い |
|---|---|---|---|
| 1 | 全ノードが起動している | `ros2 node list` | — |
| 2 | `/pumas_nav` アクションサーバが応答する | `ros2 action list \| grep pumas_nav` | **Q6** |
| 3 | TF ツリーが 1 本にまとまっている | `ros2 run tf2_tools view_frames` | **Q9** |
| 4 | 地図が出ている | RViz で `/augmented_map`, `/augmented_cost_map` | — |

**Q6 の判定**: `/pumas_nav` にゴールを投げて feedback が返らない場合、
`mvn_pln` が `motion_synth` のアクションサーバを待っている可能性がある。
その場合は `motion_synth_server` を起動対象に加える（付録B の B.1）。

**実績（2026-08-31、実機なし）**: 10 ノードが起動し、`/pumas_nav` が公開された。
`use_motion_synth:=False` のままで問題ない。

ただし `pumas_viz_goal_bridge.py` は起動時に落ちる。
コンテナの `transforms3d` 0.3.1 と `numpy` 1.26.4 の不整合によるもので、
本体 README の §4.7 に詳細と対処を記載した。**これが解消するまで
RViz の 2D Nav Goal からはゴールを指定できない。**

なお `slam_toolbox` が終了時に exit code −9 で落ちるのは launch の停止処理によるもので、
起動から 20 秒後の時点では正常に動作していることを確認済みである。

**Q9 の判定**: `go2.urdf` は `map` / `odom` リンクを内包しているため、
`robot_state_publisher` と `slam_toolbox` が同じフレームを出して TF ツリーが
分裂または競合する可能性がある。`view_frames` の出力で
`map → odom → base_link → radar` が一本の木になっていることを確認する。

---

## C.4 Phase 3: 実機計測

Go2 を**吊るすか周囲を空けた状態**で計測する。結果を付録A の値に反映する。

### ~~Q2: `/point_cloud2` は届くか~~ — 解決済み

**実測（2026-08-31、別セッション、Wi-Fi / WebRTC、有線 LAN を外した状態）**

| トピック | レート | 備考 |
|---|---|---|
| `/point_cloud2` | 15.4 Hz | 約 62,000 点、`frame_id=odom`、BEST_EFFORT |
| `/scan` | 15.1 Hz | `pointcloud_to_laserscan` 経由、`frame_id=base_link`、BEST_EFFORT |
| `/odom` | 37.6 Hz | |
| `/joint_states` | 38.4 Hz | |
| `/camera/image_raw` | 14.2 Hz | |

**注意点**
- スマートフォンの Unitree アプリを完全に終了しておくこと。接続が残っていると
  ロボットが WebRTC 接続を reject する
- `ros2 topic hz` は、ドライバのログに `validated and ready` が出る前に測ると空振りする
- LiDAR はバイナリメッセージで届く。JSON の topic フィールドだけを見ると
  受信していないように見える

### Q1: `base_link` から床までの高さ

2 通りで求め、突き合わせる。

1. `go2.urdf` の脚リンク長から立位姿勢の幾何を計算する
2. 静止状態で `/point_cloud2` を `base_link` に変換し、床面に対応する点群の z の最頻値を取る

**記録**: 立位での床の z [m]（`base_link` 基準、負値になるはず）。

### Q3: LiDAR の視野に脚がどれだけ映るか

静止状態で `/point_cloud2` を RViz に表示し、脚に対応する点群を目視で確認する。
あわせて z 方向のヒストグラムを取る。

**記録**: 脚の点群が占める z の範囲と x・y の範囲。
これが `laser_min_z` と `laser_min_x` の下限を決める。

**四足特有の問題であり、HSR には存在しない。** 本設計で最も不確実性が高い箇所である。

### Q4: 歩行時の胴体ピッチ振幅

ジョイスティックで直進させながら TF `odom → base_link` の pitch を記録する。

**記録**: pitch の振幅 [rad] と周期。
振幅が大きい場合、付録A の案 A（`base_link` 基準の固定高さ窓）では床を誤検出しうるため、
案 B（接地投影フレームの新設）に切り替える。

### Q5: `/odom` の精度・レート・遅延

```bash
ros2 topic hz /odom
```

ジョイスティックで直進 3 m させ、`/odom` の移動量と実測値を比較する。

**記録**: レート [Hz]、3 m 走行時の累積誤差 [m]。
`twist` が空であること、`position.z` に +0.07 のオフセットがあることは既知。

### Q7: 実効速度上限と最小指令速度

`tools/go2_move.py` で `Move()` を直接投げ、段階的に確認する。

**記録**: vx・vy・vyaw それぞれの実用上限。
とくに **vx = 0.05 m/s で前進するのか、その場で足踏みするのか**を確認する
（`simple_move` の `min_linear_speed` の既定値が 0.05 のため）。

### Q8: WebRTC の指令反映レートと遅延

`/cmd_vel_out` に publish してから実機が反応するまでの時間を計測する。

**記録**: 遅延 [ms]。`simple_move` は 30 Hz 制御であり、遅延が大きいと発振する。

### Q11: `radar` フレームの取付姿勢

URDF の `radar_joint` は `rpy="0 2.8782 0"`（約 165°）と大きく傾いている。
`base_link` に変換した点群が実際の周囲環境と整合するかを RViz で確認する。

---

## C.5 Phase 4: 検知ボックスの妥当性確認（必須ゲート）

**本設計で最も見落としやすく、見落とすと必ず事故になる箇所である。**

RViz で次の 2 つを表示する。

- `/detect_area_marker` — 検知ボックスの可視化
- `/pot_field_markers` — 斥力ベクトルの可視化

### 判定基準

| # | 確認 | 不合格のときに起きること |
|---|---|---|
| 1 | 検知ボックスが Go2 自身の**胴体と重なっていない** | 常に障害物ありと判定され、一歩も動けない |
| 2 | 検知ボックスが Go2 自身の**脚と重なっていない** | 同上。歩行のたびに誤検出する |
| 3 | 検知ボックスが**床面より上にある** | 床を障害物と誤検出する |
| 4 | 検知ボックスが**床面から浮きすぎていない** | 低い障害物を見落とし、そのまま衝突する |
| 5 | 障害物を前に置くと `/navigation/potential_fields/collision_risk` が `true` になる | 回避が働かない |

5 は実際に段ボール等を Go2 の前 0.5 m に置いて確認する。

---

## C.6 Phase 5: 実走行

`cmd_vel_topic` を `/cmd_vel`（`twist_mux` の navigation 入力）に戻す。

```bash
ros2 launch navigation_start go2_navigation_slam.launch.xml \
    cmd_vel_topic:=/cmd_vel \
    base_link_name:=base_ground \
    max_linear_speed:=0.3 \
    max_angular_speed:=0.5 \
    max_lateral_speed:=0.0
```

### 手順

| # | 内容 | 完了条件 |
|---|---|---|
| 1 | 障害物の無い直線 2〜3 m を走行する | 蛇行せず到達する |
| 2 | 経路上に障害物を置いて回避させる | 停止または迂回する。衝突しない |
| 3 | `max_lateral_speed` を 0.15 に上げる | ゴール直前の姿勢合わせが滑らかになる |
| 4 | `max_lateral_speed` を 0.25 に上げる | 滑り・転倒が起きない |
| 5 | `max_linear_speed` を段階的に上げる | 追従が破綻しない上限を見つける |

### Q10 の判定

走行中に `slam_toolbox` が作る地図が破綻しないことを確認する。
LiDAR の水平スライスが薄い、あるいは脚が映るといった理由で収束しない場合、
`/scan` の生成条件（高さ窓）の見直しが必要になる。

---

## C.7 Phase 6: チューニング

`navigation_tools` の `param_rw.py` を経由して**実行中にパラメータを変更できる**。
再起動せずに詰められる。使用例は
`carrobo_nav_pkg` の `carrobo_nav/dynamic_reconfig` にある。

### 推奨する順序

| # | 対象 | 見るもの |
|---|---|---|
| 1 | `laser_min_z` | 脚の自己検出が消えるまで。`/detect_area_marker` と `/pot_field_markers` |
| 2 | `inflation_radius`, `cost_radius` | 経路の壁への寄り方 |
| 3 | `laser_pot_fields_d0`, `laser_pot_fields_k_rej` | 回避の効き。避けないか、避けすぎるか |
| 4 | `control_alpha`, `control_beta` | 旋回と前進のバランス。四足の追従性 |
| 5 | `yaw_correction_omni_*` | ゴール直前の姿勢合わせ。最後に調整する |

---

## C.8 上位からの呼び出し確認

`navigation_tools/navlib.py` の `NavLib` からゴールを指定できることを確認する。

```bash
ros2 action list | grep pumas_nav
```

`carrobo_nav_pkg` の `send_path` / `go_rel` / `nav_goal_full_args` の各ノードが、
そのまま呼び出し方の実例になる。

`go_rel`（ロボット座標系での相対移動）は `SM_GOAL_REL_POSE` を使い、
`simple_move_node.cpp:1198` に「holonomic ロボットのみ」という注記がある経路を通る。
Go2 は 3 自由度を出せるため動作する見込みだが、`max_lateral_speed` が効くことを確認する。
