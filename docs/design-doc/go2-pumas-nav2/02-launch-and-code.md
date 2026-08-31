# 付録B: launch とコード変更の詳細設計

[← 設計文書 本体](README.md)

---

## B.1 追加する launch: `go2_navigation_slam.launch.xml`

配置先: `navigation_start/launch/go2_navigation_slam.launch.xml`

`navigation_slam.launch.xml`（355 行）を土台にするが、**そのまま include したり
引数で上書きしたりはできない**。理由は次の 2 点である。

1. `arm_controller` と `head_controller` が**条件分岐なしで無条件に起動**する
   （`navigation_slam.launch.xml:323-340`、いずれも `respawn="true"`）。
   Go2 には対応する関節が無く、存在しないコントローラを叩き続ける
2. `simple_move` の `move_head` が `:233` で `True` に**固定**されており、引数から変えられない

したがって Go2 用の launch を独立したファイルとして書く。

### 起動するノード

| ノード | パッケージ | 主な設定 |
|---|---|---|
| `slam_toolbox`（async） | `slam_toolbox` | `scan_topic:=/scan`, `base_frame:=base_link`, `odom_frame:=odom`, `map_frame:=map`, `mode:=mapping` |
| `prohibition_map_server` | `nav2_map_server` | `navigation_start/maps/prohibition_maps/blank/map.yaml` を流用 |
| `lifecycle_manager_map` | `nav2_lifecycle_manager` | 上記の lifecycle 管理 |
| `map_enhancer` | `augment_gridmap_online` | `add_static_obstacles:=False` |
| `map_augmenter` | `map_augmenter` | `base_link_name:=base_link`、検知ボックスは付録A、`static_map_server:=/slam_toolbox/dynamic_map`, `use_online:=True` |
| `path_planner` | `path_planner` | `use_online:=True` |
| `simple_move` | `simple_move` | `cmd_vel_topic`, `base_link_name:=base_link`, `move_head:=False`, omni 系と `max_lateral_speed` |
| `potential_fields` | `potential_fields` | `base_link_name:=base_link`, `laser_scan_topic:=/scan`, `use_point_cloud:=False` |
| `publish_enable` | `potential_fields` | 回避の有効化フラグを配信 |
| `mvn_pln` | `mvn_pln` | `base_link_name:=base_link` |
| `param_rw.py` | `navigation_tools` | 実行中のパラメータ変更（P6 で使う） |
| `pumas_viz_goal_bridge.py` | `navigation_tools` | RViz の 2D Nav Goal を `/pumas_nav` アクションに橋渡し |
| `rviz2` | `rviz2` | Fixed Frame は `map` |

### 起動しないノード

`arm_controller`, `head_controller`, `gaze_controller`, `motion_synth_server`,
`pot_fields_updator`（人追従用のプリセット切替）。

**パッケージ自体はビルド対象に残す。** 依存は `trajectory_msgs` / `control_msgs` /
`pumas_interfaces` と軽く、`COLCON_IGNORE` で除外すると差分が増えるだけで利点がない。

ただし `motion_synth_server` については、`mvn_pln` がアクションクライアントとして
接続を待つ可能性がある（Q6）。P2 で `/pumas_nav` が応答しない場合は起動対象に加える。

### 引数の既定値

`robot_name` の既定を `go2` とし、`$(env ROBOT_NAME)` への依存を切る。
その他の既定値は付録A の表に従う。

将来的にパラメータを yaml へ外出しする場合は、
`carrobo_nav_pkg` の `carrobo_slam/io/config/localization_params.yaml`
の書き方が参考になる。値の意味と単位がコメントで整理されており、そのまま手本にできる。

---

## B.2 コード変更 C1: `simple_move` に横速度の上限を追加

対象: `simple_move/src/simple_move_node.cpp`

### 背景

Go2 の横歩きは車輪式の全方向台車より遅く、滑りやすい。しかし `simple_move` には
**横速度だけを独立に制限する仕組みが無い**。

ユニサイクル制御では、斥力がそのまま `linear.y` を**上書き**する。

```cpp
// simple_move_node.cpp:738-739
if (use_pot_fields && !move_lat)
  result.linear.y = rejection_force_y;
```

last-mile omni 制御では斥力が vx・vy に加算されるが、飽和は**合成ノルム**に対してのみ効く。

```cpp
// simple_move_node.cpp:820-825 付近
float mag = sqrt(vx * vx + vy * vy);
if (mag > max_linear_speed_) { vx *= max_linear_speed_ / mag; vy *= max_linear_speed_ / mag; }
```

いずれの経路でも、`linear.y` は `max_linear_speed` と同じ大きさまで出うる。

### 変更内容

1. `max_lateral_speed` パラメータを宣言する（既定値は `max_linear_speed` と同値、
   すなわち「実質的に制限なし」とし、**HSR の挙動を変えない**）
2. 実行中のパラメータ変更に対応させる（`:400` 付近の `set_parameter` コールバックに追加）
3. `/cmd_vel` を publish する直前の 1 箇所で `linear.y` をクランプする

publish 直前の 1 箇所に置くことで、ユニサイクル経路・omni 経路・横移動指令
（`/simple_move/goal_dist_lateral`）・相対移動（`SM_GOAL_REL_POSE`）・
`mvn_pln` のリカバリ動作のすべてに一様に効く。制御則そのものには手を入れない。

### upstream への還元

既定値が「制限なし」相当であるため、HSR の挙動には影響しない。
横速度の独立制限が無いことは upstream 側の欠落でもあるため、
`devel/go2` から本家へ提案できる形にしておく。

---

## B.3 コード変更 C2: `potential_fields` の LaserScan 購読 QoS

対象: `potential_fields/src/potential_fields_node.cpp`

### 背景

`pumas_nav2` のセンサ購読はほぼ全て `rclcpp::SensorDataQoS()`（BEST_EFFORT）である。
`map_augmenter` の LaserScan・PointCloud2、`potential_fields` の PointCloud2 も同様。

しかし `potential_fields` の LaserScan 購読だけが RELIABLE だった。

```cpp
// 変更前
sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_scan_topic_, rclcpp::QoS(10).reliable(), ...);
```

Go2 の `/scan` は `pointcloud_to_laserscan` が **BEST_EFFORT** で publish する。
RELIABLE な購読は BEST_EFFORT な publisher から**何も受け取らず、エラーも出さない**。

結果として `map_augmenter` は障害物を地図に載せる一方、
`potential_fields` だけが何も見えないまま動き続ける。
経路計画は働くのに**リアクティブな衝突回避だけが静かに消える**、
最も気づきにくく最も危険な壊れ方になる。

### 変更内容

`rclcpp::SensorDataQoS()` に変更する。

BEST_EFFORT な購読は RELIABLE な publisher からも受信できるため、
LiDAR が RELIABLE で publish している HSR の構成には影響しない。
同一ノード内の点群購読、および `map_augmenter` とも整合する。

これは Go2 固有の対処ではなく `pumas_nav2` 側の不統一の解消であり、
upstream に還元すべき変更である。

### 検証

`pointcloud_to_laserscan` の `/scan`（BEST_EFFORT）に対して
`potential_fields` が BEST_EFFORT で購読することを実測で確認済み（2026-08-31）。

---

## B.4 変更しないもの

| 対象 | 理由 |
|---|---|
| `go2_robot_sdk`（`src/0_go2/3rdparty/`） | 3rdparty であり、改造すると upstream 追従性が落ちる。`/scan` 生成系の修正は別セッションが担当する |
| `go2.urdf` | 同上。基準フレームの問題は launch のパラメータ側で吸収する（付録A の A.1） |
| `map_augmenter`, `path_planner`, `potential_fields`, `mvn_pln` のコード | パラメータのみで到達できる想定。到達できない箇所が出た場合は、その時点で理由を明示してから触る |
| `navigation_slam.launch.xml` など既存の launch | HSR の運用に影響を与えない。Go2 用は独立したファイルとして追加する |
