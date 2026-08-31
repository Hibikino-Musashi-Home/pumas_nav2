# 付録A: パラメータ対応表（HSR → Go2）

[← 設計文書 本体](README.md)

`pumas_nav2` のパラメータの大半は launch XML に直書きされており、ロボット別のプロファイル機構は
ほぼ存在しない（`$(eval robot_name == 'hsrb')` による分岐が 2 箇所あるのみ）。
したがって Go2 用の値は、新設する `go2_navigation_slam.launch.xml` に記述する。

現行値の出典は `navigation_start/launch/navigation_slam.launch.xml` および
`navigation_start/launch/rtabmap_localization.launch.xml`、
そして HMA が Isaac Sim 向けに整理した
`carrobo_nav_pkg` の `carrobo_slam/io/config/localization_params.yaml` である。

---

## A.1 最重要: 基準フレームの取り違え

`pumas_nav2` の検知ボックスの高さ（`laser_min_z` = 0.022 など）は、
**HSR の `base_footprint` が接地面にある**ことを前提にした床基準の値である。

しかし Go2 の URDF は `base_footprint` を `base_link` と同一位置に定義している。

```xml
<!-- go2_robot_sdk/urdf/go2.urdf:60-64 -->
<joint name="base_footprint_joint" type="fixed">
  <parent link="base_link"/>
  <child link="base_footprint"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>   <!-- 接地面ではなく base_link と同一 -->
</joint>
```

つまり Go2 の `base_footprint` は**胴体中心**（立位でおよそ床上 0.32 m）を指す。
HSR の値をそのまま用いると、検知ボックスは胴体の上 0.022 m から 1.0 m の空間を見ることになり、
**床面上の障害物が一切検出されない**。

さらに `pumas_nav2` の内部でも既定値が統一されていない。

| ノード | `base_link_name` の既定値 | 出典 |
|---|---|---|
| `simple_move` | `base_footprint` | `simple_move_node.cpp:116` |
| `mvn_pln` | `base_footprint` | `mvn_pln_node.cpp:114` |
| `map_augmenter` | `base_footprint` | `map_augmenter_node.cpp:73` |
| `potential_fields` | **`base_link`** | `potential_fields_node.cpp:80` |

**対処**: Go2 では全ノードに `base_link_name:=base_link` を明示的に与えて統一し、
検知ボックスの高さを **`base_link`（胴体中心）基準**に読み替える。
床は `base_link` から見ておよそ −0.32 m の位置にある。

この案（案 A）で精度が足りない場合は、接地投影フレーム（yaw のみを継承し z を床に置く）を
publish する小ノードを追加する案 B に切り替える。

**2026-08-31、案 B に移行することで両セッションが合意した。**
実装は Go2 の bringup 層に置いた `ground_frame_publisher`（フレーム名 `base_ground`）が持ち、
`pointcloud_to_laserscan` の `target_frame` をそれに向ける。
`base_footprint` を使わないのは、URDF が既に（接地基準でない状態で）その名前を
使っているためである。

### pumas 側で必要な変更

**`base_link_name:=base_ground` を渡す。** これは任意ではなく必須である。

`LaserScan` は、それが表現されているフレームの **z = 0 平面**に載る。
`/scan` が `base_ground` で生成されるなら、pumas も同じフレームで扱わなければ
検知ボックスの z が意味を持たない。

| `base_link_name` | `/scan` 点の z（当該フレーム内） | 結果 |
|---|---|---|
| `base_link` | `base_ground` から変換され、z ≈ −0.32 かつ**ピッチで動く** | 高さ窓が姿勢で暴れる |
| **`base_ground`** | **z = 0（固定）** | 検知ボックスは 0 を跨ぐだけでよい |

`slam_toolbox` の `base_frame` も同じ引数を参照しているため、
`base_link_name:=base_ground` を渡すだけで両方が揃う。

これにより、pumas が本来前提としていた接地基準の `base_footprint` 相当が
初めて Go2 で手に入る（§A.1 冒頭の問題の根本的な解決）。
x・y 方向の検知ボックスは yaw が継承されるため変更不要。

---

## A.2 パラメータ一覧

「HSR 値」は `navigation_slam.launch.xml` の既定値、括弧内は `rtabmap_localization.launch.xml`
または carrobo_nav_pkg の運用値。

### フレーム

| パラメータ | HSR 値 | **Go2 初期値** | 根拠 |
|---|---|---|---|
| `base_link_name` | `base_footprint` | **`base_ground`**（暫定は `base_link`） | A.1。`base_ground` が publish されるまでは `base_link`。以降は `base_ground` が必須 |
| `odom_name` | `odom` | `odom` | 変更不要。`go2_driver_node` が同名で broadcast している |

### 検知ボックス（`base_link` 基準、単位 m）

`potential_fields` と `map_augmenter` が「障害物とみなす」直方体。
x が前方、y が左右、z が高さ。

| パラメータ | HSR 値 | **Go2 初期値** | 根拠 |
|---|---|---|---|
| `laser_min_x` | 0.17 | **0.35** | Go2 の胴体前端は `base_link` からおよそ 0.36 m（`nav2_params.yaml` のフットプリントも前方 0.36）。自己検出を避ける |
| `laser_max_x` | 0.50 | **0.90** | 停止距離に余裕を持たせる。速度を上げる際はここも伸ばす |
| `laser_min_y` / `laser_max_y` | −0.20 / 0.20 | **−0.30 / 0.30** | 脚の張り出しを含めた実効幅 0.40 m の半分 + 余裕 |
| `laser_min_z` | 0.022（HSR-B）/ 0.025 | **−0.25** | **要調査 Q1・Q3。** `base_link` 基準で床が約 −0.32 m。脚（−0.32〜0 m）を除外しつつ、床上の低い障害物を拾える高さに置く |
| `laser_max_z` | 1.00 | **0.60** | Go2 の全高は HSR より低い。天井付近まで見る必要がない |
| `cloud_*` | 各種 | — | 初期構成では点群を使わない（`use_point_cloud:=False`）ため設定しない |

`laser_min_z` は**脚の自己検出を避けるための下限**であり、四足では脚が常に視野に入るため
**最大のチューニングポイントになる**。HSR には存在しない問題である。

### 速度と制御

| パラメータ | HSR 値 | **Go2 初期値** | 根拠 |
|---|---|---|---|
| `max_linear_speed` | 0.8（carrobo_nav_pkg は 1.0） | **0.5** | 実機初回は保守的に。Q7 の実測後に見直す |
| `max_angular_speed` | 1.25 | **0.8** | 同上 |
| `max_lateral_speed` | （存在しない） | **0.25** | **新設**（付録B の C1）。Go2 の横歩きは遅く滑りやすい。P5 では 0 から開けていく |
| `min_linear_speed` | 0.05 | **要調査 Q7** | 0.05 m/s で歩くのか、その場で足踏みするのかが未確認 |
| `control_alpha` | 0.2〜0.3 | **0.3** | 旋回してから前進する度合い。四足の旋回は遅いので、まず HSR 相当から始める |
| `control_beta` | 0.8 | **0.8** | 同上 |

### 横移動（omni）

`simple_move` はゴール手前 `yaw_correction_omni_distance` [m] からユニサイクル制御を離れ、
横移動を含む holonomic 制御に切り替わる（`simple_move_node.cpp:791-828`）。

| パラメータ | HSR 値 | **Go2 初期値** | 根拠 |
|---|---|---|---|
| `yaw_correction_omni_behavior` | True | **True** | Go2 も 3 自由度を受けられるため有効にする。安全性は `max_lateral_speed` で確保する |
| `yaw_correction_omni_distance` | 0.5（carrobo_nav_pkg は 1.0） | **0.8** | 四足は姿勢を寄せるのに時間がかかる |
| `yaw_correction_omni_gain` | 4.5 | **2.0** | 四足の旋回は車輪より遅い。高すぎると指令が飽和し続ける |
| `yaw_correction_ignore_obstacles` | True | **True** | ゴール直前の姿勢合わせ中は回避を止める |
| `yaw_correction_align_head` | True | **False** | Go2 に head 制御が無い |

### コスト形状

| パラメータ | HSR 値 | **Go2 初期値** | 根拠 |
|---|---|---|---|
| `inflation_radius` | 0.12（rtabmap 運用は 0.22） | **0.32** | 通行不可として膨らませる半径。Go2 の胴体は約 0.70 × 0.31 m、脚の踏み出しを含めた実効寸法は約 0.80 × 0.40 m。`nav2_params.yaml` のフットプリントも `[0.36, 0.20]`〜`[−0.45, −0.20]` |
| `cost_radius` | 0.17（rtabmap 運用は 0.45） | **0.60** | 通行可能だがコストを与える半径。経路を壁から離す。`inflation_radius` より大きくする |
| `decay_factor` | 5（rtabmap 運用は 3） | **3** | 観測されなくなった障害物が残るサイクル数 |

### ポテンシャル場

| パラメータ | HSR 値 | **Go2 初期値** | 根拠 |
|---|---|---|---|
| `laser_pot_fields_d0` | 0.50 | **0.60** | 斥力が働き始める距離。Go2 は制動距離が長い |
| `laser_pot_fields_k_rej` | 0.30 | **0.30** | 斥力ゲイン。upstream の README も「不適切だと避けない／避けすぎる」と注意しており、実機で詰める |

### 機能の有効・無効

| パラメータ | HSR 値 | **Go2 初期値** | 根拠 |
|---|---|---|---|
| `use_lidar` | True | **True** | 主たる障害物入力 |
| `use_point_cloud` | True | **False** | HSR の頭部 RGBD 前提。Go2 に該当センサが無い。3D LiDAR を第2入力にするのは安定後 |
| `use_pot_fields` | True | **True** | 局所回避は有効にする |
| `move_head` | True（launch 内で固定） | **False** | Go2 に head 制御が無い |
| `add_static_obstacles` | True | **False** | Go2 環境向けの家具ポリゴン yaml が存在しない |
| `use_online` | True | **True** | SLAM 中は未知セルを通行可能として扱う |
| `robot_name` | `$(env ROBOT_NAME)` | **`go2`**（既定値として持つ） | 環境変数への依存を切る |
