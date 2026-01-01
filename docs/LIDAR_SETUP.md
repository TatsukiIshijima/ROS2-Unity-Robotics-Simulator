# LIDARセットアップガイド - RViz2での可視化まで

このドキュメントでは、Unity上のLidarセンサーをROS2と統合し、RViz2でmap座標系での可視化を実現する完全な手順を説明します。

## 概要

### このガイドの目的

- Unity上のRaycastLiDARセンサーからLaserScanデータをROS2に配信
- TF（Transform）フレームを配信してロボットの位置関係を管理
- RViz2でmap座標系での可視化とロボット軌跡の表示
- SLAM（地図作成）の準備を完了

### 達成できること

✅ Unity上のLidarセンサーデータをROS2に配信
✅ RViz2でmap座標系での可視化
✅ ロボット移動軌跡の確認
✅ SLAM準備完了

---

## 前提条件

- Unity Editorでプロジェクトが開かれていること
- ROS2環境（Docker）が利用可能であること
- UnitySensorsとUnitySensorsROSパッケージがインストール済み

---

## アーキテクチャ概要

### システム構成

```
Unity Simulator
  ├─ RaycastLiDARSensor      → スキャンデータ生成
  ├─ RaycastLidarScanPublisher → LaserScanメッセージ配信
  └─ RobotTFPublisher         → TFフレーム配信
         ↓
    ROS-TCP-Connector (TCP通信)
         ↓
ROS2 Environment (Docker)
  ├─ ROS-TCP-Endpoint         → Unity↔ROS2ブリッジ
  ├─ /raycast_lidar/scan トピック
  └─ /tf トピック
         ↓
      RViz2                   → 可視化
```

### TFフレーム構造

```
map (ワールド座標)
 └─ base_link (ロボット中心)
      └─ laser (Lidarセンサー位置: 上方0.13m、180度回転)
```

**TFフレームとは？**
ロボットの各部品の3D空間での位置関係を表現する仕組みです。RViz2が異なる座標系間でデータを変換するために必要です。

---

## セットアップ手順

### ステップ1: 2D ScanPatternの作成

#### 1-1. ScanPatternGeneratorを開く

Unity Editorのメニューバーから：
```
UnitySensors > LiDAR > Generate ScanPattern
```

#### 1-2. ScanPattern設定

ScanPatternGeneratorウィンドウで以下のように設定：

| 項目 | 設定値 | 説明 |
|------|--------|------|
| **Source** | `FromSpecification` | 仕様から生成 |
| **Zenith Angles** | `[0]` | 水平面のみ（配列サイズ1、値0） |
| **Min Azimuth Angle** | `0` | 開始角度0度 |
| **Max Azimuth Angle** | `360` | 終了角度360度 |
| **Azimuth Angle Resolution** | `360` | 1度刻み（360点） |

**Zenith Anglesの設定方法**:
1. Zenith Anglesの左の三角をクリックして展開
2. `Size`を`1`に設定
3. `Element 0`に`0`を入力（水平面 = 0度）

#### 1-3. ScanPatternを生成

1. `Generate ScanPattern`ボタンをクリック
2. `Assets/NewScanPattern.asset`が生成される
3. Projectウィンドウで`NewScanPattern.asset`を右クリック → Rename
4. 名前を`RPLidar_2D_360`に変更
5. `Assets/Resources/ScanPatterns/`フォルダに移動（フォルダがなければ作成）

---

### ステップ2: Unity Sceneでのセンサー設定

#### 2-1. Sceneを開く

Projectウィンドウで：
```
Assets/Scenes/LidarSlamScene.unity
```
をダブルクリックして開く

#### 2-2. laser GameObjectを選択

Hierarchyウィンドウで：
```
raspimouse_with_camera_rplidar > base_link > rplidar_multi_mount_link > laser
```
を選択

#### 2-3. RaycastLiDARSensorコンポーネントを追加

Inspectorウィンドウで：

1. `Add Component`ボタンをクリック
2. `RaycastLiDARSensor`を検索して選択
3. 以下のパラメータを設定：

| パラメータ | 設定値 | 説明 |
|-----------|--------|------|
| **Scan Pattern** | `RPLidar_2D_360` | ステップ1で作成したアセット |
| **Points Num Per Scan** | `360` | スキャン点数 |
| **Min Range** | `0.12` | 最小検出距離（m） |
| **Max Range** | `12.0` | 最大検出距離（m） |
| **Gaussian Noise Sigma** | `0.01` | ノイズ（1cm） |
| **Max Intensity** | `255.0` | 最大反射強度 |

#### 2-4. RaycastLidarScanPublisherコンポーネントを追加

同じ`laser` GameObjectのInspectorで：

1. `Add Component`ボタンをクリック
2. `RaycastLidarScanPublisher`を検索して選択
3. 以下のパラメータを設定（多くは自動設定）：

| パラメータ | 設定値 | 説明 |
|-----------|--------|------|
| **Topic Name** | `/raycast_lidar/scan` | 自動設定（Topic.RaycastLidarScan） |
| **Frame Id** | `laser` | 自動設定（FrameId.RaycastLidarScanData） |
| **Scan Frequency** | `5.5` | スキャン周波数（Hz） |
| **Range Min** | `0.12` | 最小検出距離（m） |
| **Range Max** | `12.0` | 最大検出距離（m） |

**重要**: Frame Idは自動的に`laser`に設定されます。

---

### ステップ3: TF Publisherの設定

#### 3-1. TFPublisher GameObjectの作成

1. Hierarchyウィンドウで右クリック → `Create Empty`
2. 名前を`TFPublisher`に変更
3. 位置: Sceneのルート（推奨）

#### 3-2. RobotTFPublisherコンポーネントの追加

1. `TFPublisher` GameObjectを選択
2. Inspectorで`Add Component`をクリック
3. `RobotTFPublisher`を検索して選択

#### 3-3. パラメータ設定

Inspectorで以下のように設定：

**TF Configuration:**
- **Publish Frequency**: `10` (Hz)
- **Robot Game Object**: Hierarchyから`raspimouse_with_camera_rplidar`をドラッグ&ドロップ
- **Laser Game Object**: Hierarchyから`laser`をドラッグ&ドロップ
  - パス: `raspimouse_with_camera_rplidar > base_link > rplidar_multi_mount_link > laser`

**Frame IDs:**
- **Map Frame Id**: `map` (デフォルト値のまま)
- **Base Link Frame Id**: `base_link` (デフォルト値のまま)
- **Laser Frame Id**: `laser` (デフォルト値のまま)

#### 3-4. Sceneを保存

`File > Save Scene`（または Ctrl+S / Cmd+S）

---

### ステップ4: ROS2環境の準備

#### 4-1. Docker環境の起動

プロジェクトルートの`ros2_docker`ディレクトリで：

```bash
cd ros2_docker
docker compose up
```

#### 4-2. ROS-TCP-Endpointの起動

Docker環境内で新しいターミナルを開き：

```bash
# ワークスペースのsetup.bashをソース
source /home/ubuntu/colcon_ws/install/setup.bash

# ROS-TCP-Endpointを起動
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**期待される出力**:
```
[INFO] [ros_tcp_endpoint]: Starting ROS-TCP-Endpoint...
[INFO] [ros_tcp_endpoint]: Listening on port 10000
```

---

### ステップ5: Unity Play Modeでのテスト

#### 5-1. Unity Play Modeに入る

Unity Editorで：
1. LidarSlamSceneが開かれていることを確認
2. Playボタン（▶）をクリック

#### 5-2. 接続確認

Unity Consoleウィンドウで：
- エラーがないことを確認
- `[ROS] Connected to ROS`のようなメッセージを確認

---

### ステップ6: ROS2側での確認

#### 6-1. LaserScanトピックの確認

Docker環境内の別ターミナルで：

```bash
# トピックリストの確認
ros2 topic list
```

**期待される出力**:
```
/clock
/cmd_vel
/camera/rgb/image/compressed
/raycast_lidar/scan        ← 新しく追加されたトピック
/tf                        ← TFトピック
```

```bash
# トピック情報の確認
ros2 topic info /raycast_lidar/scan
```

**期待される出力**:
```
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 0
```

```bash
# 配信周波数の確認
ros2 topic hz /raycast_lidar/scan
```

**期待される出力**:
```
average rate: 5.501
    min: 0.180s max: 0.182s std dev: 0.001s window: 10
```

```bash
# データ内容の確認
ros2 topic echo /raycast_lidar/scan --once
```

**期待される出力**:
```yaml
header:
  stamp:
    sec: 123
    nanosec: 456789000
  frame_id: laser
angle_min: 0.0
angle_max: 6.28318...
angle_increment: 0.0174532...  # 約1度
range_min: 0.12
range_max: 12.0
ranges:
- 1.523456
- 1.634567
- ...
```

#### 6-2. TF確認

```bash
# TFデータが配信されているか
ros2 topic echo /tf --once
```

**期待される出力**: TransformStampedメッセージが2つ表示される（map→base_link、base_link→laser）

```bash
# TFツリーの可視化
ros2 run tf2_tools view_frames
```

**期待される結果**: `frames.pdf`が生成され、`map → base_link → laser`のツリーが確認できる

```bash
# TF変換の動作確認
ros2 run tf2_ros tf2_echo map base_link
```

**期待される結果**: ロボットのワールド座標位置がリアルタイムで表示される（エラーなし）

```bash
# base_link → laser の変換確認
ros2 run tf2_ros tf2_echo base_link laser
```

**期待される結果**: Z軸（高さ）約0.132m、180度回転が表示される

---

### ステップ7: RViz2での可視化

#### 7-1. RViz2起動

Docker環境内で：

```bash
rviz2
```

#### 7-2. LaserScanディスプレイの追加

1. 左下の`Add`ボタンをクリック
2. `By display type`タブで`LaserScan`を選択
3. `OK`をクリック

#### 7-3. LaserScan設定

`LaserScan`ディスプレイの設定：

| 項目 | 設定値 | 説明 |
|------|--------|------|
| **Topic** | `/raycast_lidar/scan` | スキャンデータのトピック |
| **Size (m)** | `0.05` | 点のサイズ |
| **Decay Time** | `10` | **重要**: スキャンデータの表示時間（秒） |
| **Color Transformer** | `Intensity` | 反射強度で色分け |

**Decay Timeの説明**:
- `0`: 最新のスキャンのみ表示（軌跡が残らない）
- `10`: 10秒間スキャンを保持（軌跡が見える）
- **推奨**: テスト時は10秒以上、SLAM時は5-10秒

#### 7-4. Fixed Frame設定

RViz2上部の`Fixed Frame`を選択：

**選択肢と違い**:

| Fixed Frame | 動作 | 用途 |
|-------------|------|------|
| `laser` | 点群がロボットと一緒に動く | センサー視点の確認 |
| `map` | 点群がワールド座標に固定される | 軌跡表示、SLAM |

**推奨**: `map`を選択

#### 7-5. 可視化の確認

**期待される動作** (Fixed Frame = map, Decay Time = 10):

1. Unity側でロボットを前進させる
2. RViz2で観察:
   - ✅ 過去のスキャンデータがワールド座標に残る
   - ✅ ロボットが移動しても、古いスキャンは元の位置に固定
   - ✅ 新しいスキャンが新しい位置に追加される
   - ✅ ロボットの移動軌跡が可視化される

**以前の動作との違い** (Fixed Frame = laser):
- ❌ 点群がロボットと一緒に動く（センサー視点）

#### 7-6. オプション設定

**TFフレームの可視化**:
1. `Add` → `By display type` → `TF`
2. フレーム間の矢印が表示される
3. map → base_link → laser の関係を確認可能

---

## トラブルシューティング

### `/raycast_lidar/scan`トピックが見つからない

**原因**: RaycastLidarScanPublisherが動作していない

**確認**:
1. Unity Consoleでエラーをチェック
2. `laser` GameObjectに`RaycastLidarScanPublisher`がアタッチされているか
3. ROS-TCP-Endpointが起動しているか
4. コンパイルエラーが解消されているか

---

### TFエラー

**症状**: `Invalid frame ID "map"` または `frame does not exist`

**原因と対処**:

#### 1. TFがまだ配信されていない
- Unity Play Modeが起動しているか確認
- `ros2 topic echo /tf --once`でデータが来るか確認

#### 2. TFPublisher GameObjectが未設定
- Sceneに`TFPublisher` GameObjectが存在するか確認
- RobotTFPublisherコンポーネントがアタッチされているか
- Robot GameObject、Laser GameObjectが設定されているか

#### 3. 起動タイミングの問題
- 一時的なエラー: 数秒待つと解消される
- エラーメッセージの後にデータが表示されれば正常

---

### RViz2で点群が表示されない

**確認手順**:
1. `Fixed Frame`が`map`または`laser`に設定されているか
2. LaserScanの`Topic`が`/raycast_lidar/scan`になっているか
3. `ros2 topic echo /raycast_lidar/scan --once`でデータが来ているか
4. TFが正常に配信されているか（`ros2 topic echo /tf --once`）

---

### 点群がロボットと一緒に動く

**症状**: ロボットが移動すると、点群も一緒に動く

**原因**: Fixed Frame = `laser`

**対処**:
1. RViz2の`Fixed Frame`を`map`に変更

---

### スキャンデータが全て0またはInf

**原因**: ScanPatternが正しく設定されていない

**確認**:
1. `RaycastLiDARSensor`の`Scan Pattern`フィールドが空でないか
2. `RPLidar_2D_360.asset`が正しく生成されているか
3. `Points Num Per Scan`が360に設定されているか

---

### Unity FPSが大幅に低下

**原因**: Raycast負荷が高い

**対策**:
1. `Points Num Per Scan`を180に減らす（2度刻み）
2. `Scan Frequency`を3Hzに下げる
3. より軽量な`DepthBufferLiDARSensor`を検討

---

## まとめ

### セットアップ完了のチェックリスト

- [ ] ScanPattern（RPLidar_2D_360）が作成されている
- [ ] RaycastLiDARSensorが設定されている
- [ ] RaycastLidarScanPublisherが設定されている
- [ ] TFPublisher GameObjectが設定されている
- [ ] Unity Play Modeでエラーが出ない
- [ ] `/raycast_lidar/scan`トピックが配信されている
- [ ] `/tf`トピックが配信されている
- [ ] TFツリー（map → base_link → laser）が構築されている
- [ ] RViz2でmap座標系での可視化ができる
- [ ] ロボット移動時に軌跡が表示される

### 達成できたこと

✅ Unity上のLidarセンサーデータをROS2に配信
✅ RViz2でmap座標系での可視化
✅ ロボット移動軌跡の確認
✅ SLAM準備完了

### 次のステップ

**SLAM統合** (別ドキュメントまたは次のステップ):
```bash
# SLAM Toolboxのインストール
sudo apt install ros-humble-slam-toolbox

# SLAM起動
ros2 launch slam_toolbox online_async_launch.py
```

RViz2に`Map`ディスプレイを追加すると、自動生成される地図を確認できます。

---

## 付録

### 実装済みファイル

本ガイドで使用するファイルは既に実装済みです：

- `Assets/Scripts/Robotics/Simulator/Publisher/RaycastLidarScanPublisher.cs`
  - LaserScanメッセージを生成・配信
- `Assets/Scripts/Robotics/Simulator/Publisher/RobotTFPublisher.cs`
  - TFフレーム（map→base_link→laser）を配信
- `Assets/Scripts/Robotics/Simulator/Publisher/Topic.cs`
  - トピック名定数（/raycast_lidar/scan, /tf）
- `Assets/Scripts/Robotics/Simulator/Publisher/FrameId.cs`
  - フレームID定数（map, base_link, laser）

### 参考資料

- [UnitySensors GitHub](https://github.com/Field-Robotics-Japan/UnitySensors)
- [ROS2 sensor_msgs/LaserScan](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)
- [ROS2 TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
