using Robotics.Simulator.Core;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnitySensors.Sensor.LiDAR;

// Reference:
// https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/Nav2SLAMExampleProject/Assets/Scripts/LaserScanSensor.cs

namespace Robotics.Simulator.Publisher
{
    /// <summary>
    /// RaycastLiDARSensorが生成した3DポイントクラウドをROS2のLaserScan形式(2D)に変換してpublishする
    /// </summary>
    [RequireComponent(typeof(RaycastLiDARSensor))]
    public class RaycastLidarScanPublisher : MonoBehaviour
    {
        [Header("ROS Configuration")]
        [SerializeField] private string topicName = Topic.RaycastLidarScan;
        [SerializeField] private string frameId = FrameId.Laser;

        [Header("LaserScan Configuration")]
        [SerializeField] private float scanFrequency = 5.5f; // Hz (RP-LIDAR A2: ~5.5Hz)
        [SerializeField] private float rangeMin = 0.12f;     // 最小検出距離（m）: RP-LIDAR A2仕様
        [SerializeField] private float rangeMax = 12.0f;     // 最大検出距離（m）: RP-LIDAR A2仕様

        private ROSConnection _rosConnection;
        private RaycastLiDARSensor _lidarSensor;
        private float _timeAccumulator = 0f; // 周波数制御用の時間蓄積
        
        private void Awake()
        {
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RegisterPublisher<LaserScanMsg>(topicName);
            _lidarSensor = GetComponent<RaycastLiDARSensor>();
        }
        
        private void Update()
        {
            // フレーム時間を蓄積（Time.deltaTime = 前フレームからの経過時間）
            _timeAccumulator += Time.deltaTime;

            // スキャン周期を計算（5.5Hz → 約0.18秒/スキャン）
            float scanPeriod = 1.0f / scanFrequency;

            // まだ次のスキャンタイミングに達していない場合は何もしない
            if (_timeAccumulator < scanPeriod)
            {
                return;
            }
            
            PublishLaserScan();

            // 次のサイクルのために蓄積時間をリセット（余剰分は保持して精度向上）
            _timeAccumulator -= scanPeriod;
        }

        private void PublishLaserScan()
        {
            // RaycastLiDARSensorから生成されたポイントクラウドを取得
            var pointCloud = _lidarSensor.pointCloud;

            // ポイントクラウドが準備できているか確認
            // (Unity Jobs Systemで非同期生成されるため、初期フレームではnullの可能性)
            if (pointCloud.points == null || !pointCloud.points.IsCreated)
            {
                Debug.LogWarning("[RaycastLidarScanPublisher] LiDAR point cloud not ready");
                return;
            }

            // スキャンポイント数を取得（例: 360点 = 1度刻み）
            int numPoints = _lidarSensor.pointsNum;
            if (numPoints == 0)
            {
                Debug.LogWarning("[RaycastLidarScanPublisher] No points in scan");
                return;
            }

            // LaserScanメッセージ用の配列を準備
            float[] ranges = new float[numPoints];      // 各角度での距離
            float[] intensities = new float[numPoints]; // 各角度での反射強度

            // ポイントクラウド(3D)からLaserScan(2D)への変換
            // 各点のXYZ座標からユークリッド距離を計算
            for (int i = 0; i < numPoints; i++)
            {
                var point = pointCloud.points[i]; // PointXYZI構造体 (position.x, position.y, position.z, intensity)

                // 3D座標から原点までの距離を計算（ピタゴラスの定理）
                // range = √(x² + y² + z²)
                float range = Mathf.Sqrt(
                    point.position.x * point.position.x +
                    point.position.y * point.position.y +
                    point.position.z * point.position.z
                );

                ranges[i] = range;                    // 距離配列に格納
                intensities[i] = point.intensity;     // 反射強度をそのまま格納
            }

            // LaserScanの角度パラメータを計算
            // ROS規約: 反時計回り、0度 = ロボット前方（+X方向）
            float angleMin = 0f;                              // 開始角度: 0 rad (0度)
            float angleMax = 2f * Mathf.PI;                   // 終了角度: 2π rad (360度)
            float angleIncrement = (angleMax - angleMin) / numPoints; // 角度刻み（360点なら約0.0175 rad = 1度）

            // ROS2タイムスタンプを生成（Unity時間をROS時間に変換）
            var timeStamp = new TimeStamp(Clock.time);

            // LaserScanメッセージを構築
            var laserScanMsg = new LaserScanMsg
            {
                // ヘッダー情報
                header = new HeaderMsg
                {
                    frame_id = frameId,  // TFフレーム名（"laser"）
                    stamp = new TimeMsg
                    {
                        sec = timeStamp.Seconds,        // 秒
                        nanosec = timeStamp.NanoSeconds // ナノ秒
                    }
                },

                // スキャンの角度範囲
                angle_min = angleMin,           // 0.0 rad
                angle_max = angleMax,           // 6.28 rad (≈360度)
                angle_increment = angleIncrement, // 1点あたりの角度増分

                // タイミング情報
                time_increment = (1.0f / scanFrequency) / numPoints, // 点間の時間（秒）
                scan_time = 1.0f / scanFrequency, // 1スキャンにかかる時間（秒）

                // 距離の有効範囲（センサー仕様）
                range_min = rangeMin, // 最小検出距離（例: 0.12m）
                range_max = rangeMax, // 最大検出距離（例: 12.0m）

                // 測定データ
                ranges = ranges,           // 距離配列（numPoints個）
                intensities = intensities  // 強度配列（numPoints個）
            };

            // ROS2の/scanトピックにpublish
            _rosConnection.Publish(topicName, laserScanMsg);
        }
    }
}
