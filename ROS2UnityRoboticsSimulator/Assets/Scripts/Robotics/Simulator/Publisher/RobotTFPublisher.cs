using System.Collections.Generic;
using Robotics.Simulator.Core;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Robotics.Simulator.Publisher
{
    /// <summary>
    /// UnityからROS2へロボットのTransform情報（TF）を配信する
    /// SLAM（地図作成）に必要なフレーム間の位置関係を提供
    /// </summary>
    public class RobotTFPublisher : MonoBehaviour
    {
        [Header("TF Configuration")]
        [SerializeField] private float publishFrequency = 10f; // Hz (TFは通常10-30Hz)
        [SerializeField] private GameObject robotGameObject;   // base_linkに対応するGameObject
        [SerializeField] private GameObject laserGameObject;   // laserに対応するGameObject

        [Header("Frame IDs")]
        [SerializeField] private string mapFrameId = FrameId.Map;
        [SerializeField] private string baseLinkFrameId = FrameId.BaseLink;
        [SerializeField] private string laserFrameId = FrameId.Laser;

        private ROSConnection _rosConnection;
        private float _timeAccumulator = 0f; // 周波数制御用の時間蓄積

        private void Awake()
        {
            // ROS接続を取得し、/tfトピックを登録
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RegisterPublisher<TFMessageMsg>(Topic.TF);
        }

        private void Update()
        {
            // フレーム時間を蓄積
            _timeAccumulator += Time.deltaTime;

            // TF配信周期を計算（10Hz → 0.1秒/回）
            float publishPeriod = 1.0f / publishFrequency;

            // まだ次の配信タイミングに達していない場合は何もしない
            if (_timeAccumulator < publishPeriod)
            {
                return;
            }

            PublishTF();

            // 次のサイクルのために蓄積時間をリセット（余剰分は保持）
            _timeAccumulator -= publishPeriod;
        }

        private void PublishTF()
        {
            // GameObjectの参照チェック
            if (robotGameObject == null)
            {
                Debug.LogWarning("[RobotTFPublisher] Robot GameObject not assigned");
                return;
            }

            if (laserGameObject == null)
            {
                Debug.LogWarning("[RobotTFPublisher] Laser GameObject not assigned");
                return;
            }

            // ROS2タイムスタンプを生成
            var timeStamp = new TimeStamp(Clock.time);

            // 複数のTransformを格納するリスト
            var transforms = new List<TransformStampedMsg>();

            // Transform 1: map → base_link（ロボットのワールド座標位置）
            transforms.Add(CreateTransform(
                mapFrameId,
                baseLinkFrameId,
                robotGameObject.transform,
                timeStamp
            ));

            // Transform 2: base_link → laser（Lidarセンサーの相対位置）
            // laserGameObjectのワールド位置からbase_linkに対する相対変換を計算
            transforms.Add(CreateRelativeTransform(
                baseLinkFrameId,
                laserFrameId,
                robotGameObject.transform,
                laserGameObject.transform,
                timeStamp
            ));

            // TFメッセージを構築してpublish
            var tfMessage = new TFMessageMsg
            {
                transforms = transforms.ToArray()
            };

            _rosConnection.Publish(Topic.TF, tfMessage);
        }

        /// <summary>
        /// ワールド座標のTransformからTransformStampedMsgを作成（map → base_link用）
        /// </summary>
        private TransformStampedMsg CreateTransform(
            string parentFrame,
            string childFrame,
            Transform unityTransform,
            Robotics.Simulator.Core.TimeStamp timeStamp)
        {
            // Unity座標系（RUF: Right-Up-Forward）をROS座標系（FLU: Forward-Left-Up）に変換
            // Position: Unity(x, y, z) → ROS(z, -x, y)
            var rosPosition = new Vector3Msg
            {
                x = unityTransform.position.z,
                y = -unityTransform.position.x,
                z = unityTransform.position.y
            };

            // Rotation: Unity Quaternion → ROS Quaternion
            // Unity(x, y, z, w) → ROS(z, -x, y, -w)
            var unityRotation = unityTransform.rotation;
            var rosRotation = new QuaternionMsg
            {
                x = unityRotation.z,
                y = -unityRotation.x,
                z = unityRotation.y,
                w = -unityRotation.w
            };

            return new TransformStampedMsg
            {
                header = new HeaderMsg
                {
                    stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                    {
                        sec = timeStamp.Seconds,
                        nanosec = timeStamp.NanoSeconds
                    },
                    frame_id = parentFrame
                },
                child_frame_id = childFrame,
                transform = new TransformMsg
                {
                    translation = rosPosition,
                    rotation = rosRotation
                }
            };
        }

        /// <summary>
        /// 2つのGameObject間の相対TransformからTransformStampedMsgを作成（base_link → laser用）
        /// </summary>
        private TransformStampedMsg CreateRelativeTransform(
            string parentFrame,
            string childFrame,
            Transform parentTransform,
            Transform childTransform,
            Robotics.Simulator.Core.TimeStamp timeStamp)
        {
            // Unityワールド座標から相対座標を計算
            Vector3 relativePosition = parentTransform.InverseTransformPoint(childTransform.position);
            Quaternion relativeRotation = Quaternion.Inverse(parentTransform.rotation) * childTransform.rotation;

            // Unity座標系（RUF）をROS座標系（FLU）に変換
            var rosPosition = new Vector3Msg
            {
                x = relativePosition.z,
                y = -relativePosition.x,
                z = relativePosition.y
            };

            var rosRotation = new QuaternionMsg
            {
                x = relativeRotation.z,
                y = -relativeRotation.x,
                z = relativeRotation.y,
                w = -relativeRotation.w
            };

            return new TransformStampedMsg
            {
                header = new HeaderMsg
                {
                    stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                    {
                        sec = timeStamp.Seconds,
                        nanosec = timeStamp.NanoSeconds
                    },
                    frame_id = parentFrame
                },
                child_frame_id = childFrame,
                transform = new TransformMsg
                {
                    translation = rosPosition,
                    rotation = rosRotation
                }
            };
        }
    }
}
