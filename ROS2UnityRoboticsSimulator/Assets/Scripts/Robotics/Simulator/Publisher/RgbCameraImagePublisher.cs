using Robotics.Simulator.Core;
using Robotics.Simulator.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

// https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/publisher.md
// https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/Nav2SLAMExampleProject/Assets/Scripts/LaserScanSensor.cs
// https://zenn.dev/tasada038/articles/690009e63ff223#1.3%3A-joint_states%E3%81%A7unity%E4%B8%8A%E3%81%AEurdf%E3%82%92%E5%8B%95%E3%81%8B%E3%81%99
// https://github.com/taigamaru/donkey-my-sim/blob/main/Assets/Script/RGBCameraPublisher.cs
// https://github.com/Ar-Ray-code/RenderTexture2ROS2Image/blob/main/Assets/RenderTexture2ROS2Image.cs

namespace Robotics.Simulator.Publisher
{
    [RequireComponent(typeof(RgbCamera))]
    public class RgbCameraImagePublisher : MonoBehaviour
    {
        [SerializeField] private string topicName = Topic.RgbCameraCompressedImage;
        [SerializeField] private string frameId = FrameId.RgbCameraData;

        private ROSConnection _rosConnection;
        private RgbCamera _rgbCamera;

        private void Awake()
        {
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RegisterPublisher<CompressedImageMsg>(topicName);
            _rgbCamera = GetComponent<RgbCamera>();
        }

        private void Update()
        {
            _rgbCamera.LoadTexture(data =>
            {
                var timeStamp = new TimeStamp(Clock.time);
                var compressedImageMsg = new CompressedImageMsg
                {
                    header = new HeaderMsg
                    {
                        frame_id = frameId,
                        stamp = new TimeMsg
                        {
                            sec = timeStamp.Seconds,
                            nanosec = timeStamp.NanoSeconds
                        }
                    },
                    // RgbCamera側がEncodeToJPGにしているためjpegに設定
                    format = "jpeg",
                    data = data
                };
                _rosConnection.Publish(topicName, compressedImageMsg);
            }, error => Debug.LogError(error));
        }
    }
}