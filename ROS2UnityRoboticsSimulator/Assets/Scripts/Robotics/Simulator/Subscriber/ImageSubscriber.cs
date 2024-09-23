using System;
using Robotics.Simulator.Publisher;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Robotics.Simulator.Subscriber
{
    /**
     * https://github.com/unity3d-jp/Unity-ROS-MobileRobot-UI-Tutorial/blob/main/MobileRobotUITutorialProject/Assets/Scripts/ImageSubscriber.cs
     */
    public class ImageSubscriber : MonoBehaviour
    {
        [SerializeField] private string topicName = Topic.RgbCameraCompressedImage;
        [SerializeField] private bool isDebug = true;

        private ROSConnection _rosConnection;
        private bool _isImageLoading;

        public event Action<Texture2D> OnLoadImage;

        private void Awake()
        {
            _rosConnection = ROSConnection.GetOrCreateInstance();
        }

        private void Start()
        {
            _rosConnection.Subscribe<CompressedImageMsg>(topicName, OnReceiveImageData);
        }

        private void OnReceiveImageData(CompressedImageMsg imageMsg)
        {
            if (_isImageLoading) return;

            if (isDebug)
            {
                Debug.Log($"rawImage received. length : {Buffer.ByteLength(imageMsg.data)}");
            }

            LoadTexture(imageMsg);
        }

        private void LoadTexture(CompressedImageMsg imageMsg)
        {
            _isImageLoading = true;
            var texture = new Texture2D(1, 1);
            texture.LoadImage(imageMsg.data);
            texture.Apply();
            OnLoadImage?.Invoke(texture);
            _isImageLoading = false;
        }
    }
}