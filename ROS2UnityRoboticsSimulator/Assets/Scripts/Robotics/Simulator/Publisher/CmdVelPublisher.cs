using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Robotics.Simulator.Publisher
{
    /**
     * https://github.com/unity3d-jp/Unity-ROS-MobileRobot-UI-Tutorial/blob/main/MobileRobotUITutorialProject/Assets/Scripts/CmdVelPublisher.cs
     */
    public class CmdVelPublisher : MonoBehaviour
    {
        [SerializeField] private string topicName = "cmd_vel";
        [SerializeField] private float linearSpeed = 0.5f;
        [SerializeField] private float angularSpeed = 1.0f;

        private ROSConnection _rosConnection;
        private readonly TwistMsg _twistMsg = new();

        private void Awake()
        {
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RegisterPublisher<TwistMsg>(topicName);
        }

        public void Forward(float ratio = 1.0f)
        {
            _twistMsg.linear.x = linearSpeed * ratio;
        }

        public void Backward(float ratio = 1.0f)
        {
            _twistMsg.linear.x = -linearSpeed * ratio;
        }

        public void TurnLeft(float ratio = 1.0f)
        {
            _twistMsg.angular.z = -angularSpeed * ratio;
        }

        public void TurnRight(float ratio = 1.0f)
        {
            _twistMsg.angular.z = angularSpeed * ratio;
        }
        
        public void Stop()
        {
            _twistMsg.linear.x = 0;
            _twistMsg.angular.z = 0;
        }

        public void Publish()
        {
            _rosConnection.Publish(topicName, _twistMsg);
        }
    }
}