using System.Linq;
using Robotics.Simulator.Core;
using Robotics.Simulator.Core.Model;
using Robotics.Simulator.Publisher;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using Util;

namespace Robotics.Simulator.Controller
{
    /**
     * BasicController is a simple controller that moves the robot based on keyboard input or ROS.
     * This script controls the position and rotation of the robot, ignoring friction and other physical effects.
     */
    public class BasicController : MonoBehaviour
    {
        [SerializeField] private GameObject robot;
        [SerializeField] private GameObject leftWheel;
        [SerializeField] private GameObject rightWheel;
        [SerializeField] private ControlMode mode = ControlMode.Keyboard;
        [SerializeField] private float robotMass = 0.75f; // kg
        [SerializeField] private float maxLinearSpeed = 0.5f; // m/s
        [SerializeField] private float maxAngularSpeed = 1.0f; // rad/s
        [SerializeField] private float trackWidth = 0.08f; // meters Distance between tyres

        private float _leftWheelSpeed;
        private float _rightWheelSpeed;
        private float _robotLinearSpeed; // m/s
        private float _robotAngularSpeed; // rad/s
        private float _theta; // degrees

        private ROSConnection _rosConnection;
        private float _lastCmdReceivedTime;
        private const string CmdVelTopicName = Topic.CmdVel;
        private const float RosTimeout = 0.5f;

        private void Awake()
        {
            DisableArticulationBody();
            AddRigidBody();
        }

        private void Start()
        {
            if (mode != ControlMode.Ros) return;
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.Subscribe<TwistMsg>(CmdVelTopicName, OnCmdVelReceived);
        }

        private void FixedUpdate()
        {
            var deltaTimeSeconds = Clock.DeltaTimeInSeconds;

            switch (mode)
            {
                case ControlMode.Keyboard:
                    KeyboardUpdate(deltaTimeSeconds);
                    break;
                case ControlMode.Ros:
                    RosUpdate(deltaTimeSeconds);
                    break;
                default:
                    Debug.LogError("Unknown control mode");
                    break;
            }

            var newTransform = CalcRobotTransform(deltaTimeSeconds);
            robot.transform.position = newTransform.Position.ToVector3();
            robot.transform.rotation = newTransform.Rotation.ToQuaternion();
            RotateWheels(deltaTimeSeconds);
        }

        private void DisableArticulationBody()
        {
            var robotComponents = ChildFinder.GetAllChildren(robot);
            foreach (var component in robotComponents.Where(component =>
                         component.GetComponent<ArticulationBody>() != null))
            {
                component.GetComponent<ArticulationBody>().enabled = false;
            }
        }

        private void AddRigidBody()
        {
            var rigidBody = robot.AddComponent<Rigidbody>();
            rigidBody.mass = robotMass;
        }

        private void OnCmdVelReceived(TwistMsg twistMsg)
        {
            _robotLinearSpeed = (float)twistMsg.linear.x;
            _robotAngularSpeed = (float)twistMsg.angular.z;
            _lastCmdReceivedTime = Time.time;
        }

        private void RosUpdate(float deltaTimeSeconds)
        {
            if (Time.time - _lastCmdReceivedTime > RosTimeout)
            {
                _robotLinearSpeed = 0;
                _robotAngularSpeed = 0;
            }

            UpdateRobotTransform(deltaTimeSeconds);
        }

        private void KeyboardUpdate(float deltaTimeSeconds)
        {
            _robotLinearSpeed = 0;
            _robotAngularSpeed = 0;

            if (Input.GetKey(KeyCode.UpArrow))
            {
                _robotLinearSpeed = maxLinearSpeed;
            }
            else if (Input.GetKey(KeyCode.DownArrow))
            {
                _robotLinearSpeed = -maxLinearSpeed;
            }

            if (Input.GetKey(KeyCode.LeftArrow))
            {
                _robotAngularSpeed = -maxAngularSpeed;
            }
            else if (Input.GetKey(KeyCode.RightArrow))
            {
                _robotAngularSpeed = maxAngularSpeed;
            }

            UpdateRobotTransform(deltaTimeSeconds);
        }

        private void UpdateRobotTransform(float deltaTimeSeconds)
        {
            var newTransform = CalcRobotTransform(deltaTimeSeconds);
            robot.transform.position = newTransform.Position.ToVector3();
            robot.transform.rotation = newTransform.Rotation.ToQuaternion();
            RotateWheels(deltaTimeSeconds);
        }

        private RobotTransform CalcRobotTransform(float deltaTimeSeconds)
        {
            _leftWheelSpeed = _robotLinearSpeed - (_robotAngularSpeed * trackWidth / 2.0f);
            _rightWheelSpeed = _robotLinearSpeed + (_robotAngularSpeed * trackWidth / 2.0f);

            var robotLinerSpeed = (_leftWheelSpeed + _rightWheelSpeed) / 2.0f; // m/s
            var robotAngularSpeed = (_rightWheelSpeed - _leftWheelSpeed) / trackWidth; // rad/s

            var currentRobotPosition = robot.transform.position;
            var currentRobotRotation = robot.transform.rotation;

            var deltaX = robotLinerSpeed * Mathf.Sin(_theta * Mathf.Deg2Rad) * deltaTimeSeconds;
            var deltaZ = robotLinerSpeed * Mathf.Cos(_theta * Mathf.Deg2Rad) * deltaTimeSeconds;

            var newRobotPosition = new RobotPosition(
                currentRobotPosition.x + deltaX,
                currentRobotPosition.y,
                currentRobotPosition.z + deltaZ
            );

            _theta += robotAngularSpeed * Mathf.Rad2Deg * deltaTimeSeconds;

            var newRobotRotation = new RobotRotation(
                currentRobotRotation.x,
                _theta,
                currentRobotRotation.z
            );

            return new RobotTransform(newRobotPosition, newRobotRotation);
        }

        private void RotateWheels(float deltaTimeSeconds)
        {
            var leftWheelRotation = Quaternion.Euler(0, _leftWheelSpeed * Mathf.Rad2Deg * deltaTimeSeconds, 0);
            var rightWheelRotation = Quaternion.Euler(0, _rightWheelSpeed * Mathf.Rad2Deg * deltaTimeSeconds, 0);
            leftWheel.transform.rotation *= leftWheelRotation;
            rightWheel.transform.rotation *= rightWheelRotation;
        }
    }
}