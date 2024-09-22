using System.Linq;
using Robotics.Simulator.Core;
using Robotics.Simulator.Core.Model;
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

        private float leftWheelSpeed;
        private float rightWheelSpeed;
        private float linearSpeed; // m/s
        private float angularSpeed; // rad/s
        private float theta; // degrees

        private void Awake()
        {
            DisableArticulationBody();
            AddRigidBody();
        }

        private void FixedUpdate()
        {
            var deltaTimeSeconds = Clock.DeltaTimeInSeconds;

            if (mode == ControlMode.Keyboard)
            {
                KeyboardUpdate();
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

        private void KeyboardUpdate()
        {
            linearSpeed = 0;
            angularSpeed = 0;

            if (Input.GetKey(KeyCode.UpArrow))
            {
                linearSpeed = maxLinearSpeed;
            }
            else if (Input.GetKey(KeyCode.DownArrow))
            {
                linearSpeed = -maxLinearSpeed;
            }

            if (Input.GetKey(KeyCode.LeftArrow))
            {
                angularSpeed = -maxAngularSpeed;
            }
            else if (Input.GetKey(KeyCode.RightArrow))
            {
                angularSpeed = maxAngularSpeed;
            }
        }

        private float x;
        private float z;

        private RobotTransform CalcRobotTransform(float deltaTimeSeconds)
        {
            leftWheelSpeed = linearSpeed - (angularSpeed * trackWidth / 2.0f);
            rightWheelSpeed = linearSpeed + (angularSpeed * trackWidth / 2.0f);

            var robotLinerSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0f; // m/s
            var robotAngularSpeed = (rightWheelSpeed - leftWheelSpeed) / trackWidth; // rad/s

            var currentRobotPosition = robot.transform.position;
            var currentRobotRotation = robot.transform.rotation;

            var deltaX = robotLinerSpeed * Mathf.Sin(theta * Mathf.Deg2Rad) * deltaTimeSeconds;
            var deltaZ = robotLinerSpeed * Mathf.Cos(theta * Mathf.Deg2Rad) * deltaTimeSeconds;

            var newRobotPosition = new RobotPosition(
                currentRobotPosition.x + deltaX,
                currentRobotPosition.y,
                currentRobotPosition.z + deltaZ
            );

            theta += robotAngularSpeed * Mathf.Rad2Deg * deltaTimeSeconds;

            var newRobotRotation = new RobotRotation(
                currentRobotRotation.x,
                theta,
                currentRobotRotation.z
            );

            return new RobotTransform(newRobotPosition, newRobotRotation);
        }

        private void RotateWheels(float deltaTimeSeconds)
        {
            var leftWheelRotation = Quaternion.Euler(0, leftWheelSpeed * Mathf.Rad2Deg * deltaTimeSeconds, 0);
            var rightWheelRotation = Quaternion.Euler(0, rightWheelSpeed * Mathf.Rad2Deg * deltaTimeSeconds, 0);
            leftWheel.transform.rotation *= leftWheelRotation;
            rightWheel.transform.rotation *= rightWheelRotation;
        }
    }
}