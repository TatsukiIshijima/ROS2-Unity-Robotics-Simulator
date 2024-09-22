using System;
using System.Linq;
using Robotics.Simulator.Core;
using Robotics.Simulator.Core.Model;
using Unity.Robotics.UrdfImporter;
using UnityEngine;
using UnityEngine.Serialization;
using Util;

namespace Robotics.Simulator.Controller
{
    public class BasicController : MonoBehaviour
    {
        [SerializeField] private GameObject robot;
        [SerializeField] private ControlMode mode = ControlMode.Keyboard;
        [SerializeField] private float robotMass = 0.75f; // kg
        [SerializeField] private float maxLinearSpeed = 0.25f; // m/s
        [SerializeField] private float maxAngularSpeed = 1.0f; // rad/s
        // [SerializeField] private float wheelRadius = 0.024f; // meters
        [SerializeField] private float trackWidth = 0.08f; // meters Distance between tyres
        
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

        private RobotTransform CalcRobotTransform(float deltaTimeSeconds)
        {
            var leftWheelSpeed = linearSpeed - (angularSpeed * trackWidth / 2.0f);
            var rightWheelSpeed = linearSpeed + (angularSpeed * trackWidth / 2.0f);
            
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
    }
}