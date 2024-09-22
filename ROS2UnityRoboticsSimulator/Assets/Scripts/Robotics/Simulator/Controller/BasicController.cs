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
        [SerializeField] private float maxMotorTorque = 0.25f; // N/m
        [SerializeField] private float wheelMass = 0.0113f; // kg
        [SerializeField] private float wheelRadius = 0.024f; // meters
        [SerializeField] private float trackWidth = 0.08f; // meters Distance between tyres

        private float _leftTorque = 0f; // N/m
        private float _rightTorque = 0f; // N/m
        private float _inertia; // kg*m^2

        private void Awake()
        {
            DisableArticulationBody();

            // cylinder inertia = 0.5 * m * r^2
            _inertia = 0.5f * wheelMass * Mathf.Pow(wheelRadius, 2);
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

        private void KeyboardUpdate()
        {
            var moveDirection = Input.GetAxis("Vertical");
            var rotateDirection = Input.GetAxis("Horizontal");

            switch (moveDirection)
            {
                case 0:
                    _leftTorque = 0;
                    _rightTorque = 0;
                    break;
                case > 0:
                    _leftTorque = maxMotorTorque;
                    _rightTorque = maxMotorTorque;
                    break;
                case < 0:
                    _leftTorque = -1 * maxMotorTorque;
                    _rightTorque = -1 * maxMotorTorque;
                    break;
            }

            switch (rotateDirection)
            {
                case 0:
                    break;
                case > 0:
                    _leftTorque = maxMotorTorque;
                    _rightTorque = -1 * maxMotorTorque;
                    break;
                case < 0:
                    _leftTorque = -1 * maxMotorTorque;
                    _rightTorque = maxMotorTorque;
                    break;
            }
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

        private float CalculateWheelAngularVelocity(float deltaTimeSeconds, float torque)
        {
            var angularAcceleration = torque / _inertia; // rad/s^2
            return angularAcceleration * deltaTimeSeconds; // rad/s
        }

        private float CalculateWheelLinearVelocity(float angularVelocity)
        {
            return angularVelocity * wheelRadius; // m/s
        }

        private RobotTransform CalcRobotTransform(float deltaTimeSeconds)
        {
            var leftWheelAngularVelocity = CalculateWheelAngularVelocity(deltaTimeSeconds, _leftTorque); // rad/s
            var rightWheelAngularVelocity = CalculateWheelAngularVelocity(deltaTimeSeconds, _rightTorque); // rad/s
            var leftWheelLinearVelocity = CalculateWheelLinearVelocity(leftWheelAngularVelocity); // m/s
            var rightWheelLinearVelocity = CalculateWheelLinearVelocity(rightWheelAngularVelocity); // m/s

            var robotVelocity = (leftWheelLinearVelocity + rightWheelLinearVelocity) / 2; // m/s
            var robotAngularVelocity = (rightWheelLinearVelocity - leftWheelLinearVelocity) / trackWidth; // rad/s

            var deltaTheta = robotAngularVelocity * deltaTimeSeconds; // rad
            var deltaX = robotVelocity * deltaTimeSeconds * Mathf.Cos(deltaTheta);
            var deltaZ = robotVelocity * deltaTimeSeconds * Mathf.Sin(deltaTheta);

            var x = robot.transform.position.x + deltaX;
            var z = robot.transform.position.z + deltaZ;
            var theta = (Mathf.Deg2Rad * robot.transform.rotation.y) + deltaTheta; // rad
            var rotation = (Mathf.Rad2Deg * theta); // deg

            return new RobotTransform(
                new RobotPosition(x, robot.transform.position.y, z),
                new RobotRotation(robot.transform.rotation.x, rotation, robot.transform.rotation.z)
            );
        }
    }
}