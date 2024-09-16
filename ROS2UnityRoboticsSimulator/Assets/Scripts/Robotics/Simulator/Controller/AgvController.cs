using System;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;

// https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/Nav2SLAMExampleProject/Assets/Scripts/AGVController.cs

namespace Robotics.Simulator.Controller
{
    internal enum ControlMode
    {
        Keyboard,
        Ros
    };

    public class AgvController : MonoBehaviour
    {
        [SerializeField] private GameObject wheel1;
        [SerializeField] private GameObject wheel2;
        [SerializeField] private ControlMode mode = ControlMode.Keyboard;
        private ArticulationBody _wA1;
        private ArticulationBody _wA2;

        [SerializeField] private float maxLinearSpeed = 2; // m/s
        [SerializeField] private float maxRotationalSpeed = 1; //
        [SerializeField] private float wheelRadius = 0.025f; // meters
        [SerializeField] private float trackWidth = 0.1f; // meters Distance between tyres
        [SerializeField] private float forceLimit = 10;
        [SerializeField] private float damping = 10;
        // [SerializeField] private float rosTimeout = 0.5f;
        // private float lastCmdReceived = 0f;

        private RotationDirection direction;
        // private float rosLinear = 0f;
        // private float rosAngular = 0f;

        private void Awake()
        {
            _wA1 = wheel1.GetComponent<ArticulationBody>();
            _wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(_wA1);
            SetParameters(_wA2);
        }

        private void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard)
            {
                KeyboardUpdate();
            }
        }

        private void SetParameters(ArticulationBody joint)
        {
            var drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            var drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed / wheelRadius) * Mathf.Rad2Deg * (int)direction);
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }

            joint.xDrive = drive;
        }

        private void KeyboardUpdate()
        {
            var moveDirection = Input.GetAxis("Vertical");
            var turnDirection = Input.GetAxis("Horizontal");
            var inputSpeed = moveDirection switch
            {
                > 0 => maxLinearSpeed,
                < 0 => maxLinearSpeed * -1,
                _ => 0
            };
            var inputRotationSpeed = turnDirection switch
            {
                > 0 => maxRotationalSpeed,
                < 0 => maxRotationalSpeed * -1,
                _ => 0
            };
            RobotInput(inputSpeed, inputRotationSpeed);
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }

            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }

            var wheel1Rotation = speed / wheelRadius;
            var wheel2Rotation = wheel1Rotation;
            var wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }

            SetSpeed(_wA1, wheel1Rotation);
            SetSpeed(_wA2, wheel2Rotation);
        }
    }
}