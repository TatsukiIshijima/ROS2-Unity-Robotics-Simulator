using System;
using UnityEngine;

namespace Robotics.Simulator.Actuator
{
    public class ServoMotor : MonoBehaviour
    {
        [SerializeField] private WheelCollider wheelCollider;

        [SerializeField] private float maxSteeringAngle;

        private void FixedUpdate()
        {
            var steer = maxSteeringAngle * Input.GetAxis("Horizontal");
            wheelCollider.steerAngle = steer;
        }
    }
}