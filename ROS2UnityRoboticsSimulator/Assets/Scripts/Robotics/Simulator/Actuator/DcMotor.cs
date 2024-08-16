using UnityEngine;

namespace Robotics.Simulator.Actuator
{
    public class DcMotor : MonoBehaviour
    {
        [SerializeField] private WheelCollider wheelCollider;

        [SerializeField] private float maxMotorTorque;

        private void FixedUpdate()
        {
            var motor = maxMotorTorque * Input.GetAxis("Vertical");
            wheelCollider.motorTorque = motor;
        }
    }
}