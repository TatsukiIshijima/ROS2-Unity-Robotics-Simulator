using System.Collections.Generic;
using Robotics.Simulator.Production.DemoCar.Param;
using UnityEngine;

namespace Robotics.Simulator.Production.DemoCar
{
    public class DemoCarController : MonoBehaviour
    {
        public List<AxleInfo> axleInfos;
        public float maxMotorTorque;
        public float maxSteeringAngle;

        private void ApplyLocalPositionToVisuals(WheelCollider wheelCollider)
        {
            if (wheelCollider.transform.childCount == 0)
            {
                return;
            }
            
            var visualWheel = wheelCollider.transform.GetChild(0);

            wheelCollider.GetWorldPose(out var position, out var rotation);

            visualWheel.transform.position = position;
            visualWheel.transform.rotation = rotation;
        }
        
        public void FixedUpdate()
        {
            var motor = maxMotorTorque * Input.GetAxis("Vertical");
            var steering = maxSteeringAngle * Input.GetAxis("Horizontal");

            foreach (var axleInfo in axleInfos)
            {
                if (axleInfo.steering)
                {
                    axleInfo.leftWheel.steerAngle = steering;
                    axleInfo.rightWheel.steerAngle = steering;
                }

                if (axleInfo.motor)
                {
                    axleInfo.leftWheel.motorTorque = motor;
                    axleInfo.rightWheel.motorTorque = motor;
                }
                ApplyLocalPositionToVisuals(axleInfo.leftWheel);
                ApplyLocalPositionToVisuals(axleInfo.rightWheel);
            }
        }
    }
}
