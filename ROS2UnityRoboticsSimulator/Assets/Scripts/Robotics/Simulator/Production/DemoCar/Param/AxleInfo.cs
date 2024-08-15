using UnityEngine;

namespace Robotics.Simulator.Production.DemoCar.Param
{
    [System.Serializable]
    public class AxleInfo
    {
        public WheelCollider leftWheel;
        public WheelCollider rightWheel;
        public bool motor;
        public bool steering;
    }
}