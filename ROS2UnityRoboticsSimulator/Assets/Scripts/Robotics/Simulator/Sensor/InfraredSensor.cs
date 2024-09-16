using UnityEngine;

namespace Robotics.Simulator.Sensor
{
    public class InfraredSensor : MonoBehaviour
    {
        [SerializeField] private float minRange = 0.1f;
        [SerializeField] private float maxRange = 0.8f;
        
        public float MinRange => minRange;
        public float MaxRange => maxRange;
        
        internal float LoadDistance()
        {
            var forward = transform.TransformDirection(Vector3.forward);
            var distance = Physics.Raycast(transform.position, forward, out var hit, maxRange)
                ? hit.distance
                : float.NegativeInfinity;
            return distance;
        }
    }
}