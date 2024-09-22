namespace Robotics.Simulator.Core.Model
{
    public readonly struct RobotRotation
    {
        private readonly float x;
        private readonly float y;
        private readonly float z;
        
        public RobotRotation(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        
        public UnityEngine.Quaternion ToQuaternion()
        {
            return UnityEngine.Quaternion.Euler(x, y, z);
        }
    }
}