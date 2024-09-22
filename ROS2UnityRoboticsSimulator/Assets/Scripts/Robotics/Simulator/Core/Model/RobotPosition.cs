namespace Robotics.Simulator.Core.Model
{
    public readonly struct RobotPosition
    {
        private readonly float x;
        private readonly float y;
        private readonly float z;
        
        public RobotPosition(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        
        public UnityEngine.Vector3 ToVector3()
        {
            return new UnityEngine.Vector3(x, y, z);
        }
    }
}