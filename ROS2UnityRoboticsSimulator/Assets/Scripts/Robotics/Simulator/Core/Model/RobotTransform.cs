namespace Robotics.Simulator.Core.Model
{
    public readonly struct RobotTransform
    {
        public readonly RobotPosition Position;
        public readonly RobotRotation Rotation;

        public RobotTransform(RobotPosition position, RobotRotation rotation)
        {
            Position = position;
            Rotation = rotation;
        }
    }  
};