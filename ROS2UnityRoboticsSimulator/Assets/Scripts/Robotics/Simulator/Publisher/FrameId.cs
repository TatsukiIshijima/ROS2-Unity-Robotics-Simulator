namespace Robotics.Simulator.Publisher
{
    public static class FrameId
    {
        public const string RgbCameraData = "rgb_camera_data";
        public const string InfraredRangeData = "infrared_range_data";

        // TF Frame IDs for SLAM
        public const string Map = "map";
        public const string BaseLink = "base_link";
        public const string Laser = "laser";
    }
}