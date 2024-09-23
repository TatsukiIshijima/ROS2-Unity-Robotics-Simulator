using Robotics.Simulator.Core;
using Robotics.Simulator.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Robotics.Simulator.Publisher
{
    [RequireComponent(typeof(InfraredSensor))]
    public class InfraredRangePublisher : MonoBehaviour
    {
        [SerializeField] private string topicName;
        [SerializeField] private string frameId = FrameId.InfraredRangeData;

        private ROSConnection _rosConnection;
        private InfraredSensor _infraredSensor;
        
        private void Awake()
        {
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RegisterPublisher<RangeMsg>(topicName);
            _infraredSensor = GetComponent<InfraredSensor>();
        }

        private void Update()
        {
            var range = _infraredSensor.LoadDistance();
            var timeStamp = new TimeStamp(Clock.time);
            var rangeMsg = new RangeMsg
            {
                header = new HeaderMsg
                {
                    frame_id = frameId,
                    stamp = new TimeMsg
                    {
                        sec = timeStamp.Seconds,
                        nanosec = timeStamp.NanoSeconds
                    }
                },
                radiation_type = RangeMsg.INFRARED,
                field_of_view = 0.1f,
                min_range = _infraredSensor.MinRange,
                max_range = _infraredSensor.MaxRange,
                range = range,
            };
            _rosConnection.Publish(topicName, rangeMsg);
        }
    }
}