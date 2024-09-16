using System;
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
        [SerializeField] private string frameId;

        private ROSConnection _rosConnection;
        private InfraredSensor infraredSensor;
        
        private void Awake()
        {
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RegisterPublisher<RangeMsg>(topicName);
            infraredSensor = GetComponent<InfraredSensor>();
        }

        private void Update()
        {
            var range = infraredSensor.LoadDistance();
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
                min_range = infraredSensor.MinRange,
                max_range = infraredSensor.MaxRange,
                range = range,
            };
            _rosConnection.Publish(topicName, rangeMsg);
        }
    }
}