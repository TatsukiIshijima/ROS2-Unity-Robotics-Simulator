using System;
using Robotics.Simulator.Core;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

// https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/blob/main/Nav2SLAMExampleProject/Assets/Scripts/ROSClockPublisher.cs

namespace Robotics.Simulator
{
    public class RosClockPublisher : MonoBehaviour
    {
        [SerializeField] private Clock.ClockMode clockMode;

        [SerializeField, HideInInspector] private Clock.ClockMode lastSetClockMode;

        [SerializeField] private double publishRateHz = 100f;

        private double _lastPublishTimeSeconds;

        private ROSConnection _rosConnection;

        private double PublishPeriodSeconds => 1.0f / publishRateHz;

        private bool ShouldPublishMessage =>
            Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > _lastPublishTimeSeconds;

        /**
         * スクリプトがロードされた時やインスペクターの値が変更された時に呼び出される
         * https://docs.unity3d.com/ja/2018.4/ScriptReference/MonoBehaviour.OnValidate.html
         */
        private void OnValidate()
        {
            var clocks = FindObjectsOfType<RosClockPublisher>();
            if (clocks.Length > 1)
            {
                Debug.LogWarning("Found too many clock publishers in the scene, there should only be one!");
            }

            if (Application.isPlaying && lastSetClockMode != clockMode)
            {
                Debug.LogWarning("Can't change ClockMode during simulation! Setting it back...");
                clockMode = lastSetClockMode;
            }

            SetClockMode(clockMode);
        }

        private void SetClockMode(Clock.ClockMode mode)
        {
            Clock.Mode = mode;
            lastSetClockMode = mode;
        }

        /**
         * スクリプトの生存期間中に一回のみ呼び出される
         * https://docs.unity3d.com/ja/560/ScriptReference/MonoBehaviour.Start.html
         */
        private void Start()
        {
            SetClockMode(clockMode);
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RegisterPublisher<ClockMsg>("clock");
        }

        private void PublishMessage()
        {
            var publishTime = Clock.time;
            var clockMsg = new TimeMsg
            {
                sec = (int)publishTime,
                nanosec = (uint)((publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds)
            };
            _lastPublishTimeSeconds = publishTime;
            _rosConnection.Publish("clock", clockMsg);
        }

        /**
         * 毎フレーム呼び出される
         * https://docs.unity3d.com/jp/current/ScriptReference/MonoBehaviour.Update.html
         */
        private void Update()
        {
            if (ShouldPublishMessage)
            {
                PublishMessage();
            }
        }
    }
}