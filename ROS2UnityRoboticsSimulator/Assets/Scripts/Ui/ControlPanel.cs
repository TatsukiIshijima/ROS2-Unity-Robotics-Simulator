using System;
using Robotics.Simulator.Publisher;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace Ui
{
    [RequireComponent(typeof(CmdVelPublisher))]
    public class ControlPanel : MonoBehaviour
    {
        [SerializeField] private Button forwardButton;
        [SerializeField] private Button backwardButton;
        [SerializeField] private Button leftButton;
        [SerializeField] private Button rightButton;

        private CmdVelPublisher _cmdVelPublisher;

        private void Awake()
        {
            AddEventTrigger(forwardButton, OnPointerDown, OnPointerUp);
            AddEventTrigger(backwardButton, OnPointerDown, OnPointerUp);
            AddEventTrigger(leftButton, OnPointerDown, OnPointerUp);
            AddEventTrigger(rightButton, OnPointerDown, OnPointerUp);
        }

        private void AddEventTrigger(
            Button button,
            Action<string> downAction,
            Action<string> upAction
        )
        {
            var eventTrigger = button.AddComponent<EventTrigger>();

            var pointerDownEntry = new EventTrigger.Entry
            {
                eventID = EventTriggerType.PointerDown
            };
            pointerDownEntry.callback.AddListener(_ => downAction(button.name));
            eventTrigger.triggers.Add(pointerDownEntry);

            var pointerUpEntry = new EventTrigger.Entry()
            {
                eventID = EventTriggerType.PointerUp
            };
            pointerUpEntry.callback.AddListener(_ => upAction((button.name)));
            eventTrigger.triggers.Add(pointerUpEntry);
        }

        private void OnPointerDown(string buttonName)
        {
            Debug.Log($"OnPointerDown {buttonName}");
            switch (buttonName)
            {
                case "ForwardButton":
                    Debug.Log("PointerDown Forward");
                    break;
                case "BackwardButton":
                    Debug.Log("PointerDown Backward");
                    break;
                case "LeftButton":
                    Debug.Log("PointerDown Left");
                    break;
                case "RightButton":
                    Debug.Log("PointerDown Right");
                    break;
            }
        }

        private void OnPointerUp(string buttonName)
        {
            Debug.Log($"OnPointerUp {buttonName}");
            switch (buttonName)
            {
                case "ForwardButton":
                    Debug.Log("PointerUp Forward");
                    break;
                case "BackwardButton":
                    Debug.Log("PointerUp Backward");
                    break;
                case "LeftButton":
                    Debug.Log("PointerUp Left");
                    break;
                case "RightButton":
                    Debug.Log("PointerUp Right");
                    break;
            }
        }
    }
}