using System;
using UnityEngine;
using UnityEngine.Rendering;

// https://github.com/taigamaru/donkey-my-sim/blob/main/Assets/Script/RGBCamera.cs
// https://github.com/Field-Robotics-Japan/UnitySensors/blob/master/Assets/UnitySensors/Runtime/Scripts/Sensors/Camera/CameraSensor.cs

namespace Robotics.Simulator.Sensor
{
    [RequireComponent(typeof(Camera))]
    public class RgbCamera : MonoBehaviour
    {
        // 解像度
        [SerializeField] private Vector2Int resolution = new Vector2Int(640, 480);

        // 画角
        [SerializeField] private float fieldOfView = 30.0f;

        // 四角錐台の手前側の距離
        [SerializeField] private float minRange = 0.05f;

        // 四角錐台の奥側の距離
        [SerializeField] private float maxRange = 100.0f;

        [SerializeField] private float frequency = 10.0f;

        private Camera _camera;
        private RenderTexture _renderTexture;
        private Texture2D _texture;
        private float _time;
        private float _dt;
        private float _frequencyInv;

        private void Awake()
        {
            _camera = GetComponent<Camera>();
            _camera.fieldOfView = fieldOfView;
            _camera.nearClipPlane = minRange;
            _camera.farClipPlane = maxRange;

            _renderTexture = new RenderTexture(resolution.x, resolution.y, 32, RenderTextureFormat.ARGBFloat);
            _camera.targetTexture = _renderTexture;

            _texture = new Texture2D(resolution.x, resolution.y, TextureFormat.RGBAFloat, false);

            _dt = 0.0f;
            _frequencyInv = 1.0f / frequency;
        }
        
        // ReSharper disable Unity.PerformanceAnalysis
        internal void LoadTexture(Action<byte[]> data, Action<string> error)
        {
            _dt += Time.deltaTime;
            if (_dt < _frequencyInv)
            {
                return;
            }
            
            _time = Time.time;
            
            if (SystemInfo.supportsAsyncGPUReadback)
            {
                AsyncGPUReadback.Request(_renderTexture, 0, request =>
                {
                    if (request.hasError)
                    {
                        error("AsyncGPUReadback error detected.");
                        return;
                    }

                    if (!request.done)
                    {
                        error("AsyncGPUReadback is not done.");
                        return;
                    }

                    var colorArray = request.GetData<Color>();
                    _texture.LoadRawTextureData(colorArray);
                    _texture.Apply();
                    data(_texture.EncodeToJPG(50));
                });
                AsyncGPUReadback.WaitAllRequests();
            }
            else
            {
                Debug.LogWarning("AsyncGPUReadback is not supported on this platform");
                RenderTexture.active = _renderTexture;
                _texture.ReadPixels(new Rect(0, 0, _renderTexture.width, _renderTexture.height), 0, 0);
                _texture.Apply();
                data(_texture.EncodeToJPG(50));
            }

            _dt = _frequencyInv;
        }

        private void OnDestroy()
        {
            _renderTexture.Release();
        }
    }
}