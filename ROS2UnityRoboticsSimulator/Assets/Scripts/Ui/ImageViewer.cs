using Robotics.Simulator.Subscriber;
using UnityEngine;
using UnityEngine.UI;

namespace Ui
{
    [RequireComponent(typeof(ImageSubscriber))]
    public class ImageViewer : MonoBehaviour
    {
        [SerializeField] private RawImage rawImage;
        
        private ImageSubscriber _imageSubscriber;

        private void Awake()
        {
            _imageSubscriber = GetComponent<ImageSubscriber>();
        }

        private void Start()
        {
            _imageSubscriber.OnLoadImage += OnLoadImage;
        }

        private void OnDestroy()
        {
            _imageSubscriber.OnLoadImage -= OnLoadImage;
        }

        private void OnLoadImage(Texture2D texture)
        {
            rawImage.texture = texture;
        }
    }
}