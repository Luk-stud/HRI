Datenstruktur 2DDetectionArray:

- Diese Struktur wird benötigt von topic fusion_out --> state_maschine_node --> state_maschine_out --> sit_node 


{
  "header": {
    "stamp": { "sec": 1699860000, "nanosec": 123456789 },
    "frame_id": "camera_link"
  },
  "detections": [
    {
      "header": {
        "stamp": { "sec": 1699860000, "nanosec": 123456789 },
        "frame_id": "camera_link"
      },
      "bbox": {
        "center": { "x": 0.5, "y": 0.4 },
        "size_x": 0.2,
        "size_y": 0.3
      },
      "results": [
        { "id": "person", "score": 0.95 }
      ],
      "id": "follow:42"
    },
    {
      "header": {
        "stamp": { "sec": 1699860001, "nanosec": 0 },
        "frame_id": "camera_link"
      },
      "bbox": {
        "center": { "x": 0.8, "y": 0.2 },
        "size_x": 0.15,
        "size_y": 0.25
      },
      "results": [
        { "id": "unknown", "score": 0.50 }
      ],
      "id": "default"
    }
  ]
}