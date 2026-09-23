# Three-class YOLOv8n detector

This directory contains the detector weight used for the external-camera perception test in Experiment 3.

- Weight: `best_yolov8n_eraser_lock_stapler.pt`
- Classes: `eraser`, `lock`, `stapler`
- Model: YOLOv8n
- Input size: 640
- Training: 60 epochs, batch size 16, RTX 4070 Laptop GPU
- Approximate parameter count: 3 million
- Weight size: 6,249,827 bytes (about 6.3 MB)
- SHA-256: `b6902c70235390f0f69cfb101e34b245a5c0feefc6c0a0553708a8fe1a87051f`

The supplied training record reports mAP50=0.995, mAP50-95=0.995, precision=1.0, and recall=1.0 for the overall validation set and each class. The accompanying `evidence/model_detection_realtime.png` image records an external-camera run with all three classes detected, confidence values from 0.89 to 0.94, and a displayed rate of 31.2 FPS. The reported 1.1 ms value refers to single-model inference time, not the end-to-end camera display rate.

The Gazebo simulation monitor still produces deterministic `Detection2DArray` messages from model states so that the sorting task remains reproducible. This weight is archived for real-camera and real-robot integration.
