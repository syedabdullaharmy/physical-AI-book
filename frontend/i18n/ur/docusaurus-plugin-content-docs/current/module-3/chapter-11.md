---
sidebar_position: 11
---

# Chapter 11: AI-Powered Perception

## Introduction

Perception is the foundation of intelligent robotics. This chapter covers modern AI techniques for enabling robots to understand their environment through computer vision and sensor fusion.

:::tip Learning Objectives
- Implement object detection and tracking
- Perform 6-DOF pose estimation
- Use semantic segmentation
- Deploy DNNs on edge devices (Jetson)
- Integrate YOLO, DOPE, and custom models
- Implement visual SLAM
:::

## Object Detection

### YOLO (You Only Look Once)

YOLO is fast and accurate for real-time detection:

```python
import cv2
import numpy as np
from ultralytics import YOLO

# Load model
model = YOLO('yolov8n.pt')  # Nano model for speed

def detect_objects(image):
    """Run YOLO detection"""
    results = model(image)
    
    detections = []
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            
            detections.append({
                'bbox': [x1, y1, x2, y2],
                'confidence': conf,
                'class': model.names[cls]
            })
    
    return detections

# Process camera feed
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    detections = detect_objects(frame)
    
    # Draw boxes
    for det in detections:
        x1, y1, x2, y2 = map(int, det['bbox'])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{det['class']}: {det['confidence']:.2f}"
        cv2.putText(frame, label, (x1, y1-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    
    cv2.imshow('Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load model
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        
        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.det_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        
        self.get_logger().info('YOLO detector initialized')
    
    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run detection
        results = self.model(cv_image)
        
        # Create detection message
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        
        for result in results:
            for box in result.boxes:
                detection = Detection2D()
                
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                bbox = detection.bbox
                bbox.center.position.x = float((x1 + x2) / 2)
                bbox.center.position.y = float((y1 + y2) / 2)
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)
                
                # Class and confidence
                detection.results[0].hypothesis.class_id = str(int(box.cls[0]))
                detection.results[0].hypothesis.score = float(box.conf[0])
                
                det_msg.detections.append(detection)
        
        self.det_pub.publish(det_msg)

def main():
    rclpy.init()
    detector = YOLODetector()
    rclpy.spin(detector)
```

## 6-DOF Pose Estimation

### DOPE (Deep Object Pose Estimation)

```python
import torch
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

class DOPEDetector:
    def __init__(self, weights_path, camera_matrix):
        self.model = torch.load(weights_path)
        self.model.eval()
        self.K = camera_matrix  # 3x3 intrinsic matrix
        
    def detect(self, image):
        """
        Detect objects and estimate 6-DOF poses
        Returns: list of (class_name, position, orientation)
        """
        # Preprocess
        img_tensor = self.preprocess(image)
        
        with torch.no_grad():
            # Forward pass
            vertex_fields, aff_param_fields = self.model(img_tensor)
            
        # Post-process to get 3D poses
        detections = []
        for obj_id, vertices in enumerate(vertex_fields):
            # Find object keypoints
            peaks = self.find_peaks(vertices)
            
            if len(peaks) >= 8:  # Need at least 8 points
                # PnP to solve for pose
                object_points = self.get_3d_keypoints(obj_id)
                image_points = np.array(peaks)
                
                success, rvec, tvec = cv2.solvePnP(
                    object_points, image_points, self.K, None
                )
                
                if success:
                    # Convert rotation vector to quaternion
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    quat = Rotation.from_matrix(rot_matrix).as_quat()
                    
                    detections.append({
                        'class': self.class_names[obj_id],
                        'position': tvec.flatten(),
                        'orientation': quat
                    })
        
        return detections
    
    def preprocess(self, image):
        """Prepare image for model"""
        img = cv2.resize(image, (640,480))
        img = img.transpose(2, 0, 1) / 255.0
        return torch.FloatTensor(img).unsqueeze(0)
    
    def find_peaks(self, vertex_field):
        """Find keypoint peaks in belief maps"""
        peaks = []
        for belief_map in vertex_field:
            # Non-maximum suppression
            peak = self.nms(belief_map, threshold=0.1)
            peaks.extend(peak)
        return peaks
```

### Publishing Poses to ROS 2

```python
from geometry_msgs.msg import PoseStamped, Pose

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        self.dope = DOPEDetector('weights.pth', self.get_camera_matrix())
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/object_poses', 10)
    
    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections = self.dope.detect(cv_image)
        
        for det in detections:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.header.frame_id = 'camera_link'
            
            # Position
            pose_msg.pose.position.x = det['position'][0]
            pose_msg.pose.position.y = det['position'][1]
            pose_msg.pose.position.z = det['position'][2]
            
            # Orientation (quaternion)
            pose_msg.pose.orientation.x = det['orientation'][0]
            pose_msg.pose.orientation.y = det['orientation'][1]
            pose_msg.pose.orientation.z = det['orientation'][2]
            pose_msg.pose.orientation.w = det['orientation'][3]
            
            self.pose_pub.publish(pose_msg)
```

## Semantic Segmentation

### DeepLab for Scene Understanding

```python
import torch
import torchvision.transforms as T
from torchvision.models.segmentation import deeplabv3_resnet50

class SemanticSegmentation:
    def __init__(self):
        self.model = deeplabv3_resnet50(pretrained=True)
        self.model.eval()
        
        self.transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                       std=[0.229, 0.224, 0.225])
        ])
        
        self.classes = [
            'background', 'person', 'car', 'chair', 'bottle',
            # ... more classes
        ]
    
    def segment(self, image):
        """
        Returns semantic segmentation mask
        """
        # Preprocess
        input_tensor = self.transform(image).unsqueeze(0)
        
        with torch.no_grad():
            output = self.model(input_tensor)['out'][0]
        
        # Get class predictions
        predictions = output.argmax(0).cpu().numpy()
        
        return predictions
    
    def visualize(self, image, mask):
        """Create colored segmentation overlay"""
        # Color map for classes
        colors = np.random.randint(0, 255, (len(self.classes), 3))
        
        colored_mask = colors[mask]
        overlay = cv2.addWeighted(image, 0.6, colored_mask.astype(np.uint8), 0.4, 0)
        
        return overlay
```

## Object Tracking

### SORT (Simple Online Realtime Tracking)

```python
from filterpy.kalman import KalmanFilter
import numpy as np

class KalmanBoxTracker:
    count = 0
    
    def __init__(self, bbox):
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        
        # State: [x, y, s, r, vx, vy, vs]
        # s = area, r = aspect ratio
        self.kf.F = np.array([
            [1,0,0,0,1,0,0],
            [0,1,0,0,0,1,0],
            [0,0,1,0,0,0,1],
            [0,0,0,1,0,0,0],
            [0,0,0,0,1,0,0],
            [0,0,0,0,0,1,0],
            [0,0,0,0,0,0,1]
        ])
        
        self.kf.H = np.array([
            [1,0,0,0,0,0,0],
            [0,1,0,0,0,0,0],
            [0,0,1,0,0,0,0],
            [0,0,0,1,0,0,0]
        ])
        
        self.kf.R[2:,2:] *= 10.
        self.kf.P[4:,4:] *= 1000.
        self.kf.Q[-1,-1] *= 0.01
        
        self.kf.x[:4] = self.convert_bbox_to_z(bbox)
        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.hits = 0
        
    def update(self, bbox):
        self.time_since_update = 0
        self.hits += 1
        self.kf.update(self.convert_bbox_to_z(bbox))
    
    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return self.convert_x_to_bbox(self.kf.x)
    
    def get_state(self):
        return self.convert_x_to_bbox(self.kf.x)
    
    @staticmethod
    def convert_bbox_to_z(bbox):
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        x = bbox[0] + w/2.
        y = bbox[1] + h/2.
        s = w * h
        r = w / float(h)
        return np.array([x, y, s, r]).reshape((4, 1))
    
    @staticmethod
    def convert_x_to_bbox(x):
        w = np.sqrt(x[2] * x[3])
        h = x[2] / w
        return np.array([
            x[0]-w/2., x[1]-h/2.,
            x[0]+w/2., x[1]+h/2.
        ]).reshape((1,4))

class SORT:
    def __init__(self, max_age=30, min_hits=3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = []
        self.frame_count = 0
    
    def update(self, detections):
        """
        detections: Nx4 array of [x1, y1, x2, y2]
        Returns: Nx5 array of [x1, y1, x2, y2, track_id]
        """
        self.frame_count += 1
        
        # Get predicted locations
        trks = np.zeros((len(self.trackers), 5))
        to_del = []
        for t, trk in enumerate(trks):
            pos = self.trackers[t].predict()[0]
            trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
            if np.any(np.isnan(pos)):
                to_del.append(t)
        
        # Remove invalid trackers
        for t in reversed(to_del):
            self.trackers.pop(t)
        
        # Associate detections to trackers
        matched, unmatched_dets, unmatched_trks = self.associate(detections, trks)
        
        # Update matched trackers
        for m in matched:
            self.trackers[m[1]].update(detections[m[0], :])
        
        # Create new trackers for unmatched detections
        for i in unmatched_dets:
            trk = KalmanBoxTracker(detections[i,:])
            self.trackers.append(trk)
        
        # Return tracked objects
        ret = []
        for trk in self.trackers:
            if trk.hits >= self.min_hits:
                d = trk.get_state()[0]
                ret.append(np.concatenate((d, [trk.id])).reshape(1,-1))
        
        return np.concatenate(ret) if len(ret) > 0 else np.empty((0,5))
```

## Deploying on Jetson (Edge AI)

### TensorRT Optimization

```python
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class TRTInference:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        
        # Load engine
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        
        self.context = self.engine.create_execution_context()
        
        # Allocate buffers
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.stream = cuda.Stream()
        
        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            
            self.bindings.append(int(device_mem))
            
            if self.engine.binding_is_input(binding):
                self.inputs.append({'host': host_mem, 'device': device_mem})
            else:
                self.outputs.append({'host': host_mem, 'device': device_mem})
    
    def infer(self, image):
        # Copy input to device
        np.copyto(self.inputs[0]['host'], image.ravel())
        cuda.memcpy_htod_async(
            self.inputs[0]['device'],
            self.inputs[0]['host'],
            self.stream
        )
        
        # Run inference
        self.context.execute_async_v2(
            bindings=self.bindings,
            stream_handle=self.stream.handle
        )
        
        # Copy output to host
        cuda.memcpy_dtoh_async(
            self.outputs[0]['host'],
            self.outputs[0]['device'],
            self.stream
        )
        
        self.stream.synchronize()
        return self.outputs[0]['host']
```

### Converting PyTorch to TensorRT

```python
import torch
import torch.onnx
import tensorrt as trt

def convert_to_tensorrt(pytorch_model, onnx_path, trt_path):
    # Export to ONNX
    dummy_input = torch.randn(1, 3, 640, 480).cuda()
    torch.onnx.export(
        pytorch_model,
        dummy_input,
        onnx_path,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output']
    )
    
    # Convert ONNX to TensorRT
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)
    
    with open(onnx_path, 'rb') as model:
        parser.parse(model.read())
    
    config = builder.create_builder_config()
    config.max_workspace_size = 1 << 30  # 1GB
    config.set_flag(trt.BuilderFlag.FP16)  # Enable FP16 for Jetson
    
    engine = builder.build_engine(network, config)
    
    with open(trt_path, 'wb') as f:
        f.write(engine.serialize())
    
    print(f"TensorRT engine saved to {trt_path}")
```

## Summary

✅ Object detection with YOLO  
✅ 6-DOF pose estimation (DOPE)  
✅ Semantic segmentation  
✅ Object tracking (SORT)  
✅ TensorRT deployment on Jetson  

## Practice Exercises

1. Implement YOLO detector for custom objects
2. Track multiple objects across video frames
3. Deploy detection model on Jetson Nano
4. Combine detection + pose estimation

## Next Chapter

In **Chapter 12**, we'll explore:
- Reinforcement learning for robot control
- PPO and SAC algorithms
- Training in Isaac Gym
- Sim-to-real deployment

👉 [Continue to Chapter 12 →](./chapter-12.md)
