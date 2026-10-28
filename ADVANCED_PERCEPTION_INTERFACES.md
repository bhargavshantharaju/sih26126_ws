# Advanced Perception Interfaces - DRISTHI UGV

**Status:** Infrastructure ready, no trained models available  
**Date:** 2026-10-16

## Overview

The DRISTHI system provides clean interfaces for advanced semantic perception capabilities. The modular architecture supports integration of trained models when available, but currently operates with geometry-based terrain assessment only.

## Existing Perception Architecture

### 1. Perception Baseline Node

**File:** `src/sih26126_perception/sih26126_perception/perception_baseline.py`

**Interface:**
```python
class PerceptionBaseline(Node):
    def image_callback(self, msg: Image):
        """Process RGB camera images for semantic understanding."""
        # Currently: placeholder for semantic segmentation
        # Ready for: trained model integration
```

**Topics:**
- Input: `/camera/image_raw` (RGB images)
- Input: `/camera/camera_info` (camera calibration)
- Output: `/perception/semantic_class` (placeholder)
- Output: `/perception/confidence` (placeholder)

**Integration Points:**
- Can be replaced with semantic segmentation model (DeepLabV3+, SegFormer, etc.)
- Output format compatible with terrain classification pipeline
- Modular design allows model swap without changing downstream consumers

---

### 2. Terrain Geometry Node

**File:** `src/sih26126_geometry/sih26126_geometry/terrain_geometry.py`

**Capabilities:**
- ✅ Slope estimation from point clouds
- ✅ Roughness measurement from surface normals
- ✅ Ground plane fitting with RANSAC
- ✅ Confidence from geometry quality

**What It Does NOT Do:**
- ❌ Semantic classification (vegetation, rock, water)
- ❌ Material recognition (sand, gravel, mud)
- ❌ Object detection (boulders, logs, debris)

**Interface for Semantic Extension:**
```python
# Current geometric classification
terrain_class = classify_by_geometry(slope, roughness)

# Ready for semantic fusion:
# terrain_class = fuse_geometry_and_semantics(
#     geometry_class=geometric_class,
#     semantic_class=model_output,
#     geometry_confidence=geo_conf,
#     semantic_confidence=model_conf
# )
```

---

### 3. Camera Monitor Node

**File:** `src/sih26126_perception/sih26126_perception/camera_monitor.py`

**Purpose:** Health monitoring for camera streams

**Capabilities:**
- ✅ Frame rate monitoring
- ✅ Data validity checking
- ✅ Synchronization verification

**Ready For:**
- Image quality assessment (blur, exposure)
- Model inference health (latency, throughput)
- Semantic prediction monitoring

---

## Semantic Perception Interfaces (Ready, Not Implemented)

### Semantic Terrain Classification

**If a trained model becomes available:**

```python
class SemanticTerrainClassifier(Node):
    """Semantic terrain classification using deep learning."""
    
    def __init__(self):
        # Model loading interface
        self.model = load_model('path/to/weights.pth')
        
        # Topics
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.class_pub = self.create_publisher(
            String, '/terrain/semantic_class', 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, '/terrain/semantic_confidence', 10
        )
    
    def image_callback(self, msg: Image):
        # Inference
        class_label, confidence = self.model.predict(msg)
        
        # Publish
        self.class_pub.publish(String(data=class_label))
        self.confidence_pub.publish(Float32(data=confidence))
```

**Integration:** Output topics match terrain memory input interface

---

### Obstacle Semantic Classification

**If object detection model becomes available:**

```python
class SemanticObstacleDetector(Node):
    """Semantic obstacle detection and classification."""
    
    def detect_obstacles(self, image: np.ndarray):
        # YOLO/Faster-RCNN/etc. interface
        detections = self.model.detect(image)
        
        for det in detections:
            # Classify obstacle type
            if det.class_name == "vegetation":
                traversability = 0.7  # Can push through
            elif det.class_name == "rock":
                traversability = 0.2  # Hard obstacle
            elif det.class_name == "water":
                traversability = 0.0  # Unsafe
            
            # Publish with semantic information
            self.publish_semantic_obstacle(det, traversability)
```

**Integration:** Extends existing obstacle tracker with semantic labels

---

## What Models Would Be Useful

### High Priority
1. **Outdoor Terrain Segmentation**
   - Classes: vegetation, rock, soil, gravel, sand, water
   - Training data: RUGD, RELLIS-3D, or custom
   - Framework: PyTorch or ONNX for portability

2. **Traversability Estimation**
   - Direct regression: image → traversability map
   - Training: self-supervised from vehicle telemetry
   - Advantage: end-to-end learning

### Medium Priority
3. **Object Detection**
   - Classes: boulder, log, barrier, vehicle
   - Framework: YOLO for real-time performance
   - Integration: obstacle tracker enhancement

4. **Material Classification**
   - Classes: concrete, asphalt, grass, mud
   - Purpose: friction coefficient estimation
   - Integration: motion model adaptation

### Low Priority
5. **Visual Odometry Enhancement**
   - Semantic keypoint filtering
   - Dynamic object masking
   - Integration: existing RGB-D odometry

---

## Current Honest Status

### ✅ What Exists
- Clean modular interfaces
- Topic structure for semantic data
- Geometry-based fallback
- Health monitoring
- Integration points documented

### ❌ What Does NOT Exist
- No trained model weights
- No semantic segmentation
- No learned traversability
- No object classification
- No material recognition

### 🔧 What's Needed for Deployment
1. Trained model weights for target environment
2. Inference runtime (ONNX, TensorRT, etc.)
3. Preprocessing/postprocessing pipeline
4. Confidence calibration
5. Failure mode handling (model unavailable, inference timeout)

---

## Integration Example

**If semantic model becomes available:**

1. **Add model node to launch file:**
```python
semantic_classifier = Node(
    package='sih26126_perception',
    executable='semantic_terrain_classifier',
    name='semantic_classifier',
    parameters=[{'model_path': model_weights}]
)
```

2. **Terrain memory subscribes to semantic output:**
```python
# In terrain_memory.py
self.create_subscription(
    String,
    '/terrain/semantic_class',
    self.semantic_callback,
    10
)
```

3. **Fuse geometric + semantic confidence:**
```python
final_confidence = (
    0.5 * geometric_confidence +
    0.5 * semantic_confidence
)
```

No changes to downstream consumers (planner, cost map, etc.) required.

---

## Recommendation

**Current State:** Use geometry-based terrain assessment. It works and is reliable.

**Future Enhancement:** When domain-specific trained models become available:
1. Implement semantic classifier node using interfaces above
2. Add model health monitoring
3. Keep geometric fallback for robustness
4. Fuse predictions with confidence weighting

**Do NOT:**
- Claim semantic capability without actual model
- Use untrained or poorly-calibrated models
- Deploy without validation in target environment
- Remove geometric fallback (fusion is safer)

---

## Testing Without Models

Current geometry-based system can be tested immediately:
- Slope detection on ramps ✓
- Roughness on uneven terrain ✓
- Obstacle geometry from point clouds ✓

Semantic testing requires:
- Labeled validation set
- Model inference on test images
- Traversability ground truth

---

## Conclusion

**Honest Assessment:** Infrastructure is ready, but no trained models exist to integrate. Current geometry-based perception is functional for the defined mission scope.

**Status:** Feature 8 (Advanced Perception) = Infrastructure Complete, Models Not Available
