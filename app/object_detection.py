"""
Object Detection Module

Detects and classifies objects based on LiDAR scan patterns.
For single-point LiDAR, this works by analyzing:
- Distance patterns over time
- Signal strength variations
- Spatial clusters in accumulated point clouds
"""

import numpy as np
from typing import List, Dict, Optional, Tuple
from datetime import datetime
from dataclasses import dataclass, field
from collections import deque
from scipy import signal
from scipy.spatial.distance import cdist
import threading
import logging
import json
import os

from app.config import Config

logger = logging.getLogger(__name__)


@dataclass
class DetectedObject:
    """Detected object information"""
    id: int
    object_class: str
    confidence: float
    position: Tuple[float, float, float]
    size_estimate: Tuple[float, float, float]  # width, height, depth
    distance: float
    first_seen: datetime
    last_seen: datetime
    point_count: int
    features: Dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'class': self.object_class,
            'confidence': round(self.confidence, 2),
            'position': [round(p, 3) for p in self.position],
            'size': [round(s, 3) for s in self.size_estimate],
            'distance': round(self.distance, 3),
            'first_seen': self.first_seen.isoformat(),
            'last_seen': self.last_seen.isoformat(),
            'point_count': self.point_count,
            'features': self.features
        }


class DistancePatternAnalyzer:
    """
    Analyzes distance reading patterns to detect objects

    For single-point LiDAR, objects are detected by:
    - Sudden distance changes (edges)
    - Consistent readings (flat surfaces)
    - Oscillating patterns (irregular surfaces)
    """

    def __init__(self, window_size: int = 50):
        self.window_size = window_size
        self.distance_buffer = deque(maxlen=window_size)
        self.strength_buffer = deque(maxlen=window_size)
        self.timestamp_buffer = deque(maxlen=window_size)

    def add_reading(self, distance: float, strength: int, timestamp: datetime):
        """Add a new reading to the buffer"""
        self.distance_buffer.append(distance)
        self.strength_buffer.append(strength)
        self.timestamp_buffer.append(timestamp)

    def analyze(self) -> Dict:
        """
        Analyze current buffer for patterns

        Returns:
            Dict with pattern analysis results
        """
        if len(self.distance_buffer) < self.window_size // 2:
            return {'status': 'insufficient_data'}

        distances = np.array(self.distance_buffer)
        strengths = np.array(self.strength_buffer)

        # Calculate statistics
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        mean_strength = np.mean(strengths)

        # Detect edges (sudden distance changes)
        diff = np.diff(distances)
        edges = np.where(np.abs(diff) > 0.3)[0]  # 30cm threshold

        # Detect flat surfaces (low variance)
        is_flat = std_dist < 0.05  # 5cm variance threshold

        # Detect oscillating patterns
        if len(distances) > 10:
            # Simple oscillation detection using zero crossings of derivative
            zero_crossings = np.where(np.diff(np.signbit(diff)))[0]
            is_oscillating = len(zero_crossings) > len(distances) * 0.3
        else:
            is_oscillating = False

        # Classify pattern
        if len(edges) > 0:
            pattern_type = 'edge_detected'
        elif is_flat:
            pattern_type = 'flat_surface'
        elif is_oscillating:
            pattern_type = 'irregular_surface'
        else:
            pattern_type = 'unknown'

        return {
            'status': 'analyzed',
            'pattern_type': pattern_type,
            'mean_distance': round(mean_dist, 3),
            'distance_variance': round(std_dist, 4),
            'mean_strength': round(mean_strength, 1),
            'edge_count': len(edges),
            'is_flat': is_flat,
            'is_oscillating': is_oscillating
        }


class ObjectDetector:
    """
    Main object detection engine

    Uses multiple detection strategies:
    1. Distance pattern analysis
    2. Point cloud clustering
    3. Feature-based classification
    """

    def __init__(self, config=None, objects_dir: str = None):
        self.config = config or Config.object_detection
        self.objects_dir = objects_dir or Config.OBJECTS_DIR

        # Detection state
        self.detected_objects: Dict[int, DetectedObject] = {}
        self.object_counter = 0
        self.pattern_analyzer = DistancePatternAnalyzer()

        # Point accumulation for clustering
        self.point_buffer: List[np.ndarray] = []
        self.buffer_limit = 500

        # Classification model (simple rule-based for now)
        self.classification_rules = self._init_classification_rules()

        # Thread safety
        self.lock = threading.Lock()

        # Ensure directory exists
        os.makedirs(self.objects_dir, exist_ok=True)

    def _init_classification_rules(self) -> Dict:
        """Initialize classification rules"""
        return {
            'wall': {
                'min_size': (0.5, 0.5, 0.02),
                'max_size': (100, 100, 0.3),
                'pattern': 'flat_surface',
                'strength_range': (100, 65535)
            },
            'pipe': {
                'min_size': (0.05, 0.05, 0.1),
                'max_size': (0.5, 0.5, 10),
                'pattern': 'edge_detected',
                'strength_range': (50, 500)
            },
            'rock': {
                'min_size': (0.1, 0.1, 0.1),
                'max_size': (2, 2, 2),
                'pattern': 'irregular_surface',
                'strength_range': (30, 300)
            },
            'debris': {
                'min_size': (0.05, 0.05, 0.05),
                'max_size': (1, 1, 1),
                'pattern': 'irregular_surface',
                'strength_range': (20, 200)
            },
            'obstacle': {
                'min_size': (0.1, 0.1, 0.1),
                'max_size': (5, 5, 5),
                'pattern': 'edge_detected',
                'strength_range': (50, 65535)
            }
        }

    def process_reading(self, distance: float, strength: int,
                       position: Tuple[float, float, float],
                       timestamp: datetime) -> Optional[Dict]:
        """
        Process a single LiDAR reading for object detection

        Returns detection result if object found
        """
        with self.lock:
            # Add to pattern analyzer
            self.pattern_analyzer.add_reading(distance, strength, timestamp)

            # Add point to buffer
            point = np.array([position[0], position[1], position[2]])
            self.point_buffer.append(point)

            # Analyze patterns
            pattern_result = self.pattern_analyzer.analyze()

            # Check for object detection conditions
            detection = None

            if pattern_result.get('pattern_type') == 'edge_detected':
                # Potential object edge detected
                detection = self._create_detection(
                    position=position,
                    distance=distance,
                    pattern=pattern_result,
                    timestamp=timestamp
                )

            # Periodic clustering analysis
            if len(self.point_buffer) >= self.buffer_limit:
                cluster_detections = self._analyze_clusters()
                self.point_buffer = self.point_buffer[-100:]  # Keep last 100 points

                if cluster_detections:
                    for det in cluster_detections:
                        self._add_or_update_object(det)

            return detection

    def _create_detection(self, position: Tuple[float, float, float],
                         distance: float, pattern: Dict,
                         timestamp: datetime) -> DetectedObject:
        """Create a detection from pattern analysis"""
        # Classify object
        obj_class, confidence = self._classify_object(pattern, distance)

        self.object_counter += 1

        detection = DetectedObject(
            id=self.object_counter,
            object_class=obj_class,
            confidence=confidence,
            position=position,
            size_estimate=(0.5, 0.5, 0.5),  # Default estimate
            distance=distance,
            first_seen=timestamp,
            last_seen=timestamp,
            point_count=1,
            features=pattern
        )

        self._add_or_update_object(detection)
        return detection

    def _classify_object(self, pattern: Dict, distance: float) -> Tuple[str, float]:
        """Classify object based on pattern and rules"""
        pattern_type = pattern.get('pattern_type', 'unknown')
        strength = pattern.get('mean_strength', 0)

        best_class = 'unknown'
        best_confidence = 0.3

        for obj_class, rules in self.classification_rules.items():
            if rules['pattern'] == pattern_type:
                # Check strength range
                if rules['strength_range'][0] <= strength <= rules['strength_range'][1]:
                    confidence = 0.6 + (0.4 * (1 - pattern.get('distance_variance', 0.5)))
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_class = obj_class

        return best_class, min(best_confidence, 1.0)

    def _add_or_update_object(self, detection: DetectedObject):
        """Add new object or update existing if nearby"""
        # Check for nearby existing objects
        for obj_id, existing in self.detected_objects.items():
            dist = np.linalg.norm(
                np.array(detection.position) - np.array(existing.position)
            )
            if dist < 0.5:  # 50cm merge threshold
                # Update existing object
                existing.last_seen = detection.last_seen
                existing.point_count += detection.point_count
                # Update confidence with moving average
                existing.confidence = 0.9 * existing.confidence + 0.1 * detection.confidence
                return

        # Add as new object
        self.detected_objects[detection.id] = detection
        logger.info(f"New object detected: {detection.object_class} at {detection.position}")

    def _analyze_clusters(self) -> List[DetectedObject]:
        """Analyze point buffer for clusters using simple distance-based clustering"""
        if len(self.point_buffer) < self.config.clustering_min_samples:
            return []

        points = np.array(self.point_buffer)
        detections = []

        try:
            # Simple DBSCAN-like clustering
            clusters = self._simple_cluster(points, self.config.clustering_eps,
                                           self.config.clustering_min_samples)

            for cluster_points in clusters:
                if len(cluster_points) < self.config.clustering_min_samples:
                    continue

                # Calculate cluster properties
                centroid = np.mean(cluster_points, axis=0)
                size = np.max(cluster_points, axis=0) - np.min(cluster_points, axis=0)

                # Check size constraints
                if (np.all(size >= self.config.min_object_size) and
                    np.all(size <= self.config.max_object_size)):

                    self.object_counter += 1
                    detection = DetectedObject(
                        id=self.object_counter,
                        object_class='obstacle',
                        confidence=0.7,
                        position=tuple(centroid),
                        size_estimate=tuple(size),
                        distance=np.linalg.norm(centroid),
                        first_seen=datetime.now(),
                        last_seen=datetime.now(),
                        point_count=len(cluster_points)
                    )
                    detections.append(detection)

        except Exception as e:
            logger.error(f"Cluster analysis error: {e}")

        return detections

    def _simple_cluster(self, points: np.ndarray, eps: float,
                        min_samples: int) -> List[np.ndarray]:
        """Simple distance-based clustering"""
        clusters = []
        visited = set()

        for i in range(len(points)):
            if i in visited:
                continue

            # Find neighbors
            distances = np.linalg.norm(points - points[i], axis=1)
            neighbors = np.where(distances < eps)[0]

            if len(neighbors) >= min_samples:
                cluster = []
                stack = list(neighbors)

                while stack:
                    j = stack.pop()
                    if j not in visited:
                        visited.add(j)
                        cluster.append(points[j])

                        # Expand cluster
                        distances_j = np.linalg.norm(points - points[j], axis=1)
                        new_neighbors = np.where(distances_j < eps)[0]
                        if len(new_neighbors) >= min_samples:
                            stack.extend([n for n in new_neighbors if n not in visited])

                if cluster:
                    clusters.append(np.array(cluster))

        return clusters

    def get_objects(self) -> List[Dict]:
        """Get all detected objects"""
        with self.lock:
            return [obj.to_dict() for obj in self.detected_objects.values()]

    def get_nearby_objects(self, position: Tuple[float, float, float],
                          radius: float = 5.0) -> List[Dict]:
        """Get objects within radius of position"""
        with self.lock:
            pos = np.array(position)
            nearby = []
            for obj in self.detected_objects.values():
                dist = np.linalg.norm(np.array(obj.position) - pos)
                if dist <= radius:
                    obj_dict = obj.to_dict()
                    obj_dict['distance_from_query'] = round(dist, 3)
                    nearby.append(obj_dict)
            return sorted(nearby, key=lambda x: x['distance_from_query'])

    def clear_objects(self):
        """Clear all detected objects"""
        with self.lock:
            self.detected_objects.clear()
            self.object_counter = 0
            self.point_buffer.clear()
            logger.info("Object detection cleared")

    def save_objects(self, filename: str = None) -> bool:
        """Save detected objects to file"""
        try:
            if filename is None:
                filename = f"objects_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

            filepath = os.path.join(self.objects_dir, filename)

            with self.lock:
                data = {
                    'timestamp': datetime.now().isoformat(),
                    'object_count': len(self.detected_objects),
                    'objects': [obj.to_dict() for obj in self.detected_objects.values()]
                }

            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)

            logger.info(f"Objects saved to {filepath}")
            return True

        except Exception as e:
            logger.error(f"Failed to save objects: {e}")
            return False

    def get_statistics(self) -> dict:
        """Get detection statistics"""
        with self.lock:
            class_counts = {}
            for obj in self.detected_objects.values():
                class_counts[obj.object_class] = class_counts.get(obj.object_class, 0) + 1

            return {
                'total_objects': len(self.detected_objects),
                'buffer_size': len(self.point_buffer),
                'class_distribution': class_counts,
                'pattern_analysis': self.pattern_analyzer.analyze()
            }
