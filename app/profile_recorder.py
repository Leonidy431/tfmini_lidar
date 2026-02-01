"""
Navigation Profile Recorder Module

Records navigation profiles that can be replayed for autonomous navigation.
A profile consists of:
- Waypoints with positions and orientations
- Distance measurements at each point
- Timing information
- Environmental features
"""

import json
import os
import numpy as np
from typing import List, Dict, Optional, Tuple
from datetime import datetime
from dataclasses import dataclass, field, asdict
from collections import deque
import threading
import logging

from app.config import Config

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """Single waypoint in navigation profile"""
    index: int
    timestamp: datetime
    position: Tuple[float, float, float]  # x, y, z
    heading: float  # degrees
    distance_reading: float  # meters
    signal_strength: int
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    features: Dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            'index': self.index,
            'timestamp': self.timestamp.isoformat(),
            'position': list(self.position),
            'heading': self.heading,
            'distance_reading': self.distance_reading,
            'signal_strength': self.signal_strength,
            'velocity': list(self.velocity),
            'features': self.features
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'Waypoint':
        return cls(
            index=data['index'],
            timestamp=datetime.fromisoformat(data['timestamp']),
            position=tuple(data['position']),
            heading=data['heading'],
            distance_reading=data['distance_reading'],
            signal_strength=data['signal_strength'],
            velocity=tuple(data.get('velocity', [0, 0, 0])),
            features=data.get('features', {})
        )


@dataclass
class NavigationProfile:
    """Complete navigation profile"""
    name: str
    created: datetime
    waypoints: List[Waypoint] = field(default_factory=list)
    description: str = ""
    total_distance: float = 0.0
    duration_seconds: float = 0.0
    metadata: Dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'created': self.created.isoformat(),
            'description': self.description,
            'total_distance': self.total_distance,
            'duration_seconds': self.duration_seconds,
            'waypoint_count': len(self.waypoints),
            'waypoints': [wp.to_dict() for wp in self.waypoints],
            'metadata': self.metadata
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'NavigationProfile':
        profile = cls(
            name=data['name'],
            created=datetime.fromisoformat(data['created']),
            description=data.get('description', ''),
            total_distance=data.get('total_distance', 0.0),
            duration_seconds=data.get('duration_seconds', 0.0),
            metadata=data.get('metadata', {})
        )
        profile.waypoints = [Waypoint.from_dict(wp) for wp in data.get('waypoints', [])]
        return profile


class ProfileRecorder:
    """
    Records navigation profiles for later playback

    Usage:
    1. Start recording with start_recording(profile_name)
    2. Add waypoints with add_waypoint() as vehicle moves
    3. Stop with stop_recording()
    4. Save profile with save_profile()
    """

    def __init__(self, profiles_dir: str = None, config=None):
        self.profiles_dir = profiles_dir or Config.PROFILES_DIR
        self.config = config or Config.navigation

        self.current_profile: Optional[NavigationProfile] = None
        self.is_recording = False
        self.start_time: Optional[datetime] = None

        # Position tracking
        self.last_position = np.array([0.0, 0.0, 0.0])
        self.total_distance = 0.0
        self.waypoint_counter = 0

        # Thread safety
        self.lock = threading.Lock()

        # Ensure directory exists
        os.makedirs(self.profiles_dir, exist_ok=True)

    def start_recording(self, name: str, description: str = "") -> bool:
        """Start recording a new profile"""
        with self.lock:
            if self.is_recording:
                logger.warning("Already recording a profile")
                return False

            self.current_profile = NavigationProfile(
                name=name,
                created=datetime.now(),
                description=description
            )

            self.is_recording = True
            self.start_time = datetime.now()
            self.total_distance = 0.0
            self.waypoint_counter = 0
            self.last_position = np.array([0.0, 0.0, 0.0])

            logger.info(f"Started recording profile: {name}")
            return True

    def stop_recording(self) -> Optional[NavigationProfile]:
        """Stop recording and return the profile"""
        with self.lock:
            if not self.is_recording:
                logger.warning("Not currently recording")
                return None

            self.is_recording = False

            if self.current_profile and self.start_time:
                duration = (datetime.now() - self.start_time).total_seconds()
                self.current_profile.duration_seconds = duration
                self.current_profile.total_distance = self.total_distance

            logger.info(f"Stopped recording profile: {self.current_profile.name if self.current_profile else 'Unknown'}")
            return self.current_profile

    def add_waypoint(self,
                     position: Tuple[float, float, float],
                     heading: float,
                     distance_reading: float,
                     signal_strength: int,
                     velocity: Tuple[float, float, float] = (0, 0, 0),
                     features: Dict = None) -> bool:
        """Add a waypoint to the current profile"""
        with self.lock:
            if not self.is_recording or not self.current_profile:
                return False

            # Check minimum distance from last waypoint
            pos_array = np.array(position)
            distance_from_last = np.linalg.norm(pos_array - self.last_position)

            if distance_from_last < self.config.waypoint_distance_threshold and self.waypoint_counter > 0:
                return False  # Too close to last waypoint

            # Update total distance
            self.total_distance += distance_from_last
            self.last_position = pos_array

            # Create waypoint
            waypoint = Waypoint(
                index=self.waypoint_counter,
                timestamp=datetime.now(),
                position=position,
                heading=heading,
                distance_reading=distance_reading,
                signal_strength=signal_strength,
                velocity=velocity,
                features=features or {}
            )

            self.current_profile.waypoints.append(waypoint)
            self.waypoint_counter += 1

            logger.debug(f"Added waypoint {self.waypoint_counter}: {position}")
            return True

    def save_profile(self, profile: NavigationProfile = None) -> bool:
        """Save profile to disk"""
        profile = profile or self.current_profile
        if not profile:
            logger.error("No profile to save")
            return False

        try:
            filepath = os.path.join(self.profiles_dir, f"{profile.name}.json")

            with open(filepath, 'w') as f:
                json.dump(profile.to_dict(), f, indent=2)

            logger.info(f"Profile saved: {filepath}")
            return True

        except Exception as e:
            logger.error(f"Failed to save profile: {e}")
            return False

    def load_profile(self, name: str) -> Optional[NavigationProfile]:
        """Load profile from disk"""
        try:
            filepath = os.path.join(self.profiles_dir, f"{name}.json")

            if not os.path.exists(filepath):
                logger.error(f"Profile not found: {name}")
                return None

            with open(filepath, 'r') as f:
                data = json.load(f)

            profile = NavigationProfile.from_dict(data)
            logger.info(f"Profile loaded: {name} ({len(profile.waypoints)} waypoints)")
            return profile

        except Exception as e:
            logger.error(f"Failed to load profile: {e}")
            return None

    def list_profiles(self) -> List[Dict]:
        """List all saved profiles"""
        profiles = []
        try:
            for filename in os.listdir(self.profiles_dir):
                if filename.endswith('.json'):
                    filepath = os.path.join(self.profiles_dir, filename)
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                        profiles.append({
                            'name': data['name'],
                            'created': data['created'],
                            'description': data.get('description', ''),
                            'waypoint_count': data.get('waypoint_count', 0),
                            'total_distance': data.get('total_distance', 0),
                            'duration_seconds': data.get('duration_seconds', 0)
                        })
        except Exception as e:
            logger.error(f"Failed to list profiles: {e}")

        return sorted(profiles, key=lambda x: x['created'], reverse=True)

    def delete_profile(self, name: str) -> bool:
        """Delete a profile"""
        try:
            filepath = os.path.join(self.profiles_dir, f"{name}.json")
            if os.path.exists(filepath):
                os.remove(filepath)
                logger.info(f"Profile deleted: {name}")
                return True
            return False
        except Exception as e:
            logger.error(f"Failed to delete profile: {e}")
            return False

    def get_recording_status(self) -> dict:
        """Get current recording status"""
        return {
            'is_recording': self.is_recording,
            'profile_name': self.current_profile.name if self.current_profile else None,
            'waypoint_count': self.waypoint_counter,
            'total_distance': round(self.total_distance, 2),
            'duration_seconds': (datetime.now() - self.start_time).total_seconds() if self.start_time and self.is_recording else 0
        }


class ProfileNavigator:
    """
    Navigate using a recorded profile

    Provides guidance by comparing current position/readings
    with recorded waypoints
    """

    def __init__(self, config=None):
        self.config = config or Config.navigation
        self.current_profile: Optional[NavigationProfile] = None
        self.current_waypoint_index = 0
        self.is_navigating = False

        # Navigation state
        self.distance_to_waypoint = 0.0
        self.heading_error = 0.0
        self.progress_percent = 0.0

        # Deviation tracking
        self.position_deviation = 0.0
        self.heading_deviation = 0.0
        self.distance_deviation = 0.0

    def start_navigation(self, profile: NavigationProfile) -> bool:
        """Start navigating using a profile"""
        if not profile or not profile.waypoints:
            logger.error("Invalid profile for navigation")
            return False

        self.current_profile = profile
        self.current_waypoint_index = 0
        self.is_navigating = True
        self.progress_percent = 0.0

        logger.info(f"Started navigation with profile: {profile.name}")
        return True

    def stop_navigation(self):
        """Stop navigation"""
        self.is_navigating = False
        logger.info("Navigation stopped")

    def update(self,
               current_position: Tuple[float, float, float],
               current_heading: float,
               current_distance_reading: float) -> Dict:
        """
        Update navigation state and get guidance

        Returns guidance dict with:
        - target_waypoint: Next waypoint to reach
        - distance_to_waypoint: Distance to next waypoint
        - heading_correction: How much to adjust heading
        - distance_match: How well current reading matches expected
        - status: 'on_track', 'off_course', 'approaching', 'reached', 'completed'
        """
        if not self.is_navigating or not self.current_profile:
            return {'status': 'not_navigating'}

        if self.current_waypoint_index >= len(self.current_profile.waypoints):
            return {'status': 'completed', 'progress': 100.0}

        # Get target waypoint
        target_wp = self.current_profile.waypoints[self.current_waypoint_index]

        # Calculate distance to waypoint
        current_pos = np.array(current_position)
        target_pos = np.array(target_wp.position)
        self.distance_to_waypoint = np.linalg.norm(current_pos - target_pos)

        # Calculate heading error
        self.heading_error = self._normalize_angle(target_wp.heading - current_heading)

        # Calculate distance reading deviation
        self.distance_deviation = abs(current_distance_reading - target_wp.distance_reading)

        # Determine status
        status = 'on_track'
        if self.distance_to_waypoint < self.config.waypoint_distance_threshold:
            status = 'reached'
            self.current_waypoint_index += 1
        elif self.distance_to_waypoint < self.config.waypoint_distance_threshold * 2:
            status = 'approaching'
        elif abs(self.heading_error) > self.config.heading_tolerance * 2:
            status = 'off_course'

        # Calculate progress
        self.progress_percent = (self.current_waypoint_index / len(self.current_profile.waypoints)) * 100

        return {
            'status': status,
            'target_waypoint': target_wp.to_dict(),
            'waypoint_index': self.current_waypoint_index,
            'total_waypoints': len(self.current_profile.waypoints),
            'distance_to_waypoint': round(self.distance_to_waypoint, 3),
            'heading_error': round(self.heading_error, 1),
            'heading_correction': 'left' if self.heading_error > 0 else 'right',
            'distance_deviation': round(self.distance_deviation, 3),
            'progress': round(self.progress_percent, 1),
            'expected_distance': target_wp.distance_reading,
            'current_distance': current_distance_reading
        }

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to -180 to 180 degrees"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def get_status(self) -> dict:
        """Get navigation status"""
        return {
            'is_navigating': self.is_navigating,
            'profile_name': self.current_profile.name if self.current_profile else None,
            'current_waypoint': self.current_waypoint_index,
            'total_waypoints': len(self.current_profile.waypoints) if self.current_profile else 0,
            'progress': round(self.progress_percent, 1),
            'distance_to_waypoint': round(self.distance_to_waypoint, 3)
        }
