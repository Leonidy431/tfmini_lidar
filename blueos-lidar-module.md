# BlueOS LiDAR SLAM Extension для TFmini-S

## Структура проекта

```
blueos-lidar-slam-extension/
├── Dockerfile
├── docker-compose.yml
├── requirements.txt
├── app/
│   ├── main.py
│   ├── config.py
│   ├── lidar_driver.py
│   ├── slam_engine.py
│   ├── map_manager.py
│   ├── localization.py
│   └── web/
│       ├── server.py
│       ├── templates/
│       │   └── index.html
│       └── static/
│           ├── css/
│           └── js/
├── tests/
│   └── test_lidar.py
└── README.md
```

## Версия 1: Основной модуль

### 1. Dockerfile

```dockerfile
FROM python:3.11-slim-bullseye

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    libatlas-base-dev \
    libjasper-dev \
    libtiff5 \
    libharfbuzz0b \
    libwebp6 \
    libjasper1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Копирование requirements
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Копирование приложения
COPY app/ .

# Expose ports
EXPOSE 5000 8000

# Labels для BlueOS
LABEL version="1.0.0"
LABEL permissions='{"HostConfig": {"Binds": ["/usr/blueos/extensions/lidar-slam:/app/data"], "Privileged": true, "Devices": [{"PathOnHost": "/dev/ttyUSB0", "PathInContainer": "/dev/ttyUSB0", "CgroupPermissions": "rwm"}]}}'
LABEL type="device-integration"
LABEL tags="mapping navigation positioning"
LABEL company='{"name": "Custom", "website": ""}'
LABEL authors='["You"]'

CMD ["python", "main.py"]
```

### 2. requirements.txt

```txt
flask==3.0.0
flask-cors==4.0.0
pyserial==3.5
numpy==1.24.3
scipy==1.11.2
opencv-python==4.8.0.76
open3d==0.17.0
scikit-image==0.21.0
requests==2.31.0
websocket-client==1.6.2
paramiko==3.3.1
pymavlink==2.4.37
aiohttp==3.8.5
asyncio==3.4.3
```

### 3. config.py

```python
import os
from dataclasses import dataclass
from typing import Optional

@dataclass
class LiDARConfig:
    """Конфигурация TFmini-S"""
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout: float = 1.0
    frequency: int = 100  # Hz
    max_range: float = 12.0  # метры
    min_range: float = 0.05  # метры

@dataclass
class SLAMConfig:
    """Конфигурация SLAM алгоритма"""
    voxel_size: float = 0.05  # размер вокселя в метрах
    max_depth: float = 12.0
    min_depth: float = 0.05
    icp_threshold: float = 0.0001  # порог сходимости ICP
    max_correspondence_distance: float = 0.5
    
@dataclass
class LocalizationConfig:
    """Конфигурация локализации"""
    enable_icp_refinement: bool = True
    confidence_threshold: float = 0.7
    map_matching_distance: float = 2.0
    update_frequency: int = 10  # Hz
    
@dataclass
class BlueOSConfig:
    """Интеграция с BlueOS"""
    vehicle_host: str = "localhost"
    vehicle_port: int = 14550
    telemetry_enabled: bool = True
    mavlink_heartbeat_rate: int = 1  # Hz

class Config:
    """Главная конфигурация"""
    DEBUG = os.getenv("DEBUG", "False") == "True"
    DATA_DIR = "/app/data"
    MAPS_DIR = os.path.join(DATA_DIR, "maps")
    LOGS_DIR = os.path.join(DATA_DIR, "logs")
    
    lidar = LiDARConfig()
    slam = SLAMConfig()
    localization = LocalizationConfig()
    blueos = BlueOSConfig()
    
    @staticmethod
    def init_directories():
        """Инициализация директорий"""
        os.makedirs(Config.MAPS_DIR, exist_ok=True)
        os.makedirs(Config.LOGS_DIR, exist_ok=True)

config = Config()
config.init_directories()
```

### 4. lidar_driver.py

```python
import serial
import threading
import time
import struct
from typing import Optional, Callable
from dataclasses import dataclass
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

@dataclass
class LiDARReading:
    """Структура данных с LiDAR"""
    distance: float  # см
    signal_strength: int  # 0-255
    timestamp: datetime
    temperature: float  # °C
    valid: bool = True
    
    def to_dict(self):
        return {
            'distance': self.distance,
            'signal_strength': self.signal_strength,
            'timestamp': self.timestamp.isoformat(),
            'temperature': self.temperature,
            'valid': self.valid
        }

class TFminiSDriver:
    """Драйвер для Benewake TFmini-S LiDAR"""
    
    # UART протокол
    FRAME_HEADER = [0x59, 0x59]
    FRAME_LENGTH = 9
    BAUD_RATE = 115200
    
    def __init__(self, port: str, timeout: float = 1.0):
        self.port = port
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.is_running = False
        self.thread: Optional[threading.Thread] = None
        self.callbacks = []
        self.buffer = bytearray()
        
    def connect(self) -> bool:
        """Подключение к датчику"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.BAUD_RATE,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            logger.info(f"✓ Подключено к TFmini-S на {self.port}")
            
            # Очистка буфера
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            time.sleep(0.5)
            
            return True
            
        except serial.SerialException as e:
            logger.error(f"✗ Ошибка подключения: {e}")
            return False
    
    def disconnect(self):
        """Отключение от датчика"""
        if self.serial_conn:
            self.serial_conn.close()
            logger.info("✓ Отключено от TFmini-S")
    
    def start(self):
        """Запуск читаемого потока"""
        if not self.serial_conn or not self.serial_conn.is_open:
            if not self.connect():
                raise RuntimeError("Не удалось подключиться к TFmini-S")
        
        self.is_running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        logger.info("✓ Поток чтения запущен")
    
    def stop(self):
        """Остановка потока чтения"""
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self.disconnect()
        logger.info("✓ Поток чтения остановлен")
    
    def add_callback(self, callback: Callable[[LiDARReading], None]):
        """Добавление callback для новых данных"""
        self.callbacks.append(callback)
    
    def _read_loop(self):
        """Основной цикл чтения"""
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    byte = self.serial_conn.read(1)
                    if byte:
                        self.buffer.extend(byte)
                        self._process_buffer()
                else:
                    time.sleep(0.001)
                    
            except Exception as e:
                logger.error(f"Ошибка чтения: {e}")
                time.sleep(0.1)
    
    def _process_buffer(self):
        """Обработка буфера данных"""
        while len(self.buffer) >= self.FRAME_LENGTH:
            # Поиск заголовка кадра
            if self.buffer[0] != self.FRAME_HEADER[0]:
                self.buffer.pop(0)
                continue
            
            if len(self.buffer) < 2 or self.buffer[1] != self.FRAME_HEADER[1]:
                if len(self.buffer) >= 2:
                    self.buffer.pop(0)
                else:
                    break
                continue
            
            # Извлечение данных из кадра
            if len(self.buffer) >= self.FRAME_LENGTH:
                frame = bytes(self.buffer[:self.FRAME_LENGTH])
                self.buffer = self.buffer[1:]  # Сдвиг на следующий возможный кадр
                
                reading = self._parse_frame(frame)
                if reading and reading.valid:
                    for callback in self.callbacks:
                        try:
                            callback(reading)
                        except Exception as e:
                            logger.error(f"Ошибка в callback: {e}")
    
    def _parse_frame(self, frame: bytes) -> Optional[LiDARReading]:
        """Парсинг кадра TFmini-S"""
        try:
            if len(frame) < self.FRAME_LENGTH:
                return None
            
            # Формат: [0x59, 0x59, dist_l, dist_h, strength, reserved, signal, checksum]
            dist_raw = (frame[3] << 8) | frame[2]
            strength = frame[4]
            temperature_raw = frame[6]
            
            # Проверка контрольной суммы
            checksum = sum(frame[:-1]) & 0xFF
            if checksum != frame[8]:
                return None
            
            # Конвертация
            distance_cm = float(dist_raw)
            temperature = temperature_raw - 25.0  # примерная формула
            
            return LiDARReading(
                distance=distance_cm / 100.0,  # метры
                signal_strength=strength,
                timestamp=datetime.now(),
                temperature=temperature,
                valid=distance_cm > 0
            )
            
        except (IndexError, struct.error):
            return None
    
    def get_single_reading(self) -> Optional[LiDARReading]:
        """Синхронное получение одного измерения"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            if self.serial_conn.in_waiting >= self.FRAME_LENGTH:
                frame = self.serial_conn.read(self.FRAME_LENGTH)
                reading = self._parse_frame(bytes(frame))
                if reading and reading.valid:
                    return reading
            time.sleep(0.001)
        
        return None
    
    def set_frequency(self, frequency: int) -> bool:
        """
        Установка частоты измерений (1-1000 Hz)
        Команда: 0x4B [freq_low] [freq_high] 0x00 checksum
        """
        if not (1 <= frequency <= 1000):
            logger.error("Частота должна быть 1-1000 Hz")
            return False
        
        try:
            cmd = bytearray([0x4B, frequency & 0xFF, (frequency >> 8) & 0xFF, 0x00])
            checksum = sum(cmd) & 0xFF
            cmd.append(checksum)
            
            self.serial_conn.write(cmd)
            logger.info(f"✓ Частота установлена на {frequency} Hz")
            return True
            
        except Exception as e:
            logger.error(f"Ошибка установки частоты: {e}")
            return False
```

### 5. slam_engine.py

```python
import numpy as np
import open3d as o3d
from typing import Optional, Tuple, List
from datetime import datetime
import logging
from config import Config

logger = logging.getLogger(__name__)

class PointCloud:
    """Облако точек с метаданными"""
    def __init__(self, points: np.ndarray, timestamp: datetime):
        self.points = points
        self.timestamp = timestamp
        self.pcd = None
        self._create_o3d_cloud()
    
    def _create_o3d_cloud(self):
        """Создание Open3D облака точек"""
        if len(self.points) > 0:
            self.pcd = o3d.geometry.PointCloud()
            self.pcd.points = o3d.utility.Vector3dVector(self.points)

class SLAMEngine:
    """Двигатель SLAM на основе Scan-to-Scan ICP"""
    
    def __init__(self, config=None):
        self.config = config or Config.slam
        self.reference_cloud: Optional[PointCloud] = None
        self.current_cloud: Optional[PointCloud] = None
        self.poses: List[np.ndarray] = []  # Последовательность преобразований
        self.accumulated_cloud: Optional[o3d.geometry.PointCloud] = None
        
    def process_scan(self, points_3d: np.ndarray, timestamp: datetime) -> Tuple[bool, np.ndarray]:
        """
        Обработка одного скана
        Returns: (success, transformation_matrix)
        """
        if len(points_3d) == 0:
            return False, np.eye(4)
        
        # Создание облака точек
        current_cloud = PointCloud(points_3d, timestamp)
        
        if self.reference_cloud is None:
            # Первое облако - используем как референс
            self.reference_cloud = current_cloud
            self.accumulated_cloud = current_cloud.pcd
            self.poses.append(np.eye(4))
            logger.info(f"✓ Инициализирована первая сканирование ({len(points_3d)} точек)")
            return True, np.eye(4)
        
        # ICP регистрация
        transformation, success = self._register_clouds(
            self.reference_cloud.pcd,
            current_cloud.pcd
        )
        
        if success:
            # Трансформация текущего облака
            current_cloud.pcd.transform(transformation)
            
            # Добавление в общее облако
            self.accumulated_cloud = self.accumulated_cloud + current_cloud.pcd
            
            # Ограничение размера облака (прореживание)
            if len(self.accumulated_cloud.points) > 100000:
                self.accumulated_cloud = self.accumulated_cloud.voxel_down_sample(
                    voxel_size=self.config.voxel_size * 2
                )
            
            self.poses.append(transformation)
            self.current_cloud = current_cloud
            
            logger.info(f"✓ Облако зарегистрировано (смещение: {np.linalg.norm(transformation[:3, 3]):.3f} м)")
            return True, transformation
        else:
            logger.warning("✗ Регистрация облака не удалась")
            return False, np.eye(4)
    
    def _register_clouds(self, source: o3d.geometry.PointCloud, 
                        target: o3d.geometry.PointCloud) -> Tuple[np.ndarray, bool]:
        """ICP регистрация облаков"""
        try:
            # Даунсемплинг для ускорения
            source_down = source.voxel_down_sample(voxel_size=self.config.voxel_size)
            target_down = target.voxel_down_sample(voxel_size=self.config.voxel_size)
            
            # Быстрая ICP регистрация
            result = o3d.pipelines.registration.registration_icp(
                source_down,
                target_down,
                self.config.max_correspondence_distance,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=50,
                    relative_fitness=1e-6,
                    relative_rmse=1e-6
                )
            )
            
            success = result.fitness > self.config.icp_threshold
            return result.transformation, success
            
        except Exception as e:
            logger.error(f"Ошибка ICP: {e}")
            return np.eye(4), False
    
    def get_map(self) -> Optional[np.ndarray]:
        """Получить текущую карту"""
        if self.accumulated_cloud is None:
            return None
        return np.asarray(self.accumulated_cloud.points)
    
    def save_map(self, filename: str) -> bool:
        """Сохранение карты"""
        try:
            if self.accumulated_cloud is None:
                logger.error("Нет карты для сохранения")
                return False
            
            o3d.io.write_point_cloud(filename, self.accumulated_cloud)
            logger.info(f"✓ Карта сохранена: {filename}")
            return True
        except Exception as e:
            logger.error(f"Ошибка сохранения карты: {e}")
            return False
    
    def load_map(self, filename: str) -> bool:
        """Загрузка сохранённой карты"""
        try:
            self.accumulated_cloud = o3d.io.read_point_cloud(filename)
            logger.info(f"✓ Карта загружена: {filename}")
            return True
        except Exception as e:
            logger.error(f"Ошибка загрузки карты: {e}")
            return False
```

### 6. localization.py

```python
import numpy as np
import open3d as o3d
from typing import Tuple, Optional
import logging

logger = logging.getLogger(__name__)

class LocalizationEngine:
    """Локализация дрона на известной карте"""
    
    def __init__(self, reference_map: np.ndarray, config=None):
        self.reference_pcd = self._numpy_to_pcd(reference_map)
        self.config = config
        self.last_pose = np.eye(4)
        self.confidence = 0.0
        
    def _numpy_to_pcd(self, points: np.ndarray) -> o3d.geometry.PointCloud:
        """Конвертация numpy в Open3D"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd
    
    def localize(self, current_scan: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Локализация дрона по текущему скану
        Returns: (pose_4x4, confidence)
        """
        current_pcd = self._numpy_to_pcd(current_scan)
        
        try:
            # Предварительное выравнивание (без начального предположения)
            # Использование RANSAC для надёжности
            result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                current_pcd,
                self.reference_pcd,
                o3d.pipelines.registration.FastGlobalRegistrationOption(
                    maximum_correspondence_distance=self.config.map_matching_distance,
                    iteration_number=1000
                )
            )
            
            if result.fitness < self.config.confidence_threshold:
                logger.warning(f"⚠ Низкая уверенность локализации: {result.fitness:.3f}")
                return self.last_pose, result.fitness
            
            # Уточнение с помощью ICP
            if self.config.enable_icp_refinement:
                result = o3d.pipelines.registration.registration_icp(
                    current_pcd,
                    self.reference_pcd,
                    self.config.map_matching_distance,
                    result.transformation,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(
                        max_iteration=100
                    )
                )
            
            self.last_pose = result.transformation
            self.confidence = result.fitness
            
            position = result.transformation[:3, 3]
            logger.info(f"✓ Локализация: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f} (уверенность: {result.fitness:.3f})")
            
            return result.transformation, result.fitness
            
        except Exception as e:
            logger.error(f"Ошибка локализации: {e}")
            return self.last_pose, 0.0
    
    def get_position(self) -> Tuple[float, float, float]:
        """Получить текущую XYZ позицию"""
        return tuple(self.last_pose[:3, 3])
```

### 7. map_manager.py

```python
import os
import json
import numpy as np
from datetime import datetime
from typing import List, Dict, Optional
import logging

logger = logging.getLogger(__name__)

class MapMetadata:
    """Метаданные карты"""
    def __init__(self, name: str, created: datetime):
        self.name = name
        self.created = created
        self.point_count = 0
        self.bounds = {"min": [0, 0, 0], "max": [0, 0, 0]}
        self.description = ""
    
    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'created': self.created.isoformat(),
            'point_count': self.point_count,
            'bounds': self.bounds,
            'description': self.description
        }

class MapManager:
    """Управление картами местности"""
    
    def __init__(self, maps_dir: str):
        self.maps_dir = maps_dir
        os.makedirs(maps_dir, exist_ok=True)
    
    def save_map(self, name: str, points: np.ndarray, description: str = "") -> bool:
        """Сохранение карты"""
        try:
            # Создание директории для карты
            map_dir = os.path.join(self.maps_dir, name)
            os.makedirs(map_dir, exist_ok=True)
            
            # Сохранение облака точек
            points_file = os.path.join(map_dir, "points.npy")
            np.save(points_file, points)
            
            # Сохранение метаданных
            metadata = MapMetadata(name, datetime.now())
            metadata.point_count = len(points)
            if len(points) > 0:
                metadata.bounds['min'] = points.min(axis=0).tolist()
                metadata.bounds['max'] = points.max(axis=0).tolist()
            metadata.description = description
            
            metadata_file = os.path.join(map_dir, "metadata.json")
            with open(metadata_file, 'w') as f:
                json.dump(metadata.to_dict(), f, indent=2)
            
            logger.info(f"✓ Карта '{name}' сохранена ({len(points)} точек)")
            return True
            
        except Exception as e:
            logger.error(f"Ошибка сохранения карты: {e}")
            return False
    
    def load_map(self, name: str) -> Optional[np.ndarray]:
        """Загрузка карты"""
        try:
            points_file = os.path.join(self.maps_dir, name, "points.npy")
            if not os.path.exists(points_file):
                logger.error(f"Карта '{name}' не найдена")
                return None
            
            points = np.load(points_file)
            logger.info(f"✓ Карта '{name}' загружена ({len(points)} точек)")
            return points
            
        except Exception as e:
            logger.error(f"Ошибка загрузки карты: {e}")
            return None
    
    def list_maps(self) -> List[Dict]:
        """Список всех сохранённых карт"""
        maps = []
        try:
            for map_name in os.listdir(self.maps_dir):
                map_dir = os.path.join(self.maps_dir, map_name)
                metadata_file = os.path.join(map_dir, "metadata.json")
                
                if os.path.isfile(metadata_file):
                    with open(metadata_file, 'r') as f:
                        maps.append(json.load(f))
        except Exception as e:
            logger.error(f"Ошибка получения списка карт: {e}")
        
        return maps
    
    def delete_map(self, name: str) -> bool:
        """Удаление карты"""
        try:
            map_dir = os.path.join(self.maps_dir, name)
            import shutil
            shutil.rmtree(map_dir)
            logger.info(f"✓ Карта '{name}' удалена")
            return True
        except Exception as e:
            logger.error(f"Ошибка удаления карты: {e}")
            return False
```

---

## Интеграция с BlueOS и Web интерфейс

### 8. main.py

```python
import logging
import asyncio
from datetime import datetime
from flask import Flask, jsonify, request
from flask_cors import CORS
import numpy as np
from typing import Dict, List

# Импорт модулей
from config import Config
from lidar_driver import TFminiSDriver, LiDARReading
from slam_engine import SLAMEngine
from localization import LocalizationEngine
from map_manager import MapManager
from web.server import create_web_app

# Логирование
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class LiDARSLAMApplication:
    """Основное приложение"""
    
    def __init__(self):
        self.config = Config
        self.driver = None
        self.slam_engine = SLAMEngine(self.config.slam)
        self.map_manager = MapManager(self.config.MAPS_DIR)
        self.localization_engine = None
        
        self.scan_buffer: List[LiDARReading] = []
        self.mode = "mapping"  # "mapping" или "localizing"
        self.is_running = False
        
    def initialize(self):
        """Инициализация"""
        logger.info("=" * 50)
        logger.info("BlueOS LiDAR SLAM Extension v1.0.0")
        logger.info("=" * 50)
        
        # Инициализация драйвера LiDAR
        self.driver = TFminiSDriver(
            port=self.config.lidar.port,
            timeout=self.config.lidar.timeout
        )
        
        if not self.driver.connect():
            logger.error("✗ Не удалось подключиться к LiDAR")
            return False
        
        # Установка частоты
        self.driver.set_frequency(self.config.lidar.frequency)
        
        # Добавление обработчика данных
        self.driver.add_callback(self._on_lidar_reading)
        
        logger.info("✓ Приложение инициализировано")
        return True
    
    def start(self):
        """Запуск приложения"""
        if not self.driver:
            if not self.initialize():
                return False
        
        try:
            self.driver.start()
            self.is_running = True
            logger.info("✓ Приложение запущено")
            return True
        except Exception as e:
            logger.error(f"✗ Ошибка запуска: {e}")
            return False
    
    def stop(self):
        """Остановка приложения"""
        self.is_running = False
        if self.driver:
            self.driver.stop()
        logger.info("✓ Приложение остановлено")
    
    def _on_lidar_reading(self, reading: LiDARReading):
        """Callback при получении данных с LiDAR"""
        if not reading.valid or reading.distance < self.config.lidar.min_range:
            return
        
        # Преобразование в 3D точку (в цилиндрической системе координат)
        # Предполагаем, что LiDAR смотрит вперёд (Z-ось)
        angle = np.arctan2(reading.distance, 1.0)  # Угол в плоскости XZ
        x = reading.distance * np.cos(angle)
        z = reading.distance * np.sin(angle)
        y = 0  # Отсутствие боковых измерений в одноточечном LiDAR
        
        self.scan_buffer.append((x, y, z, reading.signal_strength))
        
        # Если накопилось достаточно точек - обработать скан
        if len(self.scan_buffer) >= 100:
            self._process_scan()
    
    def _process_scan(self):
        """Обработка скана"""
        if len(self.scan_buffer) == 0:
            return
        
        # Конвертация в numpy array
        points = np.array(self.scan_buffer)[:, :3]
        self.scan_buffer.clear()
        
        if self.mode == "mapping":
            self._process_mapping_scan(points)
        elif self.mode == "localizing" and self.localization_engine:
            self._process_localization_scan(points)
    
    def _process_mapping_scan(self, points: np.ndarray):
        """Обработка скана в режиме создания карты"""
        success, transformation = self.slam_engine.process_scan(
            points, datetime.now()
        )
        
        if success:
            logger.info(f"✓ Скан обработан ({len(points)} точек)")
    
    def _process_localization_scan(self, points: np.ndarray):
        """Обработка скана в режиме локализации"""
        pose, confidence = self.localization_engine.localize(points)
        
        if confidence > self.config.localization.confidence_threshold:
            x, y, z = self.localization_engine.get_position()
            logger.info(f"✓ Позиция: ({x:.2f}, {y:.2f}, {z:.2f}) - уверенность: {confidence:.2%}")
    
    def get_status(self) -> Dict:
        """Получить статус приложения"""
        map_points = self.slam_engine.get_map()
        return {
            'running': self.is_running,
            'mode': self.mode,
            'mapped_points': len(map_points) if map_points is not None else 0,
            'mapped_poses': len(self.slam_engine.poses),
            'lidar_connected': self.driver and self.driver.serial_conn and self.driver.serial_conn.is_open
        }

# Глобальное приложение
app_instance = LiDARSLAMApplication()

# Flask API
flask_app = Flask(__name__)
CORS(flask_app)

@flask_app.route('/api/register_service', methods=['GET'])
def register_service():
    """BlueOS регистрация сервиса"""
    return jsonify({
        "name": "LiDAR SLAM Module",
        "description": "Real-time mapping and localization with TFmini-S LiDAR",
        "version": "1.0.0",
        "company": "Custom",
        "webpage": "/extension/lidar-slam/",
        "api": "/api/"
    })

@flask_app.route('/api/status', methods=['GET'])
def get_status():
    """Статус приложения"""
    return jsonify(app_instance.get_status())

@flask_app.route('/api/start', methods=['POST'])
def start():
    """Запуск"""
    if app_instance.start():
        return jsonify({'success': True}), 200
    return jsonify({'success': False}), 500

@flask_app.route('/api/stop', methods=['POST'])
def stop():
    """Остановка"""
    app_instance.stop()
    return jsonify({'success': True}), 200

@flask_app.route('/api/mode/<mode>', methods=['POST'])
def set_mode(mode):
    """Установка режима"""
    if mode in ['mapping', 'localizing']:
        app_instance.mode = mode
        return jsonify({'success': True, 'mode': mode}), 200
    return jsonify({'success': False, 'error': 'Invalid mode'}), 400

@flask_app.route('/api/map/save/<name>', methods=['POST'])
def save_map(name):
    """Сохранение карты"""
    map_points = app_instance.slam_engine.get_map()
    if map_points is None:
        return jsonify({'success': False, 'error': 'No map to save'}), 400
    
    description = request.json.get('description', '') if request.json else ''
    success = app_instance.map_manager.save_map(name, map_points, description)
    return jsonify({'success': success}), 200 if success else 500

@flask_app.route('/api/maps', methods=['GET'])
def list_maps():
    """Список карт"""
    maps = app_instance.map_manager.list_maps()
    return jsonify(maps), 200

@flask_app.route('/api/map/load/<name>', methods=['POST'])
def load_map(name):
    """Загрузка карты для локализации"""
    points = app_instance.map_manager.load_map(name)
    if points is None:
        return jsonify({'success': False}), 404
    
    app_instance.localization_engine = LocalizationEngine(
        points, app_instance.config.localization
    )
    app_instance.mode = 'localizing'
    
    return jsonify({'success': True, 'points': len(points)}), 200

@flask_app.route('/api/position', methods=['GET'])
def get_position():
    """Текущая позиция"""
    if app_instance.localization_engine:
        x, y, z = app_instance.localization_engine.get_position()
        confidence = app_instance.localization_engine.confidence
        return jsonify({
            'x': x, 'y': y, 'z': z,
            'confidence': confidence
        }), 200
    return jsonify({'error': 'Not in localization mode'}), 400

if __name__ == '__main__':
    try:
        # Инициализация
        app_instance.initialize()
        app_instance.start()
        
        # Запуск Flask сервера
        flask_app.run(host='0.0.0.0', port=5000, debug=False)
        
    except KeyboardInterrupt:
        logger.info("\nПрограмма завершена пользователем")
    except Exception as e:
        logger.error(f"Критическая ошибка: {e}")
    finally:
        app_instance.stop()
```

---

## Использование модуля

**Запуск в BlueOS:**
1. Сохраните структуру проекта
2. Создайте Docker образ: `docker build -t yourusername/blueos-lidar-slam .`
3. Загрузите на Docker Hub
4. В BlueOS → Extensions Manager → найдите ваше расширение
5. Установите и откройте веб-интерфейс

**API примеры:**
```bash
# Старт сканирования
curl -X POST http://blueos.local:5000/api/start

# Создание карты
curl -X POST http://blueos.local:5000/api/mode/mapping

# Сохранение карты
curl -X POST http://blueos.local:5000/api/map/save/basement_survey \
  -H "Content-Type: application/json" \
  -d '{"description": "First basement survey"}'

# Переключение на локализацию
curl -X POST http://blueos.local:5000/api/map/load/basement_survey

# Получить позицию
curl http://blueos.local:5000/api/position
```

---

## Следующие улучшения

- Multi-beam LiDAR интеграция (РМ300, Livox)
- Визуализация облака точек в 3D
- Интеграция с MAVProxy для автоматизации миссий
- Оптимизация SLAM для летящего БПЛА
- Экспорт в ROS
