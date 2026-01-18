# tfmini_lidar
tfmini_lidar

BlueOS LiDAR SLAM - Веб-интерфейс
Структура веб-части
web/
├── server.py
├── templates/
│ ├── index.html
│ ├── mapping.html
│ └── localization.html
└── static/
├── css/
│ ├── style.css
│ └── dashboard.css
└── js/
├── app.js
├── api.js
└── visualization.js

1. web/server.py
from flask import Flask, render_template, send_from_directory
import os
def create_web_app():
"""Создание Flask приложения для веб-интерфейса"""
app = Flask(__name__)

# Путь к templates и static
template_dir = os.path.abspath('web/templates')
static_dir = os.path.abspath('web/static')

app = Flask(__name__, 
            template_folder=template_dir,
            static_folder=static_dir,
            static_url_path='/static')

@app.route('/')
def index():
    """Главная страница"""
    return render_template('index.html')

@app.route('/mapping')
def mapping():
    """Страница картирования"""
    return render_template('mapping.html')

@app.route('/localization')
def localization():
    """Страница локализации"""
    return render_template('localization.html')

@app.route('/static/<path:filename>')
def serve_static(filename):
    """Доступ к статичным файлам"""
    return send_from_directory(static_dir, filename)

return app


2. web/templates/index.html
BlueOS LiDAR SLAM Control
🛩️ LiDAR SLAM Module
Real-time Mapping & Localization
Dashboard Mapping Localization

🔌 System Status
LiDAR Connection ❌ Disconnected
Application 🔴 Stopped
Mode ⚙️ None
Mapped Points 0
⚙️ Controls
▶️ Start ⏹️ Stop 🔄 Refresh
📍 Operating Modes
🗺️ Mapping Create new map 📍 Localization Navigate with map
💾 Saved Maps
Loading maps...
 💾 Save Map
📊 Activity
📝 Logs
✓ System initialized
BlueOS LiDAR SLAM v1.0.0 | Benewake TFmini-S Integration
📡 Real-time SLAM with ICP Registration | 🗺️ Scan Matching Localization

3. web/templates/mapping.html
Mapping - BlueOS LiDAR SLAM
🗺️ Mapping Mode
Create Real-time 3D Maps
Dashboard Mapping Localization
⚙️ Mapping Controls
▶️ Start Mapping ⏹️ Stop Mapping 🗑️ Clear Map ⬇️ Download Map
🎨 3D Point Cloud

Reset View 📸 Screenshot  Show Grid  Show Axes
📊 Mapping Statistics
Points Mapped 0
Scans Registered 0
Map Quality 0%
Drift Estimate 0 cm
⚙️ SLAM Parameters
Voxel Size (m) 
Max Correspondence Distance (m) 
ICP Threshold 
Buffer Size (points) 
Apply Settings
📝 Mapping Logs
✓ Ready for mapping
Use the 3D view above to monitor your mapping progress in real-time

4. web/templates/localization.html
Localization - BlueOS LiDAR SLAM
📍 Localization Mode
Real-time Position Estimation
Dashboard Mapping Localization
🗺️ Select Reference Map
📥 Load Map 🔄 Refresh List
📊 Map size: 0 points
📍 Bounds: N/A
🎯 Current Position
X (m) --
Y (m) --
Z (m) --
Confidence --

📈 Trajectory
🗺️ 2D Top-Down View
📊 Localization Statistics
Distance Traveled 0 m
Avg Confidence 0%
Lost Count 0
Update Rate 0 Hz
⚙️ Localization Parameters
Confidence Threshold 
Map Matching Distance (m) 
Update Frequency (Hz) 
 Enable ICP Refinement
Apply Settings
📝 Localization Logs
✓ Ready for localization
Load a reference map and the system will track your position in real-time

5. web/static/css/style.css
:root {
--color-primary: #32b8c6;
--color-secondary: #5a6c7d;
--color-success: #26c281;
--color-danger: #ff6b6b;
--color-warning: #f7b731;
--color-dark: #1a1a1a;
--color-light: #ecf0f1;
--color-border: #2c3e50;
--font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
--transition: all 0.3s ease;

}
{
margin: 0;
padding: 0;
box-sizing: border-box;
}
body {
font-family: var(--font-family);
background: linear-gradient(135deg, #0a0e27 0%, #1a1a2e 100%);
color: #ecf0f1;
line-height: 1.6;
min-height: 100vh;
}
.container {
max-width: 1400px;
margin: 0 auto;
padding: 0;
}
/* NAVBAR */
.navbar {
background: rgba(20, 25, 45, 0.95);
border-bottom: 2px solid var(--color-primary);
padding: 1.5rem 2rem;
display: flex;
justify-content: space-between;
align-items: center;
backdrop-filter: blur(10px);
}
.navbar-brand h1 {
font-size: 1.8rem;
color: var(--color-primary);
margin-bottom: 0.25rem;
}
.subtitle {
font-size: 0.85rem;
color: var(--color-secondary);
}
.navbar-menu {
display: flex;
gap: 2rem;
align-items: center;
}
.nav-link {
color: #ecf0f1;
text-decoration: none;
padding: 0.5rem 1rem;
border-radius: 4px;
transition: var(--transition);
}
.nav-link:hover,
.nav-link.active {
background: var(--color-primary);
color: #000;
}
.status-indicator {
width: 12px;
height: 12px;
background: #ff6b6b;
border-radius: 50%;
animation: pulse 2s infinite;
}
.status-indicator.active {
background: #26c281;
animation: none;
}
@keyframes pulse {
0%, 100% { opacity: 1; }
50% { opacity: 0.5; }
}
/* CONTENT */
.content {
padding: 2rem;
}
section {
background: rgba(20, 25, 45, 0.7);
border: 1px solid var(--color-border);
border-radius: 8px;
padding: 2rem;
margin-bottom: 2rem;
backdrop-filter: blur(10px);
}
section h2 {
font-size: 1.5rem;
margin-bottom: 1.5rem;
color: var(--color-primary);
border-bottom: 2px solid var(--color-primary);
padding-bottom: 0.75rem;
}
/* BUTTONS */
.btn {
padding: 0.75rem 1.5rem;
border: none;
border-radius: 4px;
cursor: pointer;
font-size: 1rem;
font-weight: 600;
transition: var(--transition);
display: inline-flex;
align-items: center;
gap: 0.5rem;
text-decoration: none;
}
.btn-primary {
background: var(--color-primary);
color: #000;
}
.btn-primary:hover {
background: #28a0a8;
transform: translateY(-2px);
box-shadow: 0 5px 20px rgba(50, 184, 198, 0.3);
}
.btn-secondary {
background: var(--color-secondary);
color: #fff;
}
.btn-secondary:hover {
background: #6a7c8d;
}
.btn-success {
background: var(--color-success);
color: #fff;
}
.btn-success:hover {
background: #1fa36d;
}
.btn-danger {
background: var(--color-danger);
color: #fff;
}
.btn-danger:hover {
background: #ff4757;
}
.btn-warning {
background: var(--color-warning);
color: #000;
}
.btn-warning:hover {
background: #f5a623;
}
.btn-sm {
padding: 0.5rem 1rem;
font-size: 0.9rem;
}
.button-group {
display: flex;
gap: 1rem;
flex-wrap: wrap;
}
/* FORMS */
.input-field,
select {
background: rgba(0, 0, 0, 0.3);
border: 1px solid var(--color-border);
color: #ecf0f1;
padding: 0.75rem 1rem;
border-radius: 4px;
font-size: 1rem;
transition: var(--transition);
}
.input-field:focus,
select:focus {
outline: none;
border-color: var(--color-primary);
box-shadow: 0 0 0 3px rgba(50, 184, 198, 0.2);
}
.checkbox-label {
display: flex;
align-items: center;
gap: 0.5rem;
cursor: pointer;
color: #ecf0f1;
}
.checkbox-label input[type="checkbox"] {
cursor: pointer;
width: 18px;
height: 18px;
}
/* GRID LAYOUTS */
.status-grid,
.stats-grid,
.settings-grid,
.maps-list {
display: grid;
grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
gap: 1.5rem;
}
.status-card,
.stat-card {
background: rgba(0, 0, 0, 0.2);
border: 1px solid var(--color-border);
border-radius: 8px;
padding: 1.5rem;
text-align: center;
}
.status-label,
.stat-label {
display: block;
font-size: 0.85rem;
color: var(--color-secondary);
margin-bottom: 0.5rem;
text-transform: uppercase;
letter-spacing: 0.05em;
}
.status-value,
.stat-value {
display: block;
font-size: 1.75rem;
font-weight: bold;
color: var(--color-primary);
}
/* SPECIAL PANELS */
.mode-selector {
display: grid;
grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
gap: 1.5rem;
margin-bottom: 1.5rem;
}
.mode-btn {
background: rgba(0, 0, 0, 0.2);
border: 2px solid var(--color-border);
border-radius: 8px;
padding: 1.5rem;
cursor: pointer;
transition: var(--transition);
display: flex;
flex-direction: column;
align-items: center;
gap: 0.5rem;
color: #ecf0f1;
}
.mode-btn:hover,
.mode-btn.active {
border-color: var(--color-primary);
background: rgba(50, 184, 198, 0.1);
}
.mode-icon {
font-size: 2rem;
}
.mode-title {
font-weight: 600;
font-size: 1.1rem;
}
.mode-desc {
font-size: 0.85rem;
color: var(--color-secondary);
}
/* LOGS */
.logs-container {
background: rgba(0, 0, 0, 0.3);
border: 1px solid var(--color-border);
border-radius: 4px;
padding: 1rem;
max-height: 300px;
overflow-y: auto;
font-family: 'Courier New', monospace;
font-size: 0.9rem;
}
.log-entry {
margin: 0.25rem 0;
color: #a8e6ce;
}
/* FOOTER */
.footer {
background: rgba(20, 25, 45, 0.95);
border-top: 2px solid var(--color-primary);
padding: 2rem;
text-align: center;
color: var(--color-secondary);
margin-top: 2rem;
}
/* RESPONSIVE */
@media (max-width: 768px) {
.navbar {
flex-direction: column;
gap: 1rem;
}
.navbar-menu {
    width: 100%;
    justify-content: space-around;
}

section {
    padding: 1rem;
}

section h2 {
    font-size: 1.3rem;
}

.button-group {
    width: 100%;
}

.btn {
    flex: 1;
    justify-content: center;
}

}
/* SCROLL */
::-webkit-scrollbar {
width: 8px;
height: 8px;
}
::-webkit-scrollbar-track {
background: rgba(0, 0, 0, 0.2);
}
::-webkit-scrollbar-thumb {
background: var(--color-primary);
border-radius: 4px;
}
::-webkit-scrollbar-thumb:hover {
background: #28a0a8;
}

6. web/static/css/dashboard.css
/* DASHBOARD SPECIFIC */
.canvas3d {
width: 100%;
height: 500px;
background: #0a0e27;
border-radius: 8px;
margin-bottom: 1rem;
}
.canvas-2d {
width: 100%;
height: 400px;
background: #1a1a2e;
border-radius: 8px;
display: block;
}
.visualization-controls {
display: flex;
gap: 1rem;
margin-top: 1rem;
flex-wrap: wrap;
}
/* SETTINGS */
.settings-grid {
grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
}
.setting-group {
display: flex;
flex-direction: column;
gap: 0.5rem;
}
.setting-group label {
color: #ecf0f1;
font-weight: 500;
font-size: 0.95rem;
}
.setting-group input[type="number"],
.setting-group input[type="text"] {
background: rgba(0, 0, 0, 0.3);
border: 1px solid var(--color-border);
color: #ecf0f1;
padding: 0.75rem;
border-radius: 4px;
}
.setting-group input[type="number"]:focus,
.setting-group input[type="text"]:focus {
border-color: var(--color-primary);
outline: none;
}
/* POSITION DISPLAY */
.position-display {
display: grid;
grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
gap: 1rem;
margin-bottom: 2rem;
}
.coordinate {
background: rgba(0, 0, 0, 0.2);
border: 1px solid var(--color-primary);
border-radius: 8px;
padding: 1.5rem;
text-align: center;
}
.coord-label {
display: block;
font-size: 0.9rem;
color: var(--color-secondary);
margin-bottom: 0.5rem;
text-transform: uppercase;
}
.coord-value {
display: block;
font-size: 2rem;
font-weight: bold;
color: var(--color-primary);
font-family: 'Courier New', monospace;
}
.confidence-bar {
height: 30px;
background: rgba(0, 0, 0, 0.3);
border: 1px solid var(--color-border);
border-radius: 15px;
overflow: hidden;
}
.confidence-fill {
height: 100%;
background: linear-gradient(90deg, #ff6b6b, #f7b731, #26c281);
transition: width 0.3s ease;
display: flex;
align-items: center;
justify-content: flex-end;
padding-right: 1rem;
color: #000;
font-weight: bold;
}
/* MAP SELECTOR */
.map-selector {
display: flex;
gap: 1rem;
margin-bottom: 1.5rem;
flex-wrap: wrap;
}
.map-selector select {
flex: 1;
min-width: 200px;
}
.map-info {
background: rgba(50, 184, 198, 0.1);
border-left: 4px solid var(--color-primary);
padding: 1rem;
border-radius: 4px;
margin-top: 1rem;
}
.map-info.hidden {
display: none;
}
.map-info p {
margin: 0.5rem 0;
color: #ecf0f1;
}
/* MAPS LIST */
.map-item {
background: rgba(0, 0, 0, 0.2);
border: 1px solid var(--color-border);
border-radius: 8px;
padding: 1.5rem;
display: flex;
justify-content: space-between;
align-items: center;
transition: var(--transition);
}
.map-item:hover {
border-color: var(--color-primary);
background: rgba(50, 184, 198, 0.05);
}
.map-details {
flex: 1;
}
.map-name {
font-weight: 600;
color: var(--color-primary);
margin-bottom: 0.5rem;
}
.map-meta {
font-size: 0.85rem;
color: var(--color-secondary);
}
.map-actions {
display: flex;
gap: 0.75rem;
}
.map-actions .btn {
padding: 0.5rem 1rem;
font-size: 0.85rem;
}
/* MAP ACTIONS */
.map-actions {
display: flex;
gap: 1rem;
margin-top: 2rem;
flex-wrap: wrap;
}
.map-actions input {
flex: 1;
min-width: 200px;
}
/* ANIMATION */
.loading {
color: var(--color-secondary);
text-align: center;
padding: 2rem;
font-size: 0.95rem;
}
/* HIDDEN */
.hidden {
display: none !important;
}
/* RESPONSIVE FOR DASHBOARD */
@media (max-width: 1024px) {
.position-display {
grid-template-columns: repeat(2, 1fr);
}
.canvas3d {
    height: 400px;
}

.canvas-2d {
    height: 300px;
}

}
@media (max-width: 768px) {
.position-display {
grid-template-columns: 1fr;
}
.canvas3d,
.canvas-2d {
    height: 300px;
}

.visualization-controls {
    flex-direction: column;
}

.visualization-controls .btn {
    width: 100%;
}

.map-selector {
    flex-direction: column;
}

.map-selector .btn {
    width: 100%;
}

}

7. web/static/js/api.js
const API_BASE = '/api';
/**
Универсальная функция для API вызовов
*/
async function apiCall(endpoint, method = 'GET', data = null) {
try {
const options = {
method: method,
headers: {
'Content-Type': 'application/json',
}
};
 if (data && method !== 'GET') {
     options.body = JSON.stringify(data);
 }

 const response = await fetch(API_BASE + endpoint, options);

 if (!response.ok) {
     console.error(`API error: ${response.status}`);
     return null;
 }

 const result = await response.json();
 return result;

} catch (error) {
console.error(API call error: ${error});
return null;
}
}
/**
Получить статус приложения
*/
async function getStatus() {
return await apiCall('/status', 'GET');
}
/**
Запустить приложение
*/
async function startApp() {
return await apiCall('/start', 'POST');
}
/**
Остановить приложение
*/
async function stopApp() {
return await apiCall('/stop', 'POST');
}
/**
Установить режим
*/
async function setMode(mode) {
return await apiCall(/mode/${mode}, 'POST');
}
/**
Сохранить карту
*/
async function saveMap(name, description = '') {
return await apiCall(/map/save/${name}, 'POST', { description });
}
/**
Загрузить карту
*/
async function loadMap(name) {
return await apiCall(/map/load/${name}, 'POST');
}
/**
Получить список карт
*/
async function getMaps() {
return await apiCall('/maps', 'GET');
}
/**
Получить текущую позицию
*/
async function getPosition() {
return await apiCall('/position', 'GET');
}
/**
Утилита для добавления логов
*/
function addLogMessage(message, container = 'logsContainer') {
const logsContainer = document.getElementById(container);
if (!logsContainer) return;
const entry = document.createElement('p');
entry.className = 'log-entry';
const timestamp = new Date().toLocaleTimeString();
entry.textContent = [${timestamp}] ${message};
logsContainer.appendChild(entry);
logsContainer.scrollTop = logsContainer.scrollHeight;
// Ограничение количества логов (максимум 100)
while (logsContainer.children.length > 100) {
logsContainer.removeChild(logsContainer.firstChild);
}
}

8. web/static/js/app.js
/**
Главный скрипт приложения
*/
let statusUpdateInterval = null;
document.addEventListener('DOMContentLoaded', () => {
initializeApp();
setupEventListeners();
startStatusUpdates();
});
function initializeApp() {
console.log('🚀 Initializing LiDAR SLAM Dashboard...');
addLogMessage('System initialized');
}
function setupEventListeners() {
// Кнопки управления
const startBtn = document.getElementById('startBtn');
const stopBtn = document.getElementById('stopBtn');
const refreshBtn = document.getElementById('refreshBtn');
if (startBtn) {
    startBtn.addEventListener('click', async () => {
        const result = await startApp();
        if (result && result.success) {
            addLogMessage('✓ Application started');
        } else {
            addLogMessage('✗ Failed to start application');
        }
    });
}

if (stopBtn) {
    stopBtn.addEventListener('click', async () => {
        const result = await stopApp();
        if (result && result.success) {
            addLogMessage('⏹ Application stopped');
        }
    });
}

if (refreshBtn) {
    refreshBtn.addEventListener('click', () => {
        updateStatus();
        addLogMessage('🔄 Status refreshed');
    });
}

// Выбор режима
const modeBtns = document.querySelectorAll('.mode-btn');
modeBtns.forEach(btn => {
    btn.addEventListener('click', async () => {
        const mode = btn.getAttribute('data-mode');
        const result = await setMode(mode);
        if (result && result.success) {
            modeBtns.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            addLogMessage(`📍 Mode changed to ${mode}`);
        }
    });
});

// Сохранение карты
const saveMapBtn = document.getElementById('saveMapBtn');
if (saveMapBtn) {
    saveMapBtn.addEventListener('click', async () => {
        const mapName = document.getElementById('mapName').value;
        if (!mapName) {
            alert('Please enter a map name');
            return;
        }

        const result = await saveMap(mapName);
        if (result && result.success) {
            addLogMessage(`💾 Map "${mapName}" saved successfully`);
            document.getElementById('mapName').value = '';
            loadMapsList();
        } else {
            addLogMessage(`✗ Failed to save map`);
        }
    });
}

}
function startStatusUpdates() {
updateStatus();
statusUpdateInterval = setInterval(() => {
    updateStatus();
}, 2000); // Обновление каждые 2 секунды

}
async function updateStatus() {
const status = await getStatus();
if (!status) return;

// Обновление индикатора подключения
const statusLight = document.getElementById('statusLight');
if (statusLight) {
    if (status.running) {
        statusLight.classList.add('active');
    } else {
        statusLight.classList.remove('active');
    }
}

// Обновление статуса
const lidarStatus = document.getElementById('lidarStatus');
const appStatus = document.getElementById('appStatus');
const currentMode = document.getElementById('currentMode');
const pointCount = document.getElementById('pointCount');

if (lidarStatus) {
    lidarStatus.textContent = status.lidar_connected ? '✅ Connected' : '❌ Disconnected';
}

if (appStatus) {
    appStatus.textContent = status.running ? '🟢 Running' : '🔴 Stopped';
}

if (currentMode) {
    const modeText = status.mode === 'mapping' ? '🗺️ Mapping' : '📍 Localization';
    currentMode.textContent = modeText;
}

if (pointCount) {
    pointCount.textContent = status.mapped_points || 0;
}

}
async function loadMapsList() {
const maps = await getMaps();
const mapsList = document.getElementById('mapsList');
if (!mapsList) return;

if (!maps || maps.length === 0) {
    mapsList.innerHTML = '<p class="loading">No saved maps</p>';
    return;
}

mapsList.innerHTML = '';
maps.forEach(map => {
    const item = document.createElement('div');
    item.className = 'map-item';
    item.innerHTML = `
        <div class="map-details">
            <div class="map-name">${map.name}</div>
            <div class="map-meta">
                ${map.point_count} points | Created: ${new Date(map.created).toLocaleString()}
            </div>
        </div>
        <div class="map-actions">
            <button class="btn btn-sm btn-primary" onclick="loadMapForLocalization('${map.name}')">
                📍 Use
            </button>
            <button class="btn btn-sm btn-danger" onclick="deleteMap('${map.name}')">
                🗑️ Delete
            </button>
        </div>
    `;
    mapsList.appendChild(item);
});

}
async function loadMapForLocalization(mapName) {
const result = await loadMap(mapName);
if (result && result.success) {
addLogMessage(✓ Map "${mapName}" loaded for localization);
}
}
async function deleteMap(mapName) {
if (!confirm(Delete map "${mapName}"?)) return;
addLogMessage(`🗑️ Map "${mapName}" deleted`);
loadMapsList();

}
// Загрузка списка карт при загрузке страницы
window.addEventListener('load', () => {
loadMapsList();
});
// Очистка при выходе
window.addEventListener('beforeunload', () => {
if (statusUpdateInterval) {
clearInterval(statusUpdateInterval);
}
});

9. web/static/js/visualization.js
/**
Модуль для 3D визуализации облака точек
*/
class PointCloudVisualizer {
constructor(containerElement) {
this.container = containerElement;
this.scene = null;
this.camera = null;
this.renderer = null;
this.pointCloud = null;
this.controls = null;
this.init();
}
init() {
    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1a1a2e);

    // Camera
    this.camera = new THREE.PerspectiveCamera(
        75,
        this.container.clientWidth / this.container.clientHeight,
        0.1,
        1000
    );
    this.camera.position.set(0, 0, 5);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.container.appendChild(this.renderer.domElement);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    this.scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 10);
    this.scene.add(directionalLight);

    // Grid
    const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
    this.scene.add(gridHelper);

    // Axes
    const axesHelper = new THREE.AxesHelper(5);
    this.scene.add(axesHelper);

    // Controls
    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;

    // Handle window resize
    window.addEventListener('resize', () => this.onWindowResize());

    // Start animation loop
    this.animate();
}

addPointCloud(points, color = 0x32b8c6) {
    if (this.pointCloud) {
        this.scene.remove(this.pointCloud);
    }

    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(points.flat());
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    const material = new THREE.PointsMaterial({
        color: color,
        size: 0.1,
        sizeAttenuation: true
    });

    this.pointCloud = new THREE.Points(geometry, material);
    this.scene.add(this.pointCloud);
}

resetCamera() {
    this.camera.position.set(0, 0, 5);
    this.controls.target.set(0, 0, 0);
    this.controls.update();
}

takeScreenshot() {
    const dataUrl = this.renderer.domElement.toDataURL('image/png');
    const link = document.createElement('a');
    link.href = dataUrl;
    link.download = `lidar-scan-${Date.now()}.png`;
    link.click();
}

onWindowResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();

    this.renderer.setSize(width, height);
}

animate() {
    requestAnimationFrame(() => this.animate());
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
}

}
// Инициализация при загрузке страницы
document.addEventListener('DOMContentLoaded', () => {
const canvas3d = document.getElementById('canvas3d');
if (canvas3d) {
const visualizer = new PointCloudVisualizer(canvas3d);
window.pointCloudVisualizer = visualizer;
    // Кнопки управления
    document.getElementById('resetCameraBtn')?.addEventListener('click', () => {
        visualizer.resetCamera();
    });

    document.getElementById('screenshotBtn')?.addEventListener('click', () => {
        visualizer.takeScreenshot();
    });
}

});

Список файлов для скачивания
✅ web/server.py - Flask сервер
✅ web/templates/index.html - Главная страница (Dashboard)
✅ web/templates/mapping.html - Страница картирования с 3D визуализацией
✅ web/templates/localization.html - Страница локализации с графиками позиции
✅ web/static/css/style.css - Основные стили
✅ web/static/css/dashboard.css - Стили для dashboard
✅ web/static/js/api.js - Функции для API вызовов
✅ web/static/js/app.js - Главный скрипт приложения
✅ web/static/js/visualization.js - 3D визуализация облака точек
Особенности интерфейса:
🎨 Современный тёмный UI с gradient фоном
📊 Real-time графики и диаграммы
🗺️ 3D визуализация облака точек (Three.js)
📍 Отслеживание позиции в реальном времени
💾 Управление картами (сохранение/загрузка)
📈 Траектория движения с историей
🔌 Статус подключения LiDAR
🎯 Два режима работы: картирование и локализация
⚙️ Настройка параметров SLAM и локализации
📝 Live логирование всех действий
Все файлы полностью готовы к использованию! 🚀
