/**
 * BlueOS LiDAR SLAM - Main Application
 */

// Global state
let socket = null;
let mappingVisualizer = null;
let localizationVisualizer = null;
let statusUpdateInterval = null;
let isRecording = false;
let isNavigating = false;

// Initialize on DOM ready
document.addEventListener('DOMContentLoaded', () => {
    initializeApp();
    setupEventListeners();
    setupWebSocket();
    startStatusUpdates();
});

/**
 * Initialize application
 */
function initializeApp() {
    console.log('Initializing BlueOS LiDAR SLAM...');
    addLog('System initialized');

    // Initialize 3D visualizer for mapping
    mappingVisualizer = new PointCloudVisualizer('mappingCanvas');

    // Initialize 2D visualizer for localization
    localizationVisualizer = new LocalizationVisualizer('localizationCanvas');

    // Load initial data
    loadMaps();
    loadProfiles();
    loadObjects();
}

/**
 * Setup all event listeners
 */
function setupEventListeners() {
    // Tab navigation
    document.querySelectorAll('.nav-link').forEach(link => {
        link.addEventListener('click', (e) => {
            e.preventDefault();
            const tabId = link.dataset.tab;
            switchTab(tabId);
        });
    });

    // System controls
    document.getElementById('startBtn')?.addEventListener('click', async () => {
        const result = await startApp();
        if (result.success) {
            addLog('Application started');
        } else {
            addLog('Failed to start: ' + (result.error || 'Unknown error'));
        }
    });

    document.getElementById('stopBtn')?.addEventListener('click', async () => {
        const result = await stopApp();
        if (result.success) {
            addLog('Application stopped');
        }
    });

    document.getElementById('refreshBtn')?.addEventListener('click', () => {
        updateStatus();
        addLog('Status refreshed');
    });

    // Mapping controls
    document.getElementById('startMappingBtn')?.addEventListener('click', async () => {
        const result = await startMapping();
        if (result.success) {
            addLog('Mapping started');
        }
    });

    document.getElementById('stopMappingBtn')?.addEventListener('click', async () => {
        const result = await stopMapping();
        if (result.success) {
            addLog('Mapping stopped');
            updateMappingVisualization();
        }
    });

    document.getElementById('clearMapBtn')?.addEventListener('click', async () => {
        const result = await clearMapping();
        if (result.success) {
            mappingVisualizer.clear();
            addLog('Map cleared');
        }
    });

    document.getElementById('saveMapBtn')?.addEventListener('click', async () => {
        const name = document.getElementById('mapNameInput').value.trim();
        const desc = document.getElementById('mapDescInput').value.trim();

        if (!name) {
            alert('Please enter a map name');
            return;
        }

        const result = await saveMap(name, desc);
        if (result.success) {
            addLog(`Map "${name}" saved`);
            document.getElementById('mapNameInput').value = '';
            document.getElementById('mapDescInput').value = '';
            loadMaps();
        } else {
            addLog('Failed to save map: ' + (result.error || 'Unknown error'));
        }
    });

    // Visualization controls
    document.getElementById('resetCameraBtn')?.addEventListener('click', () => {
        mappingVisualizer?.resetCamera();
    });

    document.getElementById('toggleGridBtn')?.addEventListener('click', () => {
        mappingVisualizer?.toggleGrid();
    });

    document.getElementById('screenshotBtn')?.addEventListener('click', () => {
        mappingVisualizer?.takeScreenshot();
    });

    // Mode buttons
    document.getElementById('recordModeBtn')?.addEventListener('click', () => {
        selectNavigationMode('record');
    });

    document.getElementById('navigateModeBtn')?.addEventListener('click', () => {
        selectNavigationMode('navigate');
    });

    document.getElementById('localizeModeBtn')?.addEventListener('click', () => {
        selectNavigationMode('localize');
    });

    // Recording controls
    document.getElementById('startRecordBtn')?.addEventListener('click', async () => {
        const name = document.getElementById('profileNameInput').value.trim();
        if (!name) {
            alert('Please enter a profile name');
            return;
        }

        const result = await startRecording(name);
        if (result.success) {
            isRecording = true;
            document.getElementById('startRecordBtn').disabled = true;
            document.getElementById('stopRecordBtn').disabled = false;
            addLog(`Recording profile "${name}"`);
        }
    });

    document.getElementById('stopRecordBtn')?.addEventListener('click', async () => {
        const result = await stopRecording();
        if (result.success) {
            isRecording = false;
            document.getElementById('startRecordBtn').disabled = false;
            document.getElementById('stopRecordBtn').disabled = true;
            document.getElementById('profileNameInput').value = '';
            addLog('Recording stopped');
            loadProfiles();
        }
    });

    // Navigation controls
    document.getElementById('loadProfileBtn')?.addEventListener('click', () => {
        loadProfiles();
    });

    document.getElementById('startNavBtn')?.addEventListener('click', async () => {
        const profileName = document.getElementById('profileSelect').value;
        if (!profileName) {
            alert('Please select a profile');
            return;
        }

        const result = await startNavigation(profileName);
        if (result.success) {
            isNavigating = true;
            document.getElementById('startNavBtn').disabled = true;
            document.getElementById('stopNavBtn').disabled = false;
            addLog(`Navigation started with "${profileName}"`);
        }
    });

    document.getElementById('stopNavBtn')?.addEventListener('click', async () => {
        const result = await stopNavigation();
        if (result.success) {
            isNavigating = false;
            document.getElementById('startNavBtn').disabled = false;
            document.getElementById('stopNavBtn').disabled = true;
            addLog('Navigation stopped');
        }
    });

    // Object detection controls
    document.getElementById('clearObjectsBtn')?.addEventListener('click', async () => {
        const result = await clearObjects();
        if (result.success) {
            addLog('Objects cleared');
            loadObjects();
        }
    });

    document.getElementById('saveObjectsBtn')?.addEventListener('click', async () => {
        const result = await saveObjects();
        if (result.success) {
            addLog('Objects saved');
        }
    });

    document.getElementById('refreshObjectsBtn')?.addEventListener('click', () => {
        loadObjects();
    });
}

/**
 * Setup WebSocket connection
 */
function setupWebSocket() {
    try {
        socket = io();

        socket.on('connect', () => {
            console.log('WebSocket connected');
            document.getElementById('statusIndicator')?.classList.add('active');
            addLog('WebSocket connected');
        });

        socket.on('disconnect', () => {
            console.log('WebSocket disconnected');
            document.getElementById('statusIndicator')?.classList.remove('active');
        });

        socket.on('status', (data) => {
            updateStatusDisplay(data);
        });

        socket.on('lidar_reading', (data) => {
            updateReadingDisplay(data);
        });

        socket.on('localization', (data) => {
            updateLocalizationDisplay(data);
        });

        socket.on('navigation', (data) => {
            updateNavigationDisplay(data);
        });

        socket.on('detection', (data) => {
            onObjectDetected(data);
        });

        socket.on('map_points', (data) => {
            if (data.points && mappingVisualizer) {
                mappingVisualizer.updatePointCloud(data.points);
            }
        });

    } catch (error) {
        console.error('WebSocket setup failed:', error);
    }
}

/**
 * Start periodic status updates
 */
function startStatusUpdates() {
    updateStatus();
    statusUpdateInterval = setInterval(updateStatus, 2000);
}

/**
 * Update status from API
 */
async function updateStatus() {
    const status = await getStatus();
    if (status) {
        updateStatusDisplay(status);
    }
}

/**
 * Update status display
 */
function updateStatusDisplay(status) {
    // LiDAR status
    const lidarConnected = status.lidar?.connected;
    document.getElementById('lidarStatus').textContent = lidarConnected ? 'Connected' : 'Disconnected';
    document.getElementById('lidarStatus').style.color = lidarConnected ? '#26c281' : '#ff6b6b';

    // App status
    document.getElementById('appStatus').textContent = status.running ? 'Running' : 'Stopped';
    document.getElementById('appStatus').style.color = status.running ? '#26c281' : '#ff6b6b';

    // Mode
    const modeDisplay = {
        'idle': 'Idle',
        'mapping': 'Mapping',
        'localizing': 'Localizing',
        'navigating': 'Navigating',
        'recording': 'Recording'
    };
    document.getElementById('currentMode').textContent = modeDisplay[status.mode] || status.mode;

    // Readings per second
    document.getElementById('readingsPerSec').textContent = status.readings_per_second || 0;

    // Mapping stats
    if (status.slam) {
        document.getElementById('mappedPoints').textContent = status.slam.total_points || 0;
        document.getElementById('registeredScans').textContent = status.slam.total_scans || 0;
        document.getElementById('driftEstimate').textContent = `${((status.slam.drift_estimate || 0) * 100).toFixed(1)} cm`;

        const mapSize = status.slam.map_size;
        if (mapSize) {
            document.getElementById('mapSize').textContent =
                `${mapSize.x} x ${mapSize.y} x ${mapSize.z} m`;
        }
    }

    // Recording stats
    if (status.recording) {
        document.getElementById('waypointCount').textContent = status.recording.waypoint_count || 0;
        document.getElementById('recordedDistance').textContent = `${(status.recording.total_distance || 0).toFixed(1)} m`;
        document.getElementById('recordingDuration').textContent = `${Math.floor(status.recording.duration_seconds || 0)} s`;
    }

    // Localization stats
    if (status.localization) {
        const pos = status.localization.position || {};
        document.getElementById('posX').textContent = (pos.x || 0).toFixed(2);
        document.getElementById('posY').textContent = (pos.y || 0).toFixed(2);
        document.getElementById('posZ').textContent = (pos.z || 0).toFixed(2);
        document.getElementById('locConfidence').textContent =
            `${((status.localization.current_confidence || 0) * 100).toFixed(0)}%`;
    }

    // Object detection stats
    if (status.object_detection) {
        document.getElementById('totalObjects').textContent = status.object_detection.total_objects || 0;

        const classes = status.object_detection.class_distribution || {};
        document.getElementById('obstacleCount').textContent = classes.obstacle || 0;
        document.getElementById('wallCount').textContent = classes.wall || 0;
        document.getElementById('unknownCount').textContent = classes.unknown || 0;
    }
}

/**
 * Update reading display
 */
function updateReadingDisplay(reading) {
    if (!reading) return;

    document.getElementById('currentDistance').textContent = reading.distance.toFixed(2);
    document.getElementById('signalStrength').textContent = reading.signal_strength;
    document.getElementById('temperature').textContent = reading.temperature.toFixed(1);

    // Update distance bar (0-12m range)
    const percent = Math.min(100, (reading.distance / 12) * 100);
    document.getElementById('distanceBar').style.width = `${percent}%`;
}

/**
 * Update localization display
 */
function updateLocalizationDisplay(data) {
    if (!data || !data.success) return;

    const pos = data.position || [0, 0, 0];
    document.getElementById('posX').textContent = pos[0].toFixed(2);
    document.getElementById('posY').textContent = pos[1].toFixed(2);
    document.getElementById('posZ').textContent = pos[2].toFixed(2);
    document.getElementById('locConfidence').textContent = `${(data.confidence * 100).toFixed(0)}%`;

    // Update 2D visualization
    localizationVisualizer?.updatePosition(pos[0], pos[1]);
}

/**
 * Update navigation display
 */
function updateNavigationDisplay(data) {
    if (!data) return;

    document.getElementById('navStatus').textContent = data.status || 'Unknown';
    document.getElementById('distToWaypoint').textContent = `${(data.distance_to_waypoint || 0).toFixed(2)} m`;
    document.getElementById('headingError').textContent = `${(data.heading_error || 0).toFixed(1)}deg`;
    document.getElementById('navProgress').textContent = `${(data.progress || 0).toFixed(0)}%`;

    // Update arrow direction
    const arrow = document.getElementById('navArrow');
    if (arrow && data.heading_error) {
        arrow.style.transform = `rotate(${-data.heading_error}deg)`;
    }
}

/**
 * Handle object detection
 */
function onObjectDetected(detection) {
    addLog(`Object detected: ${detection.class} at ${detection.distance.toFixed(2)}m`);
    loadObjects();
}

/**
 * Switch between tabs
 */
function switchTab(tabId) {
    // Update nav links
    document.querySelectorAll('.nav-link').forEach(link => {
        link.classList.toggle('active', link.dataset.tab === tabId);
    });

    // Update tab content
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.toggle('active', content.id === tabId);
    });

    // Refresh visualizations when switching tabs
    if (tabId === 'mapping') {
        updateMappingVisualization();
    }
}

/**
 * Select navigation mode
 */
function selectNavigationMode(mode) {
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.mode === mode);
    });

    document.getElementById('recordingSection').classList.toggle('hidden', mode !== 'record');
    document.getElementById('navigationSection').classList.toggle('hidden', mode !== 'navigate');
    document.getElementById('localizationSection').classList.toggle('hidden', mode !== 'localize');
}

/**
 * Update mapping visualization
 */
async function updateMappingVisualization() {
    const pointsData = await getMapPoints();
    if (pointsData && pointsData.points) {
        mappingVisualizer?.updatePointCloud(pointsData.points);
    }

    const trajData = await getTrajectory();
    if (trajData && trajData.trajectory) {
        mappingVisualizer?.updateTrajectory(trajData.trajectory);
    }
}

/**
 * Load saved maps
 */
async function loadMaps() {
    const maps = await listMaps();
    const container = document.getElementById('mapsList');

    if (!maps || maps.length === 0) {
        container.innerHTML = '<p class="loading">No saved maps</p>';
        return;
    }

    container.innerHTML = maps.map(map => `
        <div class="map-item">
            <div class="item-header">
                <span class="item-name">${map.name}</span>
            </div>
            <div class="item-meta">
                ${map.point_count || 0} points | ${formatTimestamp(map.created)}
            </div>
            <div class="item-actions">
                <button class="btn btn-sm btn-primary" onclick="loadMapForLocalization('${map.name}')">
                    Load
                </button>
                <button class="btn btn-sm btn-danger" onclick="deleteMapItem('${map.name}')">
                    Delete
                </button>
            </div>
        </div>
    `).join('');
}

/**
 * Load map for localization
 */
async function loadMapForLocalization(name) {
    const result = await loadMap(name);
    if (result.success) {
        addLog(`Map "${name}" loaded for localization`);
    }
}

/**
 * Delete map
 */
async function deleteMapItem(name) {
    if (!confirm(`Delete map "${name}"?`)) return;

    const result = await deleteMap(name);
    if (result.success) {
        addLog(`Map "${name}" deleted`);
        loadMaps();
    }
}

/**
 * Load saved profiles
 */
async function loadProfiles() {
    const profiles = await listProfiles();
    const container = document.getElementById('profilesList');
    const select = document.getElementById('profileSelect');

    // Update profiles list
    if (!profiles || profiles.length === 0) {
        container.innerHTML = '<p class="loading">No saved profiles</p>';
    } else {
        container.innerHTML = profiles.map(profile => `
            <div class="profile-item">
                <div class="item-header">
                    <span class="item-name">${profile.name}</span>
                </div>
                <div class="item-meta">
                    ${profile.waypoint_count || 0} waypoints |
                    ${(profile.total_distance || 0).toFixed(1)}m |
                    ${formatTimestamp(profile.created)}
                </div>
                <div class="item-actions">
                    <button class="btn btn-sm btn-danger" onclick="deleteProfileItem('${profile.name}')">
                        Delete
                    </button>
                </div>
            </div>
        `).join('');
    }

    // Update select dropdown
    if (select) {
        select.innerHTML = '<option value="">Select a profile...</option>' +
            (profiles || []).map(p => `<option value="${p.name}">${p.name}</option>`).join('');
    }

    // Enable/disable navigation button
    document.getElementById('startNavBtn').disabled = !(select && select.value);
}

/**
 * Delete profile
 */
async function deleteProfileItem(name) {
    if (!confirm(`Delete profile "${name}"?`)) return;

    const result = await deleteProfile(name);
    if (result.success) {
        addLog(`Profile "${name}" deleted`);
        loadProfiles();
    }
}

/**
 * Load detected objects
 */
async function loadObjects() {
    const data = await getObjects();
    const container = document.getElementById('objectsList');

    if (!data || !data.objects || data.objects.length === 0) {
        container.innerHTML = '<p class="loading">No objects detected</p>';
        return;
    }

    container.innerHTML = data.objects.map(obj => `
        <div class="object-item">
            <div class="item-header">
                <span class="item-name">${obj.class}</span>
                <span style="color: var(--color-secondary);">${(obj.confidence * 100).toFixed(0)}%</span>
            </div>
            <div class="item-meta">
                Distance: ${obj.distance.toFixed(2)}m |
                Position: (${obj.position.map(p => p.toFixed(2)).join(', ')})
            </div>
        </div>
    `).join('');

    // Update pattern display
    if (data.statistics && data.statistics.pattern_analysis) {
        const pattern = data.statistics.pattern_analysis;
        document.getElementById('currentPattern').textContent = pattern.pattern_type || '--';
    }
}

/**
 * Add log entry
 */
function addLog(message) {
    const container = document.getElementById('logsContainer');
    if (!container) return;

    const entry = document.createElement('p');
    entry.className = 'log-entry';

    const timestamp = new Date().toLocaleTimeString();
    entry.textContent = `[${timestamp}] ${message}`;

    container.appendChild(entry);
    container.scrollTop = container.scrollHeight;

    // Limit log entries
    while (container.children.length > 100) {
        container.removeChild(container.firstChild);
    }
}

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (statusUpdateInterval) {
        clearInterval(statusUpdateInterval);
    }
    if (socket) {
        socket.disconnect();
    }
});
