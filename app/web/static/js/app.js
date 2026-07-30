/**
 * BlueOS LiDAR SLAM - Main Application
 */

// Global state
let socket = null;
let mappingVisualizer = null;
let localizationVisualizer = null;
let scannerVisualizer = null;
let statusUpdateInterval = null;
let isRecording = false;
let isNavigating = false;
let isScanning = false;
let scanLayerHeight = 0.5; // synced from server config on first status update

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

    // Reflect stored API token status
    updateTokenStatus();

    // Initialize 3D visualizer for mapping
    mappingVisualizer = new PointCloudVisualizer('mappingCanvas');

    // Initialize 2D visualizer for localization
    localizationVisualizer = new LocalizationVisualizer('localizationCanvas');

    // Initialize 3D visualizer for the object scanner
    scannerVisualizer = new PointCloudVisualizer('scannerCanvas');

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
    document.getElementById('startBtn')?.addEventListener('click', async (e) => {
        const result = await withLoading(e.currentTarget, startApp);
        if (result.success) {
            addLog('Application started');
            showToast('Application started', 'success');
        } else {
            addLog('Failed to start: ' + (result.error || 'Unknown error'));
            showToast('Failed to start: ' + (result.error || 'Unknown error'), 'error');
        }
    });

    document.getElementById('stopBtn')?.addEventListener('click', async (e) => {
        const result = await withLoading(e.currentTarget, stopApp);
        if (result.success) {
            addLog('Application stopped');
            showToast('Application stopped', 'info');
        } else {
            showToast('Failed to stop: ' + (result.error || 'Unknown error'), 'error');
        }
    });

    document.getElementById('refreshBtn')?.addEventListener('click', () => {
        updateStatus();
        addLog('Status refreshed');
    });

    // API token
    document.getElementById('saveTokenBtn')?.addEventListener('click', () => {
        const token = document.getElementById('apiTokenInput').value.trim();
        setApiToken(token);
        document.getElementById('apiTokenInput').value = '';
        updateTokenStatus();
        addLog(token ? 'API token saved' : 'API token cleared');
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

    // ===== 3D Scanner controls =====

    document.getElementById('startScanBtn')?.addEventListener('click', async (e) => {
        const cx = parseFloat(document.getElementById('scanCenterX').value) || 0;
        const cy = parseFloat(document.getElementById('scanCenterY').value) || 0;
        const cz = parseFloat(document.getElementById('scanCenterZ').value) || 0;
        const radius = parseFloat(document.getElementById('scanOrbitRadius').value);

        if (!radius || radius <= 0) {
            showToast('Enter a valid orbit radius', 'error');
            return;
        }

        const result = await withLoading(e.currentTarget, () => startScan(cx, cy, cz, radius));
        if (result.success) {
            isScanning = true;
            document.getElementById('startScanBtn').disabled = true;
            document.getElementById('stopScanBtn').disabled = false;
            addLog(`3D scan started (orbit radius ${radius}m)`);
            showToast('Scan started - orbit the object', 'success');
        } else {
            showToast('Failed to start scan: ' + (result.error || 'Unknown error'), 'error');
        }
    });

    document.getElementById('stopScanBtn')?.addEventListener('click', async (e) => {
        const result = await withLoading(e.currentTarget, stopScan);
        if (result.success) {
            isScanning = false;
            document.getElementById('startScanBtn').disabled = false;
            document.getElementById('stopScanBtn').disabled = true;
            addLog('3D scan stopped');
            loadScanPoints();
        }
    });

    document.getElementById('clearScanBtn')?.addEventListener('click', async () => {
        if (!confirm('Discard the current scan data?')) return;
        const result = await clearScan();
        if (result.success) {
            scannerVisualizer?.clear();
            updateScannerDisplay(null);
            addLog('Scan data cleared');
        }
    });

    document.getElementById('scanLayerUpBtn')?.addEventListener('click', async () => {
        const current = parseFloat(document.getElementById('scanCurrentZ').textContent) || 0;
        const result = await setScanLayer(current + scanLayerHeight);
        if (result.success) addLog(`Layer set to ${result.current_z.toFixed(2)}m`);
    });

    document.getElementById('scanLayerDownBtn')?.addEventListener('click', async () => {
        const current = parseFloat(document.getElementById('scanCurrentZ').textContent) || 0;
        const result = await setScanLayer(current - scanLayerHeight);
        if (result.success) addLog(`Layer set to ${result.current_z.toFixed(2)}m`);
    });

    document.getElementById('scanLayerSetBtn')?.addEventListener('click', async () => {
        const z = parseFloat(document.getElementById('scanLayerInput').value);
        if (Number.isNaN(z)) {
            showToast('Enter a valid depth value', 'error');
            return;
        }
        const result = await setScanLayer(z);
        if (result.success) {
            addLog(`Layer set to ${z.toFixed(2)}m`);
            document.getElementById('scanLayerInput').value = '';
        }
    });

    document.getElementById('scanResetCameraBtn')?.addEventListener('click', () => {
        scannerVisualizer?.resetCamera();
    });

    document.getElementById('scanRefreshPointsBtn')?.addEventListener('click', () => {
        loadScanPoints();
    });

    document.getElementById('saveScanBtn')?.addEventListener('click', async () => {
        const name = document.getElementById('scanNameInput').value.trim();
        const desc = document.getElementById('scanDescInput').value.trim();

        if (!name) {
            alert('Please enter a scan name');
            return;
        }

        const result = await saveScan(name, desc);
        if (result.success) {
            addLog(`Scan "${name}" saved as map (${result.point_count} points)`);
            showToast(`Scan saved: ${result.point_count} points`, 'success');
            document.getElementById('scanNameInput').value = '';
            document.getElementById('scanDescInput').value = '';
            loadMaps();
        } else {
            showToast('Failed to save scan: ' + (result.error || 'Unknown error'), 'error');
        }
    });
}

/**
 * Setup WebSocket connection
 */
function setupWebSocket() {
    try {
        // Automatic reconnection with exponential backoff (UX #1). Pass the
        // API token so the handshake succeeds when REQUIRE_WS_AUTH is on.
        socket = io({
            reconnection: true,
            reconnectionAttempts: 10,
            reconnectionDelay: 1000,
            reconnectionDelayMax: 5000,
            auth: { token: getApiToken() }
        });

        socket.on('connect', () => {
            console.log('WebSocket connected');
            document.getElementById('statusIndicator')?.classList.add('active');
            addLog('WebSocket connected');
        });

        socket.on('disconnect', () => {
            console.log('WebSocket disconnected');
            document.getElementById('statusIndicator')?.classList.remove('active');
            addLog('WebSocket disconnected - reconnecting...');
        });

        socket.io.on('reconnect_attempt', (n) => {
            addLog(`Reconnection attempt ${n}...`);
        });

        socket.io.on('reconnect', () => {
            showToast('Connection restored', 'success');
            // Resync state with the server after reconnection
            updateStatus();
        });

        socket.io.on('reconnect_failed', () => {
            showToast('Could not reconnect. Check the connection and refresh.', 'error', 8000);
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

        socket.on('scanner_progress', (data) => {
            updateScannerDisplay(data);
        });

        socket.on('safety_alarm', (data) => {
            showToast(data.message || 'Safety alarm', 'error', 8000);
            addLog(`SAFETY ALARM: ${data.message}`);
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
    if (!status) return;
    // Distinguish an auth/backend error (apiCall returns {success:false,...})
    // from a real status payload (which has no `success` field), so the
    // operator sees "token required" / "backend unreachable" instead of a
    // silent Disconnected/Stopped that looks identical to a real outage
    // (Blind Spot Audit R2 domain 20).
    if (status.success === false) {
        if (typeof showToast === 'function') {
            showToast(status.unauthorized
                ? 'API token required or invalid — set it above.'
                : `Backend unreachable: ${status.error || 'unknown error'}`,
                'error');
        }
        return;
    }
    updateStatusDisplay(status);
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

    // Data quality score
    if (status.data_quality) {
        const dqEl = document.getElementById('dataQuality');
        if (dqEl) {
            const score = status.data_quality.quality_score;
            dqEl.textContent = `${(score * 100).toFixed(0)}%`;
            dqEl.style.color = score >= 0.9 ? '#26c281' : (score >= 0.7 ? '#f5a623' : '#ff6b6b');
        }
    }

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

    // 3D scanner
    if (status.scanner) {
        updateScannerDisplay(status.scanner);
    }
    if (status.config?.scanner?.layer_height) {
        scanLayerHeight = status.config.scanner.layer_height;
    }
}

/**
 * Update the 3D scanner tab from a scanner status object
 * ({is_scanning, current_z, point_count, layer_count, overall_coverage,
 *   layers: {z: {coverage, complete}}, rejected, duration_seconds, ...}).
 */
function updateScannerDisplay(scanner) {
    if (!scanner) {
        document.getElementById('scanRingCoveragePct').textContent = '0%';
        document.getElementById('scanRingCoverageBar').style.width = '0%';
        document.getElementById('scanOverallCoveragePct').textContent = '0%';
        document.getElementById('scanOverallCoverageBar').style.width = '0%';
        document.getElementById('scanPointCount').textContent = '0';
        document.getElementById('scanLayerCount').textContent = '0';
        document.getElementById('scanRejectedCount').textContent = '0';
        document.getElementById('scanDuration').textContent = '0 s';
        document.getElementById('scanLayersList').innerHTML = '';
        return;
    }

    isScanning = !!scanner.is_scanning;
    document.getElementById('startScanBtn').disabled = isScanning;
    document.getElementById('stopScanBtn').disabled = !isScanning;

    document.getElementById('scanCurrentZ').textContent = (scanner.current_z || 0).toFixed(2);
    document.getElementById('scanPointCount').textContent = scanner.point_count || 0;
    document.getElementById('scanLayerCount').textContent = scanner.layer_count || 0;
    document.getElementById('scanRejectedCount').textContent = scanner.rejected || 0;
    document.getElementById('scanDuration').textContent = `${Math.floor(scanner.duration_seconds || 0)} s`;

    const overallPct = Math.round((scanner.overall_coverage || 0) * 100);
    document.getElementById('scanOverallCoveragePct').textContent = `${overallPct}%`;
    document.getElementById('scanOverallCoverageBar').style.width = `${overallPct}%`;

    // Ring coverage for the current layer
    const layers = scanner.layers || {};
    const currentKey = Object.keys(layers).find(k => Math.abs(parseFloat(k) - (scanner.current_z || 0)) < 1e-6);
    const currentCoverage = currentKey ? layers[currentKey].coverage : 0;
    const ringPct = Math.round(currentCoverage * 100);
    document.getElementById('scanRingCoveragePct').textContent = `${ringPct}%`;
    document.getElementById('scanRingCoverageBar').style.width = `${ringPct}%`;

    // Per-layer breakdown
    const layersList = document.getElementById('scanLayersList');
    const layerKeys = Object.keys(layers).sort((a, b) => parseFloat(b) - parseFloat(a));
    if (layerKeys.length === 0) {
        layersList.innerHTML = '';
    } else {
        layersList.innerHTML = layerKeys.map(z => {
            const l = layers[z];
            const pct = Math.round(l.coverage * 100);
            return `
                <div class="layer-row ${l.complete ? 'layer-complete' : ''}">
                    <span class="layer-z">${escapeHtml(z)}m</span>
                    <span class="layer-bar-track"><span class="layer-bar-fill" style="width:${pct}%;"></span></span>
                    <span class="layer-pct">${pct}%</span>
                </div>
            `;
        }).join('');
    }
}

/**
 * Fetch and render the current scan point cloud.
 */
async function loadScanPoints() {
    const data = await getScanPoints();
    if (data && data.points && scannerVisualizer) {
        scannerVisualizer.updatePointCloud(data.points, 0xf5a623);
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

    // Update arrow direction. heading_error is compass-degrees, clockwise-
    // positive; CSS rotate() is also clockwise-positive for a positive
    // angle, so a positive (rightward) error rotates the arrow to the
    // right with no sign flip (Physics Audit H2 - the previous negation
    // pointed the arrow the wrong way).
    const arrow = document.getElementById('navArrow');
    // Guard on != null, not truthiness: a heading_error of exactly 0 (perfectly
    // aligned) is falsy and would otherwise leave the arrow at its last angle
    // instead of snapping to center (Blind Spot Audit R2 domain 20).
    if (arrow && data.heading_error != null) {
        arrow.style.transform = `rotate(${data.heading_error}deg)`;
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
    } else if (tabId === 'scanner') {
        loadScanPoints();
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

    // A failed request returns {success:false,...} (truthy, no .length), which
    // would fall through to maps.map() and TypeError, leaving the list stuck on
    // "Loading..." forever. Distinguish error from empty (Blind Spot Audit R2
    // domain 20).
    if (!Array.isArray(maps)) {
        container.innerHTML = '<p class="loading">Could not load maps (check API token / connection)</p>';
        return;
    }
    if (maps.length === 0) {
        container.innerHTML = '<p class="loading">No saved maps</p>';
        return;
    }

    container.innerHTML = maps.map(map => `
        <div class="map-item">
            <div class="item-header">
                <span class="item-name">${escapeHtml(map.name)}</span>
            </div>
            <div class="item-meta">
                ${escapeHtml(map.point_count || 0)} points | ${escapeHtml(formatTimestamp(map.created))}
            </div>
            <div class="item-actions">
                <button class="btn btn-sm btn-primary" onclick="loadMapForLocalization('${escapeJsString(map.name)}')">
                    Load
                </button>
                <button class="btn btn-sm btn-danger" onclick="deleteMapItem('${escapeJsString(map.name)}')">
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

    // Update profiles list. Distinguish a failed request (non-array) from an
    // empty list so the panel doesn't hang on "Loading..." (domain 20).
    if (!Array.isArray(profiles)) {
        container.innerHTML = '<p class="loading">Could not load profiles (check API token / connection)</p>';
        return;
    }
    if (profiles.length === 0) {
        container.innerHTML = '<p class="loading">No saved profiles</p>';
    } else {
        container.innerHTML = profiles.map(profile => `
            <div class="profile-item">
                <div class="item-header">
                    <span class="item-name">${escapeHtml(profile.name)}</span>
                </div>
                <div class="item-meta">
                    ${escapeHtml(profile.waypoint_count || 0)} waypoints |
                    ${escapeHtml((profile.total_distance || 0).toFixed(1))}m |
                    ${escapeHtml(formatTimestamp(profile.created))}
                </div>
                <div class="item-actions">
                    <button class="btn btn-sm btn-danger" onclick="deleteProfileItem('${escapeJsString(profile.name)}')">
                        Delete
                    </button>
                </div>
            </div>
        `).join('');
    }

    // Update select dropdown
    if (select) {
        select.innerHTML = '<option value="">Select a profile...</option>' +
            (profiles || []).map(p =>
                `<option value="${escapeHtml(p.name)}">${escapeHtml(p.name)}</option>`
            ).join('');
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
                <span class="item-name">${escapeHtml(obj.class)}</span>
                <span style="color: var(--color-secondary);">${escapeHtml((obj.confidence * 100).toFixed(0))}%</span>
            </div>
            <div class="item-meta">
                Distance: ${escapeHtml(obj.distance.toFixed(2))}m |
                Position: (${escapeHtml(obj.position.map(p => p.toFixed(2)).join(', '))})
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
 * Toast notification system for user-facing errors/success (UX #2).
 */
function showToast(message, type = 'info', duration = 4000) {
    let container = document.getElementById('toastContainer');
    if (!container) {
        container = document.createElement('div');
        container.id = 'toastContainer';
        container.className = 'toast-container';
        container.setAttribute('aria-live', 'polite');
        document.body.appendChild(container);
    }

    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.setAttribute('role', 'status');
    toast.textContent = message;
    container.appendChild(toast);

    // Trigger enter transition
    requestAnimationFrame(() => toast.classList.add('toast-visible'));

    setTimeout(() => {
        toast.classList.remove('toast-visible');
        setTimeout(() => toast.remove(), 300);
    }, duration);
}

/**
 * Wrap an async button action with a loading/disabled state (UX #8).
 */
async function withLoading(btn, asyncFn) {
    if (!btn) return asyncFn();
    const original = btn.textContent;
    btn.disabled = true;
    btn.dataset.loading = 'true';
    btn.textContent = 'Working...';
    try {
        return await asyncFn();
    } finally {
        btn.disabled = false;
        delete btn.dataset.loading;
        btn.textContent = original;
    }
}

/**
 * Reflect API token presence in the UI (never display the token itself).
 */
function updateTokenStatus() {
    const el = document.getElementById('tokenStatus');
    if (!el) return;
    const hasToken = !!getApiToken();
    el.textContent = hasToken ? 'Token set' : 'Not set';
    el.style.color = hasToken ? '#26c281' : '#ff6b6b';
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
    // Release Three.js / canvas resources to avoid WebGL context leaks
    mappingVisualizer?.destroy?.();
    localizationVisualizer?.destroy?.();
    scannerVisualizer?.destroy?.();
});
