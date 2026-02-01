/**
 * BlueOS LiDAR SLAM - API Module
 * Handles all REST API communication
 */

const API_BASE = '/api';

/**
 * Generic API call function
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
            const errorData = await response.json().catch(() => ({}));
            return { success: false, error: errorData.error || `HTTP ${response.status}` };
        }

        return await response.json();
    } catch (error) {
        console.error(`API call error: ${error}`);
        return { success: false, error: error.message };
    }
}

// ========== System API ==========

async function getStatus() {
    return await apiCall('/status');
}

async function startApp() {
    return await apiCall('/start', 'POST');
}

async function stopApp() {
    return await apiCall('/stop', 'POST');
}

async function setMode(mode) {
    return await apiCall(`/mode/${mode}`, 'POST');
}

// ========== Mapping API ==========

async function startMapping() {
    return await apiCall('/mapping/start', 'POST');
}

async function stopMapping() {
    return await apiCall('/mapping/stop', 'POST');
}

async function clearMapping() {
    return await apiCall('/mapping/clear', 'POST');
}

async function getMappingStatistics() {
    return await apiCall('/mapping/statistics');
}

async function getMapPoints() {
    return await apiCall('/mapping/points');
}

async function getTrajectory() {
    return await apiCall('/mapping/trajectory');
}

// ========== Maps API ==========

async function listMaps() {
    return await apiCall('/maps');
}

async function getMapInfo(name) {
    return await apiCall(`/maps/${encodeURIComponent(name)}`);
}

async function saveMap(name, description = '', tags = []) {
    return await apiCall(`/maps/${encodeURIComponent(name)}/save`, 'POST', { description, tags });
}

async function loadMap(name) {
    return await apiCall(`/maps/${encodeURIComponent(name)}/load`, 'POST');
}

async function deleteMap(name) {
    return await apiCall(`/maps/${encodeURIComponent(name)}/delete`, 'DELETE');
}

// ========== Localization API ==========

async function getPosition() {
    return await apiCall('/localization/position');
}

async function resetLocalization() {
    return await apiCall('/localization/reset', 'POST');
}

// ========== Profile API ==========

async function listProfiles() {
    return await apiCall('/profiles');
}

async function startRecording(name, description = '') {
    return await apiCall(`/profiles/${encodeURIComponent(name)}/record/start`, 'POST', { description });
}

async function stopRecording() {
    return await apiCall('/profiles/record/stop', 'POST');
}

async function startNavigation(name) {
    return await apiCall(`/profiles/${encodeURIComponent(name)}/navigate/start`, 'POST');
}

async function stopNavigation() {
    return await apiCall('/profiles/navigate/stop', 'POST');
}

async function deleteProfile(name) {
    return await apiCall(`/profiles/${encodeURIComponent(name)}/delete`, 'DELETE');
}

// ========== Object Detection API ==========

async function getObjects() {
    return await apiCall('/objects');
}

async function getNearbyObjects(x, y, z, radius = 5.0) {
    return await apiCall(`/objects/nearby?x=${x}&y=${y}&z=${z}&radius=${radius}`);
}

async function clearObjects() {
    return await apiCall('/objects/clear', 'POST');
}

async function saveObjects() {
    return await apiCall('/objects/save', 'POST');
}

// ========== Utility Functions ==========

function formatTimestamp(isoString) {
    const date = new Date(isoString);
    return date.toLocaleString();
}

function formatDistance(meters) {
    if (meters < 1) {
        return `${(meters * 100).toFixed(1)} cm`;
    }
    return `${meters.toFixed(2)} m`;
}

function formatDuration(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
}
