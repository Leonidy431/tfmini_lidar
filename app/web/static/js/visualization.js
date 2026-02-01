/**
 * BlueOS LiDAR SLAM - 3D Visualization Module
 * Uses Three.js for point cloud rendering
 */

class PointCloudVisualizer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) {
            console.error(`Container ${containerId} not found`);
            return;
        }

        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.pointCloud = null;
        this.trajectory = null;
        this.gridHelper = null;
        this.axesHelper = null;

        this.showGrid = true;
        this.showAxes = true;

        this.init();
    }

    init() {
        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0e27);

        // Camera
        const aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 1000);
        this.camera.position.set(5, 5, 5);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.container.appendChild(this.renderer.domElement);

        // Lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 10);
        this.scene.add(directionalLight);

        // Grid
        this.gridHelper = new THREE.GridHelper(20, 20, 0x32b8c6, 0x1a1a2e);
        this.scene.add(this.gridHelper);

        // Axes
        this.axesHelper = new THREE.AxesHelper(3);
        this.scene.add(this.axesHelper);

        // Controls (simple orbit simulation)
        this.setupControls();

        // Handle resize
        window.addEventListener('resize', () => this.onResize());

        // Start animation loop
        this.animate();
    }

    setupControls() {
        // Simple mouse controls
        let isDragging = false;
        let previousMousePosition = { x: 0, y: 0 };

        this.container.addEventListener('mousedown', (e) => {
            isDragging = true;
            previousMousePosition = { x: e.clientX, y: e.clientY };
        });

        this.container.addEventListener('mousemove', (e) => {
            if (!isDragging) return;

            const deltaX = e.clientX - previousMousePosition.x;
            const deltaY = e.clientY - previousMousePosition.y;

            // Rotate camera around origin
            const spherical = new THREE.Spherical();
            spherical.setFromVector3(this.camera.position);
            spherical.theta -= deltaX * 0.01;
            spherical.phi -= deltaY * 0.01;
            spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));

            this.camera.position.setFromSpherical(spherical);
            this.camera.lookAt(0, 0, 0);

            previousMousePosition = { x: e.clientX, y: e.clientY };
        });

        this.container.addEventListener('mouseup', () => {
            isDragging = false;
        });

        this.container.addEventListener('mouseleave', () => {
            isDragging = false;
        });

        // Zoom with wheel
        this.container.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomSpeed = 0.1;
            const direction = new THREE.Vector3();
            direction.subVectors(new THREE.Vector3(0, 0, 0), this.camera.position).normalize();

            if (e.deltaY < 0) {
                this.camera.position.addScaledVector(direction, zoomSpeed);
            } else {
                this.camera.position.addScaledVector(direction, -zoomSpeed);
            }

            // Limit zoom
            const dist = this.camera.position.length();
            if (dist < 1) {
                this.camera.position.setLength(1);
            } else if (dist > 50) {
                this.camera.position.setLength(50);
            }
        });
    }

    updatePointCloud(points, color = 0x32b8c6) {
        // Remove existing point cloud
        if (this.pointCloud) {
            this.scene.remove(this.pointCloud);
            this.pointCloud.geometry.dispose();
            this.pointCloud.material.dispose();
        }

        if (!points || points.length === 0) return;

        // Create geometry
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(points.flat());
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        // Create material
        const material = new THREE.PointsMaterial({
            color: color,
            size: 0.05,
            sizeAttenuation: true,
            transparent: true,
            opacity: 0.8
        });

        // Create point cloud
        this.pointCloud = new THREE.Points(geometry, material);
        this.scene.add(this.pointCloud);
    }

    updateTrajectory(positions, color = 0x26c281) {
        // Remove existing trajectory
        if (this.trajectory) {
            this.scene.remove(this.trajectory);
            this.trajectory.geometry.dispose();
            this.trajectory.material.dispose();
        }

        if (!positions || positions.length < 2) return;

        // Create line geometry
        const geometry = new THREE.BufferGeometry();
        const posArray = new Float32Array(positions.flat());
        geometry.setAttribute('position', new THREE.BufferAttribute(posArray, 3));

        // Create line material
        const material = new THREE.LineBasicMaterial({
            color: color,
            linewidth: 2
        });

        // Create line
        this.trajectory = new THREE.Line(geometry, material);
        this.scene.add(this.trajectory);
    }

    addMarker(position, color = 0xff6b6b, size = 0.2) {
        const geometry = new THREE.SphereGeometry(size, 16, 16);
        const material = new THREE.MeshBasicMaterial({ color: color });
        const marker = new THREE.Mesh(geometry, material);
        marker.position.set(position[0], position[1], position[2]);
        this.scene.add(marker);
        return marker;
    }

    resetCamera() {
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
    }

    toggleGrid() {
        this.showGrid = !this.showGrid;
        this.gridHelper.visible = this.showGrid;
    }

    toggleAxes() {
        this.showAxes = !this.showAxes;
        this.axesHelper.visible = this.showAxes;
    }

    takeScreenshot() {
        this.renderer.render(this.scene, this.camera);
        const dataUrl = this.renderer.domElement.toDataURL('image/png');
        const link = document.createElement('a');
        link.href = dataUrl;
        link.download = `lidar-map-${Date.now()}.png`;
        link.click();
    }

    clear() {
        if (this.pointCloud) {
            this.scene.remove(this.pointCloud);
            this.pointCloud = null;
        }
        if (this.trajectory) {
            this.scene.remove(this.trajectory);
            this.trajectory = null;
        }
    }

    onResize() {
        if (!this.container) return;

        const width = this.container.clientWidth;
        const height = this.container.clientHeight;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }
}

/**
 * 2D Canvas Visualizer for localization
 */
class LocalizationVisualizer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) {
            console.error(`Canvas ${canvasId} not found`);
            return;
        }

        this.ctx = this.canvas.getContext('2d');
        this.trajectory = [];
        this.currentPosition = { x: 0, y: 0 };
        this.mapPoints = [];

        this.scale = 50; // Pixels per meter
        this.offsetX = 0;
        this.offsetY = 0;

        this.resize();
        window.addEventListener('resize', () => this.resize());
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height || 350;

        this.offsetX = this.canvas.width / 2;
        this.offsetY = this.canvas.height / 2;

        this.render();
    }

    setMapPoints(points) {
        this.mapPoints = points;
        this.render();
    }

    updatePosition(x, y) {
        this.currentPosition = { x, y };
        this.trajectory.push({ x, y });

        // Limit trajectory length
        if (this.trajectory.length > 1000) {
            this.trajectory.shift();
        }

        this.render();
    }

    worldToScreen(x, y) {
        return {
            x: this.offsetX + x * this.scale,
            y: this.offsetY - y * this.scale
        };
    }

    render() {
        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // Clear
        ctx.fillStyle = '#1a1a2e';
        ctx.fillRect(0, 0, width, height);

        // Draw grid
        ctx.strokeStyle = '#2c3e50';
        ctx.lineWidth = 1;

        const gridSize = this.scale;
        for (let x = this.offsetX % gridSize; x < width; x += gridSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
            ctx.stroke();
        }
        for (let y = this.offsetY % gridSize; y < height; y += gridSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // Draw map points
        if (this.mapPoints.length > 0) {
            ctx.fillStyle = 'rgba(50, 184, 198, 0.3)';
            for (const point of this.mapPoints) {
                const screen = this.worldToScreen(point[0], point[1]);
                ctx.beginPath();
                ctx.arc(screen.x, screen.y, 2, 0, Math.PI * 2);
                ctx.fill();
            }
        }

        // Draw trajectory
        if (this.trajectory.length > 1) {
            ctx.strokeStyle = '#26c281';
            ctx.lineWidth = 2;
            ctx.beginPath();

            const start = this.worldToScreen(this.trajectory[0].x, this.trajectory[0].y);
            ctx.moveTo(start.x, start.y);

            for (let i = 1; i < this.trajectory.length; i++) {
                const pos = this.worldToScreen(this.trajectory[i].x, this.trajectory[i].y);
                ctx.lineTo(pos.x, pos.y);
            }
            ctx.stroke();
        }

        // Draw current position
        const current = this.worldToScreen(this.currentPosition.x, this.currentPosition.y);
        ctx.fillStyle = '#ff6b6b';
        ctx.beginPath();
        ctx.arc(current.x, current.y, 8, 0, Math.PI * 2);
        ctx.fill();

        // Draw position label
        ctx.fillStyle = '#ecf0f1';
        ctx.font = '12px sans-serif';
        ctx.fillText(
            `(${this.currentPosition.x.toFixed(2)}, ${this.currentPosition.y.toFixed(2)})`,
            current.x + 12,
            current.y + 4
        );

        // Draw origin marker
        const origin = this.worldToScreen(0, 0);
        ctx.strokeStyle = '#f7b731';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(origin.x - 10, origin.y);
        ctx.lineTo(origin.x + 10, origin.y);
        ctx.moveTo(origin.x, origin.y - 10);
        ctx.lineTo(origin.x, origin.y + 10);
        ctx.stroke();
    }

    clear() {
        this.trajectory = [];
        this.currentPosition = { x: 0, y: 0 };
        this.render();
    }
}

// Export for use in app.js
window.PointCloudVisualizer = PointCloudVisualizer;
window.LocalizationVisualizer = LocalizationVisualizer;
