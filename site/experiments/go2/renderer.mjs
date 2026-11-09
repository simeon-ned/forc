import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class RobotView {
  constructor(host) {
    this.host = host;
    this.dirty = true;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setClearColor(0xf0f4f7);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.domElement.setAttribute('aria-label', 'Interactive Go2 model: drag to orbit, scroll to zoom');
    this.renderer.domElement.setAttribute('role', 'img');
    host.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(42, 1, 0.01, 100);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.maxDistance = 8;
    this.controls.minDistance = 0.35;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.home();
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x6b7280, 2.8));
    const light = new THREE.DirectionalLight(0xffffff, 3);
    light.position.set(2, -3, 5); light.castShadow = true;
    light.shadow.mapSize.set(1024, 1024); light.shadow.camera.left = -2; light.shadow.camera.right = 2;
    light.shadow.camera.top = 2; light.shadow.camera.bottom = -2; light.shadow.bias = -0.0005;
    this.scene.add(light);
    const grid = new THREE.GridHelper(8, 40, 0xa0afbd, 0xd1dbe3);
    grid.rotateX(Math.PI / 2); grid.position.z = 0.001;
    this.scene.add(grid);
    this.geoms = [];
    this.geometries = new Set();
    this.materials = new Set();
    this.matrix = new THREE.Matrix4();
    this.resize = new ResizeObserver(() => {
      const { width, height } = host.getBoundingClientRect();
      this.camera.aspect = width / height; this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height, false); this.dirty = true;
    });
    this.resize.observe(host);
    this.renderer.setAnimationLoop(() => {
      if (document.hidden) return;
      const cameraChanged = this.controls.update();
      if (this.dirty || cameraChanged) { this.renderer.render(this.scene, this.camera); this.dirty = false; }
    });
  }
  home() { this.camera.position.set(1.25, -1.45, 0.9); this.controls.target.set(0, 0, 0.24); this.controls.update(); this.dirty = true; }
  load(description) {
    const meshGeometries = new Map();
    for (const [id, mesh] of Object.entries(description.meshes)) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(mesh.vertices, 3));
      geometry.setIndex(Array.from(mesh.faces));
      geometry.computeVertexNormals();
      meshGeometries.set(Number(id), geometry); this.geometries.add(geometry);
    }
    for (const g of description.geoms) {
      let geometry;
      const [x, y, z] = g.size;
      switch (g.type) {
        case 0: geometry = new THREE.PlaneGeometry(200, 200); break;
        case 2: geometry = new THREE.SphereGeometry(x, 20, 12); break;
        case 3: geometry = new THREE.CapsuleGeometry(x, 2 * y, 6, 12); geometry.rotateX(Math.PI / 2); break;
        case 4: geometry = new THREE.SphereGeometry(1, 20, 12); geometry.scale(x, y, z); break;
        case 5: geometry = new THREE.CylinderGeometry(x, x, 2 * y, 20); geometry.rotateX(Math.PI / 2); break;
        case 6: geometry = new THREE.BoxGeometry(2 * x, 2 * y, 2 * z); break;
        case 7: geometry = meshGeometries.get(g.mesh); break;
        default: throw new Error(`Unsupported geometry ${g.type}`);
      }
      if (!geometry) throw new Error('Missing compiled mesh');
      this.geometries.add(geometry);
      const material = new THREE.MeshStandardMaterial({ color: new THREE.Color(...g.rgba.slice(0, 3)), roughness: 0.65,
        metalness: 0.12, opacity: g.rgba[3], transparent: g.rgba[3] < 1 });
      if (g.type === 0) { material.color.set(0xe6ecf1); material.metalness = 0; }
      this.materials.add(material);
      const object = new THREE.Mesh(geometry, material);
      object.matrixAutoUpdate = false;
      object.castShadow = g.type !== 0; object.receiveShadow = true;
      object.visible = g.group !== 3;
      this.scene.add(object); this.geoms.push({ object, ...g });
    }
  }
  collisionOnly(enabled) {
    this.dirty = true;
    for (const g of this.geoms) g.object.visible = g.type === 0 || (enabled ? g.group === 3 : g.group !== 3);
  }
  update({ positions: p, rotations: r }) {
    this.dirty = true;
    for (const { id, object } of this.geoms) {
      const j = 9 * id, i = 3 * id;
      object.matrix.set(r[j], r[j + 1], r[j + 2], p[i], r[j + 3], r[j + 4], r[j + 5], p[i + 1],
        r[j + 6], r[j + 7], r[j + 8], p[i + 2], 0, 0, 0, 1);
      object.matrixWorldNeedsUpdate = true;
    }
  }
  dispose() {
    this.renderer.setAnimationLoop(null); this.resize.disconnect(); this.controls.dispose();
    for (const geometry of this.geometries) geometry.dispose();
    for (const material of this.materials) material.dispose();
    this.renderer.dispose(); this.renderer.domElement.remove();
  }
}
