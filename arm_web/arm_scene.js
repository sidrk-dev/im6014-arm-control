import * as THREE from './vendor/three.module.js';

export class ArmScene {
  constructor(canvasContainer) {
    this.container = canvasContainer;
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.statusEl = canvasContainer.querySelector('#model-status');
    this.sourceEl = canvasContainer.querySelector('#model-source');
    this.baseGroup = null;
    this.j1Pivot = null;
    this.j2Pivot = null;
    this.j3Pivot = null;
    this.j1Origin = new THREE.Vector3(0.0, 0.0, 0.0643);
    this.j1Axis = new THREE.Vector3(0.0, 0.0, -1.0).normalize();

    this.j2OriginWorld = new THREE.Vector3(0.026856, -0.001573, 0.137201);
    this.j2Axis = new THREE.Vector3(0.998289, -0.058467, 0.0).normalize();

    this.j3OriginWorld = new THREE.Vector3(-0.035525, -0.278126, 0.348443);
    this.j3Axis = new THREE.Vector3(0.998289, -0.058468, 0.0).normalize();
    this.isDragging = false;
    this.prevMouse = { x: 0, y: 0 };
    this.spherical = { radius: 1.2, theta: Math.PI / 4, phi: Math.PI / 3 };
    this.target = new THREE.Vector3(0, -0.1, 0.2);

    this.init();
  }

  init() {
    const width = this.container.clientWidth || 600;
    const height = this.container.clientHeight || 450;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xe7e8e9);
    THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 10.0);
    this.updateCameraPosition();

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = false;
    this.container.appendChild(this.renderer.domElement);
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight1.position.set(2, 3, 4);
    this.scene.add(dirLight1);

    const grid = new THREE.GridHelper(1.2, 24, 0x888888, 0xbbbbbb);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
    this.buildKinematicsTree();
    this.setupInteraction();
    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
    this.loadModelAssets();
  }

  buildKinematicsTree() {
    this.baseGroup = new THREE.Group();
    this.scene.add(this.baseGroup);
    this.j1Pivot = new THREE.Group();
    this.j1Pivot.position.copy(this.j1Origin);
    this.baseGroup.add(this.j1Pivot);
    this.j2Pivot = new THREE.Group();
    const j2Rel = this.j2OriginWorld.clone().sub(this.j1Origin);
    this.j2Pivot.position.copy(j2Rel);
    this.j1Pivot.add(this.j2Pivot);
    this.j3Pivot = new THREE.Group();
    const j3Rel = this.j3OriginWorld.clone().sub(this.j2OriginWorld);
    this.j3Pivot.position.copy(j3Rel);
    this.j2Pivot.add(this.j3Pivot);

  }

  setModelStatus(message, isError = false) {
    if (!this.statusEl) return;
    this.statusEl.textContent = message;
    this.statusEl.classList.toggle('error', isError);
  }

  showPath(points) {
    if(this.pathLine){this.scene.remove(this.pathLine);this.pathLine.geometry.dispose();this.pathLine.material.dispose();this.pathLine=null;}
    if(!points?.length)return;
    const geometry=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...p.map(v=>v/1000))));
    this.pathLine=new THREE.Line(geometry,new THREE.LineBasicMaterial({color:0x555555}));
    this.scene.add(this.pathLine);
  }

  async loadModelAssets() {
    this.setModelStatus('Loading CAD preview…');
    try {
      const resp = await fetch('/arm_assets/model.json');
      if (!resp.ok) throw new Error(`model.json HTTP ${resp.status}`);
      const model = await resp.json();
      this.model = model;
      if (!Array.isArray(model.joints) || model.joints.length < 3) throw new Error('model has no three-joint kinematics');
      const joints = model.joints.slice(0, 3);
      for (const joint of joints) {
        if (!Array.isArray(joint.origin) || joint.origin.length !== 3
          || !joint.origin.every(Number.isFinite)
          || !Array.isArray(joint.axis) || joint.axis.length !== 3
          || !joint.axis.every(Number.isFinite)
          || Math.hypot(...joint.axis) < 1e-9) {
          throw new Error('model contains invalid joint kinematics');
        }
      }

      const [j0, j1, j2] = joints;
      this.j1Origin.set(j0.origin[0] / 1000, j0.origin[1] / 1000, j0.origin[2] / 1000);
      this.j1Axis.set(...j0.axis).normalize();
      this.j2OriginWorld.set(j1.origin[0] / 1000, j1.origin[1] / 1000, j1.origin[2] / 1000);
      this.j2Axis.set(...j1.axis).normalize();
      this.j3OriginWorld.set(j2.origin[0] / 1000, j2.origin[1] / 1000, j2.origin[2] / 1000);
      this.j3Axis.set(...j2.axis).normalize();
      this.j1Pivot.position.copy(this.j1Origin);
      this.j2Pivot.position.copy(this.j2OriginWorld.clone().sub(this.j1Origin));
      this.j3Pivot.position.copy(this.j3OriginWorld.clone().sub(this.j2OriginWorld));

      if (!model.bodies || typeof model.bodies !== 'object') throw new Error('model has no body meshes');
      const loaded = await this.loadBodiesMeshes(model.bodies);
      if (!loaded) throw new Error('model contains no renderable body meshes');


      const archive = model.source?.archive_file || 'source archive not recorded';
      const urdf = model.cleaned_urdf?.file || model.source?.urdf_file || 'URDF source not recorded';
      if (this.sourceEl) {
        this.sourceEl.textContent = `CAD preview · ${archive} → ${urdf} · J1 physical option; J2/J3 virtual`;
      }
      this.setModelStatus(`CAD meshes loaded · ${loaded} parts · visualization only`);
    } catch (error) {
      this.setModelStatus(`CAD preview failed: ${error.message}`, true);
      if (this.sourceEl) this.sourceEl.textContent = 'CAD unavailable · motor status comes from Check';
    }
  }

  async loadBodiesMeshes(bodies) {
    const materials = {
      hardware: new THREE.MeshLambertMaterial({ color: 0xb4bac2, flatShading: true }),
      frame: new THREE.MeshLambertMaterial({ color: 0x7d91ad, flatShading: true }),
      motor: new THREE.MeshLambertMaterial({ color: 0x2e333b, flatShading: true }),
      hub: new THREE.MeshLambertMaterial({ color: 0x4a515c, flatShading: true }),
      link: new THREE.MeshLambertMaterial({ color: 0xe27d3c, flatShading: true }),
      tube: new THREE.MeshLambertMaterial({ color: 0x3b4048, flatShading: true })
    };
    const bodyParents = {
      base: { parent: this.baseGroup, offset: new THREE.Vector3(0, 0, 0) },
      j1: { parent: this.j1Pivot, offset: this.j1Origin.clone().negate() },
      j2: { parent: this.j2Pivot, offset: this.j2OriginWorld.clone().negate() },
      j3: { parent: this.j3Pivot, offset: this.j3OriginWorld.clone().negate() }
    };
    const pendingGroups = [];
    const geometries = [];
    let meshCount = 0;
    try {
      for (const [bodyName, mats] of Object.entries(bodies)) {
        const config = bodyParents[bodyName];
        if (!config || !mats || typeof mats !== 'object') throw new Error(`unsupported body ${bodyName}`);
        const bodyGroup = new THREE.Group();
        bodyGroup.position.copy(config.offset);

        for (const [matName, fileInfo] of Object.entries(mats)) {
          if (!fileInfo || typeof fileInfo.file !== 'string'
            || !/^meshes\/[A-Za-z0-9._-]+\.bin$/.test(fileInfo.file)) {
            throw new Error(`invalid mesh path for ${bodyName}/${matName}`);
          }
          const vCount = fileInfo.vertices;
          const tCount = fileInfo.triangles;
          if (!Number.isSafeInteger(vCount) || vCount < 3 || vCount > 200000
            || !Number.isSafeInteger(tCount) || tCount < 1 || tCount > 400000) {
            throw new Error(`invalid mesh dimensions for ${bodyName}/${matName}`);
          }

          const meshResp = await fetch(`/arm_assets/${fileInfo.file}`);
          if (!meshResp.ok) throw new Error(`${fileInfo.file} HTTP ${meshResp.status}`);
          const buffer = await meshResp.arrayBuffer();
          const positionBytes = vCount * 3 * Float32Array.BYTES_PER_ELEMENT;
          const indexBytes = tCount * 3 * Uint32Array.BYTES_PER_ELEMENT;
          if (buffer.byteLength !== positionBytes + indexBytes) {
            throw new Error(`${fileInfo.file} has an invalid byte length`);
          }

          const rawPositions = new Float32Array(buffer, 0, vCount * 3);
          const rawIndices = new Uint32Array(buffer, positionBytes, tCount * 3);
          if (!rawPositions.every(Number.isFinite)) throw new Error(`${fileInfo.file} has non-finite vertices`);
          if (!rawIndices.every(index => index < vCount)) throw new Error(`${fileInfo.file} has an out-of-range index`);

          const positions = Float32Array.from(rawPositions, value => value / 1000);
          const geometry = new THREE.BufferGeometry();
          geometries.push(geometry);
          geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
          geometry.setIndex(new THREE.BufferAttribute(rawIndices, 1));
          geometry.computeVertexNormals();
          bodyGroup.add(new THREE.Mesh(geometry, materials[matName] || materials.frame));
          meshCount++;
        }
        pendingGroups.push({ parent: config.parent, bodyGroup });
      }
    } catch (error) {
      for (const geometry of geometries) geometry.dispose();
      throw error;
    }

    for (const { parent, bodyGroup } of pendingGroups) parent.add(bodyGroup);
    return meshCount;
  }

  updateJoints(q1 = 0, q2 = 0, q3 = 0) {
    if (this.j1Pivot) {
      this.j1Pivot.setRotationFromAxisAngle(this.j1Axis, q1);
    }
    if (this.j2Pivot) {
      this.j2Pivot.setRotationFromAxisAngle(this.j2Axis, q2);
    }
    if (this.j3Pivot) {
      this.j3Pivot.setRotationFromAxisAngle(this.j3Axis, q3);
    }
  }

  updateCameraPosition() {
    const { radius, theta, phi } = this.spherical;
    const x = this.target.x + radius * Math.sin(phi) * Math.cos(theta);
    const y = this.target.y + radius * Math.sin(phi) * Math.sin(theta);
    const z = this.target.z + radius * Math.cos(phi);
    this.camera.position.set(x, y, z);
    this.camera.lookAt(this.target);
  }

  setupInteraction() {
    const el = this.renderer.domElement;

    el.addEventListener('mousedown', (e) => {
      this.isDragging = true;
      this.prevMouse = { x: e.clientX, y: e.clientY };
    });

    window.addEventListener('mouseup', () => {
      this.isDragging = false;
    });

    window.addEventListener('mousemove', (e) => {
      if (!this.isDragging) return;
      const dx = e.clientX - this.prevMouse.x;
      const dy = e.clientY - this.prevMouse.y;
      this.prevMouse = { x: e.clientX, y: e.clientY };

      this.spherical.theta -= dx * 0.008;
      this.spherical.phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, this.spherical.phi - dy * 0.008));
      this.updateCameraPosition();
    });

    el.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.spherical.radius = Math.max(0.3, Math.min(3.0, this.spherical.radius + e.deltaY * 0.001));
      this.updateCameraPosition();
    }, { passive: false });

    window.addEventListener('resize', () => {
      const w = this.container.clientWidth;
      const h = this.container.clientHeight;
      if (w && h) {
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
      }
    });
  }

  resetCamera() {
    this.spherical = { radius: 1.2, theta: Math.PI / 4, phi: Math.PI / 3 };
    this.target = new THREE.Vector3(0, -0.1, 0.2);
    this.updateCameraPosition();
  }

  animate() {
    requestAnimationFrame(this.animate);
    this.renderer.render(this.scene, this.camera);
  }
}
