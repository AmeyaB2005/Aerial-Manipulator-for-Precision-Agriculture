// EFT E610P Digital Twin Studio & Photography Suite
// Advanced Three.js Visualizer with PBR Materials, TDCR Arm, & 4K Screenshot Engine

// --- Scene, Camera, Renderer ---
const container = document.getElementById('canvas-container');
let scene, camera, renderer, controls;
let droneRoot, frameMesh, tankGroup, rotorsGroup, landingGearGroup, chinCamGroup, tdcrArmGroup;
let shadowPlane, gridHelper, dirLight, fillLight, rimLight, ambLight;
let animProps = true, turntableSpin = false;
let explodedFactor = 0;
let tdcrBendAngle = 0;
let tdcrYawAngle = 0;
let rotorMeshes = [];
let propGeometryCW = null, propGeometryCCW = null;
let currentEnv = 'studio-white';
let currentLivery = 'carbon';

// Materials dictionary
let materials = {};

const CAM_PRESETS = {
  'hero': { pos: [1.6, 1.2, 1.8], target: [0, 0.45, 0], fov: 45, name: 'HERO 3/4 PERSPECTIVE' },
  'front': { pos: [0, 0.6, 2.2], target: [0, 0.45, 0], fov: 40, name: 'FRONT ELEVATION' },
  'side': { pos: [2.2, 0.6, 0], target: [0, 0.45, 0], fov: 40, name: 'SIDE PROFILE' },
  'top': { pos: [0, 2.6, 0.001], target: [0, 0.45, 0], fov: 38, name: 'TOP-DOWN PLAN' },
  'underside': { pos: [1.2, -0.4, 1.4], target: [0, 0.35, 0], fov: 45, name: 'UNDERSIDE PAYLOAD' },
  'camera-fpv': { pos: [0.35, 0.62, 0.6], target: [0.8, 0.2, 1.2], fov: 60, name: 'FPV CHIN CAM POV' },
  'motor': { pos: [0.75, 0.85, 0.25], target: [0.585, 0.613, 0], fov: 30, name: 'MOTOR & ROTOR CLOSE-UP' },
  'arm-focus': { pos: [0.6, 0.2, 0.8], target: [0, 0.15, 0], fov: 35, name: 'TDCR CONTINUUM ARM FOCUS' }
};

let targetCamPos = new THREE.Vector3(...CAM_PRESETS['hero'].pos);
let targetCamTarget = new THREE.Vector3(...CAM_PRESETS['hero'].target);
let targetCamFov = CAM_PRESETS['hero'].fov;

function init() {
  // 1. Scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0d14);

  // 2. Camera
  camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.05, 100);
  camera.position.set(...CAM_PRESETS['hero'].pos);

  // 3. Renderer
  renderer = new THREE.WebGLRenderer({
    antialias: true,
    preserveDrawingBuffer: true,
    powerPreference: 'high-performance'
  });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  renderer.outputEncoding = THREE.sRGBEncoding;
  container.appendChild(renderer.domElement);

  // 4. Controls
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.target.set(0, 0.45, 0);
  controls.maxPolarAngle = Math.PI / 2 + 0.15;
  controls.minDistance = 0.3;
  controls.maxDistance = 8.0;

  // 5. Materials
  initMaterials();

  // 6. Lighting & Studio Environment
  initStudioLighting();

  // 7. Ground / Shadow Catcher
  initGround();

  // 8. Load Models & Digital Twin Structure
  initDigitalTwin();

  // 9. Event Listeners
  initUI();
  window.addEventListener('resize', onWindowResize);

  // 10. Load Existing Photos in Gallery
  refreshPhotoGallery();

  // 11. Render Loop
  animate();
}

function initMaterials() {
  materials.carbon = new THREE.MeshStandardMaterial({
    color: 0x181a20,
    roughness: 0.38,
    metalness: 0.45,
    envMapIntensity: 1.2
  });

  materials.stealth = new THREE.MeshStandardMaterial({
    color: 0x0f1115,
    roughness: 0.7,
    metalness: 0.15
  });

  materials.orange = new THREE.MeshStandardMaterial({
    color: 0xff6600,
    roughness: 0.3,
    metalness: 0.2
  });

  materials.white = new THREE.MeshStandardMaterial({
    color: 0xf5f7fa,
    roughness: 0.25,
    metalness: 0.1
  });

  materials.tank = new THREE.MeshPhysicalMaterial({
    color: 0xf0f3f6,
    roughness: 0.2,
    transmission: 0.75, // Translucent HDPE spray tank
    opacity: 0.92,
    transparent: true,
    ior: 1.45,
    thickness: 0.04
  });

  materials.tankMount = new THREE.MeshStandardMaterial({
    color: 0x1a1a1a,
    roughness: 0.5,
    metalness: 0.6
  });

  materials.motor = new THREE.MeshStandardMaterial({
    color: 0x242830,
    roughness: 0.2,
    metalness: 0.85
  });

  materials.motorGold = new THREE.MeshStandardMaterial({
    color: 0xcc9922,
    roughness: 0.25,
    metalness: 0.85
  });

  materials.propeller = new THREE.MeshStandardMaterial({
    color: 0x151618,
    roughness: 0.35,
    metalness: 0.3
  });

  materials.cameraBody = new THREE.MeshStandardMaterial({
    color: 0x111317,
    roughness: 0.4,
    metalness: 0.5
  });

  materials.cameraLens = new THREE.MeshPhysicalMaterial({
    color: 0x051530,
    roughness: 0.05,
    metalness: 0.9,
    transmission: 0.6,
    transparent: true
  });

  materials.tdcrDisk = new THREE.MeshStandardMaterial({
    color: 0x00e5ff,
    roughness: 0.25,
    metalness: 0.7
  });

  materials.tdcrBackbone = new THREE.MeshStandardMaterial({
    color: 0x8899aa,
    roughness: 0.4,
    metalness: 0.5
  });

  materials.wireframe = new THREE.MeshBasicMaterial({
    color: 0x00e5ff,
    wireframe: true
  });
}

function initStudioLighting() {
  ambLight = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(ambLight);

  // Key Light (Main soft shadow caster)
  dirLight = new THREE.DirectionalLight(0xffffff, 1.8);
  dirLight.position.set(3, 5, 3);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.camera.near = 0.5;
  dirLight.shadow.camera.far = 15;
  dirLight.shadow.camera.left = -1.8;
  dirLight.shadow.camera.right = 1.8;
  dirLight.shadow.camera.top = 1.8;
  dirLight.shadow.camera.bottom = -1.8;
  dirLight.shadow.bias = -0.0003;
  scene.add(dirLight);

  // Fill Light (Soft cool fill)
  fillLight = new THREE.DirectionalLight(0x99ccff, 0.9);
  fillLight.position.set(-3, 3, -2);
  scene.add(fillLight);

  // Rim / Kicker Light
  rimLight = new THREE.DirectionalLight(0xffffff, 1.2);
  rimLight.position.set(0, 4, -4);
  scene.add(rimLight);

  // Bottom bounce light
  const bounceLight = new THREE.DirectionalLight(0xffffff, 0.4);
  bounceLight.position.set(0, -2, 0);
  scene.add(bounceLight);
}

function initGround() {
  const planeGeo = new THREE.PlaneGeometry(30, 30);
  const planeMat = new THREE.ShadowMaterial({
    opacity: 0.35
  });
  shadowPlane = new THREE.Mesh(planeGeo, planeMat);
  shadowPlane.rotation.x = -Math.PI / 2;
  shadowPlane.position.y = 0;
  shadowPlane.receiveShadow = true;
  scene.add(shadowPlane);

  gridHelper = new THREE.GridHelper(10, 20, 0x00e5ff, 0x223344);
  gridHelper.position.y = 0.001;
  scene.add(gridHelper);
}

function initDigitalTwin() {
  droneRoot = new THREE.Group();
  scene.add(droneRoot);

  tankGroup = new THREE.Group();
  rotorsGroup = new THREE.Group();
  landingGearGroup = new THREE.Group();
  chinCamGroup = new THREE.Group();
  tdcrArmGroup = new THREE.Group();

  droneRoot.add(tankGroup);
  droneRoot.add(rotorsGroup);
  droneRoot.add(landingGearGroup);
  droneRoot.add(chinCamGroup);
  droneRoot.add(tdcrArmGroup);

  // 1. Load frame.STL (High detail CAD model)
  const stlLoader = new THREE.STLLoader();
  stlLoader.load('assets/frame.STL', function (geometry) {
    geometry.computeVertexNormals();
    geometry.center();

    // In raw STL, height was Y, min Y was -325mm, so offset to sit on ground Y=0
    frameMesh = new THREE.Mesh(geometry, materials.carbon);
    frameMesh.scale.set(0.001, 0.001, 0.001); // mm to meters
    frameMesh.position.set(0, 0.325, 0); // Put landing skids on ground
    frameMesh.castShadow = true;
    frameMesh.receiveShadow = true;
    droneRoot.add(frameMesh);

    document.getElementById('stat-poly').textContent = `${(geometry.attributes.position.count / 3 / 1000).toFixed(0)}k Polygons`;
  }, undefined, function (error) {
    console.error('Error loading frame.STL:', error);
  });

  // 2. Build 10L Spray Payload Tank Sub-Assembly
  buildSprayTank();

  // 3. Load & Mount 6x Motors and 24" Folding Propellers
  buildRotors();

  // 4. Build Chin FPV Camera Gimbal
  buildChinCamera();

  // 5. Build Tendon-Driven Continuum Manipulator Arm (TDCR)
  buildTDCRArm();
}

function buildSprayTank() {
  // Main HDPE Tank body
  const tankGeo = new THREE.BoxGeometry(0.38, 0.18, 0.38);
  const tankMesh = new THREE.Mesh(tankGeo, materials.tank);
  tankMesh.position.set(0, 0.22, 0);
  tankMesh.castShadow = true;
  tankMesh.receiveShadow = true;
  tankGroup.add(tankMesh);

  // Top mount plate
  const mountGeo = new THREE.BoxGeometry(0.24, 0.015, 0.28);
  const mountMesh = new THREE.Mesh(mountGeo, materials.tankMount);
  mountMesh.position.set(0, 0.315, -0.04);
  mountMesh.castShadow = true;
  tankGroup.add(mountMesh);

  // Tank Cap / Lid
  const capGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.03, 24);
  const capMesh = new THREE.Mesh(capGeo, materials.tankMount);
  capMesh.position.set(0, 0.32, 0.1);
  capMesh.castShadow = true;
  tankGroup.add(capMesh);

  // Bottom Spray Manifold & Pump mount
  const pumpGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.08, 16);
  const pumpMesh = new THREE.Mesh(pumpGeo, materials.motor);
  pumpMesh.rotation.z = Math.PI / 2;
  pumpMesh.position.set(0, 0.11, 0);
  tankGroup.add(pumpMesh);
}

function buildRotors() {
  const motorAngles = [0, 60, 120, 180, 240, 300]; // 6 arms at 60 deg
  const R = 0.5854; // Arm radius in meters
  const H = 0.6128; // Height above ground in meters

  const objLoader = new THREE.OBJLoader();

  // Try loading converted OBJ propellers or fallback to realistic procedural blades
  objLoader.load('assets/prop_cw.obj', function (objCW) {
    propGeometryCW = objCW.children[0].geometry;
    objLoader.load('assets/prop_ccw.obj', function (objCCW) {
      propGeometryCCW = objCCW.children[0].geometry;
      createRotorInstances(motorAngles, R, H);
    }, undefined, function () {
      createRotorInstances(motorAngles, R, H);
    });
  }, undefined, function () {
    createRotorInstances(motorAngles, R, H);
  });
}

function createRotorInstances(motorAngles, R, H) {
  rotorsGroup.clear();
  rotorMeshes = [];

  motorAngles.forEach((deg, idx) => {
    const rad = THREE.MathUtils.degToRad(deg);
    const x = R * Math.cos(rad);
    const z = R * Math.sin(rad);

    const singleRotorGroup = new THREE.Group();
    singleRotorGroup.position.set(x, H, z);
    singleRotorGroup.userData = { originalX: x, originalZ: z, originalY: H, rad: rad, idx: idx };

    // Motor Base Stator & Bell
    const statorGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.024, 24);
    const statorMesh = new THREE.Mesh(statorGeo, materials.motor);
    statorMesh.castShadow = true;
    singleRotorGroup.add(statorMesh);

    const bellGeo = new THREE.CylinderGeometry(0.03, 0.03, 0.015, 24);
    const bellMesh = new THREE.Mesh(bellGeo, materials.motorGold);
    bellMesh.position.y = 0.012;
    singleRotorGroup.add(bellMesh);

    // Propeller spinning hub
    const propHub = new THREE.Group();
    propHub.position.y = 0.022;

    const isCW = (idx % 2 === 0);
    let propMesh;

    if (isCW && propGeometryCW) {
      propMesh = new THREE.Mesh(propGeometryCW, materials.propeller);
      propMesh.scale.set(0.0022, 0.0022, 0.0022);
    } else if (!isCW && propGeometryCCW) {
      propMesh = new THREE.Mesh(propGeometryCCW, materials.propeller);
      propMesh.scale.set(0.0022, 0.0022, 0.0022);
    } else {
      // Procedural folding 24" carbon blade
      propMesh = createProceduralPropeller(isCW);
    }

    propMesh.castShadow = true;
    propHub.add(propMesh);
    singleRotorGroup.add(propHub);

    rotorMeshes.push({
      hub: propHub,
      isCW: isCW,
      speed: isCW ? 1.0 : -1.0,
      group: singleRotorGroup
    });

    rotorsGroup.add(singleRotorGroup);
  });
}

function createProceduralPropeller(isCW) {
  const propGroup = new THREE.Group();
  // Central hub
  const hubGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.01, 16);
  const hub = new THREE.Mesh(hubGeo, materials.carbon);
  propGroup.add(hub);

  // 2 Folding Blades (24 inch span = ~0.6m diameter)
  for (let i = 0; i < 2; i++) {
    const bladeGeo = new THREE.BoxGeometry(0.28, 0.004, 0.032);
    bladeGeo.translate(0.15, 0, 0);
    const blade = new THREE.Mesh(bladeGeo, materials.propeller);
    blade.rotation.y = (i * Math.PI) + (isCW ? 0.08 : -0.08);
    propGroup.add(blade);
  }
  return propGroup;
}

function buildChinCamera() {
  chinCamGroup.position.set(0, 0.35, 0.3);

  // Mount rod
  const rodGeo = new THREE.CylinderGeometry(0.006, 0.006, 0.1, 12);
  rodGeo.rotateX(Math.PI / 2);
  const rod = new THREE.Mesh(rodGeo, materials.carbon);
  chinCamGroup.add(rod);

  // Gimbal housing
  const camBoxGeo = new THREE.BoxGeometry(0.05, 0.05, 0.05);
  const camBox = new THREE.Mesh(camBoxGeo, materials.cameraBody);
  camBox.position.set(0, -0.03, 0.06);
  camBox.rotation.x = THREE.MathUtils.degToRad(35); // 35 deg downward FPV tilt
  camBox.castShadow = true;

  // Lens
  const lensGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.015, 20);
  lensGeo.rotateX(Math.PI / 2);
  const lens = new THREE.Mesh(lensGeo, materials.cameraLens);
  lens.position.set(0, 0, 0.026);
  camBox.add(lens);

  chinCamGroup.add(camBox);
}

function buildTDCRArm() {
  // Tendon-Driven Continuum Robot (TDCR) with 3 segments
  tdcrArmGroup.position.set(0, 0.12, 0); // Mount under tank/frame
  updateTDCRArmGeometry(0, 0);
}

function updateTDCRArmGeometry(bendDeg, yawDeg) {
  tdcrArmGroup.clear();

  const numDisks = 14;
  const totalLength = 0.28; // 280 mm continuum arm
  const bendRad = THREE.MathUtils.degToRad(bendDeg);
  const yawRad = THREE.MathUtils.degToRad(yawDeg);

  // Base mounting collar
  const baseCollarGeo = new THREE.CylinderGeometry(0.035, 0.035, 0.015, 24);
  const baseCollar = new THREE.Mesh(baseCollarGeo, materials.motor);
  tdcrArmGroup.add(baseCollar);

  // Calculate backbone spline points
  const points = [];
  for (let i = 0; i <= numDisks; i++) {
    const t = i / numDisks;
    // Constant curvature bending formula
    let curBend = bendRad * (t * t); // Progressive bending along length
    let dx = Math.sin(curBend) * Math.sin(yawRad) * (t * totalLength);
    let dz = Math.sin(curBend) * Math.cos(yawRad) * (t * totalLength);
    let dy = -Math.cos(curBend) * (t * totalLength);
    points.push(new THREE.Vector3(dx, dy, dz));
  }

  // Draw Central Backbone Tube
  const curve = new THREE.CatmullRomCurve3(points);
  const tubeGeo = new THREE.TubeGeometry(curve, 32, 0.005, 12, false);
  const tubeMesh = new THREE.Mesh(tubeGeo, materials.tdcrBackbone);
  tubeMesh.castShadow = true;
  tdcrArmGroup.add(tubeMesh);

  // Tendon Spacer Disks along backbone
  for (let i = 1; i <= numDisks; i++) {
    const pt = points[i];
    const prevPt = points[i - 1];
    const dir = new THREE.Vector3().subVectors(pt, prevPt).normalize();

    const diskGeo = new THREE.CylinderGeometry(0.024, 0.024, 0.004, 20);
    const diskMesh = new THREE.Mesh(diskGeo, materials.tdcrDisk);
    diskMesh.position.copy(pt);
    diskMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    diskMesh.castShadow = true;
    tdcrArmGroup.add(diskMesh);
  }

  // End-Effector (Precision Agriculture Spray Nozzle / Gripper)
  const tipPt = points[points.length - 1];
  const tipDir = new THREE.Vector3().subVectors(tipPt, points[points.length - 2]).normalize();

  const toolGeo = new THREE.ConeGeometry(0.018, 0.04, 16);
  const toolMesh = new THREE.Mesh(toolGeo, materials.orange);
  toolMesh.position.copy(tipPt);
  toolMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, -1, 0), tipDir);
  toolMesh.castShadow = true;
  tdcrArmGroup.add(toolMesh);
}

function updateExplodedView(factor) {
  explodedFactor = factor;

  // Frame stays or moves slightly up
  if (frameMesh) {
    frameMesh.position.y = 0.325 + factor * 0.05;
  }

  // Tank moves downward
  if (tankGroup) {
    tankGroup.position.y = -factor * 0.25;
  }

  // TDCR Arm moves even further downward
  if (tdcrArmGroup) {
    tdcrArmGroup.position.y = 0.12 - factor * 0.45;
  }

  // Chin Camera moves forward
  if (chinCamGroup) {
    chinCamGroup.position.z = 0.3 + factor * 0.25;
  }

  // Rotors explode radially outward & upward
  rotorsGroup.children.forEach(r => {
    const origX = r.userData.originalX;
    const origZ = r.userData.originalZ;
    const origY = r.userData.originalY;
    const rad = r.userData.rad;

    const radialOffset = factor * 0.35;
    const verticalOffset = factor * 0.2;

    r.position.x = origX + Math.cos(rad) * radialOffset;
    r.position.z = origZ + Math.sin(rad) * radialOffset;
    r.position.y = origY + verticalOffset;
  });
}

function setEnvironment(env) {
  currentEnv = env;
  document.querySelectorAll('.env-btn').forEach(b => b.classList.toggle('active', b.dataset.env === env));

  switch (env) {
    case 'studio-white':
      scene.background = new THREE.Color(0xf5f7fb);
      ambLight.color.setHex(0xffffff);
      ambLight.intensity = 1.0;
      dirLight.color.setHex(0xffffff);
      dirLight.intensity = 1.6;
      fillLight.color.setHex(0xd0e0ff);
      fillLight.intensity = 0.8;
      rimLight.intensity = 0.6;
      shadowPlane.material.opacity = 0.22;
      gridHelper.visible = false;
      break;

    case 'cyber-dark':
      scene.background = new THREE.Color(0x090b10);
      ambLight.color.setHex(0x1a2030);
      ambLight.intensity = 0.6;
      dirLight.color.setHex(0x00e5ff);
      dirLight.intensity = 1.4;
      fillLight.color.setHex(0xff0055);
      fillLight.intensity = 1.2;
      rimLight.color.setHex(0x0070f3);
      rimLight.intensity = 1.5;
      shadowPlane.material.opacity = 0.5;
      gridHelper.visible = true;
      break;

    case 'farmland':
      scene.background = new THREE.Color(0x1e2d24);
      ambLight.color.setHex(0xfffaed);
      ambLight.intensity = 0.8;
      dirLight.color.setHex(0xffbb55);
      dirLight.intensity = 2.0;
      fillLight.color.setHex(0x669977);
      fillLight.intensity = 0.7;
      rimLight.intensity = 0.5;
      shadowPlane.material.opacity = 0.35;
      gridHelper.visible = true;
      break;

    case 'wireframe':
      scene.background = new THREE.Color(0x05070a);
      ambLight.color.setHex(0x003344);
      ambLight.intensity = 1.0;
      dirLight.intensity = 0.5;
      fillLight.intensity = 0.3;
      rimLight.intensity = 0.5;
      shadowPlane.material.opacity = 0.1;
      gridHelper.visible = true;
      break;

    case 'transparent':
      scene.background = null;
      ambLight.color.setHex(0xffffff);
      ambLight.intensity = 1.0;
      dirLight.intensity = 1.6;
      fillLight.intensity = 0.8;
      rimLight.intensity = 0.6;
      shadowPlane.material.opacity = 0.0;
      gridHelper.visible = false;
      break;
  }

  // Update wireframe material if requested
  if (frameMesh) {
    if (env === 'wireframe') {
      frameMesh.material = materials.wireframe;
    } else {
      setLivery(currentLivery);
    }
  }
}

function setLivery(livery) {
  currentLivery = livery;
  document.querySelectorAll('.mat-btn').forEach(b => b.classList.toggle('active', b.dataset.mat === livery));

  let mat;
  switch (livery) {
    case 'carbon': mat = materials.carbon; break;
    case 'agri-orange': mat = materials.orange; break;
    case 'stealth-black': mat = materials.stealth; break;
    case 'clean-white': mat = materials.white; break;
    default: mat = materials.carbon;
  }

  if (frameMesh && currentEnv !== 'wireframe') {
    frameMesh.material = mat;
  }
}

function setCameraPreset(presetKey) {
  const preset = CAM_PRESETS[presetKey];
  if (!preset) return;

  targetCamPos.set(...preset.pos);
  targetCamTarget.set(...preset.target);
  targetCamFov = preset.fov;

  document.querySelectorAll('.cam-btn').forEach(b => b.classList.toggle('active', b.dataset.cam === presetKey));
  document.getElementById('hud-cam-name').textContent = preset.name;

  document.getElementById('slider-fov').value = preset.fov;
  document.getElementById('fov-val').textContent = `${preset.fov}°`;
}

// --- High-Resolution Photo Capture Engine ---
async function takeClearPhoto() {
  const resKey = document.getElementById('photo-res-select').value;
  let targetWidth, targetHeight;

  switch (resKey) {
    case '4k':
      targetWidth = 3840; targetHeight = 2160;
      break;
    case '2k':
      targetWidth = 2560; targetHeight = 1440;
      break;
    case '1080p':
      targetWidth = 1920; targetHeight = 1080;
      break;
    default:
      targetWidth = window.innerWidth; targetHeight = window.innerHeight;
  }

  // Save current renderer state
  const origWidth = renderer.domElement.width;
  const origHeight = renderer.domElement.height;
  const origAspect = camera.aspect;

  // Temporarily resize renderer for ultra crisp render
  camera.aspect = targetWidth / targetHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(targetWidth, targetHeight, false);
  renderer.render(scene, camera);

  // Extract high-res PNG Data URL
  const dataUrl = renderer.domElement.toDataURL('image/png', 1.0);

  // Restore screen size
  camera.aspect = origAspect;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight, true);

  const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
  const filename = `eft_e610p_${resKey}_${timestamp}.png`;

  // 1. Trigger Direct Browser Download
  const link = document.createElement('a');
  link.download = filename;
  link.href = dataUrl;
  link.click();

  // 2. Save directly to the repository's renders/ folder via the server API (server.py)
  try {
    const resp = await fetch('/api/save_photo', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ image: dataUrl, filename: filename })
    });
    const res = await resp.json();
    if (res.status === 'success') {
      showToast(`Saved photo to renders/${filename} (${(res.size / 1024 / 1024).toFixed(2)} MB)`);
      refreshPhotoGallery();
    }
  } catch (err) {
    console.warn('Could not save to local renders folder via API:', err);
    showToast(`Captured & Downloaded: ${filename}`);
  }
}

async function refreshPhotoGallery() {
  try {
    const resp = await fetch('/api/list_photos');
    const photos = await resp.json();

    const grid = document.getElementById('gallery-grid');
    const badge = document.getElementById('gallery-badge');
    badge.textContent = photos.length;

    if (!photos || photos.length === 0) {
      grid.innerHTML = '<div class="gallery-empty">No photos captured yet. Click the shutter button to take clear photos!</div>';
      return;
    }

    grid.innerHTML = photos.map(p => `
      <div class="gallery-card">
        <img src="${p.url}" class="gallery-thumb" alt="${p.filename}" loading="lazy" />
        <div class="gallery-info">
          <div class="gallery-filename" title="${p.filename}">${p.filename}</div>
          <div style="font-size: 9px; color: var(--text-muted); font-family: monospace;">${(p.size / 1024 / 1024).toFixed(2)} MB</div>
          <div class="gallery-actions">
            <a href="${p.url}" download="${p.filename}" class="gallery-btn">Download</a>
            <a href="${p.url}" target="_blank" class="gallery-btn">Full View</a>
          </div>
        </div>
      </div>
    `).join('');
  } catch (err) {
    console.warn('Gallery refresh error:', err);
  }
}

function showToast(msg) {
  const toast = document.getElementById('toast');
  document.getElementById('toast-msg').textContent = msg;
  toast.classList.add('show');
  setTimeout(() => toast.classList.remove('show'), 3500);
}

function initUI() {
  // Camera buttons
  document.querySelectorAll('.cam-btn').forEach(btn => {
    btn.addEventListener('click', () => setCameraPreset(btn.dataset.cam));
  });

  // Environment buttons
  document.querySelectorAll('.env-btn').forEach(btn => {
    btn.addEventListener('click', () => setEnvironment(btn.dataset.env));
  });

  // Livery buttons
  document.querySelectorAll('.mat-btn').forEach(btn => {
    btn.addEventListener('click', () => setLivery(btn.dataset.mat));
  });

  // FOV Slider
  document.getElementById('slider-fov').addEventListener('input', e => {
    const fov = parseFloat(e.target.value);
    targetCamFov = fov;
    camera.fov = fov;
    camera.updateProjectionMatrix();
    document.getElementById('fov-val').textContent = `${fov}°`;
  });

  // Turntable Toggle
  document.getElementById('toggle-turntable').addEventListener('change', e => {
    turntableSpin = e.target.checked;
  });

  // Propeller Spin Toggle
  document.getElementById('toggle-props').addEventListener('change', e => {
    animProps = e.target.checked;
  });

  // Specs HUD Toggle
  document.getElementById('toggle-specs').addEventListener('change', e => {
    document.getElementById('tech-hud').style.display = e.target.checked ? 'block' : 'none';
  });

  // Component Toggles
  document.getElementById('comp-cad-frame').addEventListener('change', e => {
    if (frameMesh) frameMesh.visible = e.target.checked;
  });
  document.getElementById('comp-tank').addEventListener('change', e => {
    tankGroup.visible = e.target.checked;
  });
  document.getElementById('comp-rotors').addEventListener('change', e => {
    rotorsGroup.visible = e.target.checked;
  });
  document.getElementById('comp-chin-cam').addEventListener('change', e => {
    chinCamGroup.visible = e.target.checked;
  });
  document.getElementById('comp-tdcr-arm').addEventListener('change', e => {
    tdcrArmGroup.visible = e.target.checked;
  });

  // Exploded View Slider
  document.getElementById('slider-exploded').addEventListener('input', e => {
    const val = parseFloat(e.target.value) / 100;
    document.getElementById('exploded-val').textContent = `${e.target.value}%`;
    updateExplodedView(val);
  });

  // TDCR Arm Bending Slider
  document.getElementById('slider-tdcr').addEventListener('input', e => {
    tdcrBendAngle = parseFloat(e.target.value);
    document.getElementById('tdcr-val').textContent = `${tdcrBendAngle}°`;
    updateTDCRArmGeometry(tdcrBendAngle, tdcrYawAngle);
  });

  // TDCR Arm Yaw Slider
  document.getElementById('slider-tdcr-yaw').addEventListener('input', e => {
    tdcrYawAngle = parseFloat(e.target.value);
    document.getElementById('tdcr-yaw-val').textContent = `${tdcrYawAngle}°`;
    updateTDCRArmGeometry(tdcrBendAngle, tdcrYawAngle);
  });

  // Shutter Buttons
  document.getElementById('btn-snap-top').addEventListener('click', takeClearPhoto);
  document.getElementById('btn-quick-snap').addEventListener('click', takeClearPhoto);

  // Gallery Modal
  const galleryModal = document.getElementById('gallery-modal');
  document.getElementById('btn-gallery-toggle').addEventListener('click', () => {
    refreshPhotoGallery();
    galleryModal.classList.add('open');
  });
  document.getElementById('btn-close-gallery').addEventListener('click', () => {
    galleryModal.classList.remove('open');
  });
  galleryModal.addEventListener('click', e => {
    if (e.target === galleryModal) galleryModal.classList.remove('open');
  });
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

// Smooth camera lerp
function updateCameraLerp() {
  if (camera.position.distanceTo(targetCamPos) > 0.005) {
    camera.position.lerp(targetCamPos, 0.08);
  }
  if (controls.target.distanceTo(targetCamTarget) > 0.005) {
    controls.target.lerp(targetCamTarget, 0.08);
  }
  if (Math.abs(camera.fov - targetCamFov) > 0.1) {
    camera.fov += (targetCamFov - camera.fov) * 0.08;
    camera.updateProjectionMatrix();
  }
}

// FPS Counter
let frameCount = 0, lastFpsUpdate = performance.now();
function updateFPS() {
  frameCount++;
  const now = performance.now();
  if (now - lastFpsUpdate >= 500) {
    const fps = Math.round((frameCount * 1000) / (now - lastFpsUpdate));
    document.getElementById('stat-fps').textContent = fps;
    frameCount = 0;
    lastFpsUpdate = now;
  }
}

function animate() {
  requestAnimationFrame(animate);

  updateCameraLerp();
  controls.update();

  // Rotate propellers
  if (animProps && rotorMeshes.length > 0) {
    rotorMeshes.forEach(r => {
      r.hub.rotation.y += r.speed * 0.35;
    });
  }

  // 360 Turntable rotation
  if (turntableSpin && droneRoot) {
    droneRoot.rotation.y += 0.008;
  }

  updateFPS();
  renderer.render(scene, camera);
}

// Start
init();
