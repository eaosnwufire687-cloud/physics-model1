// main.js - 完整整合版 (含正常摆动模式)
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import GUI from 'https://cdn.jsdelivr.net/npm/lil-gui@0.19/+esm';

// ================= 1. 参数与物理状态 =================
const params = {
    W_cm: 0.0,           // 驱动速度 (cm/s) 默认设为0，看正常摆动
    R_cm: 8.0,           // 圆盘半径 (cm)
    init_Y_cm: -3.0,     // 初始水平位置 (偏开一点才能摆)
    init_Z_cm: -6.0,     // 初始竖直位置 (负号代表在下方，正号在上方倒立)
    O_offset_cm: 0.0,    // 支点水平偏移
    g_val: 9.81,         // 重力
    friction_scale: 1.0, // 摩擦
    show_trajectory: true,
    preset_pendulum: () => {
        // 一键设置为正常阻尼摆
        params.W_cm = 0.0;
        params.init_Y_cm = -4.0;
        params.init_Z_cm = -6.0;
        gui.controllers.forEach(c => c.updateDisplay());
        resetSimulation();
    },
    reset: () => { resetSimulation(); }
};

let I_C = 0.5 * Math.pow(params.R_cm / 100, 2); 
const k0_base = 0.012, k1_base = 0.103, k2_base = 0.055;

let state = [0.0, params.init_Y_cm / 100, params.init_Z_cm / 100]; 
let beta = 0.0; 
const SCALE = 5; 

// ================= 2. 物理引擎 (RK4) =================
function frictionCoeff(Omega) {
    const k0 = k0_base * params.friction_scale;
    const k1 = k1_base * params.friction_scale;
    const k2 = k2_base * params.friction_scale;
    return Math.max(k1 - k2 * Math.abs(Omega), k0);
}

function derivatives(st, W) {
    const [Omega, Y, Z] = st;
    const k = frictionCoeff(Omega);
    const g = params.g_val; 
    
    const dOmega_dt = (g * Y - k * Omega - W * Z * Omega) / I_C;
    const dY_dt = Z * Omega;
    const dZ_dt = -Y * Omega + W;
    return [dOmega_dt, dY_dt, dZ_dt];
}

function updatePhysics(W_cm, dt) {
    const W = W_cm / 100.0; 
    const subSteps = 10;
    const subDt = dt / subSteps;
    
    for (let i = 0; i < subSteps; i++) {
        const k1 = derivatives(state, W);
        const k2 = derivatives(state.map((v, j) => v + 0.5 * subDt * k1[j]), W);
        const k3 = derivatives(state.map((v, j) => v + 0.5 * subDt * k2[j]), W);
        const k4 = derivatives(state.map((v, j) => v + subDt * k3[j]), W);

        state = state.map((v, j) => 
            v + (subDt / 6) * (k1[j] + 2*k2[j] + 2*k3[j] + k4[j])
        );
    }
    beta += state[0] * dt;
}

// ================= 3. 3D 场景搭建 =================
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a); 

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(40, 30, 60); 

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
scene.add(ambientLight);
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(20, 50, 20);
scene.add(dirLight);

// 参考线和网格
const vertLineGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, -60, 0), new THREE.Vector3(0, 60, 0)]);
const vertLineMat = new THREE.LineBasicMaterial({ color: 0x444444, dashed: true });
scene.add(new THREE.Line(vertLineGeo, vertLineMat));

const gridHelper = new THREE.GridHelper(100, 20, 0x333333, 0x222222);
gridHelper.position.y = -40; // 网格放低一点
scene.add(gridHelper);

// ================= 4. 物理模型元素 =================
const pivotGroup = new THREE.Group();
scene.add(pivotGroup);

const pivotMesh = new THREE.Mesh(new THREE.SphereGeometry(0.5, 16, 16), new THREE.MeshStandardMaterial({ color: 0xffff00, emissive: 0x888800 }));
pivotGroup.add(pivotMesh);

const wheelGeo = new THREE.CylinderGeometry(1.5, 1.5, 2.0, 32);
const wheelMat = new THREE.MeshStandardMaterial({ color: 0x666666, metalness: 0.8, roughness: 0.2 });
const wheel1 = new THREE.Mesh(wheelGeo, wheelMat);
const wheel2 = new THREE.Mesh(wheelGeo, wheelMat);
wheel1.rotation.x = Math.PI / 2; 
wheel2.rotation.x = Math.PI / 2;
wheel1.position.set(0, 0, 1.2);
wheel2.position.set(0, 0, -1.2);
pivotGroup.add(wheel1, wheel2);

const diskGroup = new THREE.Group();
scene.add(diskGroup);

const diskGeo = new THREE.CylinderGeometry(8.0 * SCALE, 8.0 * SCALE, 0.4, 64); 
const diskMat = new THREE.MeshStandardMaterial({ color: 0x3388ff, transparent: true, opacity: 0.3, side: THREE.DoubleSide, depthWrite: false });
const diskMesh = new THREE.Mesh(diskGeo, diskMat);
diskMesh.rotation.x = Math.PI / 2; 
diskGroup.add(diskMesh);

const ringGeo = new THREE.TorusGeometry(8.0 * SCALE, 0.15, 16, 100);
const ringMat = new THREE.MeshBasicMaterial({ color: 0x3388ff });
const ringMesh = new THREE.Mesh(ringGeo, ringMat);
diskMesh.add(ringMesh);

const centerMesh = new THREE.Mesh(new THREE.SphereGeometry(1.2, 16, 16), new THREE.MeshStandardMaterial({ color: 0x00ff00, emissive: 0x00aa00 }));
diskGroup.add(centerMesh);

const markerGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(8.0 * SCALE, 0, 0)]);
const markerLine = new THREE.Line(markerGeo, new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 2 }));
diskMesh.add(markerLine);

const lineOCGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const lineOCMat = new THREE.LineDashedMaterial({ color: 0xffff00, dashSize: 1.5, gapSize: 1.5 });
const lineOC = new THREE.Line(lineOCGeo, lineOCMat);
scene.add(lineOC);

// ================= 5. 动态轨迹 =================
const MAX_POINTS = 20000;
const positions = new Float32Array(MAX_POINTS * 3);
const trajectoryGeo = new THREE.BufferGeometry();
trajectoryGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
const trajectoryMat = new THREE.LineBasicMaterial({ color: 0x00ffff, opacity: 0.5, transparent: true });
const trajectoryLine = new THREE.Line(trajectoryGeo, trajectoryMat);
scene.add(trajectoryLine);

let drawCount = 0;

function addTrajectoryPoint() {
    if (drawCount >= MAX_POINTS) return; 
    const abs_y_cm = params.O_offset_cm + state[1] * 100; 
    const z_cm = state[2] * 100;
    const omega = state[0];

    positions[drawCount * 3] = abs_y_cm * SCALE;    
    positions[drawCount * 3 + 1] = z_cm * SCALE; 
    positions[drawCount * 3 + 2] = omega * 2; 
    
    drawCount++;
    trajectoryGeo.setDrawRange(0, drawCount);
    trajectoryGeo.attributes.position.needsUpdate = true;
}

// ================= 6. UI 控制面板 =================
const gui = new GUI({ title: '真实情景物理引擎' });
gui.add(params, 'preset_pendulum').name('🌟 一键看正常单摆');

const folderPhysics = gui.addFolder('物理参数与初始条件');
folderPhysics.add(params, 'W_cm', 0, 10, 0.1).name('驱动速度 W (cm/s)').onChange(() => resetSimulation());
folderPhysics.add(params, 'init_Y_cm', -6.0, 6.0, 0.1).name('初始水平偏移 Y (cm)').onChange(() => resetSimulation());
folderPhysics.add(params, 'init_Z_cm', -8.0, 8.0, 0.5).name('初始竖直位置 Z (cm)').onChange(() => resetSimulation());
folderPhysics.add(params, 'O_offset_cm', -10.0, 10.0, 0.5).name('夹轮(支点)位置 (cm)').onChange(() => resetSimulation());

const folderMore = gui.addFolder('高级参数');
folderMore.add(params, 'R_cm', 4.0, 12.0, 0.5).name('圆盘半径 R (cm)').onChange((val) => {
    I_C = 0.5 * Math.pow(val / 100, 2);
    const scale = val / 8.0;
    diskMesh.scale.set(scale, scale, scale);
    resetSimulation();
});
folderMore.add(params, 'g_val', 0.1, 20.0, 0.1).name('重力 g (m/s²)').onChange(() => resetSimulation());
folderMore.add(params, 'friction_scale', 0.1, 3.0, 0.1).name('摩擦系数比例').onChange(() => resetSimulation());

const folderView = gui.addFolder('视图选项');
folderView.add(params, 'show_trajectory').name('显示相空间轨迹').onChange((val) => { trajectoryLine.visible = val; });
folderView.add(params, 'reset').name('重置仿真');

function resetSimulation() {
    state = [0.0, params.init_Y_cm / 100, params.init_Z_cm / 100]; 
    beta = 0.0;
    drawCount = 0;
    trajectoryGeo.setDrawRange(0, 0);
    updateVisuals(); 
}

// ================= 7. 动画循环 =================
const clock = new THREE.Clock();

function updateVisuals() {
    const offset3D = params.O_offset_cm * SCALE;
    const absX = offset3D + state[1] * 100 * SCALE;
    const absY = state[2] * 100 * SCALE;

    pivotGroup.position.set(offset3D, 0, 0);
    diskGroup.position.set(absX, absY, 0);
    diskGroup.rotation.z = -beta; 

    const positionsOC = lineOC.geometry.attributes.position.array;
    positionsOC[0] = offset3D; 
    positionsOC[1] = 0; 
    positionsOC[2] = 0;
    positionsOC[3] = absX; 
    positionsOC[4] = absY; 
    positionsOC[5] = 0;
    lineOC.geometry.attributes.position.needsUpdate = true;
    lineOC.computeLineDistances(); 
}

function animate() {
    requestAnimationFrame(animate);
    const dt = Math.min(clock.getDelta(), 0.05); 
    
    updatePhysics(params.W_cm, dt);
    if (params.show_trajectory) addTrajectoryPoint();
    updateVisuals();
    
    controls.update();
    renderer.render(scene, camera);
}

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// 启动
resetSimulation();
animate();