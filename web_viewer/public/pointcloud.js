import * as THREE from './vendor/three.module.js';
import {OrbitControls} from './vendor/OrbitControls.js';
import {PCDLoader} from './vendor/PCDLoader.js';

const el = id => document.getElementById(id);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xe9ebed);
const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 10000);
camera.up.set(0, 0, 1);
const center = new THREE.Vector3();
let renderer, controls, cloud, grid;
let radius = 10, generation = 0;

// 标尺位于相机焦点深度，每米的像素数随透视缩放实时更新。
function draw() {
    const distance = camera.position.distanceTo(controls.target);
    const pixels = el('scene').clientHeight / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * distance);
    el('scale-line').style.width = pixels + 'px';
    renderer.render(scene, camera);
}

// 视角始终以点云包围球为中心，竖直视角留出极小偏移以确定朝北方向。
function setView(view) {
    const offset = {orbit:[1,-1,0.8], top:[0,-0.00001,1], front:[1,0,0], side:[0,-1,0]}[view];
    const halfFov = Math.atan(Math.tan(THREE.MathUtils.degToRad(22.5)) * Math.min(1, camera.aspect));
    const distance = radius * 1.1 / Math.sin(halfFov);
    camera.position.fromArray(offset).normalize().multiplyScalar(distance).add(center);
    controls.target.copy(center);
    controls.update();
    document.querySelectorAll('[data-view]').forEach(button => button.classList.toggle('active', button.dataset.view === view));
    draw();
}

// 清除上一幅点云及其 GPU 缓冲。
function discard(object) {
    if (!object) return;
    scene.remove(object);
    object.geometry.dispose();
    object.material.dispose();
}

// 网格使用米制原坐标；对大坐标点云仅平移视图，无任何缩放。
function display(points, label, total) {
    const geometry = points.geometry;
    const pos = geometry.getAttribute('position');
    if (!pos?.count) throw Error('PCD 没有 XYZ 点');
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
    if (!Number.isFinite(geometry.boundingSphere.radius)) throw Error('点云坐标含无效值');
    discard(cloud);
    discard(grid);
    cloud = points;
    center.copy(geometry.boundingSphere.center);
    radius = Math.max(1, geometry.boundingSphere.radius);
    const size = Math.min(2048, Math.max(20, 2 * Math.ceil(radius)));
    grid = new THREE.GridHelper(size, size, 0x9a9fa4, 0xc8cdd1);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(Math.round(center.x), Math.round(center.y), Math.floor(geometry.boundingBox.min.z));
    scene.add(grid, cloud);
    points.material.size = Number(el('point-size').value);
    points.material.sizeAttenuation = false;
    if (!points.material.vertexColors) points.material.color.set(0x39576a);
    camera.far = Math.max(10000, radius * 100);
    camera.updateProjectionMatrix();
    el('dataset-label').textContent = label;
    el('point-count').textContent = total > pos.count ? '预览 ' + pos.count.toLocaleString() + ' / ' + total.toLocaleString() + ' 点' : pos.count.toLocaleString() + ' 点';
    el('loading').hidden = true;
    setView('orbit');
}

// 读取地图的抽样 XYZI 数据；独立 PCD 文件走官方加载器。
async function load(file) {
    const token = ++generation;
    el('loading').hidden = false;
    el('loading').textContent = '正在读取点云…';
    try {
        let points, label, total;
        if (file) {
            const data = await file.arrayBuffer();
            if (token !== generation) return;
            points = new PCDLoader().parse(data);
            label = file.name;
        } else {
            const dataset = new URLSearchParams(location.search).get('dataset');
            if (!dataset) throw Error('请选择地图或打开 PCD 文件');
            const response = await fetch('/api/datasets/' + encodeURIComponent(dataset) + '/pointcloud');
            if (!response.ok) throw Error('地图点云读取失败');
            const raw = new Float32Array(await response.arrayBuffer());
            if (token !== generation) return;
            if (raw.length % 4) throw Error('点云长度无效');
            const xyz = new Float32Array(raw.length / 4 * 3);
            for (let i = 0, j = 0; i < raw.length; i += 4) {
                xyz[j++] = raw[i]; xyz[j++] = raw[i+1]; xyz[j++] = raw[i+2];
            }
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(xyz, 3));
            points = new THREE.Points(geometry, new THREE.PointsMaterial());
            total = Number(response.headers.get('X-Source-Points'));
            label = '地图点云';
        }
        display(points, label, total);
    } catch (error) {
        if (generation === token) el('loading').textContent = error.message;
    }
}

// 按容器实际尺寸更新透视投影，切换工作区后保留相机姿态。
function resize() {
    const {clientWidth:w, clientHeight:h} = el('scene-wrap');
    if (!w || !h) return;
    renderer.setSize(w,h,false);
    camera.aspect = w/h;
    camera.updateProjectionMatrix();
    draw();
}

try {
    renderer = new THREE.WebGLRenderer({canvas:el('scene'), antialias:true});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    controls = new OrbitControls(camera, renderer.domElement);
    controls.mouseButtons = {LEFT:THREE.MOUSE.ROTATE, MIDDLE:THREE.MOUSE.PAN, RIGHT:THREE.MOUSE.DOLLY};
    controls.minDistance = 0.2;
    controls.addEventListener('change', draw);
    scene.add(new THREE.AxesHelper(5));
    setView('orbit');
    resize();
    new ResizeObserver(resize).observe(el('scene-wrap'));
    document.querySelectorAll('[data-view]').forEach(button => button.onclick = () => setView(button.dataset.view));
    el('reset-view').onclick = () => setView('orbit');
    el('file').onchange = event => { if (event.target.files[0]) load(event.target.files[0]); };
    el('point-size').oninput = event => {
        el('point-size-value').textContent = event.target.value + ' px';
        if (cloud) cloud.material.size = Number(event.target.value);
        draw();
    };
    load();
} catch (error) {
    el('loading').textContent = '3D 视图初始化失败：' + error.message;
}
