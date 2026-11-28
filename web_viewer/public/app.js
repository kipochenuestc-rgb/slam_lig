const $ = id => document.getElementById(id);
const canvas = $('cloud-canvas'), ctx = canvas.getContext('2d');
const stage = $('map-stage'), raster = document.createElement('canvas');
const state = { meta: null, points: null, colors: null, tracks: null, mode: 'local', map: null, layer: null, bounds: null, cx: 0, cy: 0, scale: 1, width: 0, height: 0, visible: 0, drag: null };
const controls = ['cloud-toggle', 'path-toggle', 'gnss-toggle', 'opacity', 'point-size', 'z-min', 'z-max'];
// The localization marker is separate from the cloud raster so scrubbing never repaints the cloud.
let liveXYZ = null, liveDot = null, satelliteDot = null;
if (new URLSearchParams(location.search).has('localization')) {
  document.body.classList.add('localization-map');
  liveDot = document.createElement('span');
  liveDot.className = 'localization-dot';
  liveDot.hidden = true;
  stage.appendChild(liveDot);
  window.addEventListener('message', e => {
    if (e.origin !== location.origin || e.source !== parent || e.data?.type !== 'localization-position') return;
    liveXYZ = e.data.xyz;
    updatePosition();
  });
  parent.postMessage({type:'localization-ready'}, location.origin);
}
function updatePosition() {
  if (!liveDot) return;
  liveDot.hidden = !liveXYZ || !state.meta || state.mode !== 'local';
  if (!liveDot.hidden) {
    const [x,y] = worldToScreen(...liveXYZ);
    liveDot.style.left = x+'px'; liveDot.style.top = y+'px';
  }
  if (!state.map) return;
  if (!liveXYZ || state.mode !== 'amap') { satelliteDot?.hide(); return; }
  // Use the same bounds as the projected point cloud for the overlay marker.
  const [west,south,east,north] = state.meta.boundsENU;
  const sw = state.bounds.getSouthWest(), ne = state.bounds.getNorthEast();
  const lng = sw.lng+(ne.lng-sw.lng)*(liveXYZ[0]-west)/(east-west);
  const lat = sw.lat+(ne.lat-sw.lat)*(liveXYZ[1]-south)/(north-south);
  if (!satelliteDot) satelliteDot = new AMap.CircleMarker({map:state.map,center:[lng,lat],radius:7,
      fillColor:'#e34c32',fillOpacity:1,strokeColor:'#fff',strokeWeight:2,zIndex:200});
  satelliteDot.setCenter([lng,lat]); satelliteDot.show();
}
function reportView(status, details={}) { fetch('/api/view-state',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({status,...details})}).catch(()=>{}); }
function toast(message) { $('toast').textContent = message; $('toast').classList.add('visible'); clearTimeout(toast.timer); toast.timer = setTimeout(() => $('toast').classList.remove('visible'), 3800); }
function color(z) { const [lo, hi] = state.meta.zColorRange; const t = Math.max(0, Math.min(1, (z - lo) / (hi - lo || 1))); const a = t < .5 ? [75,154,181] : [117,195,168], b = t < .5 ? [117,195,168] : [229,215,138]; const f = t < .5 ? t * 2 : (t - .5) * 2; return `rgb(${a.map((v,i) => Math.round(v + (b[i] - v) * f)).join(',')})`; }
function worldToScreen(x, y) { return [state.width / 2 + (x - state.cx) * state.scale, state.height / 2 - (y - state.cy) * state.scale]; }
function screenToWorld(x, y) { return [state.cx + (x - state.width / 2) / state.scale, state.cy - (y - state.height / 2) / state.scale]; }
function drawGrid() {
  const step = 10 ** Math.floor(Math.log10(90 / state.scale)); const spacing = step * state.scale;
  ctx.strokeStyle = '#b9c0c6'; ctx.lineWidth = .5; ctx.beginPath();
  const [sx, sy] = worldToScreen(0, 0);
  for (let x = ((sx % spacing) + spacing) % spacing; x < state.width; x += spacing) { ctx.moveTo(x,0); ctx.lineTo(x,state.height); }
  for (let y = ((sy % spacing) + spacing) % spacing; y < state.height; y += spacing) { ctx.moveTo(0,y); ctx.lineTo(state.width,y); } ctx.stroke();
}
function pathOn(context, rows, project, lineColor, dashed = false) {
  if (!rows?.length) return; context.globalAlpha = 1; context.strokeStyle = lineColor; context.lineWidth = 2; context.lineJoin = 'round'; context.setLineDash(dashed ? [5, 4] : []); context.beginPath();
  rows.forEach((row,i) => { const [x,y] = project(row[0], row[1]); if(i) context.lineTo(x,y); else context.moveTo(x,y); }); context.stroke(); context.setLineDash([]);
}
function paint(context, project, size) {
  const { points } = state; let count = 0; const low = Number($('z-min').value), high = Number($('z-max').value);
  context.globalAlpha = Number($('opacity').value) / 100;
  if ($('cloud-toggle').checked) for (let i = 0; i < points.length; i += 4) {
    const z = points[i+2]; if (z < low || z > high) continue; count++;
    const [x,y] = project(points[i], points[i+1]); if (x < -5 || y < -5 || x > context.canvas.width + 5 || y > context.canvas.height + 5) continue;
    const c = (i / 4) * 3;
    context.fillStyle = state.colors ? 'rgb(' + state.colors[c] + ',' + state.colors[c+1] + ',' + state.colors[c+2] + ')' : color(z); context.fillRect(x-size/2,y-size/2,size,size);
  }
  context.globalAlpha = 1;
  if ($('gnss-toggle').checked) pathOn(context, state.tracks.gnss, project, '#9aaec4', true);
  if ($('path-toggle').checked) {
    pathOn(context, state.tracks.fused, project, '#eb9468');
    const start = state.tracks.fused[0]; if(start) { const [x,y] = project(start[0],start[1]); context.fillStyle='#e4edb8';context.strokeStyle='#1c4739';context.lineWidth=2;context.beginPath();context.arc(x,y,4.5,0,Math.PI*2);context.fill();context.stroke(); }
  }
  state.visible = count; $('point-count').textContent = count.toLocaleString('zh-CN');
}
function draw() {
  updatePosition();
  if (!state.meta) return;
  $('opacity-value').textContent = `${$('opacity').value}%`; $('size-value').textContent = `${$('point-size').value} px`;
  if (Number($('z-min').value) > Number($('z-max').value)) { toast('最低高度不能高于最高高度'); return; }
  if (state.mode === 'amap') { renderRaster(); return; }
  const dpr = window.devicePixelRatio || 1; ctx.setTransform(dpr,0,0,dpr,0,0); ctx.fillStyle='#eceef0'; ctx.fillRect(0,0,state.width,state.height);drawGrid();paint(ctx, worldToScreen, Number($('point-size').value));
  const meters = 10 ** Math.floor(Math.log10(75 / state.scale)); $('scale').querySelector('i').style.width=`${meters*state.scale}px`; $('scale').querySelector('span').textContent=`${meters} m`;
}
function renderRaster() {
  const b = state.meta.boundsENU, w = b[2]-b[0], h = b[3]-b[1]; raster.width = Math.min(2600, Math.max(800, Math.round(w/.15))); raster.height = Math.min(2600, Math.max(800, Math.round(raster.width*h/w)));
  const rc = raster.getContext('2d');rc.clearRect(0,0,raster.width,raster.height);paint(rc,(x,y)=>[(x-b[0])/w*raster.width,(b[3]-y)/h*raster.height],Number($('point-size').value)*1.4);if(state.layer){state.layer.setCanvas(raster);state.layer.reFresh();}
}
function resize() { const r = stage.getBoundingClientRect();state.width=r.width;state.height=r.height;const dpr=window.devicePixelRatio||1;canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);draw(); }
function fit() {
  if(!state.meta)return;
  if(state.mode==='amap' && state.map && state.bounds){state.map.setBounds(state.bounds,false,[35,35,35,35]);return;}
  const b=state.meta.boundsENU;state.cx=(b[0]+b[2])/2;state.cy=(b[1]+b[3])/2;state.scale=Math.min((state.width-70)/(b[2]-b[0]),(state.height-70)/(b[3]-b[1]));draw();
}
function zoom(factor, x=state.width/2,y=state.height/2) {if(state.mode==='amap'){state.map.setZoom(state.map.getZoom()+(factor>1?1:-1));return;}const p=screenToWorld(x,y);state.scale=Math.max(.2,Math.min(80,state.scale*factor));state.cx=p[0]-(x-state.width/2)/state.scale;state.cy=p[1]+(y-state.height/2)/state.scale;draw();}
canvas.addEventListener('wheel', e=>{e.preventDefault();const r=canvas.getBoundingClientRect();zoom(e.deltaY<0?1.15:1/1.15,e.clientX-r.left,e.clientY-r.top);},{passive:false});
canvas.addEventListener('pointerdown',e=>{state.drag={x:e.clientX,y:e.clientY,cx:state.cx,cy:state.cy};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{const r=canvas.getBoundingClientRect();if(state.drag){state.cx=state.drag.cx-(e.clientX-state.drag.x)/state.scale;state.cy=state.drag.cy+(e.clientY-state.drag.y)/state.scale;draw();}const p=screenToWorld(e.clientX-r.left,e.clientY-r.top);$('coordinates').textContent=`E ${p[0].toFixed(1)} m　 N ${p[1].toFixed(1)} m`;});
canvas.addEventListener('pointerup',()=>state.drag=null);canvas.addEventListener('pointercancel',()=>state.drag=null);canvas.addEventListener('dblclick',()=>zoom(1.5));
controls.forEach(id=>$(id).addEventListener('input',draw));$('zoom-in').onclick=()=>zoom(1.4);$('zoom-out').onclick=()=>zoom(1/1.4);$('fit').onclick=fit;$('home-view').onclick=fit;
$('reset-filter').onclick=()=>{if(!state.meta)return;$('z-min').value=Math.floor(state.meta.zRange[0]);$('z-max').value=Math.ceil(state.meta.zRange[1]);$('opacity').value=85;$('point-size').value=1.5;draw();};
function showDialog(){if(!state.meta){toast('请等待点云数据加载');return;}$('connect-dialog').showModal();}
$('connect-side').onclick=showDialog;$('close-dialog').onclick=()=>$('connect-dialog').close();
function mode(m){state.mode=m;$('local-mode').classList.toggle('selected',m==='local');$('amap-mode').classList.toggle('selected',m==='amap');canvas.style.display=m==='local'?'block':'none';$('amap').style.display=m==='amap'?'block':'none';$('view-status').textContent=m==='amap'?'高德卫星影像已连接':'本地坐标视图';$('coord-label').textContent=m==='amap'?'高德 GCJ‑02 · 卫星影像 + 点云':'本地 ENU · 单位：米';$('scale').style.display=m==='amap'?'none':'block';$('coordinates').textContent=m==='amap'?'卫星影像 © 高德地图 / AutoNavi':'拖动平移 · 滚轮缩放';draw();if(m==='amap')state.map.resize();}
$('local-mode').onclick=()=>mode('local');$('amap-mode').onclick=()=>state.map?mode('amap'):showDialog();
async function loadSDK(key,code,serviceHost){if(window.AMap)return window.AMap;window._AMapSecurityConfig=serviceHost?{serviceHost}:{securityJsCode:code};return new Promise((resolve,reject)=>{const s=document.createElement('script');const q=new URLSearchParams({v:'2.0',key,callback:'__fieldAtlasMapReady'});window.__fieldAtlasMapReady=()=>{clearTimeout(timer);resolve(window.AMap);};s.src=`https://webapi.amap.com/maps?${q}`;s.onerror=()=>reject(new Error('高德脚本加载失败，请检查网络和 Key。'));const timer=setTimeout(()=>reject(new Error('高德加载超时，请检查 Key、域名白名单和网络。')),20000);document.head.appendChild(s);});}
async function connectMap(key,code,serviceHost){
  const btn=$('connect-button');btn.disabled=true;btn.textContent='正在连接…';$('map-error').textContent='';
  try{
    reportView('sdk-loading');await loadSDK(key,code,serviceHost);reportView('sdk-loaded');const b=state.meta.boundsWGS84;
    const loc=await new Promise((resolve,reject)=>window.AMap.convertFrom([[b[0],b[1]],[b[2],b[3]]],'gps',(status,result)=>status==='complete'&&result.locations?.length===2?resolve(result.locations):reject(new Error(`坐标转换失败：${result.info||status}。请核对 Key、密钥和域名设置。`))));
    if(!serviceHost){sessionStorage.setItem('atlas-amap-key',key);sessionStorage.setItem('atlas-amap-code',code);}
    state.bounds=new AMap.Bounds(loc[0],loc[1]);$('amap').style.display='block';state.map=new AMap.Map('amap',{viewMode:'2D',zoom:18,center:state.bounds.getCenter(),layers:[new AMap.TileLayer.Satellite(),new AMap.TileLayer.RoadNet()],showLabel:true});
    state.map.on('complete',()=>reportView('satellite-map-complete',{pointCount:state.meta.pointCount,gnssFactors:state.meta.gnssFactors,datasetId:state.meta.datasetId}));
    renderRaster();state.layer=new AMap.CanvasLayer({canvas:raster,bounds:state.bounds,zooms:[3,22],opacity:1,zIndex:120});state.map.add(state.layer);mode('amap');fit();$('connect-dialog').close();toast('高德卫星影像已连接，点云已叠加');reportView('satellite-overlay-ready');
  }catch(error){$('map-error').textContent=error.message;mode('local');toast(error.message);reportView('map-error',{message:error.message});}finally{btn.disabled=false;btn.textContent='连接高德卫星地图 →';}
}
$('connect-form').onsubmit=async e=>{e.preventDefault();const key=$('amap-key').value.trim(),code=$('amap-code').value.trim();if(key&&code)await connectMap(key,code);};
$('download').onclick=()=>{if(!state.meta){toast('数据还未加载完成');return;}const a=document.createElement('a');a.href='./data/pointcloud_2d.geojson';a.download='pointcloud_2d_wgs84.geojson';a.click();toast('正在导出 WGS‑84 二维点云数据');};
async function start(){
  try{const [mr,pr,tr]=await Promise.all([fetch('./data/metadata.json'),fetch('./data/points.bin'),fetch('./data/tracks.json')]);if(!mr.ok||!pr.ok||!tr.ok)throw Error('数据文件读取失败');state.meta=await mr.json();state.points=new Float32Array(await pr.arrayBuffer());state.tracks=await tr.json();if(state.points.length!==state.meta.pointCount*4)throw Error('点云文件长度校验失败');
    if(state.meta.colorFile){
      const response=await fetch('./data/'+state.meta.colorFile);
      if(!response.ok)throw Error('RGB颜色文件读取失败');
      state.colors=new Uint8Array(await response.arrayBuffer());
      if(state.colors.length!==state.meta.pointCount*3)throw Error('RGB颜色长度校验失败');
      document.querySelector('.height-legend').style.display='none';
      document.querySelector('.cloud-swatch').parentElement.querySelector('small').textContent='原始相机 RGB';
    }
    $('dataset-state').textContent=state.meta.isPreview?'预览 · GNSS 约束构建中':(state.meta.datasetLabel||'GNSS 约束建图已完成');$('distance').textContent=`${state.meta.distanceM.toFixed(0)} m`;$('duration').textContent=`${Math.floor(state.meta.durationS/60)} 分 ${Math.round(state.meta.durationS%60)} 秒`;$('factor-count').textContent=state.meta.gnssFactors?`${state.meta.gnssFactors} 个`:'等待结果';$('z-min').value=Math.floor(state.meta.zRange[0]);$('z-max').value=Math.ceil(state.meta.zRange[1]);$('legend-low').textContent=`${state.meta.zColorRange[0].toFixed(1)}`;$('legend-high').textContent=`${state.meta.zColorRange[1].toFixed(1)}`;$('quality-note').textContent=state.meta.qualityNote;$('pipeline-label').textContent=state.meta.datasetLabel||'GNSS 约束建图';$('pipeline-detail').textContent=state.meta.fusedFrames?`${state.meta.fusedFrames.toLocaleString('zh-CN')} 帧 · 滤波前 ${(state.meta.inputPoints/1e8).toFixed(2)} 亿点 · 滤波后 ${(state.meta.filteredPoints/1e8).toFixed(2)} 亿点`:'';$('filter-detail').textContent=state.meta.filterDescription||'';$('empty-state').style.display='none';resize();fit();$('amap-key').value=sessionStorage.getItem('atlas-amap-key')||'';$('amap-code').value=sessionStorage.getItem('atlas-amap-code')||'';
    try{if(new URLSearchParams(location.search).get('view')==='local')return;const response=await fetch('/api/amap-config');if(response.ok){const config=await response.json();if(!config.enabled && liveDot) toast('未配置高德地图，当前使用本地点云');if(config.enabled){$('view-status').textContent='正在连接高德地图…';await connectMap(config.key,null,config.serviceHost);}}}catch{/* Local mode remains usable without configured map credentials. */}
  }catch(error){$('empty-state').textContent=`加载失败：${error.message}`;$('dataset-state').textContent='数据未加载';}
}
new ResizeObserver(resize).observe(stage);start();
