import {displayInputs} from './rig.js';
let inputsReady = false;
const el = id => document.getElementById(id);
let selected = '', parent = '', running = true, pending = false, version = 0;
let activeJob = null, exporting = false, exportJob = null;
function canExport(job) {
  return !!(job?.map || (job?.state === 'running' && job.frames >= 10));
}
const update = () => {
  el('save-map').disabled = pending || exporting || activeJob?.saving || !canExport(activeJob);
  el('stop').disabled = exporting || pending || !activeJob?.canStop; el('start').disabled = !selected || !inputsReady || pending || running || exporting || activeJob?.saving; };

async function api(url, options) {
  const response = await fetch(url, options);
  const value = await response.json();
  if (!response.ok) throw Error(value.error || response.statusText);
  return value;
}

async function browse(dir) {
  const sequence = ++version;
  inputsReady = false;
  selected = '';
  displayInputs(null,false);
  el('rig-context').textContent='正在读取bag TF…';
  update();
  el('selection').textContent = '';
  el('folders').replaceChildren();
  el('error').textContent = '';
  try {
    const data = await api('/api/mapping/folders?dir='+encodeURIComponent(dir));
    if (version !== sequence) return;
    parent = data.parent;
    el('dir').value = data.dir;
    el('up').disabled = data.parent === data.dir;
    selected = data.bag ? data.dir : '';
    el('selection').textContent = data.bag ? '已选择 bag：'+data.dir : '请选择 bag 所在的文件夹';
    for (const name of data.folders) {
      const button = document.createElement('button');
      button.textContent = name + ' /';
      button.onclick = () => browse(data.dir + '/' + name);
      el('folders').append(button);
    }
    const inputs = await api('/api/mapping/inputs'+(selected?'?dir='+encodeURIComponent(selected):''));
    if (version !== sequence) return;
    displayInputs(inputs,selected);
    inputsReady = Boolean(selected && inputs.filesOk && inputs.rows.every(row=>row.count > 0 && row.pose));
    el('rig-context').textContent = selected ?
      selected + (inputsReady?' · 6 路输入与 TF 已就绪':(inputs.filesOk?' · 输入或 TF 不完整，见下表':' · 缺少 bag 数据文件')) :
      '选择 bag 后读取六路输入的 frame_id 和录包 TF';
    update();
  } catch (error) { if (sequence === version) {el('error').textContent = error.message; el('rig-context').textContent='未能读取所选目录或 TF';} }
}

let editingRate=false, applyingRate=false;
function setRateDisplay(rate) {
  el('rate').value=String(rate);
  el('rate-value').textContent=rate+'×';
}
el('rate').oninput=()=>{editingRate=true;setRateDisplay(Number(el('rate').value));};
el('rate').onpointercancel=()=>{editingRate=false;};
el('rate').onchange=async()=>{
  editingRate=false;
  if (activeJob?.state !== 'running') return;
  const job=activeJob, rate=Number(el('rate').value);
  applyingRate=true; el('rate').disabled=true; el('error').textContent='';
  try {
    await api('/api/mapping',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action:'rate',runDir:job.runDir,rate})});
    if (activeJob?.runDir===job.runDir) activeJob.rate=rate;
  } catch(error) {setRateDisplay(job.rate ?? 1);el('error').textContent=error.message;}
  finally {applyingRate=false;el('rate').disabled=activeJob?.state==='stopping';}
};
function show(job) {
  if(job.state==='running' && !editingRate && !applyingRate) setRateDisplay(job.rate ?? 1);
  el('rate').disabled=applyingRate || job.state==='stopping';
  activeJob = job;
  running = ['running','stopping'].includes(job.state);
  const progress = job.frames == null ? '等待里程计数据…' : '已收到 '+job.frames+' 帧里程计数据';
  const detail = job.state === 'stopping' ? '正在停止回放并保存当前地图…' : running ? progress : job.message;
  el('status').textContent = job.state === 'idle' ? '尚未开始建图' :
    [detail, job.rvizStatus,
      job.bag && '输入 bag：'+job.bag,
      job.runDir && '输出文件夹：'+job.runDir,
      job.map && '地图文件：'+job.map].filter(Boolean).join('\n');
  update();
}

async function poll() {
  try { if (!pending) {const job = await api('/api/mapping'); if (!pending) show(job);} }
  catch { running = true; activeJob = null; update(); el('status').textContent = '无法连接建图服务，正在重试…'; }
  setTimeout(poll,2000);
}

el('dir').oninput = () => {version++; selected=''; inputsReady=false; displayInputs(null,false); el('rig-context').textContent='打开目录以读取 TF'; el('selection').textContent=''; update();};
el('browse').onsubmit = event => {event.preventDefault(); browse(el('dir').value.trim());};
el('up').onclick = () => browse(parent);
el('start').onclick = async () => {
  pending = true; update(); el('error').textContent = '';
  try {
    show(await api('/api/mapping',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({bag:selected,rviz:el('rviz').checked,rate:Number(el('rate').value)})}));
  } catch (error) {el('error').textContent=error.message;}
  finally {pending=false; update();}
};
el('stop').onclick = async () => {
  if (!activeJob?.canStop || pending) return;
  pending = true; update(); el('error').textContent = '';
  try {
    const job = await api('/api/mapping',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action:'stop',runDir:activeJob.runDir})});
    show(job);
  } catch(error) {el('error').textContent=error.message;}
  finally {pending=false; update();}
};
function exportPreview() {
  el('export-files').textContent = el('export-format').value === 'autoware'
    ? 'pointcloud_map.pcd · map_projector_info.yaml · pointcloud_map_metadata.yaml'
    : 'pointcloud_map.pcd（SLAM 本地坐标）';
  el('export-dir').textContent = '保存位置：'+exportJob.runDir+'/exports/'+el('export-name').value;
}
el('save-map').onclick = () => {
  exportJob = activeJob;
  const stamp = new Intl.DateTimeFormat('sv-SE',{timeZone:'Asia/Shanghai',dateStyle:'short',timeStyle:'medium'}).format(new Date());
  el('export-name').value='map_'+stamp.replace(/[-:]/g,'').replace(' ','_');
  el('export-status').textContent=''; exportPreview(); el('export-dialog').showModal();
};
el('export-name').oninput=exportPreview;
el('export-format').onchange=exportPreview;
el('export-close').onclick=()=>el('export-dialog').close();
el('export-form').onsubmit=async event=>{
  event.preventDefault();
  if(exporting) return;
  exporting=true; update(); el('export-submit').disabled=true;
  el('export-status').textContent='正在写入地图…';
  try {
    const result=await api('/api/mapping',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action:'export',runDir:exportJob.runDir,name:el('export-name').value.trim(),format:el('export-format').value})});
    el('export-status').textContent='已保存到 '+result.dir;
  } catch(error) {el('export-status').textContent=error.message;}
  finally {exporting=false; el('export-submit').disabled=false; update();}
};
async function initialize() {
  let dir='';
  try {const job=await api('/api/mapping'); show(job); dir=job.bag || '';} catch {}
  if (!version) browse(dir);
  poll();
}
initialize();
