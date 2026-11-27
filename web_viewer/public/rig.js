const canvas = document.querySelector('#rig-canvas');
const ctx = canvas.getContext('2d');
const tbody = document.querySelector('#rig-table tbody');
let rows = [], active = 0, yaw = -.76, pitch = .52, zoom = 1, markers = [];
const colors = ['#b84b49','#308459','#426ab2'];
const origin = [0,0,0];
const position = pose => pose.slice(0,3).map(row=>row[3]);

const center = row => row.position || position(row.pose);

function draw() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(w*ratio); canvas.height = Math.round(h*ratio);
  ctx.setTransform(ratio,0,0,ratio,0,0);
  markers = [];
  const posed = rows.filter(r=>r.pose || r.position);
  if (!posed.length) return;
  const radius = Math.max(.35,...posed.map(r=>Math.hypot(...center(r))));
  const scale = Math.min(w,h)*.37*zoom/radius;
  const cy=Math.cos(yaw), sy=Math.sin(yaw), cp=Math.cos(pitch), sp=Math.sin(pitch);
  function project([x,y,z]) {
    const depth=x*cy+y*sy;
    return [w/2+scale*(-x*sy+y*cy),h*.52-scale*(z*cp-depth*sp),depth*cp+z*sp];
  }
  function line(a,b,color,width=1) {
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();
    ctx.moveTo(...project(a).slice(0,2));ctx.lineTo(...project(b).slice(0,2));ctx.stroke();
  }
  for(let i=-4;i<=4;i++) {
    const v=i*radius/4;
    line([-radius,v,0],[radius,v,0],'#dce5df');line([v,-radius,0],[v,radius,0],'#dce5df');
  }
  const [ox,oy] = project(origin);
  for (let axis=0; axis<3; axis++) {
    const tip=[0,0,0]; tip[axis]=radius*.32;
    line(origin,tip,colors[axis],3);
  }
  for (const row of [...posed].sort((a,b)=>project(center(a))[2]-project(center(b))[2])) {
    const m=row.pose,p=center(row),[x,y]=project(p);
    const selected=row===rows[active];
    line(origin,p,'#a0b2aa');
    for(let axis=0;m && axis<3;axis++) {
      const end=p.map((v,k)=>v+m[k][axis]*radius*.22);
      line(p,end,colors[axis],selected?3:1.5);
    }
    ctx.fillStyle=selected?'#00705d':'#344d44';ctx.beginPath();ctx.arc(x,y,selected?6:4,0,2*Math.PI);ctx.fill();
    ctx.font='bold 12px sans-serif';ctx.fillText(row.name,x+7,y-7);
    markers.push({x,y,index:rows.indexOf(row)});
  }
  ctx.strokeStyle='#183d34';ctx.lineWidth=2;ctx.beginPath();ctx.arc(ox,oy,7,0,2*Math.PI);ctx.stroke();
  ctx.font='bold 13px sans-serif';ctx.fillStyle='#183d34';ctx.fillText('base_link',ox+9,oy+18);
}

function select(index) {
  active=index;
  [...tbody.rows].forEach((tr,i)=>tr.classList.toggle('selected',i===active));
  const r=rows[index], m=r?.pose;
  const detail=document.querySelector('#rig-detail');
  if(!r) detail.textContent='';
  else if(!m) detail.textContent=r.name+' · '+(r.error || '等待 bag TF');
  else {
    const angles=[Math.atan2(m[2][1],m[2][2]),Math.asin(Math.max(-1,Math.min(1,-m[2][0]))),Math.atan2(m[1][0],m[0][0])];
    detail.textContent=r.name+' · '+r.frame+' · XYZ (m)  '+position(m).map(v=>v.toFixed(3)).join(' / ')+
      '     · 滚转 / 俯仰 / 偏航 (°)  '+angles.map(v=>(v*180/Math.PI).toFixed(1)).join(' / ');
  }
  draw();
}

export function displayInputs(data,bag) {
  rows=data?.rows || [];tbody.replaceChildren();
  for(const [i,r] of rows.entries()) {
    const tr=tbody.insertRow(), button=document.createElement('button'),topic=document.createElement('small');
    button.textContent=r.name+(r.error?' · '+r.error:'');topic.textContent=r.topic;button.append(topic);
    button.onclick=()=>select(i);tr.insertCell().append(button);
    const frameCell=tr.insertCell(); frameCell.className='frame-id'; frameCell.textContent=r.frame || (bag?'缺失':'待选包');
    tr.insertCell().textContent=bag?(r.count==null?'缺失':r.count.toLocaleString()):'待选包';
  }
  document.querySelector('#rig-files').textContent=data?.source || '选择 bag 后读取其 /tf_static';
  select(0);
}

for(const button of document.querySelectorAll('[data-view]')) button.onclick=()=>{
  [yaw,pitch]=({iso:[-.76,.52],top:[0,Math.PI/2],front:[0,0],side:[Math.PI/2,0]})[button.dataset.view];zoom=1;draw();
};
let drag;
canvas.onpointerdown=e=>{canvas.setPointerCapture(e.pointerId);drag={x:e.clientX,y:e.clientY,yaw,pitch};};
canvas.onpointermove=e=>{if(!drag)return;yaw=drag.yaw+(e.clientX-drag.x)*.012;
  pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,drag.pitch+(e.clientY-drag.y)*.012));draw();};
canvas.onpointerup=e=>{
  if(drag && Math.hypot(e.clientX-drag.x,e.clientY-drag.y)<5){
    const box=canvas.getBoundingClientRect(),x=e.clientX-box.left,y=e.clientY-box.top;
    const hit=markers.map(m=>({...m,d:Math.hypot(m.x-x,m.y-y)})).sort((a,b)=>a.d-b.d)[0];
    if(hit && hit.d<15)select(hit.index);
  }drag=null;
};
canvas.onpointercancel=()=>{drag=null;};
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.min(4,Math.max(.4,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
new ResizeObserver(draw).observe(canvas);
