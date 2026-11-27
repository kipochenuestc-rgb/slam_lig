import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {readFile, readdir, stat, writeFile, link, unlink} from 'node:fs/promises';
import {promisify} from 'node:util';
import {spawn, execFile} from 'node:child_process';
import {randomUUID} from 'node:crypto';

const project = fileURLToPath(new URL('../', import.meta.url));
const windows = process.platform === 'win32';
const lastJobFile = path.join(project,'runs','.web_last_job.json');
let current = {state:'idle'};
let saving = false;
let settingRate = false;
try {
  const last = JSON.parse(await readFile(lastJobFile,'utf8'));
  if (['done','failed','stopped'].includes(last.state)) current = last;
} catch {}

function linuxPath(value) {
  if (!windows) return value;
  if (!/^[a-z]:[\\/]/i.test(value)) throw Error('请选择本地磁盘上的目录');
  return '/mnt/' + value[0].toLowerCase() + value.slice(2).replaceAll('\\', '/');
}

async function isFile(file) {
  try { return (await stat(file)).isFile(); } catch { return false; }
}

async function browse(folder) {
  const dir = path.resolve(folder || path.dirname(project));
  const entries = await readdir(dir, {withFileTypes:true});
  const bag = await isFile(path.join(dir,'metadata.yaml'));
  const folders = bag ? [] : entries.filter(e => e.isDirectory() && !e.name.startsWith('.'))
    .map(e => e.name).sort((a,b) => a.localeCompare(b));
  return {dir, parent:path.dirname(dir), folders, bag};
}

async function stopJob(job) {
  if (job.state === 'stopping' || job.state === 'stopped') return;
  if (job.state !== 'running') throw Error('当前任务已结束');
  const request = path.join(job.runDir,'stop_request.json');
  const temp = request + '.' + randomUUID();
  await writeFile(temp, '{}', {flag:'wx'});
  try {
    await link(temp,request);
  } catch(error) {
    if (error.code !== 'EEXIST') throw error;
  } finally {
    await unlink(temp);
  }
  if (job.state === 'running') job.state = 'stopping';
}

function launch(bag, rviz, rate) {
  const run = 'web_' + randomUUID();
  const runDir = path.join(project,'runs',run);
  const args = [linuxPath(path.join(project,'scripts/ros_env.sh')), 'python3', '-u',
    linuxPath(path.join(project,'scripts/web_mapping.py')), linuxPath(bag), run, '--rate',String(rate)];
  if (rviz) args.push('--rviz');
  const job = {state:'running', bag, rviz, rate, runDir, message:'正在校验 bag 并启动 ROS 节点…'};
  const child = windows
    ? spawn('wsl.exe',['-d','Ubuntu-24.04','--exec','bash',...args],{windowsHide:true})
    : spawn('bash',args);
  current = job;
  let tail = '';
  for (const stream of [child.stdout,child.stderr]) {
    stream.setEncoding('utf8');
    stream.on('data',chunk => {tail = (tail + chunk).slice(-3000);});
  }
  child.on('error',error => {job.state='failed'; job.message=error.message;});
  child.on('close',async code => {
    if (job.state === 'failed') return;
    const map = path.join(runDir,'map','GlobalMap.pcd');
    let interrupted = false;
    try {interrupted = JSON.parse(await readFile(path.join(runDir,'run_status.json'),'utf8')).interrupted;} catch {}
    if (code === 0 && interrupted) {
      const saved = await isFile(map);
      Object.assign(job,{state:'stopped', message:saved?'已停止，当前地图已保存':'已停止，有效里程计不足，未生成地图'});
      if (saved) job.map = map;
    } else if (code === 0 && await isFile(map)) {
      Object.assign(job,{state:'done',message:'建图完成',map});
    } else {
      Object.assign(job,{state:'failed',message:tail.trim() || '建图失败，请查看运行目录的日志。'});
    }
    await writeFile(lastJobFile,JSON.stringify(job)).catch(error=>console.error(error.message));
  });
}

export async function mappingApi(req,res,url,port) {
  function reply(status,value) {
    res.writeHead(status,{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'}).end(JSON.stringify(value));
  }
  const hosts = ['localhost:'+port,'127.0.0.1:'+port];
  if (!hosts.includes(req.headers.host)) return reply(403,{error:'仅支持本机访问'});
  try {
    if (url.pathname === '/api/mapping/inputs' && req.method === 'GET') {
      const dir = url.searchParams.get('dir');
      const args = [linuxPath(path.join(project,'scripts/web_mapping.py')), '--inspect'];
      if (dir) {
        if (!path.isAbsolute(dir) || !await isFile(path.join(dir,'metadata.yaml')))
          return reply(400,{error:'请选择有效的 bag 目录'});
        args.push(linuxPath(path.resolve(dir)));
      }
      const result = await promisify(execFile)(windows ? 'wsl.exe' : 'python3',
        windows ? ['-d','Ubuntu-24.04','--exec','python3',...args] : args,
        {windowsHide:true,timeout:20000});
      return reply(200,JSON.parse(result.stdout));
    }
    if (url.pathname === '/api/mapping/folders' && req.method === 'GET')
      return reply(200,await browse(url.searchParams.get('dir')));
    if (url.pathname !== '/api/mapping') return reply(404,{error:'未知接口'});
    if (req.method === 'GET') {
      const job = current;
      let frames, rvizStatus;
      if (job.state === 'running') {
        if (job.rviz) {
          try {rvizStatus=JSON.parse(await readFile(path.join(job.runDir,'rviz_gui_status.json'),'utf8')).message;} catch {}
        }
        try { frames = JSON.parse(await readFile(path.join(job.runDir,'quality.json'),'utf8')).odom_count; } catch {}
      }
      return reply(200,{...job,frames,rvizStatus,saving,canStop:!saving && job.state === 'running' && await isFile(path.join(job.runDir,'processes.json'))});
    }
    if (req.method !== 'POST') return reply(405,{error:'不支持的请求'});
    if (req.headers.origin !== 'http://'+req.headers.host || req.headers['content-type'] !== 'application/json')
      return reply(403,{error:'请从本机建图页面启动'});
    let body='';
    for await (const chunk of req) {body+=chunk;if(body.length>8192) return reply(413,{error:'请求过大'});}
    const {bag,rviz,action,runDir,name,format,rate=1} = JSON.parse(body);
    if ((!action || action === 'rate') && (!Number.isFinite(rate) || rate < 0.5 || rate > 4))
      return reply(400,{error:'回放倍率范围为 0.5× 到 4×'});
    if (action === 'rate') {
      const job = current;
      if (job.state !== 'running' || runDir !== job.runDir) return reply(409,{error:'当前任务已变更'});
      if (settingRate) return reply(409,{error:'正在调整回放倍率'});
      settingRate = true;
      try {
        const args = [linuxPath(path.join(project,'scripts/ros_env.sh')), 'ros2','service','call',
          '/rosbag2_player/set_rate','rosbag2_interfaces/srv/SetRate',JSON.stringify({rate})];
        const result = await promisify(execFile)(windows?'wsl.exe':'bash',
          windows?['-d','Ubuntu-24.04','--exec','bash',...args]:args,{windowsHide:true,timeout:12000});
        if (!result.stdout.includes('success=True')) throw Error('播放器未确认新倍率');
        job.rate = rate;
        return reply(200,{rate});
      } catch { return reply(409,{error:'播放器未确认新倍率，请检查回放是否仍在运行'}); }
      finally { settingRate=false; }
    }
    if (action && !['stop','export'].includes(action)) return reply(400,{error:'未知操作'});
    if (saving) return reply(409,{error:'地图正在写入，请稍后再试'});
    if (action === 'export') {
      if (!current.runDir || runDir !== current.runDir || !(current.map || current.state === 'running'))
        return reply(409,{error:'当前任务没有可导出的地图'});
      if (typeof name !== 'string' || !/^[\p{L}\p{N} _-]{1,64}$/u.test(name) || !['autoware','pcd'].includes(format))
        return reply(400,{error:'请检查地图名称和格式'});
      saving = true;
      try {
        const args = [linuxPath(path.join(project,'scripts/export_web_map.py')),'--',linuxPath(runDir),name,format];
        const result = await promisify(execFile)(windows ? 'wsl.exe' : 'python3',
          windows ? ['-d','Ubuntu-24.04','--exec','python3',...args] : args,
          {windowsHide:true,timeout:600000});
        return reply(201,{dir:path.join(runDir,'exports',name),...JSON.parse(result.stdout)});
      } catch(error) {
        return reply(400,{error:error.stderr?.trim() || error.message});
      } finally { saving = false; }
    }
    if (action === 'stop') {
      if (!current.runDir || runDir !== current.runDir) return reply(409,{error:'当前任务已变更，请刷新任务状态'});
      if (!['running','stopping','stopped'].includes(current.state)) return reply(409,{error:'当前任务已结束'});
      await stopJob(current);
      return reply(202,current);
    }
    if (typeof bag !== 'string' || !path.isAbsolute(bag) || typeof rviz !== 'boolean' ||
        !await isFile(path.join(bag,'metadata.yaml'))) return reply(400,{error:'请选择含 metadata.yaml 的 ROS 2 bag 目录'});
    if (['running','stopping'].includes(current.state)) return reply(409,{error:'已有建图任务正在运行'});
    launch(path.resolve(bag),rviz,rate);
    reply(202,current);
  } catch(error) { reply(400,{error:error.message}); }
}
