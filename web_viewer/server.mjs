import {mappingApi} from './mapping.mjs';
import http from 'node:http';
import {readFile,access} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';

const directory=path.dirname(fileURLToPath(import.meta.url));
const root=path.join(directory,'public');
const args=process.argv.slice(2), index=args.indexOf('--port');
const port=Number(index>=0?args[index+1]:(process.env.PORT||8765));
if(!Number.isInteger(port)||port<1||port>65535)throw Error('Invalid port');
const local=`http://localhost:${port}/?view=satellite`;
let localConfig={};
try{localConfig=JSON.parse(await readFile(path.join(directory,'config/amap.local.json'),'utf8'));}
catch(error){if(error.code!=='ENOENT')throw Error('Invalid config/amap.local.json; check the JSON format.');}
let amapConfig={key:process.env.AMAP_JS_KEY||localConfig.key||'',code:process.env.AMAP_SECURITY_CODE||localConfig.securityJsCode||''};
let viewState={},input='';
const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.json':'application/json; charset=utf-8','.geojson':'application/geo+json; charset=utf-8','.bin':'application/octet-stream','.png':'image/png','.jpg':'image/jpeg'};
async function dataReady(){try{await Promise.all(['metadata.json','points.bin','tracks.json'].map(name=>access(path.join(root,'data',name))));return true;}catch{return false;}}
function openBrowser(){
  const command=process.platform==='win32'?'explorer.exe':process.platform==='darwin'?'open':'xdg-open';
  const child=spawn(command,[local],{windowsHide:true,detached:true,stdio:'ignore'});
  child.on('error',()=>process.stdout.write(`Open in your browser: ${local}\n`));child.unref();
}
if(args.includes('--check')){
  const ready=await dataReady();
  if(ready){const metadata=JSON.parse(await readFile(path.join(root,'data/metadata.json'),'utf8'));const points=await readFile(path.join(root,'data/points.bin'));if(points.length!==metadata.pointCount*16)throw Error('Point data length mismatch');if(metadata.colorFile){const colors=await readFile(path.join(root,'data',metadata.colorFile));if(colors.length!==metadata.pointCount*3)throw Error('RGB data length mismatch');}}
  console.log(JSON.stringify({service:'slam_lig_web_viewer',node:process.version,dataReady:ready}));
  process.exit(0);
}
const server=http.createServer(async(req,res)=>{
  try{
    const url=new URL(req.url,`http://localhost:${port}`);
    if(url.pathname.startsWith('/api/mapping')){await mappingApi(req,res,url,port);return;}
    if(url.pathname==='/api/health'){res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'}).end(JSON.stringify({service:'slam_lig_web_viewer',dataReady:await dataReady()}));return;}
    if(url.pathname==='/api/view-state'){
      if(req.method==='POST'){let body='';for await(const chunk of req){body+=chunk;if(body.length>3000)throw Error('Too large');}viewState={...JSON.parse(body),time:new Date().toISOString()};res.writeHead(204).end();}
      else res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'}).end(JSON.stringify(viewState));return;
    }
    if(url.pathname==='/api/amap-config'){res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'}).end(JSON.stringify({enabled:!!(amapConfig.key&&amapConfig.code),key:amapConfig.key,serviceHost:`http://localhost:${port}/_AMapService`}));return;}
    if(url.pathname.startsWith('/_AMapService/')){
      if(!amapConfig.code||req.method!=='GET'){res.writeHead(403).end();return;}
      const name=url.pathname.slice('/_AMapService'.length);
      if(!name.startsWith('/v3/')&&!name.startsWith('/v4/')){res.writeHead(404).end();return;}
      const upstream=new URL(name,name.startsWith('/v4/map/styles')?'https://webapi.amap.com':'https://restapi.amap.com');
      upstream.search=url.search;upstream.searchParams.set('jscode',amapConfig.code);upstream.searchParams.set('key',amapConfig.key);
      const response=await fetch(upstream,{headers:{referer:req.headers.referer||`http://localhost:${port}/`,origin:req.headers.origin||`http://localhost:${port}`,'user-agent':req.headers['user-agent']||'Mozilla/5.0'}});
      res.writeHead(response.status,{'Content-Type':response.headers.get('content-type')||'application/json','Cache-Control':'no-store'}).end(Buffer.from(await response.arrayBuffer()));return;
    }
    if(req.method!=='GET'&&req.method!=='HEAD'){res.writeHead(405).end();return;}
    let name=decodeURIComponent(url.pathname);if(name.endsWith('/'))name+='index.html';
    const target=path.resolve(root,'.'+name);if(!target.startsWith(root+path.sep)){res.writeHead(403).end();return;}
    const data=await readFile(target);res.writeHead(200,{'Content-Type':types[path.extname(target)]||'application/octet-stream','Cache-Control':'no-store'}).end(req.method==='HEAD'?undefined:data);
  }catch{res.writeHead(404,{'Content-Type':'text/plain; charset=utf-8'}).end('Not found');}
});
server.on('error',async error=>{
  if(error.code==='EADDRINUSE'){
    try{const response=await fetch(`http://127.0.0.1:${port}/api/health`,{signal:AbortSignal.timeout(1500)});const health=await response.json();if(health.service==='slam_lig_web_viewer'){console.log(`Already running: ${local}`);if(args.includes('--open')&&!args.includes('--no-open'))openBrowser();process.exit(0);}}
    catch{}
    console.error(`Port ${port} is used by another application. Close it or use --port <number>.`);
  }else console.error(error.message);
  process.exit(1);
});
function stop(){server.close(()=>process.exit(0));setTimeout(()=>process.exit(0),1500).unref();}
process.on('SIGINT',stop);process.on('SIGTERM',stop);
if(process.stdin.isTTY)process.stdin.setRawMode(true);
process.stdin.on('data',chunk=>{
  if(chunk.includes(3)){stop();return;}
  input+=chunk.toString();if(input.length>8192){input='';return;}
  let position;while((position=input.search(/[\r\n]/))>=0){
    const line=input.slice(0,position).trim();input=input.slice(position+1);if(!line)continue;
    try{const next=JSON.parse(line);if(typeof next.key!=='string'||typeof next.code!=='string')throw Error('Invalid config');amapConfig=next;console.log('Map runtime configuration updated');}
    catch{console.log('Invalid map runtime configuration');}
  }
});
server.listen(port,'127.0.0.1',async()=>{console.log(`Local: ${local}`);console.log('Press Ctrl+C to stop.');if(!await dataReady())console.log('Map data is missing: place exports in public/data.');if(args.includes('--open')&&!args.includes('--no-open'))openBrowser();});
