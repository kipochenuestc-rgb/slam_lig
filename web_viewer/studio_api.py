"""Local FastAPI service for the mapping and Gaode projection desktop."""
import hashlib
import json
import os
import shutil
import subprocess
import time
from contextlib import asynccontextmanager
from pathlib import Path

import httpx
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware

from rosbags.rosbag2 import ReaderError
from localization_view import bag_info, map_positions
from projection import _pcd_sample, project, project_autoware

HERE = Path(__file__).resolve().parent
PUBLIC = HERE / "public"
RUNS = HERE.parent / "runs"
WORKER = "http://127.0.0.1:8767"
FILES = {"metadata.json", "points.bin", "tracks.json", "colors.bin", "pointcloud_2d.geojson"}


def ensure_worker():
    def ready(client):
        try:
            return client.get(WORKER + "/api/health").json().get("service") == "slam_lig_web_viewer"
        except (httpx.HTTPError, ValueError):
            return False

    with httpx.Client(trust_env=False, timeout=1) as client:
        if ready(client):
            return
        node = shutil.which("node") or next(iter(Path("C:/Program Files/WindowsApps").glob(
            "*CodexPrimaryRuntime*/dependencies/node/bin/node.exe")), None)
        if not node:
            raise RuntimeError("找不到建图服务所需的 Node.js")
        child = subprocess.Popen([str(node), str(HERE / "server.mjs"), "--port", "8767", "--no-open"],
                                 stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                 creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        for _ in range(80):
            if ready(client):
                return
            if child.poll() is not None:
                break
            time.sleep(.1)
        raise RuntimeError("建图服务启动失败，请检查端口 8767")


@asynccontextmanager
async def lifespan(app):
    ensure_worker()
    async with httpx.AsyncClient(base_url=WORKER, trust_env=False, timeout=630) as client:
        app.state.worker = client
        yield


app = FastAPI(lifespan=lifespan)
app.add_middleware(TrustedHostMiddleware, allowed_hosts=["127.0.0.1", "localhost"])


external_maps = {}


def sources():
    for key, folder in tuple(external_maps.items()):
        yield key, f"Autoware 地图 · {folder.name}", folder, "autoware"
    candidates = {
        "original": HERE.parents[1] / "gnss-map-web/web/data",
        "viewer": PUBLIC / "data",
    }
    for key, folder in candidates.items():
        if all((folder / name).is_file() for name in ("metadata.json", "points.bin", "tracks.json")):
            meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
            yield key, meta.get("datasetLabel", key), folder, False
    for run in sorted(RUNS.glob("web_*"), key=lambda p: p.stat().st_mtime, reverse=True):
        if all((run / name).is_file() for name in ("map/GlobalMap.pcd", "gnss_alignment.json", "run_status.json", "quality.json")):
            when = time.strftime("%m-%d %H:%M", time.localtime((run/"run_status.json").stat().st_mtime))
            yield run.name, f"建图结果 · {when} · {run.name[4:12]}", run, "run"
        for folder in (run / "exports").glob("*"):
            if (folder / "pointcloud_map.pcd").is_file() and (folder / "map_projector_info.yaml").is_file():
                yield f"{run.name}__{folder.name}", f"Autoware 地图 · {folder.name}", folder, "autoware"


@app.get("/api/localization/bag")
def localization_bag(directory: str):
    try:
        return bag_info(directory)
    except (ReaderError, ValueError, OSError, RuntimeError) as error:
        raise HTTPException(400, f"bag 读取失败：{error}") from error


@app.get("/api/localization/maps")
def localization_maps():
    return [{"id": key, "label": label} for key, label, _, kind in sources() if kind == "autoware"]


@app.get("/api/localization/positions")
def localization_positions(directory: str, topic: str, dataset: str):
    for key, _, folder, kind in sources():
        if key == dataset and kind == "autoware":
            try:
                return map_positions(directory, topic, folder)
            except (ValueError, KeyError, OSError, RuntimeError) as error:
                raise HTTPException(400, f"读取定位数据失败：{error}") from error
    raise HTTPException(404, "请选择已导入的 Autoware 地图")


@app.get("/api/datasets")
def datasets():
    return [{"id": key, "label": label} for key, label, _, _ in sources()]


@app.get("/api/datasets/{dataset}/pointcloud")
def dataset_pointcloud(dataset: str):
    """按地图编号返回 PCD 文件，或为已有网页地图返回 XYZ+I 二进制预览。"""
    for key, _, folder, source_type in sources():
        if key != dataset:
            continue
        if source_type == "run":
            pointcloud = folder / "map/GlobalMap.pcd"
        elif source_type == "autoware":
            pointcloud = folder / "pointcloud_map.pcd"
        else:
            pointcloud = folder / "points.bin"
        if not pointcloud.is_file():
            break
        if pointcloud.suffix.lower() == ".pcd":
            points, count = _pcd_sample(pointcloud)
            return Response(points.tobytes(), media_type="application/octet-stream",
                            headers={"X-Source-Points": str(count)})
        return FileResponse(pointcloud, media_type="application/octet-stream")
    raise HTTPException(404, "所选地图没有可读取的点云文件")


@app.post("/api/datasets/import")
def import_map(request: Request, payload: dict):
    if request.headers.get("origin") != str(request.base_url).rstrip("/"):
        raise HTTPException(403, "请从本机界面操作")
    value = payload.get("directory")
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise HTTPException(400, "请选择地图文件夹或输入绝对路径")
    folder = Path(value).resolve()
    if not all((folder / name).is_file() for name in ("pointcloud_map.pcd", "map_projector_info.yaml")):
        raise HTTPException(400, "目录中需要 pointcloud_map.pcd 和 map_projector_info.yaml")
    try:
        project_autoware(folder)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        raise HTTPException(400, f"地图读取失败：{error}") from error
    for key, _, existing, kind in sources():
        if kind == "autoware" and existing == folder:
            return {"id": key}
    key = "external_" + hashlib.sha256(str(folder).encode()).hexdigest()
    external_maps[key] = folder
    return {"id": key}


@app.get("/atlas/{dataset}/data/{name}")
def dataset_file(dataset: str, name: str):
    if name in FILES:
        for key, _, folder, source_type in sources():
            if key == dataset:
                if source_type == "autoware":
                    base = project_autoware(folder)
                elif source_type == "run":
                    base = project(folder)
                else:
                    base = folder
                path = base / name
                if path.is_file():
                    return FileResponse(path)
    raise HTTPException(404, "找不到投影数据")


@app.get("/atlas/{dataset}/{asset}")
def atlas(dataset: str, asset: str):
    if asset not in ("index.html", "app.js", "style.css"):
        raise HTTPException(404)
    return FileResponse(PUBLIC / asset)


@app.get("/")
def home():
    return FileResponse(PUBLIC / "desktop.html")


@app.api_route("/api/{route:path}", methods=["GET", "POST"])
@app.get("/_AMapService/{route:path}")
async def existing_service(request: Request, route: str):
    # A single worker owns ROS processes for both the web and desktop clients.
    path = request.url.path
    if path not in {"/api/mapping", "/api/mapping/inputs", "/api/mapping/folders", "/api/health",
                    "/api/view-state", "/api/amap-config"} and not path.startswith("/_AMapService/"):
        raise HTTPException(404)
    headers = {}
    if request.method == "POST":
        if request.headers.get("origin") != str(request.base_url).rstrip("/"):
            return JSONResponse({"error":"请从本机界面操作"}, status_code=403)
        headers = {"origin": WORKER, "content-type": request.headers.get("content-type", "")}
    try:
        response = await app.state.worker.request(request.method, path, params=request.query_params.multi_items(),
                                                  content=await request.body(), headers=headers)
    except httpx.HTTPError:
        return JSONResponse({"error":"建图服务连接失败"}, status_code=502)
    if path == "/api/amap-config" and response.is_success:
        return {**response.json(), "serviceHost": str(request.base_url).rstrip("/") + "/_AMapService"}
    return Response(response.content, status_code=response.status_code,
                    headers={"content-type":response.headers.get("content-type","application/json"),
                             "cache-control":"no-store"})


app.mount("/", StaticFiles(directory=PUBLIC), name="public")
