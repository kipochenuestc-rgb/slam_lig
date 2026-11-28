# 点云与卫星地图查看器

双击 `start.cmd`，自动启动本地服务并打开 `http://localhost:8765/?view=satellite`。窗口保持打开，Ctrl+C 停止服务。重复启动会打开现有服务，不重复占用端口。

启动脚本先查找已安装的 Node.js，再查找本机 Codex 附带的 Node.js；不需要 npm install 或编译。手动启动：`node server.mjs --open`。

- `public/index.html`、`app.js`、`style.css` 是网页源码。
- `public/data/` 是本地地图数据：metadata.json、points.bin、colors.bin、tracks.json、pointcloud_2d.geojson。
- 当前显示 SAM 3 全量强清理地图的 400,000 个均匀抽样点，完整来源为 13,315,896 点。colors.bin 为逐点 uint8 RGB，长度为 pointCount×3；旧数据没有 colorFile 字段时继续按高度着色。
- 地图数据受上级 `.gitignore` 排除，不随源码上传；新电脑需要另外复制或导出上述数据文件。

高德配置在 `config/amap.local.json`：`key` 是 Web JS API Key，`securityJsCode` 是对应安全密钥。修改后重启服务生效。按仓库所有者要求，实际配置随源码提交，新电脑克隆后即可使用；另保留 `config/amap.example.json` 空白模板供替换配置。

服务代理不会把安全密钥返回页面。也支持环境变量覆盖配置，或在网页入口按浏览器会话填写。没有高德配置时仍可查看本地点云。

`node server.mjs --check` 检查 Node 和地图数据长度；`--port 8766` 可更换端口。

使用 ?view=local 可直接进入本地坐标视图。当前查看地址为 http://localhost:8766/?view=local，服务采用 --port 8766 --no-open 启动。Web 是二维俯视与卫星图叠加，完整三维测点保存在 runs/sam3_full_outlier_strong_20261127/sam3_full_clean_rgb.ply。旧网页数据备份在 runs/web_viewer_data_before_sam3。

建图页的「停止建图」结束回放，有效地图保存至运行目录的 map/GlobalMap.pcd。「保存地图」可导出实时快照或已完成的地图，目标为本次运行的 exports/地图名称。Autoware 格式依赖本次 GNSS 对齐，生成 LocalCartesian/WGS84 点云、map_projector_info.yaml 和 pointcloud_map_metadata.yaml。本地 PCD 格式保留 SLAM 坐标。道路矢量图 lanelet2_map.osm 需要独立制作。

## 桌面版（Windows）

首次使用请在本目录执行 `python -m pip install -r requirements.txt`。双击项目根目录的 `start_desktop.cmd` 即可打开 pywebview 窗口。Windows 使用 Edge WebView2。桌面服务默认选择可用的本地端口，也可运行 `python desktop.py --web --port 8770` 在浏览器中使用。

- **离线建图**沿用原有 ROS 2 / WSL 建图服务，桌面和原 Web 页面操作同一个任务。关闭窗口后建图服务继续运行。
- **高德投影**可切换已有的点云数据集，包括工作区中 `gnss-map-web/web/data` 下的原始投影和本项目的 `public/data`。
- 建图结束后点击 **在高德上查看建图结果**。程序读取该次运行的 PCD、GNSS 对齐和轨迹文件，首次查看时在运行目录的 `atlas_preview` 中生成抽样预览。卫星底图使用现有高德配置。

Vue 运行时随程序提供。FastAPI 提供桌面页面、数据集和投影接口，并连接本机 8767 端口的既有 Node 建图服务。启动时会复用或拉起该服务。
- 在高德投影页选择地图并点击 **载入地图**。列表包含已有投影数据、已完成建图结果和保存的 Autoware 地图。也可点击 **打开地图文件夹**，从本机载入含有 `pointcloud_map.pcd` 与 `map_projector_info.yaml` 的 Autoware 地图目录。
**点云视图**可载入当前地图，或者通过“打开 PCD”选择独立文件。地图列表中的大型 PCD 使用后端抽样预览，独立文件在本机解析。左键旋转、中键平移，右键或滚轮缩放。地面网格每格 1 米，屏幕标尺按相机焦点处的距离计算。`public/pointcloud.js` 负责三维渲染和相机控制，`studio_api.py` 的 pointcloud 接口提供地图预览点。

定位页通过 `localization_view.py` 读取 ROS 2 bag 所选 NavSatFix topic，以 bag 记录时间驱动回放，取当前时刻之前最近一条采样。输出为 GNSS 天线在 LocalCartesian / WGS84 Autoware 地图中的 XYZ（米）。可拖动时间条、播放或暂停，并在本地点云和高德卫星底图间切换。高德底图使用同一套地图投影和定位结果。
界面主题源文件为 workspace.pcss。工作区导航使用 Vue 和 Ant Design Vue。在 web_viewer 下运行 npm ci 和 npm run build:ui 可重新生成本地 JS 与 Tailwind CSS，桌面运行时直接使用 public 内的构建产物。
