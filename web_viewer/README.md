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
