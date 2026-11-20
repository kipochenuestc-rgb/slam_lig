# 点云与卫星地图查看器

双击 `start.cmd`，自动启动本地服务并打开 `http://localhost:8765/?view=satellite`。窗口保持打开，Ctrl+C 停止服务。重复启动会打开现有服务，不重复占用端口。

启动脚本先查找已安装的 Node.js，再查找本机 Codex 附带的 Node.js；不需要 npm install 或编译。手动启动：`node server.mjs --open`。

- `public/index.html`、`app.js`、`style.css` 是网页源码。
- `public/data/` 是本地地图数据：metadata.json、points.bin、tracks.json、pointcloud_2d.geojson。
- 本次保留已有四雷达拼接滤波建图的 120,000 个显示点、优化轨迹和 GNSS 约束统计。
- 地图数据受上级 `.gitignore` 排除，不随源码上传；新电脑需要另外复制或导出这四个数据文件。

高德配置在 `config/amap.local.json`：`key` 是 Web JS API Key，`securityJsCode` 是对应安全密钥。修改后重启服务生效。按仓库所有者要求，实际配置随源码提交，新电脑克隆后即可使用；另保留 `config/amap.example.json` 空白模板供替换配置。

服务代理不会把安全密钥返回页面。也支持环境变量覆盖配置，或在网页入口按浏览器会话填写。没有高德配置时仍可查看本地点云。

`node server.mjs --check` 检查 Node 和地图数据长度；`--port 8766` 可更换端口。
