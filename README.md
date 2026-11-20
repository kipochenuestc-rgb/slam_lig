# slam_lig：四雷达实时融合建图

运行环境：WSL Ubuntu 24.04、ROS 2 Jazzy。当前工程包含实际使用的 LIO-SAM C++ 源码、实时拼接与滤波、IMU 偏置补偿、GNSS 桥接，以及 RViz 配置。

建图链路：**四路传感器点云 → 按 1 号雷达时间拼接 → 实时滤波 → LIO-SAM + IMU + GNSS → RViz**。输入可来自真实驱动话题或已有 bag，不需要提前生成滤波 bag。

## Windows 使用

- 首次或修改 C++ 后：双击 `build.cmd`。
- 模式选择：双击 `start_mapping.cmd`，选择 1 实时建图、2 只录制数据、3 离线建图。离线模式可输入 Windows 或 WSL 的 bag 文件夹路径，回车使用配置的默认数据。
- `start_live_mapping.cmd` 可直接进入实时建图。先启动传感器驱动，使用相同 ROS domain 50；本工程不会启动厂家的硬件驱动。
- 查看已保存地图：双击 `open_map.cmd`。
- 单元测试：双击 `test.cmd`。
- Web 地图：双击 `start_web_viewer.cmd`，代码位于 `web_viewer/`，也可直接运行其中的 `start.cmd`。高德配置为 `web_viewer/config/amap.local.json`，按仓库所有者要求随源码提交；本地地图数据不提交 Git。
- 保存地图并停止任务：双击 `stop.cmd`。包含主窗口关闭后遗留的后台节点；录制器会正常结束并写出 bag 元数据。

启动窗口中按 Ctrl+C 会停止本次进程。正常播放结束后保存地图并退出；可以用 `open_map.cmd` 查看结果。命令行可用 `--leave-running` 保留节点，但再次启动前需要停止该会话，工程会阻止重复运行。

出现“已有建图任务正在运行”时，先使用 `stop.cmd`。停止前的地图保存在原运行目录的 `map_stop_<时间>/`；地图保存失败时默认保留旧任务，详情写入 `stop_save_map.log`。脚本根据本工程配置、运行目录和进程启动标识确认归属，避免按通用进程名批量结束其他 ROS 任务。

启动建图时会自动关闭本工程的静态地图查看器，避免两个发布源交替显示。其他工程窗口不会被关闭。

任务结束后启动菜单会继续留在 WSL 内，建议暂时保留菜单窗口，以减少 WSLg 会话被完全销毁后再创建时的 COPY MODE 故障。输入 0 可以正常退出，不会安装后台常驻服务。

只录制模式不启动融合、滤波、SLAM 或 RViz；原始四雷达、IMU、GNSS 和固定 TF 保存在 `runs/<运行名称>/bag`。按 Ctrl+C 正常完成录制后，再用离线模式处理该文件夹。模式默认值与录制话题在 `config/runtime.yaml`。

## WSL 命令

```bash
# 编译独立工作区 /home/kipoc/slam_lig_ws
bash /mnt/d/slam/slam_lig/scripts/build.sh

# 单元测试
bash /mnt/d/slam/slam_lig/scripts/test.sh

# 实时传感器建图，不启动 bag 播放器，也不要求 /clock
python3 /mnt/d/slam/slam_lig/scripts/run_slam.py --mode live

# 只采集原始数据，Ctrl+C 结束
python3 /mnt/d/slam/slam_lig/scripts/run_slam.py --mode record

# 之后使用采集的数据建图
python3 /mnt/d/slam/slam_lig/scripts/run_slam.py --mode replay --bag /path/to/recording/bag

# 150 秒验证，自动保存后退出；run 名称必须是新目录
python3 /mnt/d/slam/slam_lig/scripts/run_slam.py --run validation_example --duration 150 --no-rviz

# 指定原始 bag
python3 /mnt/d/slam/slam_lig/scripts/run_slam.py --bag /path/to/rosbag --input raw

# 从已拼接的 bag 输入，仍然实时滤波
python3 /mnt/d/slam/slam_lig/scripts/run_slam.py --input fused

# 查看指定结果
bash /mnt/d/slam/slam_lig/scripts/open_map.sh /path/to/run
```

默认数据位置和已保存结果位置在 `config/dataset.json`，大体积 bag 和历史成果与代码分开。`--start-offset` 默认 0 秒，仅影响回放；如需复现旧验证可指定 40 秒。IMU 常量偏置使用已有标定文件，不会预读当前数据。`--duration` 在回放模式表示传感器时长，在实时/录制模式表示墙钟时长（0 为持续运行）。`--rate` 仅控制回放倍率。

## 目录

| 目录 | 内容 |
|---|---|
| `src/LIO-SAM` | 当前实际编译的 C++ 核心、ROS 消息与服务、许可证、上游版本号 |
| `scripts` | 运行控制、在线拼接、在线滤波、GNSS/IMU 桥接、结果检查与查看 |
| `config` | 传感器外参、LIO 参数、GNSS 初始化、RViz 和通信配置 |
| `launch` | 四个 LIO-SAM 节点与固定坐标变换 |
| `tests` | 拼接匹配、坐标与逐点时间、滤波正确性测试 |
| `runs` | 每次独立的日志、质量统计、轨迹与地图 |
| `cmake/gtsam` | 本机 GTSAM 4.2 包的 CMake 导出兼容配置 |

没有保留离线滤波转换器、重复启动脚本、失败试验代码和编译产物。构建输出在 WSL 工作区，不放入源码目录。

## 输入、输出和时间

- 原始输入：`/point_cloud_hasco1`～`/point_cloud_hasco4`、`/imu/data`。GNSS 直接订阅 `/gnss/fix_fixed`（`sensor_msgs/NavSatFix`），不读取 CSV，不按回放时钟定时发送。话题名可修改 `config/bridge.yaml`，或使用 `--gnss-topic /your/fix`。
- 融合输出：`/lidars/fused_points`，frame 为 `hasco1_lio`，原点为 1 号雷达，方向与 IMU 对齐。
- 滤波输出：`/lidars/filtered_points`，这是 LIO-SAM 的点云输入。
- 每组必须四路齐全；以 1 号帧头匹配其余最近帧，容差 ±20 ms，单帧不重复使用。缺失组跳过。
- 点时间为本路帧头相对 1 号帧头的偏移，加上原始点的微秒时间。滤波保留真实采样点，不平均时间戳。
- 滤波顺序：1.5–100 m 距离裁剪、0.2 m 体素、20 近邻统计滤波（均值 + 2 倍标准差）。4 个计算进程并行，按输入顺序发布；16 帧上限，超载计数明确报告。
- `deskewReferenceToHeader=true`：即使帧首点被滤掉，仍以 1 号帧头时间作为 IMU 去畸变基准。

`runs/<本次运行>/fusion_status.json`、`bridge_status.json` 和 `live_filter_status.json` 可查看输入、输出、积压、丢弃及耗时。`quality.json` 为在线里程计；`map/transformations.pcd` 才是最终图优化关键帧。地图服务成功后，文件复制到本次 `map/`。

```bash
python3 /mnt/d/slam/slam_lig/scripts/analyze_run.py /mnt/d/slam/slam_lig/runs/<运行名称>
```

生成最终关键帧 CSV、二维预览和 GNSS 约束残差。最后静止区间可能没有新增关键帧，关键帧轨迹跨度会短于完整播放时间。

## GNSS 在线坐标对齐

默认 `alignment_mode: online`：第一个有效 GNSS 点建立 ENU 原点，按照消息时间戳与 SLAM 位姿插值配对。至少 12 对位置、15 m 水平运动且拟合通过后，冻结 ENU 到 map 的刚体变换，再开始发布新到达的 GNSS 位置约束。对齐期间 SLAM 正常运行；静止或没有匹配里程计时，GNSS 约束等待，不猜测航向。

GNSS 输出保持接收机原始测量时间，不改为系统当前时间。无定位、非有限坐标、无效或乱序时间戳会被拒绝；已知 ENU 协方差旋转到 map，未知协方差使用可配置的保守值。节点默认使用系统时钟；只有回放模式才给 SLAM/桥接设置 `use_sim_time=true`。所有传感器测量时间需要同一时间基准。

每次运行写出 `bridge_status.json`、`gnss_alignment.json` 与 `gnss_constraints.csv`；后者是本次实际发布的 GNSS 位置日志，不是节点输入。若已经有可靠的原点与航向标定，可显式选择 `alignment_mode: fixed` 并设置 `georeference_file`，否则不要沿用历史采集的全局坐标。`apply_imu_bias` 可以关闭；启用时应使用当前设备的 IMU 标定文件。

## 已知边界

这是针对本次数据的适配：固态雷达用仰角分箱代替原生机械 ring；候选雷达外参和 GNSS 天线杆臂仍需标定验证。GNSS 使用启动后的因果在线对齐，最大配对窗口 120 秒，不依赖历史轨迹或全轨迹事后拟合。GNSS 残差不能作为独立精度；上一版末段高度残差约 1.9 m。轮速未作为优化约束。

ROS 通信使用独立 domain 50，Linux 本机地址 `10.255.255.254` 和共享内存传输，配置在 `fastdds_local.xml`。如该地址变化需同步更新。测试使用独立 domain 89，不向正在建图的话题注入测试数据。RViz 带 Move Camera 工具，左键旋转、中键平移、滚轮缩放。

## RViz 窗口透明或无法出现

`launch_rviz.py` 会检查 WSLg 的 `use_gfxredir` 状态。若检测到 COPY MODE，不会把一个透明窗口当作正常启动；日志会提示修复，建图可继续。

双击 `repair_rviz.cmd`：只重启 **Ubuntu-24.04 的图形后台**，然后重新连接正在运行的建图话题。该发行版的 Linux 图形窗口会关闭，非图形建图节点和终端任务继续运行。若图形后台已经正常，则只重新打开 RViz。此入口不重新播放 bag、不重新建图。

WSLg 底层共享内存缺陷仍可能复发，菜单保留和启动检查是防护而不是永久修复。若单独修复图形服务仍失败，先保存所有 WSL 工作，再考虑完整重启 WSL。

依赖 ROS Jazzy 的 C++/消息/TF/PCL/OpenCV 开发包、GTSAM 4.2、Python numpy/scipy/matplotlib 和 RViz。本机已安装；`build.sh` 会实际验证 C++ 依赖是否齐全。`cmake/gtsam` 仅修正本机导出包引用不存在的测试库及库文件名问题，不包含 GTSAM 本体。
