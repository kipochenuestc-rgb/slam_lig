# SAM 3 固定基线

当前固定版本为 sam3-dense-road-120-150-v1，使用原 NDT/GNSS/EKF 轨迹，采集包第 120～150 秒，7.5 厘米体素。最终点云 1,297,109 点，统计滤波剔除 25,005 点。

主要变化是四路原始鱼眼图逐帧使用本机 SAM 3，以 sky、person、car、truck、bus、bicycle、motorcycle 为提示；在原始雷达逐点运动补偿和取色后，排除掩码内观测，对人车轮廓额外执行同深度边界检查。全段累积后执行 20 近邻平均距离统计过滤，阈值为全局均值加 2 倍标准差。

## 运行入口

在 WSL Ubuntu 24.04 内从工程目录运行。

1. /home/kipoc/.venvs/sam3-cu128/bin/python scripts/sam3_baseline/segment.py
2. bash scripts/ros_env.sh python3 scripts/sam3_baseline/rebuild.py

segment.py 默认本地权重为 /mnt/d/slam/sam3/weights/sam3.pt，需要本机 /mnt/d/slam/sam3 的 Python 源码与 CUDA 环境。输入为 /mnt/d/slam/session_20261124_100923_5cbed2 和 /mnt/d/slam/pic，轨迹位于 runs/ndt_ekf_full_v3/ekf_trajectory.csv。输出位于 runs/sam3_dense_road_120_150。现存逐帧掩码会复用，重新推理须选择独立输出位置或保留旧掩码后清理对应文件。

## 文件职责

- scripts/sam3_baseline/segment.py 生成原始照片尺寸的类别掩码和实例置信度。
- scripts/sam3_baseline/core.py 复用 rgb_core 的标定、点云字段和位姿插值接口，提供高频 CSV 轨迹加载、MCAP 流式读取、逐点投影、PLY 读回与俯视绘图。
- scripts/sam3_baseline/rebuild.py 累积原始点观测，执行掩码与轮廓深度过滤、体素去重和离群点过滤，写出完整结果。
- docs/sam3_baseline_120_150/baseline.json 固定结果、模型、标定、轨迹的 SHA-256 和本地路径。
- report.json、verification.json 记录当次重建和定向验证结果，comparison.jpg 为同时间窗过滤前后预览。

SAM 3 权重、输入 bag、照片、轨迹以及 PLY 在本地保存，Git 提交代码、参数、校验签名、报告和预览。复现需要这些输入文件，校验签名用于确认所用数据版本。

这版没有额外位姿图优化。候选外参未重新标定，语义分割仍可能误分或漏分，统计滤波可能剔除远处稀疏细枝。已核查的人体样本在本次原图中出现的 14 点，最终保留 0 点，这一结果仅覆盖指定样本。

提交同时包含本聊天前期已完成的 run_slam.py 生命周期重构，保留实时、录制和回放入口。
