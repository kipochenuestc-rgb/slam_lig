# LIO-SAM core used by slam_lig

Upstream: https://github.com/TixiaoShan/LIO-SAM (ROS 2 branch).
The exact base commit is in `UPSTREAM_COMMIT`; see `LICENSE` for attribution.

This copy contains the source actually built by `../../scripts/build.sh`.
Adaptations include Jazzy/Eigen/C++17 compatibility, IMU queue guards and TF metadata,
non-destructive map saving, GNSS factor logging, and the optional
`deskewReferenceToHeader` setting (default false, enabled for this dataset).
Fusion, synthetic elevation bins and online filtering live outside the core in
the project's `scripts/` directory. No Docker files or duplicate upstream launch
and configuration files are needed by this project.

Additional opt-in solid-state support: `solidStateSurfaceOnly` bypasses synthetic
scan-line feature extraction and fits centered local planes in 3D. Set
`useImuOdometryPrediction: false` to retain rotational deskew and attitude input
without feeding preintegrated pose corrections back into scan registration.
The registration degeneracy projection persists across iterations, and OpenMP
selection flags use independent bytes instead of packed `vector<bool>` bits.
