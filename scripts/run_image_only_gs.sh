#!/bin/bash
# Usage (WSL): bash scripts/run_image_only_gs.sh images_scene output_root semantics_dir
set -euo pipefail
scripts_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image_source="${1:?Pass a scene with rectified images and SLAM camera priors}"
result_root="${2:?Pass a new output root}"
semantic_source="${3:-}"
if [ -e "$result_root" ]; then
  echo "Use a new output root: $result_root" >&2
  exit 1
fi
mkdir -p "$result_root"
if [ -z "$semantic_source" ]; then
  semantic_source="$result_root/semantics"
  bash "$scripts_dir/gs_env.sh" "$scripts_dir/prepare_scene_semantics.py" \
    --scene "$image_source" --output "$semantic_source"
fi
bash "$scripts_dir/gs_env.sh" "$scripts_dir/bundle_adjust_rig.py" \
  --scene "$image_source" --semantics "$semantic_source" --output "$result_root/bundle"
bash "$scripts_dir/gs_env.sh" "$scripts_dir/prepare_image_only_scene.py" \
  --images-scene "$image_source" --bundle "$result_root/bundle" \
  --semantics "$semantic_source" --output "$result_root/scene"
bash "$scripts_dir/gs_env.sh" "$scripts_dir/refine_local_gs.py" \
  --scene "$result_root/scene" --output "$result_root/model" \
  --image-only --no-pose-opt --mask-dir "$result_root/scene/valid_masks" \
  --steps "${SLAM_IMAGE_GS_STEPS:-20000}" --point-cap "${SLAM_IMAGE_GS_CAP:-700000}" --patience 5
echo "Image-only model: $result_root/model/gaussians.ply"
