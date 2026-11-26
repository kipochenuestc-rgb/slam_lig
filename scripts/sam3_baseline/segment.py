"""用本机 SAM3 对短路段四路原始鱼眼图逐帧分割。"""

import json, sys
from pathlib import Path
import cv2
import numpy as np
import torch
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/sam3_dense_road_120_150"
OUT.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, "/mnt/d/slam/sam3")
from sam3.model.sam3_image_processor import Sam3Processor
from sam3.model_builder import build_sam3_image_model

PIC = ROOT.parent / "pic"
PROMPTS = ["sky", "person", "car", "truck", "bus", "bicycle", "motorcycle"]


@torch.inference_mode()
def main():
    """缓存原分辨率掩码、实例置信度及叠图，支持中途续跑。"""
    torch.set_num_threads(4)
    model = build_sam3_image_model(
        device="cuda", checkpoint_path="/mnt/d/slam/sam3/weights/sam3.pt", load_from_HF=False
    )
    proc = Sam3Processor(model, device="cuda", confidence_threshold=0.30)
    a = np.load(PIC / "poses.npz")
    m = a["camera_id"] == 0
    ts = a["stamp_ns"][m] * 1e-9
    gs = a["group_index"][m]
    zero = 1795486169.500616
    groups = gs[(ts >= zero + 119.8) & (ts <= zero + 150.2)]
    (OUT / "masks").mkdir(exist_ok=True)
    for g in groups:
        masks = []
        reports = []
        for cam in range(4):
            dest = OUT / f"masks/{g:06d}_{cam}.png"
            if dest.exists():
                continue
            frame = Image.open(PIC / f"images/cam{cam}/{g:06d}.png").convert("RGB")
            labels = np.zeros((640, 800), np.uint8)
            report = {}
            with torch.autocast("cuda", dtype=torch.bfloat16):
                state = proc.set_image(frame)
                for prompt in PROMPTS:
                    proc.reset_all_prompts(state)
                    result = proc.set_text_prompt(state=state, prompt=prompt)
                    ms = result["masks"].flatten(0, 1)
                    combined = ms.any(dim=0).cpu().numpy()
                    report[prompt] = {
                        "instances": len(ms),
                        "scores": result["scores"].float().cpu().tolist(),
                        "pixels": int(combined.sum()),
                    }
                    labels[combined] = 1 if prompt == "sky" else 2 if prompt == "person" else 3
                del state, result, ms
            cv2.imwrite(str(dest), labels)
            (dest.with_suffix(".json")).write_text(json.dumps(report))
            if int(g) in [840, 864, 875]:
                img = np.asarray(frame).copy()
                colors = np.array(
                    [[0, 0, 0], [235, 160, 40], [255, 65, 60], [255, 170, 40]], np.uint8
                )
                hit = labels != 0
                img[hit] = (0.45 * img[hit] + 0.55 * colors[labels[hit]]).astype("u1")
                Image.fromarray(img).save(OUT / f"overlay_{g}_{cam}.jpg")
            print(int(g), cam, flush=True)
    (OUT / "mask_run.json").write_text(
        json.dumps(
            {
                "groups": groups.tolist(),
                "prompts": PROMPTS,
                "threshold": 0.30,
                "checkpoint": "/mnt/d/slam/sam3/weights/sam3.pt",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
