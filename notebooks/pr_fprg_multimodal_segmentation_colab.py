# %% [markdown]
# # PR-FPRG 多模态钢筋面分割训练 Colab
#
# 目标：不用 RGB，只用 IR / depth / worldCoord / 组合响应等非 RGB 模态，训练二段式主链第一阶段：
#
# ```text
# 多模态输入
# -> U-Net++ / U-Net / DeepLabV3+ 分割 ordinary_rebar / beam_rebar / floor_seam / occluder
# -> 导出模型
# -> 后续接 skeleton / graph 找普通钢筋交点，再做梁筋 +/-13cm 过滤
# ```
#
# 推荐 zip 数据集结构：
#
# ```text
# dataset.zip
# └── dataset/
#     ├── samples.csv                       # 可选，推荐
#     ├── images/
#     │   ├── ir/<id>.npy 或 .png                       # 兼容 rectified_ir
#     │   ├── depth_z/<id>.npy 或 .png                  # 兼容 depth / rectified_depth / raw_world_z
#     │   ├── worldcoord_height_response/<id>.npy 或 .png
#     │   ├── depth_response/<id>.npy 或 .png
#     │   ├── depth_gradient/<id>.npy 或 .png
#     │   ├── combined_response/<id>.npy 或 .png
#     │   ├── hessian_ridge/<id>.npy 或 .png
#     │   ├── frangi_like/<id>.npy 或 .png
#     │   ├── fused_instance_response/<id>.npy 或 .png
#     │   ├── valid_mask/<id>.npy 或 .png               # 训练 ignore mask
#     │   └── workspace_mask/<id>.npy 或 .png           # 训练 ignore mask
#     └── masks/<id>.png                    # 像素值为类别 id，255 为 ignore
# ```
#
# 也兼容：
#
# ```text
# dataset/samples/<id>/ir.npy
# dataset/samples/<id>/depth_z.npy
# dataset/samples/<id>/worldcoord_height_response.npy
# dataset/samples/<id>/depth_response.npy
# dataset/samples/<id>/depth_gradient.npy
# dataset/samples/<id>/combined_response.npy
# dataset/samples/<id>/hessian_ridge.npy
# dataset/samples/<id>/frangi_like.npy
# dataset/samples/<id>/fused_instance_response.npy
# dataset/samples/<id>/valid_mask.npy
# dataset/samples/<id>/workspace_mask.npy
# dataset/samples/<id>/mask.png
# ```
#
# 类别约定：
#
# ```text
# 0 background
# 1 ordinary_rebar
# 2 beam_rebar
# 3 floor_seam
# 4 occluder
# 255 ignore
# ```

# %%
# =========================
# 0. 安装依赖
# =========================
import os
import sys
import subprocess


def in_colab():
    return "google.colab" in sys.modules or os.path.exists("/content")


if in_colab():
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "-q",
            "segmentation-models-pytorch>=0.3.3",
            "albumentations>=1.4.0",
            "opencv-python-headless",
            "scikit-learn",
            "tqdm",
            "matplotlib",
            "pandas",
            "onnx",
            "onnxruntime",
        ],
        check=True,
    )

# %%
# =========================
# 1. 配置区：只改这里就能开跑
# =========================
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import json
import math
import random
import shutil
import time
import zipfile

import cv2
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset
from sklearn.model_selection import train_test_split
from tqdm.auto import tqdm
import matplotlib.pyplot as plt


SEED = 20260430
random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)
torch.backends.cudnn.benchmark = True

# Google Drive 中的 zip 路径。Colab 里按自己的文件名改这里。
DRIVE_ZIP_PATH = "/content/drive/MyDrive/rebar_dataset.zip"

# 解压和输出目录。
WORK_DIR = Path("/content/pr_fprg_multimodal_seg")
DATA_ROOT = WORK_DIR / "dataset"
OUTPUT_DIR = WORK_DIR / "outputs"

# 输入模态：默认覆盖当前讨论的全部非 RGB 语义底图。
# IR/rectified_ir、depth/rectified_depth/raw_world_z 这类同义名由 CHANNEL_ALIASES 兼容，
# 不作为重复通道喂给网络。
INPUT_CHANNELS = [
    "ir",
    "depth_z",
    "worldcoord_height_response",
    "depth_response",
    "depth_gradient",
    "combined_response",
    "hessian_ridge",
    "frangi_like",
    "fused_instance_response",
]

TRAINING_MASK_CHANNELS = [
    "valid_mask",
    "workspace_mask",
]
REQUIRE_TRAINING_MASKS = True

CHANNEL_ALIASES = {
    "ir": ["ir", "rectified_ir", "infrared", "rectified_infrared"],
    "depth_z": ["depth_z", "depth", "rectified_depth", "raw_world_z", "raw_world_coord_z", "world_z"],
    "worldcoord_height_response": [
        "worldcoord_height_response",
        "worldCoord_height_response",
        "worldcoord_height",
        "world_coord_height",
        "worldCoord_height",
    ],
    "depth_response": ["depth_response", "depth_darkline_response", "depth_dark_line_response"],
    "depth_gradient": ["depth_gradient", "depth_grad"],
    "combined_response": ["combined_response", "depth_ir_combined_response", "combined_depth_ir_response"],
    "hessian_ridge": ["hessian_ridge", "hessian", "Hessian_ridge"],
    "frangi_like": ["frangi_like", "frangi", "multiscale_frangi_like"],
    "fused_instance_response": ["fused_instance_response", "instance_fused_response", "instance_response"],
    "valid_mask": ["valid_mask", "rectified_valid", "valid"],
    "workspace_mask": ["workspace_mask", "manual_workspace_mask", "roi_mask"],
}

CLASS_NAMES = [
    "background",
    "ordinary_rebar",
    "beam_rebar",
    "floor_seam",
    "occluder",
]
NUM_CLASSES = len(CLASS_NAMES)
IGNORE_INDEX = 255

# 图像训练尺寸。rectified 数据建议 512 起步；显存不足改 384。
IMAGE_SIZE = 512
BATCH_SIZE = 8
NUM_WORKERS = 2
EPOCHS = 60
LR = 3e-4
WEIGHT_DECAY = 1e-4
AMP = True

# 第一版推荐：UnetPlusPlus + efficientnet-b0。
# 可选：Unet, UnetPlusPlus, DeepLabV3Plus。
EXPERIMENTS = [
    {
        "name": "unetpp_effb0_9ch",
        "arch": "UnetPlusPlus",
        "encoder_name": "efficientnet-b0",
        "encoder_weights": None,
    },
    # 想做精度对照时取消注释：
    # {
    #     "name": "deeplabv3plus_mobilenetv2_9ch",
    #     "arch": "DeepLabV3Plus",
    #     "encoder_name": "mobilenet_v2",
    #     "encoder_weights": None,
    # },
]

# 如果 zip 内无 samples.csv，脚本会自动按目录扫描。
VAL_RATIO = 0.18
TEST_RATIO = 0.0

# 输出结果是否复制回 Google Drive。
SAVE_TO_DRIVE = True
DRIVE_OUTPUT_DIR = "/content/drive/MyDrive/pr_fprg_multimodal_seg_outputs"

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# %%
# =========================
# 2. 从 Google Drive 导入 zip 数据集
# =========================
try:
    from google.colab import drive

    drive.mount("/content/drive")
except Exception as exc:
    print(f"[warn] 未在 Colab 或无法挂载 Google Drive：{exc}")


def unzip_dataset(zip_path: str, work_dir: Path, data_root: Path):
    zip_path = Path(zip_path)
    if not zip_path.exists():
        raise FileNotFoundError(
            f"找不到数据集 zip：{zip_path}\n"
            "请把 DRIVE_ZIP_PATH 改成 Google Drive 里的实际路径。"
        )
    extract_dir = work_dir / "_extract"
    if data_root.exists():
        shutil.rmtree(data_root)
    if extract_dir.exists():
        shutil.rmtree(extract_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    extract_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path, "r") as archive:
        archive.extractall(extract_dir)
    candidates = [p for p in extract_dir.iterdir() if p.is_dir()]
    if len(candidates) == 1:
        extracted_root = candidates[0]
    elif (extract_dir / "images").exists() or (extract_dir / "samples").exists():
        extracted_root = extract_dir
    else:
        extracted_root = extract_dir
    shutil.move(str(extracted_root), str(data_root))
    if extract_dir.exists():
        shutil.rmtree(extract_dir)
    print(f"[ok] 数据集解压到：{data_root}")


unzip_dataset(DRIVE_ZIP_PATH, WORK_DIR, DATA_ROOT)

# %%
# =========================
# 3. 数据集扫描与读取
# =========================
IMAGE_EXTS = [".npy", ".npz", ".png", ".jpg", ".jpeg", ".tif", ".tiff"]


def channel_aliases(channel: str) -> List[str]:
    aliases = [str(channel)]
    aliases.extend(CHANNEL_ALIASES.get(str(channel), []))
    deduped = []
    for alias in aliases:
        if alias and alias not in deduped:
            deduped.append(alias)
    return deduped


def has_path_value(value) -> bool:
    if value is None:
        return False
    if isinstance(value, float) and math.isnan(value):
        return False
    text = str(value).strip()
    return bool(text) and text.lower() != "nan"


def find_file_by_stem(directory: Path, sample_id: str) -> Optional[Path]:
    for ext in IMAGE_EXTS:
        path = directory / f"{sample_id}{ext}"
        if path.exists():
            return path
    matches = list(directory.glob(f"{sample_id}.*"))
    return matches[0] if matches else None


def find_mask_file(data_root: Path, sample_id: str) -> Optional[Path]:
    candidates = [
        data_root / "masks",
        data_root / "labels",
        data_root / "mask",
        data_root / "label",
    ]
    for directory in candidates:
        if directory.exists():
            path = find_file_by_stem(directory, sample_id)
            if path is not None:
                return path
    sample_dir = data_root / "samples" / sample_id
    if sample_dir.exists():
        for name in ["mask", "label", "segmentation"]:
            path = find_file_by_stem(sample_dir, name)
            if path is not None:
                return path
    return None


def find_channel_file(data_root: Path, sample_id: str, channel: str) -> Optional[Path]:
    for channel_name in channel_aliases(channel):
        candidates = [
            data_root / "images" / channel_name,
            data_root / channel_name,
            data_root / "modalities" / channel_name,
            data_root / "responses" / channel_name,
            data_root / "masks" / channel_name,
        ]
        for directory in candidates:
            if directory.exists():
                path = find_file_by_stem(directory, sample_id)
                if path is not None:
                    return path
    sample_dir = data_root / "samples" / sample_id
    if sample_dir.exists():
        for channel_name in channel_aliases(channel):
            path = find_file_by_stem(sample_dir, channel_name)
            if path is not None:
                return path
    return None


def find_channel_file_from_csv_row(data_root: Path, table: pd.DataFrame, row, channel: str) -> Optional[Path]:
    for channel_name in channel_aliases(channel):
        if channel_name in table.columns and has_path_value(row.get(channel_name)):
            return data_root / str(row[channel_name]).strip()
    return None


def read_scalar_image(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix == ".npy":
        array = np.load(path)
    elif suffix == ".npz":
        loaded = np.load(path)
        first_key = loaded.files[0]
        array = loaded[first_key]
    else:
        array = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if array is None:
            raise ValueError(f"无法读取图像：{path}")
        if array.ndim == 3:
            array = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
    array = np.asarray(array)
    if array.ndim == 3:
        array = array[..., 0]
    return array.astype(np.float32)


def robust_normalize(image: np.ndarray, lower=1.0, upper=99.0) -> np.ndarray:
    image = np.asarray(image, dtype=np.float32)
    finite = np.isfinite(image)
    if not np.any(finite):
        return np.zeros_like(image, dtype=np.float32)
    values = image[finite]
    lo = float(np.percentile(values, lower))
    hi = float(np.percentile(values, upper))
    if hi <= lo + 1e-6:
        lo = float(np.min(values))
        hi = float(np.max(values))
    if hi <= lo + 1e-6:
        out = np.zeros_like(image, dtype=np.float32)
        out[finite] = 0.5
        return out
    out = np.clip((image - lo) / (hi - lo), 0.0, 1.0).astype(np.float32)
    out[~finite] = 0.0
    return out


def read_mask(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix == ".npy":
        mask = np.load(path)
    elif suffix == ".npz":
        loaded = np.load(path)
        mask = loaded[loaded.files[0]]
    else:
        mask = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if mask is None:
            raise ValueError(f"无法读取 mask：{path}")
        if mask.ndim == 3:
            # 如果是彩色调色板导出，默认取第一个通道；推荐直接保存类别 id 灰度图。
            mask = mask[..., 0]
    return np.asarray(mask).astype(np.uint8)


def scan_samples_from_csv(data_root: Path) -> Optional[pd.DataFrame]:
    csv_path = data_root / "samples.csv"
    if not csv_path.exists():
        return None
    table = pd.read_csv(csv_path)
    if "id" not in table.columns:
        raise ValueError("samples.csv 必须包含 id 列")
    rows = []
    for _, row in table.iterrows():
        sample_id = str(row["id"])
        record = {"id": sample_id, "split": str(row.get("split", ""))}
        for channel in INPUT_CHANNELS:
            path = find_channel_file_from_csv_row(data_root, table, row, channel)
            if path is None:
                path = find_channel_file(data_root, sample_id, channel)
            if path is None:
                raise FileNotFoundError(f"样本 {sample_id} 缺少通道 {channel}")
            record[channel] = str(path)
        for mask_channel in TRAINING_MASK_CHANNELS:
            path = find_channel_file_from_csv_row(data_root, table, row, mask_channel)
            if path is None:
                path = find_channel_file(data_root, sample_id, mask_channel)
            if path is None and REQUIRE_TRAINING_MASKS:
                raise FileNotFoundError(f"样本 {sample_id} 缺少训练 mask 通道 {mask_channel}")
            record[mask_channel] = "" if path is None else str(path)
        mask_path = data_root / str(row["mask"]).strip() if "mask" in table.columns and has_path_value(row.get("mask")) else find_mask_file(data_root, sample_id)
        if mask_path is None:
            raise FileNotFoundError(f"样本 {sample_id} 缺少 mask")
        record["mask"] = str(mask_path)
        rows.append(record)
    return pd.DataFrame(rows)


def scan_samples_auto(data_root: Path) -> pd.DataFrame:
    table = scan_samples_from_csv(data_root)
    if table is not None:
        print("[ok] 使用 samples.csv")
        return table

    sample_ids = set()
    for primary_alias in channel_aliases(INPUT_CHANNELS[0]):
        primary_dir = data_root / "images" / primary_alias
        if primary_dir.exists():
            sample_ids.update(path.stem for path in primary_dir.iterdir() if path.suffix.lower() in IMAGE_EXTS)
    samples_dir = data_root / "samples"
    if samples_dir.exists():
        sample_ids.update(path.name for path in samples_dir.iterdir() if path.is_dir())
    if not sample_ids:
        raise RuntimeError("无法自动发现样本。请使用推荐目录结构或提供 samples.csv。")

    rows = []
    for sample_id in sorted(sample_ids):
        record = {"id": sample_id, "split": ""}
        missing = []
        for channel in INPUT_CHANNELS:
            path = find_channel_file(data_root, sample_id, channel)
            if path is None:
                missing.append(channel)
            else:
                record[channel] = str(path)
        for mask_channel in TRAINING_MASK_CHANNELS:
            path = find_channel_file(data_root, sample_id, mask_channel)
            if path is None:
                if REQUIRE_TRAINING_MASKS:
                    missing.append(mask_channel)
                record[mask_channel] = ""
            else:
                record[mask_channel] = str(path)
        mask_path = find_mask_file(data_root, sample_id)
        if mask_path is None:
            missing.append("mask")
        else:
            record["mask"] = str(mask_path)
        if missing:
            print(f"[skip] {sample_id} 缺少：{missing}")
            continue
        rows.append(record)
    if not rows:
        raise RuntimeError("没有完整样本。")
    return pd.DataFrame(rows)


samples = scan_samples_auto(DATA_ROOT)
print(samples.head())
print(f"[ok] 样本数：{len(samples)}")

# %%
# =========================
# 4. train/val/test 划分
# =========================
if "split" in samples.columns and samples["split"].str.len().gt(0).any():
    train_df = samples[samples["split"].str.lower() == "train"].reset_index(drop=True)
    val_df = samples[samples["split"].str.lower().isin(["val", "valid", "validation"])].reset_index(drop=True)
    test_df = samples[samples["split"].str.lower() == "test"].reset_index(drop=True)
    if len(val_df) == 0:
        train_df, val_df = train_test_split(train_df, test_size=VAL_RATIO, random_state=SEED)
        train_df = train_df.reset_index(drop=True)
        val_df = val_df.reset_index(drop=True)
else:
    train_df, val_df = train_test_split(samples, test_size=VAL_RATIO, random_state=SEED)
    train_df = train_df.reset_index(drop=True)
    val_df = val_df.reset_index(drop=True)
    test_df = pd.DataFrame(columns=samples.columns)

print(f"train={len(train_df)} val={len(val_df)} test={len(test_df)}")

# %%
# =========================
# 5. Dataset / Augmentation
# =========================
import albumentations as A


def resize_image_and_mask(image: np.ndarray, mask: np.ndarray, size: int):
    image_resized = cv2.resize(image, (size, size), interpolation=cv2.INTER_LINEAR)
    mask_resized = cv2.resize(mask, (size, size), interpolation=cv2.INTER_NEAREST)
    return image_resized, mask_resized


def resize_support_mask(mask: np.ndarray, size: int) -> np.ndarray:
    return cv2.resize(np.asarray(mask), (size, size), interpolation=cv2.INTER_NEAREST)


def support_mask_inside(mask: np.ndarray) -> np.ndarray:
    mask = np.asarray(mask, dtype=np.float32)
    finite = np.isfinite(mask)
    return finite & (mask > 0)


def apply_training_support_masks(target: np.ndarray, support_masks: Dict[str, np.ndarray]) -> np.ndarray:
    target = np.asarray(target).copy()
    ignore_mask = np.zeros(target.shape[:2], dtype=bool)
    for _name, support_mask in support_masks.items():
        if support_mask is None:
            continue
        if support_mask.shape[:2] != target.shape[:2]:
            support_mask = cv2.resize(
                np.asarray(support_mask),
                (target.shape[1], target.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        ignore_mask |= ~support_mask_inside(support_mask)
    target[ignore_mask] = IGNORE_INDEX
    return target


train_aug = A.Compose(
    [
        A.HorizontalFlip(p=0.5),
        A.VerticalFlip(p=0.5),
        A.RandomRotate90(p=0.5),
        A.ShiftScaleRotate(
            shift_limit=0.05,
            scale_limit=0.12,
            rotate_limit=8,
            border_mode=cv2.BORDER_CONSTANT,
            value=0,
            mask_value=IGNORE_INDEX,
            p=0.55,
        ),
        A.GaussNoise(var_limit=(0.0005, 0.006), p=0.25),
    ]
)


class RebarMultimodalSegDataset(Dataset):
    def __init__(self, table: pd.DataFrame, image_size: int, training: bool = False):
        self.table = table.reset_index(drop=True)
        self.image_size = int(image_size)
        self.training = bool(training)

    def __len__(self):
        return len(self.table)

    def load_sample(self, index: int) -> Tuple[np.ndarray, np.ndarray, str]:
        row = self.table.iloc[index]
        channels = []
        for channel in INPUT_CHANNELS:
            image = read_scalar_image(Path(row[channel]))
            channels.append(robust_normalize(image))
        image = np.stack(channels, axis=-1).astype(np.float32)
        mask = read_mask(Path(row["mask"]))
        image, mask = resize_image_and_mask(image, mask, self.image_size)
        support_masks = {}
        for mask_channel in TRAINING_MASK_CHANNELS:
            mask_path = str(row.get(mask_channel, "") or "").strip()
            if mask_path:
                support_masks[mask_channel] = resize_support_mask(
                    read_scalar_image(Path(mask_path)),
                    self.image_size,
                )
        mask = apply_training_support_masks(mask, support_masks)
        return image, mask, str(row["id"])

    def __getitem__(self, index: int):
        image, mask, sample_id = self.load_sample(index)
        if self.training:
            augmented = train_aug(image=image, mask=mask)
            image = augmented["image"]
            mask = augmented["mask"]
        image = np.transpose(image, (2, 0, 1)).astype(np.float32)
        image_tensor = torch.from_numpy(image)
        mask_tensor = torch.from_numpy(mask.astype(np.int64))
        return image_tensor, mask_tensor, sample_id


train_ds = RebarMultimodalSegDataset(train_df, IMAGE_SIZE, training=True)
val_ds = RebarMultimodalSegDataset(val_df, IMAGE_SIZE, training=False)
train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True, num_workers=NUM_WORKERS, pin_memory=True)
val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False, num_workers=NUM_WORKERS, pin_memory=True)

images, masks, ids = next(iter(train_loader))
print(images.shape, masks.shape, ids[:2])

# %%
# =========================
# 6. 可视化一批训练样本
# =========================
PALETTE = np.array(
    [
        [20, 20, 20],       # background
        [40, 220, 150],     # ordinary_rebar
        [40, 60, 230],      # beam_rebar
        [255, 180, 40],     # floor_seam
        [220, 40, 180],     # occluder
    ],
    dtype=np.uint8,
)


def mask_to_rgb(mask: np.ndarray) -> np.ndarray:
    mask = np.asarray(mask)
    rgb = np.zeros((*mask.shape[:2], 3), dtype=np.uint8)
    valid = (mask >= 0) & (mask < len(PALETTE))
    rgb[valid] = PALETTE[mask[valid]]
    rgb[mask == IGNORE_INDEX] = [128, 128, 128]
    return rgb


def channel_index(channel_name: str) -> int:
    return INPUT_CHANNELS.index(channel_name)


def show_sample_batch(batch, max_items=4):
    images, masks, ids = batch
    n = min(max_items, images.shape[0])
    preview_channels = [
        "ir",
        "depth_z",
        "combined_response",
        "fused_instance_response",
    ]
    fig, axes = plt.subplots(n, len(preview_channels) + 1, figsize=(18, 4 * n))
    if n == 1:
        axes = axes[None, :]
    for i in range(n):
        image = images[i].numpy()
        mask = masks[i].numpy()
        for column_index, channel_name in enumerate(preview_channels):
            cmap = "gray" if channel_name == "ir" else "turbo"
            axes[i, column_index].imshow(image[channel_index(channel_name)], cmap=cmap)
            title = f"{ids[i]} / {channel_name}" if column_index == 0 else channel_name
            axes[i, column_index].set_title(title)
        axes[i, len(preview_channels)].imshow(mask_to_rgb(mask))
        axes[i, len(preview_channels)].set_title("mask")
        for ax in axes[i]:
            ax.axis("off")
    plt.tight_layout()
    plt.show()


show_sample_batch(next(iter(train_loader)))

# %%
# =========================
# 7. 模型、loss、metrics
# =========================
import segmentation_models_pytorch as smp


def build_model(config: Dict) -> nn.Module:
    arch = config["arch"]
    kwargs = dict(
        encoder_name=config.get("encoder_name", "efficientnet-b0"),
        encoder_weights=config.get("encoder_weights", None),
        in_channels=len(INPUT_CHANNELS),
        classes=NUM_CLASSES,
        activation=None,
    )
    if arch == "UnetPlusPlus":
        return smp.UnetPlusPlus(**kwargs)
    if arch == "Unet":
        return smp.Unet(**kwargs)
    if arch == "DeepLabV3Plus":
        return smp.DeepLabV3Plus(**kwargs)
    raise ValueError(f"Unsupported arch: {arch}")


class DiceLoss(nn.Module):
    def __init__(self, num_classes: int, ignore_index: int = 255, eps: float = 1e-6):
        super().__init__()
        self.num_classes = int(num_classes)
        self.ignore_index = int(ignore_index)
        self.eps = float(eps)

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        valid = target != self.ignore_index
        target_clamped = target.clone()
        target_clamped[~valid] = 0
        probs = torch.softmax(logits, dim=1)
        one_hot = F.one_hot(target_clamped, num_classes=self.num_classes).permute(0, 3, 1, 2).float()
        valid = valid.unsqueeze(1).float()
        probs = probs * valid
        one_hot = one_hot * valid
        dims = (0, 2, 3)
        intersection = torch.sum(probs * one_hot, dims)
        cardinality = torch.sum(probs + one_hot, dims)
        dice = (2.0 * intersection + self.eps) / (cardinality + self.eps)
        return 1.0 - dice.mean()


class SegLoss(nn.Module):
    def __init__(self):
        super().__init__()
        self.ce = nn.CrossEntropyLoss(ignore_index=IGNORE_INDEX)
        self.dice = DiceLoss(NUM_CLASSES, IGNORE_INDEX)

    def forward(self, logits, target):
        return self.ce(logits, target) + self.dice(logits, target)


@torch.no_grad()
def compute_confusion_matrix(logits: torch.Tensor, target: torch.Tensor, num_classes: int):
    pred = torch.argmax(logits, dim=1).detach().cpu().numpy().reshape(-1)
    truth = target.detach().cpu().numpy().reshape(-1)
    valid = truth != IGNORE_INDEX
    pred = pred[valid]
    truth = truth[valid]
    valid_class = (truth >= 0) & (truth < num_classes) & (pred >= 0) & (pred < num_classes)
    encoded = (truth[valid_class] * num_classes) + pred[valid_class]
    cm = np.bincount(encoded, minlength=num_classes * num_classes)
    return cm.reshape(num_classes, num_classes).astype(np.int64)


def metrics_from_confusion(cm: np.ndarray) -> Dict:
    tp = np.diag(cm).astype(np.float64)
    fp = cm.sum(axis=0) - tp
    fn = cm.sum(axis=1) - tp
    precision = tp / np.maximum(tp + fp, 1.0)
    recall = tp / np.maximum(tp + fn, 1.0)
    iou = tp / np.maximum(tp + fp + fn, 1.0)
    return {
        "precision": precision.tolist(),
        "recall": recall.tolist(),
        "iou": iou.tolist(),
        "miou": float(np.mean(iou)),
        "ordinary_rebar_recall": float(recall[1]) if len(recall) > 1 else 0.0,
        "ordinary_rebar_precision": float(precision[1]) if len(precision) > 1 else 0.0,
        "beam_rebar_iou": float(iou[2]) if len(iou) > 2 else 0.0,
    }

# %%
# =========================
# 8. 训练 / 验证
# =========================
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("device:", device)


def train_one_epoch(model, loader, optimizer, scaler, loss_fn):
    model.train()
    total_loss = 0.0
    progress = tqdm(loader, desc="train", leave=False)
    for images, masks, _ids in progress:
        images = images.to(device, non_blocking=True)
        masks = masks.to(device, non_blocking=True)
        optimizer.zero_grad(set_to_none=True)
        with torch.cuda.amp.autocast(enabled=AMP and device.type == "cuda"):
            logits = model(images)
            loss = loss_fn(logits, masks)
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
        scaler.step(optimizer)
        scaler.update()
        total_loss += float(loss.item()) * images.size(0)
        progress.set_postfix(loss=float(loss.item()))
    return total_loss / max(1, len(loader.dataset))


@torch.no_grad()
def validate(model, loader, loss_fn):
    model.eval()
    total_loss = 0.0
    cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=np.int64)
    for images, masks, _ids in tqdm(loader, desc="val", leave=False):
        images = images.to(device, non_blocking=True)
        masks = masks.to(device, non_blocking=True)
        logits = model(images)
        loss = loss_fn(logits, masks)
        total_loss += float(loss.item()) * images.size(0)
        cm += compute_confusion_matrix(logits, masks, NUM_CLASSES)
    metrics = metrics_from_confusion(cm)
    metrics["loss"] = total_loss / max(1, len(loader.dataset))
    return metrics


def run_experiment(config: Dict) -> Dict:
    exp_name = config["name"]
    exp_dir = OUTPUT_DIR / exp_name
    exp_dir.mkdir(parents=True, exist_ok=True)

    model = build_model(config).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=EPOCHS)
    scaler = torch.cuda.amp.GradScaler(enabled=AMP and device.type == "cuda")
    loss_fn = SegLoss()

    best_miou = -1.0
    history = []
    best_path = exp_dir / "best_model.pt"
    last_path = exp_dir / "last_model.pt"

    for epoch in range(1, EPOCHS + 1):
        train_loss = train_one_epoch(model, train_loader, optimizer, scaler, loss_fn)
        val_metrics = validate(model, val_loader, loss_fn)
        scheduler.step()
        record = {
            "epoch": epoch,
            "train_loss": train_loss,
            **val_metrics,
            "lr": float(optimizer.param_groups[0]["lr"]),
        }
        history.append(record)
        print(
            f"[{exp_name}] epoch={epoch:03d} "
            f"train_loss={train_loss:.4f} val_loss={val_metrics['loss']:.4f} "
            f"mIoU={val_metrics['miou']:.4f} "
            f"ordinary_recall={val_metrics['ordinary_rebar_recall']:.4f} "
            f"ordinary_precision={val_metrics['ordinary_rebar_precision']:.4f}"
        )
        torch.save({"model": model.state_dict(), "config": config, "channels": INPUT_CHANNELS, "classes": CLASS_NAMES}, last_path)
        if val_metrics["miou"] > best_miou:
            best_miou = val_metrics["miou"]
            torch.save({"model": model.state_dict(), "config": config, "channels": INPUT_CHANNELS, "classes": CLASS_NAMES}, best_path)

    pd.DataFrame(history).to_csv(exp_dir / "history.csv", index=False)
    return {"name": exp_name, "best_miou": best_miou, "best_path": str(best_path), "history": history}


experiment_results = []
for config in EXPERIMENTS:
    experiment_results.append(run_experiment(config))

print(json.dumps(experiment_results, indent=2, ensure_ascii=False))

# %%
# =========================
# 9. 可视化预测
# =========================
def load_best_model(result: Dict):
    checkpoint = torch.load(result["best_path"], map_location=device)
    model = build_model(checkpoint["config"]).to(device)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    return model, checkpoint


@torch.no_grad()
def save_prediction_grid(model, dataset, output_path: Path, max_items=8):
    n = min(max_items, len(dataset))
    fig, axes = plt.subplots(n, 5, figsize=(18, 4 * n))
    if n == 1:
        axes = axes[None, :]
    for i in range(n):
        image, mask, sample_id = dataset[i]
        logits = model(image[None].to(device))
        pred = torch.argmax(logits, dim=1)[0].cpu().numpy().astype(np.uint8)
        image_np = image.numpy()
        axes[i, 0].imshow(image_np[0], cmap="gray")
        axes[i, 0].set_title(f"{sample_id} / ir")
        axes[i, 1].imshow(image_np[channel_index("combined_response")], cmap="turbo")
        axes[i, 1].set_title("combined")
        axes[i, 2].imshow(mask_to_rgb(mask.numpy()))
        axes[i, 2].set_title("gt")
        axes[i, 3].imshow(mask_to_rgb(pred))
        axes[i, 3].set_title("pred")
        overlay = np.repeat(image_np[0][..., None], 3, axis=2)
        overlay = (overlay * 255).astype(np.uint8)
        overlay = cv2.addWeighted(overlay, 0.58, mask_to_rgb(pred), 0.42, 0)
        axes[i, 4].imshow(overlay)
        axes[i, 4].set_title("overlay")
        for ax in axes[i]:
            ax.axis("off")
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=140)
    plt.show()


best_result = sorted(experiment_results, key=lambda item: item["best_miou"], reverse=True)[0]
best_model, best_checkpoint = load_best_model(best_result)
save_prediction_grid(best_model, val_ds, OUTPUT_DIR / best_result["name"] / "val_predictions.png")

# %%
# =========================
# 10. 导出 ONNX 并做单帧耗时测试
# =========================
def export_onnx(model: nn.Module, output_path: Path):
    model.eval()
    dummy = torch.randn(1, len(INPUT_CHANNELS), IMAGE_SIZE, IMAGE_SIZE, device=device)
    torch.onnx.export(
        model,
        dummy,
        output_path,
        input_names=["input"],
        output_names=["logits"],
        opset_version=17,
        dynamic_axes={
            "input": {0: "batch", 2: "height", 3: "width"},
            "logits": {0: "batch", 2: "height", 3: "width"},
        },
    )
    print(f"[ok] ONNX exported: {output_path}")


@torch.no_grad()
def benchmark_torch(model: nn.Module, repeats=80, warmup=15):
    model.eval()
    x = torch.randn(1, len(INPUT_CHANNELS), IMAGE_SIZE, IMAGE_SIZE, device=device)
    for _ in range(warmup):
        _ = model(x)
    if device.type == "cuda":
        torch.cuda.synchronize()
    start = time.perf_counter()
    for _ in range(repeats):
        _ = model(x)
    if device.type == "cuda":
        torch.cuda.synchronize()
    elapsed_ms = (time.perf_counter() - start) * 1000.0 / repeats
    return elapsed_ms


best_exp_dir = OUTPUT_DIR / best_result["name"]
onnx_path = best_exp_dir / "best_model.onnx"
export_onnx(best_model, onnx_path)
torch_ms = benchmark_torch(best_model)
print(f"[benchmark] torch single-frame: {torch_ms:.2f} ms")

try:
    import onnxruntime as ort

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if device.type == "cuda" else ["CPUExecutionProvider"]
    session = ort.InferenceSession(str(onnx_path), providers=providers)
    x_np = np.random.randn(1, len(INPUT_CHANNELS), IMAGE_SIZE, IMAGE_SIZE).astype(np.float32)
    for _ in range(10):
        _ = session.run(None, {"input": x_np})
    start = time.perf_counter()
    for _ in range(50):
        _ = session.run(None, {"input": x_np})
    ort_ms = (time.perf_counter() - start) * 1000.0 / 50
    print(f"[benchmark] onnxruntime single-frame: {ort_ms:.2f} ms")
except Exception as exc:
    ort_ms = None
    print(f"[warn] ONNXRuntime benchmark failed: {exc}")

# %%
# =========================
# 11. 写训练摘要并打包回 Google Drive
# =========================
summary = {
    "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    "dataset_root": str(DATA_ROOT),
    "num_samples": int(len(samples)),
    "train": int(len(train_df)),
    "val": int(len(val_df)),
    "test": int(len(test_df)),
    "input_channels": INPUT_CHANNELS,
    "training_mask_channels": TRAINING_MASK_CHANNELS,
    "require_training_masks": REQUIRE_TRAINING_MASKS,
    "channel_aliases": CHANNEL_ALIASES,
    "class_names": CLASS_NAMES,
    "ignore_index": IGNORE_INDEX,
    "image_size": IMAGE_SIZE,
    "batch_size": BATCH_SIZE,
    "epochs": EPOCHS,
    "experiments": experiment_results,
    "best_experiment": best_result,
    "torch_single_frame_ms": float(torch_ms),
    "onnxruntime_single_frame_ms": None if ort_ms is None else float(ort_ms),
    "next_runtime_path": "分割输出 ordinary_rebar / beam_rebar 后，接 skeleton/graph 找普通钢筋交点，再做梁筋 +/-13cm 点级过滤。",
}
(OUTPUT_DIR / "training_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
print(json.dumps(summary, indent=2, ensure_ascii=False))

archive_path = shutil.make_archive(str(WORK_DIR / "pr_fprg_multimodal_seg_outputs"), "zip", OUTPUT_DIR)
print(f"[ok] output archive: {archive_path}")

if SAVE_TO_DRIVE:
    drive_output = Path(DRIVE_OUTPUT_DIR)
    drive_output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(archive_path, drive_output / Path(archive_path).name)
    shutil.copy2(OUTPUT_DIR / "training_summary.json", drive_output / "training_summary.json")
    print(f"[ok] copied outputs to: {drive_output}")

# %% [markdown]
# ## 训练完成后怎么接机器人主链
#
# 1. 把 `best_model.onnx` 放回机器人工作站。
# 2. 运行时构造同样的 9 通道输入：
#    - `ir`
#    - `depth_z`
#    - `worldcoord_height_response`
#    - `depth_response`
#    - `depth_gradient`
#    - `combined_response`
#    - `hessian_ridge`
#    - `frangi_like`
#    - `fused_instance_response`
# 3. 模型输出语义 mask：
#    - `ordinary_rebar`
#    - `beam_rebar`
#    - `floor_seam`
#    - `occluder`
# 4. 对 `ordinary_rebar` 做 skeleton / graph。
# 5. 找普通钢筋与普通钢筋交点。
# 6. 去掉 `beam_rebar` 本体和 +/-13cm 范围内的点。
# 7. `valid_mask` / `workspace_mask` 继续作为运行时 ROI/ignore 区域约束，避免工作区外干扰进入图谱。
# 8. 用 PR-FPRG-MS 的频相拓扑做几何一致性校验和兜底。
