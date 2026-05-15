"""Draw the current camera-visible area directly on backend IR frames."""

import cv2
import numpy as np


def build_live_visible_area_mask(world_coord_image):
    """Return a uint8 mask for pixels with valid current-frame 3D data."""
    if world_coord_image is None:
        return None

    source = np.asarray(world_coord_image)
    if source.ndim == 3 and source.shape[2] >= 3:
        x_channel = source[:, :, 0].astype(np.float32, copy=False)
        y_channel = source[:, :, 1].astype(np.float32, copy=False)
        z_channel = source[:, :, 2].astype(np.float32, copy=False)
        valid_mask = (
            np.isfinite(x_channel)
            & np.isfinite(y_channel)
            & np.isfinite(z_channel)
            & (z_channel > 0.0)
        )
    elif source.ndim == 2:
        z_channel = source.astype(np.float32, copy=False)
        valid_mask = np.isfinite(z_channel) & (z_channel > 0.0)
    else:
        return None

    return (valid_mask.astype(np.uint8) * 255)


def _resize_mask_to_image(mask, image):
    image_height, image_width = image.shape[:2]
    if mask.shape[:2] == (image_height, image_width):
        return mask
    return cv2.resize(mask, (image_width, image_height), interpolation=cv2.INTER_NEAREST)


def _largest_external_contour(mask, min_area_px=16.0):
    contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < float(min_area_px):
        return None
    return largest_contour


def draw_live_visible_area_boundary(
    result_image,
    world_coord_image,
    gray_value=180,
    thickness=2,
):
    """Draw the current valid raw-world image boundary onto result_image."""
    if result_image is None:
        return False

    valid_mask = build_live_visible_area_mask(world_coord_image)
    if valid_mask is None:
        return False

    valid_mask = _resize_mask_to_image(valid_mask, result_image)
    contour = _largest_external_contour(valid_mask)
    if contour is None:
        return False

    gray = int(np.clip(gray_value, 0, 255))
    color = gray if result_image.ndim == 2 else (gray, gray, gray)
    cv2.drawContours(
        result_image,
        [contour],
        -1,
        color,
        max(1, int(thickness)),
        lineType=cv2.LINE_8,
    )
    return True
