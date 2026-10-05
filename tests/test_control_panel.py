"""Test control panel camera encoding and batch frame processing."""

from __future__ import annotations

import io

import numpy as np
from PIL import Image

from mh3_genesis.control_panel import MH3ControlPanel


def test_encode_rgb() -> None:
    """Verify RGB image encoding produces a valid JPEG payload.
    :return: None when encoded bytes deserialize into an RGB image.
    :rtype: None
    """
    # Create a synthetic 8-bit RGB test pattern.
    raw_rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    raw_rgb[:, :, 0] = 120
    raw_rgb[:, :, 1] = 200
    # Compress RGB array to JPEG image bytes.
    encoded_jpeg = MH3ControlPanel._encode_rgb(raw_rgb, format_name="jpeg", quality=75)
    # Confirm JPEG header magic bytes.
    assert encoded_jpeg.startswith(b"\xff\xd8\xff")
    image_jpeg = Image.open(io.BytesIO(encoded_jpeg))
    assert image_jpeg.format == "JPEG"
    assert image_jpeg.size == (640, 480)

    # Compress RGB array to PNG image bytes.
    encoded_png = MH3ControlPanel._encode_rgb(raw_rgb, format_name="png")
    assert encoded_png.startswith(b"\x89PNG\r\n\x1a\n")
    image_png = Image.open(io.BytesIO(encoded_png))
    assert image_png.format == "PNG"
    assert image_png.size == (640, 480)


def test_encode_depth() -> None:
    """Verify metric depth encoding produces a valid grayscale JPEG or PNG preview.
    :return: None when depth array transforms and encodes into requested image formats.
    :rtype: None
    """
    # Create a synthetic floating-point metric depth array containing non-finite and distant values.
    raw_depth = np.full((480, 640), 2.5, dtype=np.float32)
    raw_depth[0, 0] = np.nan
    raw_depth[0, 1] = np.inf
    raw_depth[0, 2] = -1.0
    raw_depth[0, 3] = 10.0
    # Compress metric depth buffer to JPEG image bytes.
    encoded_jpeg = MH3ControlPanel._encode_depth(raw_depth, near_m=0.1, far_m=5.0, format_name="jpeg", quality=85)
    # Confirm JPEG header magic bytes.
    assert encoded_jpeg.startswith(b"\xff\xd8\xff")
    image_jpeg = Image.open(io.BytesIO(encoded_jpeg))
    assert image_jpeg.format == "JPEG"
    assert image_jpeg.size == (640, 480)

    # Compress metric depth buffer to PNG image bytes.
    encoded_png = MH3ControlPanel._encode_depth(raw_depth, near_m=0.1, far_m=5.0, format_name="png")
    assert encoded_png.startswith(b"\x89PNG\r\n\x1a\n")
    image_png = Image.open(io.BytesIO(encoded_png))
    assert image_png.format == "PNG"
    assert image_png.size == (640, 480)


def test_all_camera_frames() -> None:
    """Verify all_camera_frames packages cached frame buffers into data URLs with proper MIME types.
    :return: None when frame dictionary is formatted as base64 data URLs.
    :rtype: None
    """
    import threading
    import time

    # Create an uninitialized panel instance to test frame batching in isolation.
    panel = MH3ControlPanel.__new__(MH3ControlPanel)
    panel._lock = threading.Lock()
    panel._last_camera_request_at = 0.0
    panel._camera_frames = {
        "head-left-rgb": b"\xff\xd8\xff\xe0test_jpeg",
        "head-depth": b"\x89PNG\r\n\x1a\ntest_png",
    }
    # Retrieve formatted frame payload mapping.
    payload = panel.all_camera_frames()
    # Confirm timestamp was refreshed and data URLs were correctly constructed with respective MIME types.
    assert time.monotonic() - panel._last_camera_request_at < 1.0
    assert payload["head-left-rgb"].startswith("data:image/jpeg;base64,")
    assert payload["head-depth"].startswith("data:image/png;base64,")
