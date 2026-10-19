#!/usr/bin/env python3
"""Upload the GIFData frames in ikun.txt to the board over USART1."""

from __future__ import annotations

import argparse
import re
import struct
import sys
import time
import zlib
from pathlib import Path
from typing import Any

FRAME_COUNT = 744
FRAME_SIZE = 1024
CHUNK_SIZE = 256
BAUD_RATE = 115200
READY = b"RDY!"
DATA = b"DATA"
ACK = b"\x06"
PASS = b"PASS"


def parse_video_source(path: Path) -> bytes:
    text = path.read_text(encoding="ascii")
    declaration = re.search(
        r"\bGIFData\s*\[\s*(\d+)\s*\]\s*\[\s*(\d+)\s*\]\s*=", text
    )
    if declaration is None:
        raise ValueError("Missing GIFData[frame_count][frame_size] declaration")

    frame_count, frame_size = map(int, declaration.groups())
    if (frame_count, frame_size) != (FRAME_COUNT, FRAME_SIZE):
        raise ValueError(
            f"Expected GIFData[{FRAME_COUNT}][{FRAME_SIZE}], "
            f"found GIFData[{frame_count}][{frame_size}]"
        )

    payload = text[declaration.end() :]
    frames = bytes(
        int(value, 16)
        for value in re.findall(r"0x([0-9A-Fa-f]{1,2})", payload)
    )
    expected_length = FRAME_COUNT * FRAME_SIZE
    if len(frames) != expected_length:
        raise ValueError(f"Expected {expected_length} frame bytes, found {len(frames)}")
    return frames


def read_exact(port: Any, length: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    result = bytearray()
    while len(result) < length and time.monotonic() < deadline:
        result.extend(port.read(length - len(result)))
    if len(result) != length:
        raise TimeoutError(f"Timed out waiting for {length} serial bytes")
    return bytes(result)


def upload(port_name: str, frames: bytes) -> None:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required; install it with: python -m pip install pyserial"
        ) from exc

    checksum = zlib.crc32(frames) & 0xFFFFFFFF
    with serial.Serial(port_name, BAUD_RATE, timeout=0.1, write_timeout=5) as port:
        time.sleep(1.0)
        port.reset_input_buffer()
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            port.write(b"IKUP")
            port.flush()
            try:
                response = read_exact(port, 4, 1.0)
            except TimeoutError:
                continue
            if response == READY:
                break
            raise RuntimeError(f"Unexpected board response while connecting: {response!r}")
        else:
            raise TimeoutError("No board response; reset the MCU and retry")

        port.write(struct.pack("<II", len(frames), checksum))
        port.flush()
        response = read_exact(port, 4, 120.0)
        if response != DATA:
            raise RuntimeError(f"Board refused upload metadata: {response!r}")

        total_chunks = len(frames) // CHUNK_SIZE
        for chunk_index in range(total_chunks):
            start = chunk_index * CHUNK_SIZE
            port.write(frames[start : start + CHUNK_SIZE])
            port.flush()
            response = read_exact(port, 1, 5.0)
            if response != ACK:
                raise RuntimeError(
                    f"Board rejected chunk {chunk_index + 1}/{total_chunks}: {response!r}"
                )
            if (chunk_index + 1) % 64 == 0 or chunk_index + 1 == total_chunks:
                print(f"Uploaded {chunk_index + 1}/{total_chunks} pages")

        response = read_exact(port, 4, 20.0)
        if response != PASS:
            raise RuntimeError(f"Board failed flash verification: {response!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, for example COM3")
    parser.add_argument(
        "--file",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "ikun.txt",
        help="source text file (default: repository-root ikun.txt)",
    )
    args = parser.parse_args()

    try:
        frames = parse_video_source(args.file)
        print(f"Validated {FRAME_COUNT} frames ({len(frames)} bytes)")
        upload(args.port, frames)
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"Upload failed: {exc}", file=sys.stderr)
        return 1

    print("Upload verified; playback starts on the board.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
