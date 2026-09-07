#!/usr/bin/env python3
"""Generate synthetic sequences for smoke testing ORB-SLAM2.

There is no test suite and the TUM/KITTI datasets are large downloads, so this
builds sequences from scratch: a random-dot plane that the camera pans across at a
fixed rate. That is enough texture for ORB to find corners and for all three sensor
modes to initialise and track.

The point is that the motion is known, so the result can be checked rather than
just watched for a crash. Stereo and RGB-D recover true metric scale, so the
expected translation is printed in metres; monocular has no scale, so only the
direction ratio is meaningful.

    tools/make_synth_seq.py mono   /tmp/mono
    tools/make_synth_seq.py stereo /tmp/stereo
    tools/make_synth_seq.py rgbd   /tmp/rgbd

Use tools/smoke_test.sh to generate, run and check all three in one go.
"""
import os
import random
import struct
import sys
import zlib


def png(path, w, h, rows, bitdepth=8):
    """Minimal 8/16-bit greyscale PNG writer (zlib is stdlib; OpenCV reads these)."""
    raw = b"".join(b"\x00" + bytes(r) for r in rows)

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xffffffff)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, bitdepth, 0, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6))
                + chunk(b"IEND", b""))


def canvas(w, h, margin, dots, seed):
    """A plane of random 3x3 dots, large enough to pan a w*h window over."""
    random.seed(seed)
    cw, ch = w + 2 * margin, h + 2 * margin
    buf = bytearray(b"\x80" * (cw * ch))
    for _ in range(dots):
        x, y = random.randrange(2, cw - 2), random.randrange(2, ch - 2)
        v = random.choice((0, 255))
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                buf[(y + dy) * cw + (x + dx)] = v
    return buf, cw


def window(buf, cw, ox, oy, w, h):
    return [buf[(oy + r) * cw + ox:(oy + r) * cw + ox + w] for r in range(h)]


def mono(out):
    """TUM monocular. Pans diagonally so the trajectory's x:y ratio is checkable."""
    W, H, N, DX, DY = 640, 480, 80, 1.2, 0.6
    margin = 120
    buf, cw = canvas(W, H, margin, 60000, seed=7)
    os.makedirs(os.path.join(out, "rgb"), exist_ok=True)
    lines = []
    for i in range(N):
        ox, oy = margin + int(round(DX * i)), margin + int(round(DY * i))
        name = "rgb/%04d.png" % i
        png(os.path.join(out, name), W, H, window(buf, cw, ox, oy, W, H))
        lines.append("%.6f %s" % (1305031102.175304 + i / 30.0, name))
    with open(os.path.join(out, "rgb.txt"), "w") as f:
        f.write("# color images\n# synthetic\n# timestamp filename\n" + "\n".join(lines) + "\n")
    print("mono: %d frames -> %s" % (N, out))
    print("expect trajectory x:y ratio ~ %.2f" % (DX / DY))


def stereo(out):
    """KITTI stereo. Constant disparity puts the plane at a known depth."""
    W, H, N = 1241, 376, 50                      # Examples/Stereo/KITTI00-02.yaml
    FX, BF, DEPTH, DX = 718.856, 386.1448, 10.0, 2.0
    disp = int(round(BF / DEPTH))                # right image is the left shifted by this
    margin = 200 + disp + int(DX * N)
    buf, cw = canvas(W, H, margin, 400000, seed=11)
    os.makedirs(os.path.join(out, "image_0"), exist_ok=True)
    os.makedirs(os.path.join(out, "image_1"), exist_ok=True)
    times = []
    for i in range(N):
        ox, oy = margin + int(round(DX * i)), margin
        png("%s/image_0/%06d.png" % (out, i), W, H, window(buf, cw, ox, oy, W, H))
        png("%s/image_1/%06d.png" % (out, i), W, H, window(buf, cw, ox + disp, oy, W, H))
        times.append("%.6e" % (i / 10.0))
    with open(os.path.join(out, "times.txt"), "w") as f:
        f.write("\n".join(times) + "\n")
    shift = int(round(DX * (N - 1)))
    print("stereo: %d pairs -> %s (disparity %d px = %.1f m)" % (N, out, disp, DEPTH))
    print("expect translation %.4f m" % (shift * DEPTH / FX))


def rgbd(out):
    """TUM RGB-D. Depth is a constant fronto-parallel plane, scaled by DepthMapFactor."""
    W, H, N = 640, 480, 50                       # Examples/RGB-D/TUM1.yaml
    FX, FACTOR, DEPTH, DX = 517.306408, 5000.0, 2.0, 1.5
    margin = 150 + int(DX * N)
    buf, cw = canvas(W, H, margin, 150000, seed=23)
    os.makedirs(os.path.join(out, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(out, "depth"), exist_ok=True)
    drow = struct.pack(">H", int(round(DEPTH * FACTOR))) * W
    assoc = []
    for i in range(N):
        ox = margin + int(round(DX * i))
        png("%s/rgb/%04d.png" % (out, i), W, H, window(buf, cw, ox, margin, W, H))
        png("%s/depth/%04d.png" % (out, i), W, H, [drow] * H, bitdepth=16)
        t = 1305031102.175304 + i / 30.0
        assoc.append("%.6f rgb/%04d.png %.6f depth/%04d.png" % (t, i, t, i))
    with open(os.path.join(out, "associations.txt"), "w") as f:
        f.write("\n".join(assoc) + "\n")
    shift = int(round(DX * (N - 1)))
    print("rgbd: %d rgb+depth pairs -> %s (plane at %.1f m)" % (N, out, DEPTH))
    print("expect translation %.4f m" % (shift * DEPTH / FX))


MODES = {"mono": mono, "stereo": stereo, "rgbd": rgbd}

if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in MODES:
        sys.exit("usage: make_synth_seq.py {%s} <out_dir>" % "|".join(MODES))
    MODES[sys.argv[1]](sys.argv[2])
