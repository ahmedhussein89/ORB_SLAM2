#!/usr/bin/env python3
"""Generate a synthetic TUM-format monocular sequence.

A smoke test for when no real dataset is available: a random-dot plane translated
at a constant rate, which ORB-SLAM2 can initialise and track. Because the motion is
known, the recovered trajectory can be sanity-checked -- the x:y ratio of the
translation should match DX:DY below.

    python3 tools/make_synth_seq.py /tmp/synthseq
    ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt \
        Examples/Monocular/TUM1.yaml /tmp/synthseq
"""
import os
import random
import sys

W, H, N, MARGIN = 640, 480, 80, 120     # TUM1.yaml is 640x480
DX, DY = 1.2, 0.6                       # px/frame; trajectory x:y should match this


def main(out_dir):
    random.seed(7)
    cw, ch = W + 2 * MARGIN, H + 2 * MARGIN
    canvas = bytearray(b"\x80" * (cw * ch))
    for _ in range(60000):               # 3x3 dots -> plenty of FAST corners
        x, y, v = random.randrange(2, cw - 2), random.randrange(2, ch - 2), random.choice((0, 255))
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                canvas[(y + dy) * cw + (x + dx)] = v

    os.makedirs(os.path.join(out_dir, "rgb"), exist_ok=True)
    lines = []
    for i in range(N):
        ox, oy = MARGIN + int(round(DX * i)), MARGIN + int(round(DY * i))
        name = "rgb/%04d.pgm" % i
        with open(os.path.join(out_dir, name), "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (W, H))
            f.write(b"".join(bytes(canvas[(oy + r) * cw + ox:(oy + r) * cw + ox + W]) for r in range(H)))
        lines.append("%.6f %s" % (1305031102.175304 + i / 30.0, name))

    with open(os.path.join(out_dir, "rgb.txt"), "w") as f:
        f.write("# color images\n# synthetic\n# timestamp filename\n" + "\n".join(lines) + "\n")
    print("wrote %d frames to %s" % (N, out_dir))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "synthseq")
