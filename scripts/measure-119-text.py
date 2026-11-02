import numpy as np
from PIL import Image

im = Image.open("/mnt/e/workspace/Pictures/119.jpg").convert("L")
a = np.asarray(im).astype(np.int16)
H, W = a.shape
mask = a > 180

# horizontal profile over the whole lower 2/3 to find text rows
prof = mask[300:1650, 20:1250].sum(axis=1)
rows = []
inrow = False
for i, v in enumerate(prof):
    if v > 8 and not inrow:
        start = i
        inrow = True
    elif v <= 8 and inrow:
        rows.append((300 + start, 300 + i - 1, int(prof[start:i].max())))
        inrow = False
if inrow:
    rows.append((300 + start, 300 + len(prof) - 1, int(prof[start:].max())))
print("text row bands (y0,y1,max):")
for r in rows:
    print(r)

print()
print("per-band x extent and best pitch:")
for y0, y1, _ in rows:
    band = mask[y0:y1 + 1, :]
    cols = band.any(axis=0)
    xs = np.nonzero(cols)[0]
    if len(xs) == 0:
        continue
    x0, x1 = xs[0], xs[-1]
    colprof = band[:, x0:x1 + 1].sum(axis=0).astype(float)
    colprof -= colprof.mean()
    best = (0, 0)
    for lag in range(8, 120):
        s = float((colprof[:-lag] * colprof[lag:]).sum())
        if s > best[0]:
            best = (s, lag)
    print(f"y {y0}-{y1}  x {x0}-{x1}  w={x1-x0+1}  pitch={best[1]}")
