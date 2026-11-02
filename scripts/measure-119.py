import numpy as np
from PIL import Image

im = Image.open("/mnt/e/workspace/Pictures/119.jpg").convert("L")
a = np.asarray(im).astype(np.int16)
H, W = a.shape
print("photo", W, H)

mask = np.zeros((H, W), dtype=bool)
mask[:700, 650:] = a[:700, 650:] > 200
visited = np.zeros_like(mask)
boxes = []
ys, xs = np.nonzero(mask)
for sy, sx in zip(ys, xs):
    if visited[sy, sx]:
        continue
    stack = [(sy, sx)]
    visited[sy, sx] = True
    minx = maxx = sx
    miny = maxy = sy
    area = 0
    while stack:
        y, x = stack.pop()
        area += 1
        if x < minx: minx = x
        if x > maxx: maxx = x
        if y < miny: miny = y
        if y > maxy: maxy = y
        for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
            if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not visited[ny, nx]:
                visited[ny, nx] = True
                stack.append((ny, nx))
    w, h = maxx - minx + 1, maxy - miny + 1
    if 25 <= w <= 70 and 25 <= h <= 70 and area > 0.75 * w * h:
        boxes.append((minx, miny, w, h, area))
boxes.sort(key=lambda b: (b[1] // 10, b[0]))
print("square blobs:", len(boxes))
for b in boxes:
    print(b)
