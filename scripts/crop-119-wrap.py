from PIL import Image, ImageDraw

SRC = "/mnt/e/workspace/Pictures/119.jpg"
im = Image.open(SRC).convert("RGB")

def band(x0, y0, x1, y1, scale, name, grid=50):
    c = im.crop((x0, y0, x1, y1)).resize(((x1 - x0) * scale, (y1 - y0) * scale), Image.LANCZOS)
    d = ImageDraw.Draw(c)
    for gx in range(0, x1 - x0 + 1, grid):
        d.line([(gx * scale, 0), (gx * scale, c.height)], fill=(0, 128, 255), width=1)
    for gy in range(0, y1 - y0 + 1, 50):
        d.line([(0, gy * scale), (c.width, gy * scale)], fill=(0, 128, 255), width=1)
    c.save("/mnt/e/workspace/Pictures/" + name)
    print(name, c.size)

band(100, 1155, 800, 1275, 4, "119_wrap_zoom.png", 50)
band(100, 1395, 800, 1515, 4, "119_tail_zoom.png", 50)
