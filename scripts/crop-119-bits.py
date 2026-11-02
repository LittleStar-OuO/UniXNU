from PIL import Image

SRC = "/mnt/e/workspace/Pictures/119.jpg"
im = Image.open(SRC)

# d117_bits rows 0..5 live at fb x=8..294, y=900..1124.
# Approx photo mapping (from marker grid): x_p = 134 + 0.635*x_f, y_p = 27 + 0.603*y_f
crops = {
    "119_bits_rows_wide.png": (110, 540, 420, 730, 6),
    "119_bits_rows_zoom.png": (125, 645, 340, 720, 10),
}
for name, (x0, y0, x1, y1, scale) in crops.items():
    c = im.crop((x0, y0, x1, y1))
    c = c.resize((c.width * scale, c.height * scale), Image.LANCZOS)
    c.save("/mnt/e/workspace/Pictures/" + name)
    print(name, c.size)
