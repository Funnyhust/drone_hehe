import fitz
from PIL import Image

svg_path = r"d:\DATN\FLY\report\Images\altium_logo.svg"
png_path = r"d:\DATN\FLY\report\Images\altium_logo.png"

try:
    print("Converting SVG to PNG with alpha...")
    doc = fitz.open(svg_path)
    page = doc[0]
    pix = page.get_pixmap(dpi=300, alpha=True)
    pix.save(png_path)

    print("Adding white background...")
    img = Image.open(png_path).convert("RGBA")
    bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
    bg.paste(img, (0, 0), img)
    bg.convert("RGB").save(png_path, "PNG")
    print("Done")
except Exception as e:
    print(f"Error: {e}")
