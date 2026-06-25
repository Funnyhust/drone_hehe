import urllib.request
import ssl
import fitz
from PIL import Image
import os

url = "https://upload.wikimedia.org/wikipedia/en/c/c2/Altium_Designer_Logo.svg"
svg_path = r"d:\DATN\FLY\report\Images\wiki_altium.svg"
png_path = r"d:\DATN\FLY\report\Images\altium_logo.png"

try:
    context = ssl._create_unverified_context()
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req, context=context) as response:
        with open(svg_path, "wb") as f:
            f.write(response.read())
    
    # Convert SVG to PNG
    doc = fitz.open(svg_path)
    page = doc[0]
    pix = page.get_pixmap(dpi=300, alpha=True)
    temp_png = r"d:\DATN\FLY\report\Images\temp_wiki.png"
    pix.save(temp_png)

    # Add white background
    img = Image.open(temp_png).convert("RGBA")
    bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
    # Paste using the image's own alpha channel as a mask
    bg.paste(img, (0, 0), img)
    bg.convert("RGB").save(png_path, "PNG")
    print("Success")
except Exception as e:
    print(f"Error: {e}")
