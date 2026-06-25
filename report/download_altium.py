import urllib.request
import ssl
from svglib.svglib import svg2rlg
from reportlab.graphics import renderPM

url = "https://xplm.com/wp-content/uploads/2025/09/Altium_Designer_Standard_21.svg"
svg_path = "d:\\DATN\\FLY\\report\\Images\\altium_logo.svg"
png_path = "d:\\DATN\\FLY\\report\\Images\\altium_logo.png"

try:
    print("Downloading SVG...")
    context = ssl._create_unverified_context()
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'})
    with urllib.request.urlopen(req, context=context) as response, open(svg_path, 'wb') as out_file:
        out_file.write(response.read())
    print("Converting SVG to PNG...")
    drawing = svg2rlg(svg_path)
    renderPM.drawToFile(drawing, png_path, fmt="PNG", dpi=300)
    print("Success")
except Exception as e:
    print(f"Error: {e}")
