import base64
import re
from PIL import Image

svg_path = r"d:\DATN\FLY\report\Images\altium_logo.svg"
png_path = r"d:\DATN\FLY\report\Images\altium_logo.png"

try:
    with open(svg_path, 'r') as f:
        content = f.read()

    # Find the base64 string
    match = re.search(r'xlink:href="data:image/png;base64,([^"]+)"', content)
    if match:
        b64_data = match.group(1)
        with open(png_path, 'wb') as f:
            f.write(base64.b64decode(b64_data))
        print("Extracted successfully!")
        
        # Add white background just in case it's transparent
        print("Adding white background...")
        img = Image.open(png_path).convert("RGBA")
        bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
        bg.paste(img, (0, 0), img)
        bg.convert("RGB").save(png_path, "PNG")
        print("Done converting background.")
    else:
        print("No embedded PNG found.")
except Exception as e:
    print(f"Error: {e}")
