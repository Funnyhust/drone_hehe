from PIL import Image
import base64
import re

svg_path = r"d:\DATN\FLY\report\Images\altium_logo.svg"
png_path = r"d:\DATN\FLY\report\Images\altium_logo.png"

with open(svg_path, 'r') as f:
    content = f.read()

match = re.search(r'xlink:href="data:image/png;base64,([^"]+)"', content)
b64_data = match.group(1)
with open(png_path, 'wb') as f:
    f.write(base64.b64decode(b64_data))

img = Image.open(png_path).convert("RGBA")
alpha = img.split()[3] # The text is in the alpha channel

bg = Image.new("RGB", img.size, (255, 255, 255)) # White background
black = Image.new("RGB", img.size, (0, 0, 0)) # Black text

# Paste black text onto white background using the alpha mask
bg.paste(black, (0, 0), alpha)
bg.save(png_path, "PNG")
print("Done")
