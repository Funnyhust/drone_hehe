import urllib.request
import ssl
from PIL import Image

url = "https://upload.wikimedia.org/wikipedia/en/thumb/c/c2/Altium_Designer_Logo.svg/800px-Altium_Designer_Logo.svg.png"
png_path = r"d:\DATN\FLY\report\Images\altium_logo.png"

try:
    context = ssl._create_unverified_context()
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req, context=context) as response:
        with open(png_path, "wb") as f:
            f.write(response.read())
    print("Downloaded successfully from Wikipedia")
    
    # Ensure white background
    img = Image.open(png_path).convert("RGBA")
    bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
    bg.paste(img, (0, 0), img)
    bg.convert("RGB").save(png_path, "PNG")
    print("White background applied")
except Exception as e:
    print(f"Error: {e}")
