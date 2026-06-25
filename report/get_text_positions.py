import pypdf

reader = pypdf.PdfReader("main.pdf")
page = reader.pages[0]

def visitor_body(text, cm, tm, font_dict, font_size):
    if text.strip():
        print(f"Text: {text.strip().encode('ascii', 'replace').decode('ascii'):<40} Position: x={tm[4]:.2f}, y={tm[5]:.2f}")

page.extract_text(visitor_text=visitor_body)
