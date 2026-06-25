import pypdf

reader = pypdf.PdfReader("main.pdf")
print("Total pages:", len(reader.pages))

# Search for the GCS flowchart text across all pages to find the exact page index
for idx, page in enumerate(reader.pages):
    text = page.extract_text()
    if "Hàng đợi ghi hình" in text or "Hang doi ghi hinh" in text or "Event Replay Queue" in text:
        print(f"Found on page {idx + 1}:")
        print(text.encode('ascii', 'replace').decode('ascii'))
        print("-" * 50)
