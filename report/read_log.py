with open("main.log", "r", encoding="utf-8", errors="ignore") as f:
    content = f.read()

print("Chuong4 in log:", "Chuong4" in content)
print("Chuong4.tex in log:", "Chuong4.tex" in content)

# Find all occurrences of "Chuong4" and print surrounding text
import re
for m in re.finditer(r"Chuong4", content, re.IGNORECASE):
    start = max(0, m.start() - 50)
    end = min(len(content), m.end() + 50)
    print(f"Match: {content[start:end].replace(chr(10), ' ')}")
