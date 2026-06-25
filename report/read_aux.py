with open("main.aux", "r", encoding="utf-8") as f:
    lines = f.readlines()
for i in range(max(0, 80), min(len(lines), 130)):
    print(f"{i+1}: {lines[i].encode('ascii', 'replace').decode('ascii')}", end="")
