$downloads = "c:\Users\hustv\Downloads"
$file = Get-ChildItem -Path $downloads -Filter "*HXT_20224163*.docx" | Select-Object -First 1
Write-Output "Processing file: $($file.FullName)"
$tempDir = New-Item -ItemType Directory -Path ".\temp_docx" -Force

# Copy file .docx to a temporary .zip file
$zipFile = Join-Path $tempDir.FullName "temp_doc.zip"
Copy-Item $file.FullName $zipFile -Force

# Expand the zip file
Expand-Archive -Path $zipFile -DestinationPath $tempDir.FullName -Force

# Read and parse XML
[xml]$doc = Get-Content (Join-Path $tempDir.FullName "word/document.xml") -Raw
$ns = New-Object System.Xml.XmlNamespaceManager($doc.NameTable)
$ns.AddNamespace("w", "http://schemas.openxmlformats.org/wordprocessingml/2006/main")
$texts = $doc.SelectNodes("//w:t", $ns) | ForEach-Object { $_.InnerText }
$fullText = $texts -join " "
$fullText | Out-File -FilePath ".\scratch_docx_content.txt" -Encoding utf8

# Clean up
Remove-Item -Recurse -Force $tempDir.FullName
Write-Output "Success. Length of text: $($fullText.Length)"
