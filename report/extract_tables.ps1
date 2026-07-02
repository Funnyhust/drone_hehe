$downloads = "c:\Users\hustv\Downloads"
$file = Get-ChildItem -Path $downloads -Filter "*HXT_20224163*.docx" | Select-Object -First 1
$tempDir = New-Item -ItemType Directory -Path ".\temp_docx_tables" -Force
$zipFile = Join-Path $tempDir.FullName "temp_doc.zip"
Copy-Item $file.FullName $zipFile -Force
Expand-Archive -Path $zipFile -DestinationPath $tempDir.FullName -Force

[xml]$doc = Get-Content (Join-Path $tempDir.FullName "word/document.xml") -Raw
$ns = New-Object System.Xml.XmlNamespaceManager($doc.NameTable)
$ns.AddNamespace("w", "http://schemas.openxmlformats.org/wordprocessingml/2006/main")

$tables = $doc.SelectNodes("//w:tbl", $ns)
$tableIdx = 1
foreach ($table in $tables) {
    Write-Output "--- Table $tableIdx ---"
    $rows = $table.SelectNodes(".//w:tr", $ns)
    foreach ($row in $rows) {
        $cells = $row.SelectNodes(".//w:tc", $ns)
        $cellTexts = @()
        foreach ($cell in $cells) {
            $texts = $cell.SelectNodes(".//w:t", $ns) | ForEach-Object { $_.InnerText }
            $cellTexts += ($texts -join "")
        }
        Write-Output ($cellTexts -join " | ")
    }
    $tableIdx++
}

Remove-Item -Recurse -Force $tempDir.FullName
