import os
try:
    import fitz # PyMuPDF
    print("PyMuPDF installed")
except ImportError:
    print("PyMuPDF not installed")

try:
    from pdf2image import convert_from_path
    print("pdf2image installed")
except ImportError:
    print("pdf2image not installed")

try:
    import pypdf
    print("pypdf installed")
except ImportError:
    print("pypdf not installed")
