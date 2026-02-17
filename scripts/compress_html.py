import gzip
import os
import sys

# PlatformIO pre-build script hook
# When called by PlatformIO, Import is available
try:
    Import("env")
    is_pio = True
except:
    is_pio = False

def compress_html():
    # Resolve paths relative to project root
    # In SCons/PlatformIO context __file__ is not defined, use cwd instead
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
    except NameError:
        project_dir = os.getcwd()

    input_path = os.path.join(project_dir, "web", "index.html")
    output_path = os.path.join(project_dir, "include", "html_page_gz.h")

    if not os.path.exists(input_path):
        print(f"[compress_html] ERROR: {input_path} not found!")
        sys.exit(1)

    with open(input_path, "r", encoding="utf-8") as f:
        html_content = f.read()

    compressed = gzip.compress(html_content.encode("utf-8"), compresslevel=9)

    with open(output_path, "w") as f:
        f.write("// Auto-generated — do not edit manually\n")
        f.write("// Source: web/index.html\n")
        f.write(f"// Original size: {len(html_content)} bytes\n")
        f.write(f"// Compressed size: {len(compressed)} bytes\n")
        f.write(f"// Compression ratio: {len(compressed)*100//len(html_content)}%\n\n")
        f.write("#pragma once\n")
        f.write("#include <pgmspace.h>\n\n")
        f.write(f"const size_t html_page_gz_len = {len(compressed)};\n\n")
        f.write("const uint8_t html_page_gz[] PROGMEM = {\n")

        for i in range(0, len(compressed), 16):
            chunk = compressed[i:i+16]
            hex_str = ", ".join(f"0x{b:02x}" for b in chunk)
            f.write(f"    {hex_str},\n")

        f.write("};\n")

    print(f"[compress_html] Compressed {len(html_content)} -> {len(compressed)} bytes ({len(compressed)*100//len(html_content)}%)")

# Run compression
compress_html()
