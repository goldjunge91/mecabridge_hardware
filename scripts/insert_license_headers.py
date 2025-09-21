#!/usr/bin/env python3
"""
Insert standard license headers into source files of various types.
Supports: C++, Python, CMake, XML, launch, shell scripts.

Usage:
  python scripts/insert_license_headers.py <root_folder> <ext1> <ext2> ...
Example:
  python scripts/insert_license_headers.py src/mecabridge_hardware cpp hpp h py cmake xml xacro launch sh

Idempotent: will not insert duplicate headers.
"""
import sys
import re
from pathlib import Path

LICENSE_HEADER = {
    'cpp': [
        '/*\n',
        ' * Copyright (c) 2024 MecaBridge Project\n',
        ' * SPDX-License-Identifier: Apache-2.0\n',
        ' */\n',
        '\n',
    ],
    'h': [
        '/*\n',
        ' * Copyright (c) 2024 MecaBridge Project\n',
        ' * SPDX-License-Identifier: Apache-2.0\n',
        ' */\n',
        '\n',
    ],
    'hpp': [
        '/*\n',
        ' * Copyright (c) 2024 MecaBridge Project\n',
        ' * SPDX-License-Identifier: Apache-2.0\n',
        ' */\n',
        '\n',
    ],
    'py': [
        '#\n',
        '# Copyright (c) 2024 MecaBridge Project\n',
        '# SPDX-License-Identifier: Apache-2.0\n',
        '#\n',
        '\n',
    ],
    'sh': [
        '#\n',
        '# Copyright (c) 2024 MecaBridge Project\n',
        '# SPDX-License-Identifier: Apache-2.0\n',
        '#\n',
        '\n',
    ],
    'cmake': [
        '#\n',
        '# Copyright (c) 2024 MecaBridge Project\n',
        '# SPDX-License-Identifier: Apache-2.0\n',
        '#\n',
        '\n',
    ],
    'xml': [
        '<!--\n',
        '  Copyright (c) 2024 MecaBridge Project\n',
        '  SPDX-License-Identifier: Apache-2.0\n',
        '-->\n',
        '\n',
    ],
    'xacro': [
        '<!--\n',
        '  Copyright (c) 2024 MecaBridge Project\n',
        '  SPDX-License-Identifier: Apache-2.0\n',
        '-->\n',
        '\n',
    ],
    'launch': [
        '#\n',
        '# Copyright (c) 2024 MecaBridge Project\n',
        '# SPDX-License-Identifier: Apache-2.0\n',
        '#\n',
        '\n',
    ],
}

HEADER_MARKERS = {
    'cpp': 'Copyright (c) 2024 MecaBridge Project',
    'h': 'Copyright (c) 2024 MecaBridge Project',
    'hpp': 'Copyright (c) 2024 MecaBridge Project',
    'py': 'Copyright (c) 2024 MecaBridge Project',
    'sh': 'Copyright (c) 2024 MecaBridge Project',
    'cmake': 'Copyright (c) 2024 MecaBridge Project',
    'xml': 'Copyright (c) 2024 MecaBridge Project',
    'xacro': 'Copyright (c) 2024 MecaBridge Project',
    'launch': 'Copyright (c) 2024 MecaBridge Project',
}

def insert_header(path: Path, ext: str):
    try:
        text = path.read_text(encoding='utf-8')
    except Exception:
        return False
    marker = HEADER_MARKERS[ext]
    if marker in text[:200]:
        return False  # Already present
    header = LICENSE_HEADER[ext]
    # For XML, insert after <?xml ...?> if present
    if ext in ('xml', 'xacro') and text.lstrip().startswith('<?xml'):
        first_line_end = text.find('\n') + 1
        new_text = text[:first_line_end] + ''.join(header) + text[first_line_end:]
    else:
        new_text = ''.join(header) + text
    try:
        path.write_text(new_text, encoding='utf-8')
        return True
    except Exception:
        return False

def main():
    if len(sys.argv) < 3:
        print('Usage: python scripts/insert_license_headers.py <root_folder> <ext1> <ext2> ...')
        sys.exit(1)
    root = Path(sys.argv[1])
    exts = set(sys.argv[2:])
    changed = 0
    for ext in exts:
        for f in root.rglob(f'*.{ext}'):
            if insert_header(f, ext):
                print(f'Inserted header: {f}')
                changed += 1
    print(f'Total files modified: {changed}')

if __name__ == '__main__':
    main()
