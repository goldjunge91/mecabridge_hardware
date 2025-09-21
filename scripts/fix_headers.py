#!/usr/bin/env python3
"""Bulk style maintenance helper for mecabridge_hardware.

Focus (non-invasive):
 1. Ensure trailing newline at EOF.
 2. Strip trailing whitespace.
 3. Normalize header guards for headers missing or malformed.
 4. Insert missing #include <memory> / <algorithm> when explicitly required by cpplint diagnostics
    (We perform heuristic: look for shared_ptr / unique_ptr / make_shared and absence of <memory>.)
 5. Remove duplicate self-include lines inside the same file (e.g., watchdog.hpp duplicate include).

Copyright headers: delegated to existing bash script. Run that first.

Guard naming scheme:
  <PATH_WITH_SLASHES_REPLACED_BY_DOUBLE_UNDERSCORES><FILENAME_UPPER>_<EXT>_
  Root prefix deduced: mecabridge_hardware/ or mecabridge_utils/ preserved per cpplint suggestions.

The script is idempotent.
"""
from __future__ import annotations
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SCOPE = 'src'  # operate across entire src tree
TARGET = ROOT / DEFAULT_SCOPE
HEADER_EXTS = {'.h', '.hpp'}
SOURCE_EXTS = {'.c', '.cc', '.cxx', '.cpp'}

INCLUDE_GUARD_RE = re.compile(r'^#ifndef\s+([A-Z0-9_]+)\s*$')
ENDIF_GUARD_RE = re.compile(r'^#endif\s*(//.*)?$')
SHARED_PTR_RE = re.compile(r'\b(std::)?shared_ptr<')
UNIQUE_PTR_RE = re.compile(r'\b(std::)?unique_ptr<')
MAKE_SHARED_RE = re.compile(r'\bstd::make_shared<')
ALGO_COPY_RE = re.compile(r'\bstd::copy\b|\bcopy\s*\(')
ALGO_MINMAX_RE = re.compile(r'\bstd::(min|max)\b')

# Known files needing <algorithm> from cpplint output (copy, min usage) may be safer than broad heuristic
FORCE_ALGO = {
    'src/mecabridge_hardware/src/mecabridge_utils/serial/serial_backend.hpp',
    'src/mecabridge_hardware/test/mecabridge/testable_mecabridge_serial_protocol.cpp',
}
FORCE_MEMORY = set()  # cpplint explicitly mentioned one header lacking <memory>

# Map of specific file -> required additional includes (deduped later)
EXPLICIT_INCLUDES = {
    'src/mecabridge_hardware/include/mecabridge_hardware/mecabridge_serial_protocol.h': ['<memory>'],
}

def desired_guard(path: Path) -> str:
    """Compute header guard approximating cpplint suggestions.

    Empirically derived rules from current cpplint diagnostics:
      * Strip leading 'src/' component (repo build root noise).
      * Choose the LAST occurrence of a root token (so nested mecabridge_utils wins over outer mecabridge_hardware).
      * Root tokens (ordered by preference when multiple exist): mecabridge_utils, mecabridge_hardware, mecabridge.
      * Skip structural directories: include, src, test.
      * Preserve remaining directory names (deduplicate consecutive duplicates).
      * File component becomes STEM_EXT (e.g. config.hpp -> CONFIG_HPP) to match cpplint style.
      * Join components with double underscores and append trailing underscore.
    """
    rel_parts = list(path.relative_to(ROOT).parts)
    if rel_parts and rel_parts[0] == 'src':
        rel_parts = rel_parts[1:]
    if not rel_parts:
        return ''
    root_tokens = ['mecabridge_utils', 'mecabridge_hardware', 'mecabridge']
    # Find last matching root token (prefer earlier in root_tokens ordering if multiple at same index?)
    root_index = 0
    last_match = -1
    preferred_token = None
    for i, part in enumerate(rel_parts):
        if part in root_tokens:
            # weight by index priority (lower index in root_tokens => higher priority)
            priority_rank = root_tokens.index(part)
            # Accept later occurrences OR higher priority at same position
            if i >= last_match or preferred_token is None or priority_rank < root_tokens.index(preferred_token):
                last_match = i
                root_index = i
                preferred_token = part
    after = rel_parts[root_index:]
    structural = {'include', 'src', 'test'}
    path_tokens: list[str] = []
    for t in after[:-1]:
        if t in structural:
            continue
        if not path_tokens or path_tokens[-1] != t:
            path_tokens.append(t)
    filename = after[-1]
    if '.' in filename:
        stem, ext = filename.rsplit('.', 1)
        file_token = f"{stem}_{ext}".upper()
    else:
        file_token = filename.upper()
    components = path_tokens + [file_token]
    collapsed: list[str] = []
    for comp in components:
        up = comp.upper()
        if not collapsed or collapsed[-1].upper() != up:
            collapsed.append(comp)
    return '__'.join(c.upper().replace('.', '_') for c in collapsed) + '_'


def process_header_guard(lines: list[str], path: Path) -> list[str]:
    # Detect existing guard pattern (#ifndef ... #define ... ... #endif ...)
    guard_name = desired_guard(path)
    # cpplint examples show double underscore between directory and file, maintain scheme above.
    has_ifndef = None
    has_define = None
    endif_index = None
    for i, line in enumerate(lines[:10]):  # usually guards at top
        if line.startswith('#ifndef '):
            has_ifndef = i
        if line.startswith('#define '):
            has_define = i
        if has_ifndef is not None and has_define is not None:
            break
    for j in range(len(lines)-1, -1, -1):
        if lines[j].startswith('#endif'):
            endif_index = j
            break
    # If any piece missing or names mismatched, rebuild guards.
    need_rewrite = False
    if has_ifndef is None or has_define is None or endif_index is None:
        need_rewrite = True
    else:
        current_ifndef = lines[has_ifndef].split()[1]
        if current_ifndef != guard_name:
            need_rewrite = True

    if not need_rewrite:
        # Ensure #endif comment style matches cpplint (only if endif_index detected)
        if endif_index is not None and not re.search(re.escape(guard_name), lines[endif_index]):
            lines[endif_index] = f'#endif  // {guard_name}\n'
        return lines

    # Remove any existing leading guard lines to avoid duplicates
    new_body_start = 0
    if has_ifndef is not None and has_define is not None and has_define == has_ifndef + 1:
        # Skip old pattern until after define
        new_body_start = has_define + 1
    new_lines = []
    new_lines.append(f'#ifndef {guard_name}\n')
    new_lines.append(f'#define {guard_name}\n\n')
    new_lines.extend(lines[new_body_start:])
    # Ensure single trailing newline
    if not new_lines[-1].endswith('\n'):
        new_lines[-1] += '\n'
    # Fix final endif
    if endif_index is not None:
        # remove existing endif if mismatched to append new
        if not re.search(re.escape(guard_name), new_lines[-1]):
            if new_lines[-1].startswith('#endif'):
                new_lines.pop()
        new_lines.append(f'#endif  // {guard_name}\n')
    else:
        new_lines.append(f'#endif  // {guard_name}\n')
    return new_lines


def needs_memory(lines: list[str]) -> bool:
    use = any(r.search(''.join(lines)) for r in (SHARED_PTR_RE, UNIQUE_PTR_RE, MAKE_SHARED_RE))
    if not use:
        return False
    for l in lines[:50]:
        if '#include <memory>' in l:
            return False
    return True


def needs_algorithm(lines: list[str], rel: str) -> bool:
    if rel in FORCE_ALGO:
        for l in lines[:80]:
            if '#include <algorithm>' in l:
                return False
        return True
    text = ''.join(lines)
    if ALGO_COPY_RE.search(text) or ALGO_MINMAX_RE.search(text):
        for l in lines[:80]:
            if '#include <algorithm>' in l:
                return False
        return True
    return False


def insert_includes(lines: list[str], rel: str) -> list[str]:
    add = []
    if rel in EXPLICIT_INCLUDES:
        add.extend([inc for inc in EXPLICIT_INCLUDES[rel] if not any(inc in l for l in lines)])
    if needs_memory(lines):
        add.append('<memory>')
    if needs_algorithm(lines, rel):
        add.append('<algorithm>')
    if not add:
        return lines
    # insert after first block of includes or after guard lines
    insertion_index = 0
    # Skip guard
    if lines and lines[0].startswith('#ifndef') and len(lines) > 2 and lines[1].startswith('#define'):
        insertion_index = 2
        while insertion_index < len(lines) and lines[insertion_index].strip() == '':
            insertion_index += 1
    # Find last consecutive include starting at top
    i = insertion_index
    while i < len(lines) and lines[i].startswith('#include'):
        i += 1
    insertion_index = i
    snippet = ''.join(f'#include {inc}\n' for inc in add)
    return lines[:insertion_index] + [snippet] + lines[insertion_index:]


def reorder_includes(lines: list[str]) -> list[str]:
    """Reorder contiguous include block to: project, C system, C++ system, other.

    Project includes are quoted or starting with mecabridge_. C system heuristically: <stdio.h>, <stdint.h>, etc.
    C++ system: <vector>, <string>, <memory>, <algorithm>, etc.
    Other: remaining angle-bracket includes.
    """
    include_block_start = None
    include_block_end = None
    for i, l in enumerate(lines[:100]):
        if l.startswith('#include'):
            if include_block_start is None:
                include_block_start = i
            include_block_end = i
        elif include_block_start is not None:
            break
    if include_block_start is None or include_block_end is None:
        return lines
    block = lines[include_block_start:include_block_end+1]
    project = []
    c_system = []
    cpp_system = []
    other = []
    c_pattern = re.compile(r'<(assert.h|ctype.h|errno.h|inttypes.h|limits.h|math.h|signal.h|stdarg.h|stdbool.h|stddef.h|stdint.h|stdio.h|stdlib.h|string.h|time.h)>')
    cpp_pattern = re.compile(r'<(algorithm|array|chrono|deque|functional|future|initializer_list|iostream|iterator|map|memory|mutex|optional|set|string|thread|tuple|type_traits|unordered_map|unordered_set|utility|vector)>')
    for raw in block:
        line = raw.rstrip('\n')
        if not line.startswith('#include'):
            continue
        if '"' in line:
            project.append(line)
        elif c_pattern.search(line):
            c_system.append(line)
        elif cpp_pattern.search(line):
            cpp_system.append(line)
        else:
            # treat <mecabridge_...> as project if ever used in angle form
            if '<mecabridge_' in line:
                project.append(line)
            else:
                other.append(line)
    # If no diversity, skip
    classified_total = len(project)+len(c_system)+len(cpp_system)+len(other)
    if classified_total < 2:
        return lines
    # Preserve relative ordering within each group (already in encounter order)
    new_block = []
    for group in (project, c_system, cpp_system, other):
        if not group:
            continue
        new_block.extend(group)
        new_block.append('')  # blank line separator
    if new_block and new_block[-1] == '':
        new_block.pop()
    # Replace
    new_block_lines = [b+'\n' for b in new_block]
    return lines[:include_block_start] + new_block_lines + lines[include_block_end+1:]


def remove_duplicate_self_include(lines: list[str], path: Path) -> list[str]:
    # If a line includes its own header twice, remove duplicates
    filename = path.name
    # Match either just the filename or any relative path ending with the filename
    include_pattern_simple = f'#include "{filename}"'
    include_pattern_suffix = f'/{filename}"'
    seen = False
    new = []
    for l in lines:
        if include_pattern_simple in l or (include_pattern_suffix in l and '#include "' in l):
            if seen:
                continue
            seen = True
        new.append(l)
    return new


def strip_trailing_ws_and_ensure_newline(text: str) -> str:
    lines = text.splitlines()
    lines = [re.sub(r'[ \t]+$', '', l) for l in lines]
    return '\n'.join(lines) + '\n'


def remove_using_namespace(lines: list[str]) -> list[str]:
    """Remove blanket 'using namespace X;' lines. Keep explicit 'using std::foo;' untouched.

    This is conservative: remove only lines that match exactly 'using namespace <word>;' optionally
    prefixed/suffixed by whitespace.
    """
    new = []
    for l in lines:
        if re.match(r'\s*using\s+namespace\s+[A-Za-z0-9_:]+\s*;\s*$', l):
            # drop blanket using namespace lines
            continue
        new.append(l)
    return new


def strip_namespace_indentation(lines: list[str], ns: str) -> list[str]:
    """If a namespace block 'namespace <ns> {' exists, remove one level of indentation
    for lines between the opening and matching closing brace. This normalizes style where
    files were indented inside namespace.
    """
    out = []
    in_ns = False
    brace_depth = 0
    for l in lines:
        stripped = l.lstrip('\t ')
        if not in_ns:
            out.append(l)
            if re.match(rf'\s*namespace\s+{re.escape(ns)}\s*{{', l):
                in_ns = True
                # count braces starting on this line
                brace_depth = l.count('{') - l.count('}')
        else:
            brace_depth += l.count('{') - l.count('}')
            if brace_depth <= 0:
                # end of namespace block
                out.append(l)
                in_ns = False
                continue
            # remove a single level of leading indentation (tab or 2-4 spaces)
            if l.startswith('\t'):
                out.append(l[1:])
            elif l.startswith('    '):
                out.append(l[4:])
            elif l.startswith('  '):
                out.append(l[2:])
            else:
                out.append(l)
    return out


def ensure_self_header_first(lines: list[str], path: Path) -> list[str]:
    """Ensure that if a matching header exists (same stem, .h/.hpp), it is the first include.

    If the header is missing, do nothing. If present but not first (after guard), move it to the
    top of the include block.
    """
    stem = path.stem
    dirp = path.parent
    candidates = [dirp / f"{stem}.hpp", dirp / f"{stem}.h"]
    header_name = None
    for c in candidates:
        if c.exists():
            header_name = c.name
            break
    if not header_name:
        return lines
    # find include block
    include_block_start = None
    include_block_end = None
    for i, l in enumerate(lines[:120]):
        if l.startswith('#include'):
            if include_block_start is None:
                include_block_start = i
            include_block_end = i
        elif include_block_start is not None:
            break
    if include_block_start is None:
        return lines
    if include_block_end is None:
        include_block_end = include_block_start
    block = lines[include_block_start:include_block_end+1]
    # find self include if present
    self_include_idx = None
    for i, l in enumerate(block):
        if f'"{header_name}"' in l or f'<{header_name}>' in l:
            self_include_idx = i
            break
    if self_include_idx is None:
        # not present -> insert at top of include block
        return lines[:include_block_start] + [f'#include "{header_name}"\n'] + lines[include_block_start:]
    if self_include_idx == 0:
        return lines
    # move it to first position
    new_block = [block[self_include_idx]] + block[:self_include_idx] + block[self_include_idx+1:]
    return lines[:include_block_start] + new_block + lines[include_block_end+1:]


def transform_file(path: Path):
    rel = path.relative_to(ROOT).as_posix()
    original = path.read_text(encoding='utf-8', errors='ignore')
    new_text = original
    # Normalize CRLF -> LF to avoid cpplint \r complaints
    if '\r\n' in new_text:
        new_text = new_text.replace('\r\n', '\n')
    # Step 1 strip trailing whitespace + ensure final newline early
    new_text = strip_trailing_ws_and_ensure_newline(new_text)
    lines = new_text.splitlines(keepends=True)

    if path.suffix in HEADER_EXTS:
        lines = process_header_guard(lines, path)
    # Remove duplicate self-include (particular issue in watchdog.hpp)
    lines = remove_duplicate_self_include(lines, path)
    # Remove blanket using namespace directives (they trigger readability/namespace issues)
    lines = remove_using_namespace(lines)
    # Strip namespace indentation for mecabridge_hardware to match project style
    lines = strip_namespace_indentation(lines, 'mecabridge_hardware')
    # Insert missing includes heuristically
    if path.suffix in HEADER_EXTS or path.suffix in SOURCE_EXTS:
        lines = insert_includes(lines, rel)
        lines = reorder_includes(lines)
        # Ensure source files include their matching header first
        if path.suffix in SOURCE_EXTS:
            lines = ensure_self_header_first(lines, path)

    final_text = ''.join(lines)
    if final_text != original:
        path.write_text(final_text, encoding='utf-8')
        return True
    return False


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Normalize headers/guards/includes across source tree.')
    parser.add_argument('--scope', action='append', default=[DEFAULT_SCOPE],
                        help='Path relative to repo root. Can be repeated. Default: src')
    args = parser.parse_args()
    total_changed = 0
    total_examined = 0
    for scope in args.scope:
        scope_path = (ROOT / scope).resolve()
        if not scope_path.exists():
            print(f"[WARN] Scope path does not exist, skipping: {scope_path}", file=sys.stderr)
            continue
        changed = 0
        examined = 0
        for p in scope_path.rglob('*'):
            if not p.is_file():
                continue
            if p.suffix not in (HEADER_EXTS | SOURCE_EXTS):
                continue
            examined += 1
            if transform_file(p):
                changed += 1
        total_changed += changed
        total_examined += examined
        print(f"Scope {scope}: processed {examined} files. Modified {changed}.")
    print(f"TOTAL: processed {total_examined} files. Modified {total_changed}.")

if __name__ == '__main__':
    main()
