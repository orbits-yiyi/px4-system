#!/usr/bin/env python3
"""Check course links, anchors, fences, Bash syntax and inventory.

Run from any directory with Python 3 and markdown-it-py installed.
Parses Markdown tokens so C++ lambdas and code examples are not treated as links.
External URLs are not fetched. This is a structural check, not factual review.
"""
import ast
import re
import subprocess
import unicodedata
from collections import Counter
from pathlib import Path
from urllib.parse import unquote, urlsplit

from markdown_it import MarkdownIt


ROOT = Path(__file__).resolve().parents[2]
PARSER = MarkdownIt('commonmark')


def slug(text):
    return ''.join(c for c in text.lower()
                   if c in ' -_' or unicodedata.category(c)[0] in 'LNM').replace(' ', '-')


def main():
    documents = [ROOT / 'README.md', *sorted((ROOT / 'learn').rglob('*.md'))]
    errors = []
    anchor_cache = {}
    link_count = 0
    shell_count = 0

    def fail(path, line, message):
        errors.append(f'{path.relative_to(ROOT)}:{line}: {message}')

    def anchors(path):
        if path in anchor_cache:
            return anchor_cache[path]
        body = path.read_text()
        result = set(re.findall(r'<a\s+(?:id|name)=["\']([^"\']+)', body))
        seen = Counter()
        tokens = PARSER.parse(body)
        for index, token in enumerate(tokens):
            if token.type == 'heading_open':
                inline = tokens[index + 1]
                heading = ''.join(t.content for t in (inline.children or [])
                                  if t.type in ('text', 'code_inline', 'html_inline'))
                base = slug(re.sub('<[^>]+>', '', heading))
                number = seen[base]
                seen[base] += 1
                result.add(base if number == 0 else f'{base}-{number}')
        anchor_cache[path] = result
        return result

    for path in documents:
        body = path.read_text()
        lines = body.splitlines()
        for token in PARSER.parse(body):
            line = token.map[0] + 1 if token.map else 1
            if token.type == 'fence':
                last = lines[token.map[1] - 1].strip()
                if not re.fullmatch(re.escape(token.markup[0]) + '{' +
                                    str(len(token.markup)) + ',}', last):
                    fail(path, line, 'unclosed code fence')
                if token.info.strip() in ('bash', 'sh'):
                    shell_count += 1
                    check = subprocess.run(['bash', '-n'], input=token.content,
                                           text=True, capture_output=True)
                    if check.returncode:
                        fail(path, line, 'Bash syntax: ' + check.stderr.strip())
            for child in token.children or []:
                if child.type not in ('link_open', 'image'):
                    continue
                value = child.attrGet('href' if child.type == 'link_open' else 'src')
                if not value:
                    continue
                parsed = urlsplit(value)
                if parsed.scheme or parsed.netloc:
                    continue
                link_count += 1
                target = (path.parent / unquote(parsed.path)).resolve() if parsed.path else path
                if not target.exists():
                    fail(path, line, f'missing file: {value}')
                elif parsed.fragment and target.suffix.lower() == '.md':
                    if unquote(parsed.fragment) not in anchors(target):
                        fail(path, line, f'missing anchor: {value}')

    modules = {p.name for p in (ROOT / 'src/modules').iterdir() if p.is_dir()}
    coverage = (ROOT / 'learn/records/IMPLEMENTATION.md').read_text()
    rows = re.findall(r'^\| `([a-z0-9_]+)` \|', coverage, re.MULTILINE)
    if set(rows) != modules or len(rows) != len(modules):
        errors.append(f'coverage mismatch: missing={modules - set(rows)}, '
                      f'extra={set(rows) - modules}, rows={len(rows)}')
    workbook = ROOT / 'learn/WORKBOOK.md'
    body = workbook.read_text()
    chapters = re.findall(r'<a id="c(\d{2})"></a>', body)
    if chapters != [f'{i:02}' for i in range(1, 21)]:
        errors.append('expected exactly workbook chapters c01..c20 in order')
    parts = re.split(r'<a id="c\d{2}"></a>', body)[1:]
    for index, part in enumerate(parts, 1):
        for required in ('**目标**', '独立', '<details>', '完成标准', '恢复'):
            if required not in part:
                fail(workbook, 1, f'chapter {index} missing teaching element: {required}')
    for path in (ROOT / 'learn').rglob('*.py'):
        try:
            ast.parse(path.read_text(), filename=str(path))
        except SyntaxError as error:
            fail(path, error.lineno, str(error))
    if errors:
        print('\n'.join(errors))
        raise SystemExit(1)
    print(f'PASS: {len(documents)} Markdown files, {link_count} local links, '
          f'{shell_count} Shell blocks, {len(chapters)} workbook chapters, '
          f'{len(modules)}/{len(modules)} module coverage; Python syntax valid')


if __name__ == '__main__':
    main()
