#!/usr/bin/env python3
"""Translate source comments (<!-- -->, /* */, // ...) that contain CJK in html/en files.

  comments_i18n.py extract DIR OUT.txt     -> listing  === file#cNNN === / comment text
  comments_i18n.py inject DIR OUT.txt TRANS.txt
The comment delimiters are kept by the script; only the inner text is listed/replaced.
"""
import re, sys, os, glob

CJK = re.compile(r'[一-鿿]')
SEP = re.compile(r'^=== (\S+) ===$', re.M)


def find_comments(s):
    """Return list of (start, end) of the inner text of comments containing CJK."""
    out = []
    for m in re.finditer(r'<!--(.*?)-->', s, re.S):
        if CJK.search(m.group(1)):
            out.append((m.start(1), m.end(1)))
    for m in re.finditer(r'/\*(.*?)\*/', s, re.S):
        if CJK.search(m.group(1)):
            out.append((m.start(1), m.end(1)))
    for m in re.finditer(r'//([^\n]*)', s):
        if not CJK.search(m.group(1)):
            continue
        line_start = s.rfind('\n', 0, m.start()) + 1
        before = s[line_start:m.start()]
        # skip if // is inside a string literal (odd number of quotes before it) or is part of a URL
        if before.count('"') % 2 or before.count("'") % 2 or before.count('`') % 2:
            continue
        if before.endswith(':'):
            continue
        out.append((m.start(1), m.end(1)))
    out.sort()
    # drop nested/overlapping (e.g. // inside /* */)
    res = []
    last_end = -1
    for a, b in out:
        if a < last_end:
            continue
        res.append((a, b))
        last_end = b
    return res


def read_listing(path):
    data = open(path, encoding='utf-8').read()
    parts = SEP.split(data)
    out = {}
    for i in range(1, len(parts), 2):
        body = parts[i + 1]
        if body.startswith('\n'):
            body = body[1:]
        if body.endswith('\n'):
            body = body[:-1]
        out[parts[i]] = body
    return out


def cmd_extract(d, outp):
    n = 0
    with open(outp, 'w', encoding='utf-8') as f:
        for fp in sorted(glob.glob(os.path.join(d, '*.html'))):
            s = open(fp, encoding='utf-8').read()
            base = os.path.basename(fp)
            for i, (a, b) in enumerate(find_comments(s), 1):
                f.write(f'=== {base}#c{i:03d} ===\n{s[a:b]}\n')
                n += 1
    print(f'{n} comments listed in {outp}')


def cmd_inject(d, listing, trans):
    tr = read_listing(trans)
    orig = read_listing(listing)
    missing = [k for k in orig if k not in tr]
    if missing:
        print(f'ERROR {len(missing)} ids missing in {trans}: {missing[:10]}')
        sys.exit(1)
    for fp in sorted(glob.glob(os.path.join(d, '*.html'))):
        s = open(fp, encoding='utf-8').read()
        base = os.path.basename(fp)
        spans = find_comments(s)
        edits = []
        for i, (a, b) in enumerate(spans, 1):
            k = f'{base}#c{i:03d}'
            if k not in tr:
                continue
            if orig.get(k) != s[a:b]:
                print(f'WARN drift {k}, skipped')
                continue
            en = tr[k]
            if CJK.search(en):
                print(f'WARN CJK left {k}: {en[:80]!r}')
            if '-->' in en or '*/' in en:
                print(f'WARN delimiter in translation {k}')
                continue
            edits.append((a, b, en))
        for a, b, en in reversed(edits):
            s = s[:a] + en + s[b:]
        if edits:
            open(fp, 'w', encoding='utf-8').write(s)
            print(f'{base}: {len(edits)} comments replaced')


if __name__ == '__main__':
    if sys.argv[1] == 'extract':
        cmd_extract(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == 'inject':
        cmd_inject(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        print(__doc__)
