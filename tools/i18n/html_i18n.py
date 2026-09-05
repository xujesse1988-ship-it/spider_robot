#!/usr/bin/env python3
"""Extract translatable Chinese segments from a self-contained HTML file and inject
English translations back, touching nothing but the text.

Usage:
  html_i18n.py extract IN.html SEG.txt [--tm TM.json]
      Writes segments needing translation as a plain listing:
          === s0001 ===
          <original segment>
      Segments already in the translation memory (TM) are skipped in the
      listing but recorded in SEG.txt.map (id -> original) for injection.
  html_i18n.py inject IN.html SEG.txt TRANS.txt OUT.html [--tm TM.json]
      TRANS.txt uses the same listing format with English text.
      Every segment id must resolve (from TRANS.txt or TM); tag-structure of
      each translation is checked against the original.
  html_i18n.py check OUT.html
      Reports remaining CJK text outside <script>/comments.

Segmentation rules
  * Text nodes plus *inline* tags between them form one segment, so a
    sentence like  吸盘 <b>30 mm</b> 时  is translated as a whole.
  * Block tags, comments, <script>, <style> flush the segment.
  * Attributes with CJK on block tags become their own segments.
  * Inside <script>: string literals ("...", '...', `...`) containing CJK.
  * lang="zh*" is rewritten to lang="en" automatically.
"""
import json
import re
import sys

CJK = re.compile(r'[　-〿㐀-䶿一-鿿豈-﫿＀-￯]')
INLINE = {
    'a', 'abbr', 'b', 'bdi', 'bdo', 'br', 'cite', 'code', 'data', 'dfn', 'em',
    'i', 'kbd', 'mark', 'q', 'rp', 'rt', 'ruby', 's', 'samp', 'small', 'span',
    'strong', 'sub', 'sup', 'time', 'u', 'var', 'wbr', 'tspan', 'del', 'ins',
    'input',
}
TOKEN = re.compile(
    r'(?P<comment><!--.*?-->)'
    r'|(?P<script><script\b[^>]*>.*?</script\s*>)'
    r'|(?P<style><style\b[^>]*>.*?</style\s*>)'
    r'|(?P<tag></?[A-Za-z][^>]*>)'
    r'|(?P<text>[^<]+)',
    re.S,
)
TAGNAME = re.compile(r'</?\s*([A-Za-z][A-Za-z0-9:-]*)')
ATTR = re.compile(r'''([A-Za-z_:][-A-Za-z0-9_:.]*)\s*=\s*("([^"]*)"|'([^']*)')''')
JS_STR = re.compile(
    r'"(?:[^"\\\n]|\\.)*"'
    r"|'(?:[^'\\\n]|\\.)*'"
    r'|`(?:[^`\\]|\\.)*`',
    re.S,
)
SEP = re.compile(r'^=== (\S+) ===$', re.M)


def tokens(html):
    for m in TOKEN.finditer(html):
        yield m.lastgroup, m.start(), m.end()


def tagname(s):
    m = TAGNAME.match(s)
    return m.group(1).lower() if m else ''


def extract_segments(html):
    """Return list of (start, end, kind) for every translatable range."""
    segs = []
    buf = []  # list of (start, end, has_text)

    def flush():
        if not buf:
            return
        start = buf[0][0]
        end = buf[-1][1]
        s = html[start:end]
        if CJK.search(s):
            # trim whitespace
            ls = len(s) - len(s.lstrip())
            rs = len(s) - len(s.rstrip())
            segs.append((start + ls, end - rs, 'text'))
        buf.clear()

    for kind, a, b in tokens(html):
        s = html[a:b]
        if kind == 'text':
            buf.append((a, b))
        elif kind == 'tag':
            name = tagname(s)
            if name in INLINE:
                buf.append((a, b))
            else:
                flush()
                # attributes on block tags
                for am in ATTR.finditer(s):
                    val = am.group(3) if am.group(3) is not None else am.group(4)
                    if val and CJK.search(val):
                        vs = a + am.start(2) + 1
                        segs.append((vs, vs + len(val), 'attr'))
        elif kind == 'script':
            flush()
            open_end = s.index('>') + 1
            close = s.rfind('</script')
            body = s[open_end:close]
            for jm in JS_STR.finditer(body):
                lit = jm.group(0)
                if CJK.search(lit):
                    vs = a + open_end + jm.start() + 1
                    segs.append((vs, vs + len(lit) - 2, 'js'))
        else:  # comment / style
            flush()
            if kind == 'style':
                open_end = s.index('>') + 1
                close = s.rfind('</style')
                body = s[open_end:close]
                for jm in JS_STR.finditer(body):
                    lit = jm.group(0)
                    if CJK.search(lit):
                        vs = a + open_end + jm.start() + 1
                        segs.append((vs, vs + len(lit) - 2, 'css'))
    flush()
    segs.sort()
    return segs


def write_listing(path, items):
    with open(path, 'w', encoding='utf-8') as f:
        for sid, text in items:
            f.write(f'=== {sid} ===\n{text}\n')


def read_listing(path):
    data = open(path, encoding='utf-8').read()
    out = {}
    parts = SEP.split(data)
    # parts: [pre, id1, body1, id2, body2, ...]
    for i in range(1, len(parts), 2):
        sid = parts[i]
        body = parts[i + 1]
        if body.startswith('\n'):
            body = body[1:]
        if body.endswith('\n'):
            body = body[:-1]
        out[sid] = body
    return out


def load_tm(path):
    if not path:
        return {}
    try:
        return json.load(open(path, encoding='utf-8'))
    except FileNotFoundError:
        return {}


def save_tm(path, tm):
    if path:
        json.dump(tm, open(path, 'w', encoding='utf-8'), ensure_ascii=False, indent=0)


def tag_signature(s):
    return sorted(re.findall(r'</?[A-Za-z][^>]*>', s))


def tag_names_multiset(s):
    return sorted(tagname(t) + ('/' if t.startswith('</') else '') for t in re.findall(r'</?[A-Za-z][^>]*>', s))


def cmd_extract(inp, segfile, tm_path=None):
    html = open(inp, encoding='utf-8').read()
    segs = extract_segments(html)
    tm = load_tm(tm_path)
    items = []
    need = []
    seen = {}
    for i, (a, b, kind) in enumerate(segs, 1):
        sid = f's{i:04d}'
        text = html[a:b]
        items.append((sid, text))
        if text in tm:
            continue
        if text in seen:
            continue
        seen[text] = sid
        need.append((sid, text))
    write_listing(segfile + '.map', items)
    write_listing(segfile, need)
    n_tm = sum(1 for _, t in items if t in tm)
    print(f'{inp}: {len(items)} segments, {len(need)} unique to translate, {n_tm} from TM, '
          f'{len(items) - len(need) - n_tm} in-file duplicates')
    print(f'chars to translate: {sum(len(t) for _, t in need)}')


def cmd_inject(inp, segfile, transfile, out, tm_path=None):
    html = open(inp, encoding='utf-8').read()
    segs = extract_segments(html)
    items = read_listing(segfile + '.map')
    trans = read_listing(transfile) if transfile != '-' else {}
    tm = load_tm(tm_path)
    # build original->english map
    o2e = dict(tm)
    for sid, en in trans.items():
        if sid not in items:
            print(f'WARN: unknown id {sid} in {transfile}')
            continue
        o2e[items[sid]] = en
    missing = []
    warns = 0
    edits = []
    for i, (a, b, kind) in enumerate(segs, 1):
        sid = f's{i:04d}'
        orig = html[a:b]
        assert items[sid] == orig, f'segment drift at {sid}'
        if orig not in o2e:
            missing.append((sid, orig))
            continue
        en = o2e[orig]
        if tag_names_multiset(orig) != tag_names_multiset(en):
            print(f'WARN tag mismatch {sid}:\n  ZH: {orig[:200]!r}\n  EN: {en[:200]!r}')
            warns += 1
        if kind == 'attr':
            q = html[a - 1]
            if q in en:
                print(f'WARN quote {q} inside attr translation {sid}: {en[:100]!r}')
                warns += 1
        if kind == 'js':
            q = html[a - 1]
            if q != '`':
                bad = re.search(r'(?<!\\)' + re.escape(q), en)
                if bad:
                    print(f'WARN unescaped quote {q} inside JS string {sid}: {en[:100]!r}')
                    warns += 1
        if CJK.search(en):
            print(f'WARN CJK left in translation {sid}: {en[:100]!r}')
            warns += 1
        edits.append((a, b, en))
    if missing:
        print(f'ERROR: {len(missing)} segments missing translation:')
        for sid, orig in missing[:20]:
            print(f'  {sid}: {orig[:120]!r}')
        sys.exit(1)
    for a, b, en in reversed(edits):
        html = html[:a] + en + html[b:]
    html = re.sub(r'(<html\b[^>]*\blang=")zh[^"]*(")', r'\1en\2', html, count=1)
    open(out, 'w', encoding='utf-8').write(html)
    # update TM with all pairs
    for sid, en in trans.items():
        if sid in items:
            tm[items[sid]] = en
    save_tm(tm_path, tm)
    print(f'wrote {out}: {len(edits)} segments replaced, {warns} warnings')


def cmd_check(path):
    html = open(path, encoding='utf-8').read()
    n = 0
    for kind, a, b in tokens(html):
        s = html[a:b]
        if kind in ('text', 'tag'):
            for m in CJK.finditer(s):
                line = html.count('\n', 0, a + m.start()) + 1
                ctx = s[max(0, m.start() - 40):m.start() + 40].replace('\n', ' ')
                print(f'line {line}: ...{ctx}...')
                n += 1
                break
        elif kind == 'script':
            for m in CJK.finditer(s):
                line = html.count('\n', 0, a + m.start()) + 1
                ctx = s[max(0, m.start() - 40):m.start() + 40].replace('\n', ' ')
                print(f'line {line} [script]: ...{ctx}...')
                n += 1
                break
    print(f'{path}: {n} tokens still containing CJK')


if __name__ == '__main__':
    args = sys.argv[1:]
    tm = None
    if '--tm' in args:
        i = args.index('--tm')
        tm = args[i + 1]
        del args[i:i + 2]
    cmd = args[0]
    if cmd == 'extract':
        cmd_extract(args[1], args[2], tm)
    elif cmd == 'inject':
        cmd_inject(args[1], args[2], args[3], args[4], tm)
    elif cmd == 'check':
        cmd_check(args[1])
    else:
        print(__doc__)
        sys.exit(2)
