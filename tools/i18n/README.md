# i18n helpers for html/en and docs/en

`html/en/*.html` and `docs/en/*.md` are English mirrors of the Chinese originals (generated 2026-09-05). The Chinese files are the maintained source. These two scripts were used to build the HTML mirrors and can be reused when a Chinese page changes.

## html_i18n.py — text segments

```
python3 tools/i18n/html_i18n.py extract html/X.html seg.txt --tm tools/i18n/tm.json
#   seg.txt lists every Chinese segment that is not already in the translation memory:
#   === s0001 ===
#   <text, inline tags kept>
# write trans.txt in the same format with the English text, then
python3 tools/i18n/html_i18n.py inject html/X.html seg.txt trans.txt html/en/X.html --tm tools/i18n/tm.json
python3 tools/i18n/html_i18n.py check html/en/X.html      # reports any CJK left
```

Segmentation: text nodes plus the inline tags between them form one segment (so a sentence with `<b>` inside is translated whole); block tags flush; attributes with Chinese on block tags, JavaScript string literals and `lang="zh*"` are handled too. Markup, CSS, SVG geometry and embedded base64 images are never touched. `inject` warns on tag mismatches, unescaped quotes in JS strings and leftover CJK. `tm.json` is the translation memory (Chinese segment → English) accumulated from all pages; unchanged segments are filled from it automatically.

## comments_i18n.py — source comments

```
python3 tools/i18n/comments_i18n.py extract html/en comments.txt
# translate into comments_en.txt (same format), then
python3 tools/i18n/comments_i18n.py inject html/en comments.txt comments_en.txt
```

Handles `<!-- -->`, `/* */` and `//` comments containing Chinese; delimiters stay in place.

Markdown files in `docs/en/` were translated whole; there is no tool for them beyond a structure check (heading / fence / table-row counts must match the Chinese file).
