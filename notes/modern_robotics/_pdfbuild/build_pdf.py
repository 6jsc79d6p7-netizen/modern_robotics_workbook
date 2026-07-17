#!/usr/bin/env python3
"""Build a single combined PDF ("hard copy") from the Modern Robotics notes.

Pipeline: markdown -> HTML (python-markdown + pymdownx.arithmatex) -> a styled,
self-contained HTML doc with vendored MathJax (tex-svg) -> PDF via headless
Chrome. Math written as $...$ / $$...$$ renders as real notation; code fences
and inline `code` stay monospace.

Usage:
    ../../.venv/bin/python build_pdf.py            # -> build/modern_robotics.pdf
    ../../.venv/bin/python build_pdf.py --html-only  # stop after HTML (fast debug)
"""
from __future__ import annotations
import argparse
import html
import os
import re
import subprocess
import sys
from pathlib import Path

import markdown

HERE = Path(__file__).resolve().parent
NOTES_DIR = HERE.parent            # notes/modern_robotics/
BUILD_DIR = HERE / "build"
MATHJAX = HERE / "tex-svg.js"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

TITLE = "Modern Robotics"
SUBTITLE = "Mechanics, Planning, and Control — study notes"

# Explicit chapter order (natural sort is almost right, but the "learned SOTA"
# note that replaces classical ch.9+10 belongs AFTER both of them).
def ordered_notes() -> list[Path]:
    files = sorted(p for p in NOTES_DIR.glob("*.md") if p.name[0].isdigit())
    learned = NOTES_DIR / "09_10_learned_sota.md"
    if learned in files:
        files.remove(learned)
        # place right after 10_motion_planning.md if present, else at end
        anchor = NOTES_DIR / "10_motion_planning.md"
        idx = files.index(anchor) + 1 if anchor in files else len(files)
        files.insert(idx, learned)
    return files


def make_md() -> markdown.Markdown:
    return markdown.Markdown(
        extensions=[
            "extra",            # tables, fenced_code, footnotes, attr_list, ...
            "sane_lists",
            "pymdownx.arithmatex",
            "pymdownx.tilde",
            "pymdownx.caret",
            "codehilite",
        ],
        extension_configs={
            "pymdownx.arithmatex": {"generic": True},
            "codehilite": {"guess_lang": False, "noclasses": False},
        },
    )


def absolutize_images(body_html: str, base_dir: Path) -> str:
    """Rewrite relative <img src="..."> to absolute file:// URIs so images
    resolve regardless of where the built HTML lives."""
    def repl(m: re.Match) -> str:
        src = m.group(2)
        if re.match(r"(?:[a-z]+:|//|data:|/)", src):  # already absolute / data / scheme
            return m.group(0)
        abs_path = (base_dir / src).resolve()
        return f'{m.group(1)}{abs_path.as_uri()}{m.group(3)}'
    return re.sub(r'(<img\b[^>]*?\bsrc=")([^"]+)(")', repl, body_html)


def first_h1(text: str, fallback: str) -> str:
    for line in text.splitlines():
        m = re.match(r"#\s+(.*)", line.strip())
        if m:
            return m.group(1).strip()
    return fallback


def build_html(notes: list[Path]) -> str:
    md = make_md()
    chapters = []          # (anchor_id, title, html)
    for i, p in enumerate(notes):
        raw = p.read_text(encoding="utf-8")
        title = first_h1(raw, p.stem)
        anchor = f"ch{i}"
        md.reset()
        body = md.convert(raw)
        body = absolutize_images(body, p.parent)
        chapters.append((anchor, title, body))

    toc_items = "\n".join(
        f'<li><a href="#{a}"><span class="toc-t">{html.escape(t)}</span>'
        f'<span class="toc-f">{html.escape(notes[i].name)}</span></a></li>'
        for i, (a, t, _) in enumerate(chapters)
    )
    chapters_html = "\n".join(
        f'<section class="chapter" id="{a}"><div class="chapter-file">{html.escape(notes[i].name)}</div>\n{b}\n</section>'
        for i, (a, t, b) in enumerate(chapters)
    )

    mathjax_src = MATHJAX.read_text(encoding="utf-8")
    return TEMPLATE.format(
        title=html.escape(TITLE),
        subtitle=html.escape(SUBTITLE),
        n=len(chapters),
        toc=toc_items,
        chapters=chapters_html,
        mathjax=mathjax_src,
    )


def html_to_pdf(html_path: Path, pdf_path: Path) -> None:
    cmd = [
        CHROME, "--headless=new", "--disable-gpu", "--no-sandbox",
        "--no-pdf-header-footer",
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=60000",
        f"--print-to-pdf={pdf_path}",
        html_path.as_uri(),
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--html-only", action="store_true")
    args = ap.parse_args()

    if not MATHJAX.exists():
        sys.exit(f"missing vendored MathJax at {MATHJAX} (run the fetch step)")
    BUILD_DIR.mkdir(exist_ok=True)
    notes = ordered_notes()
    print(f"[build] {len(notes)} notes")
    doc = build_html(notes)
    html_path = BUILD_DIR / "modern_robotics.html"
    html_path.write_text(doc, encoding="utf-8")
    print(f"[build] wrote {html_path} ({len(doc)//1024} KB)")
    if args.html_only:
        return 0
    pdf_path = BUILD_DIR / "modern_robotics.pdf"
    if not Path(CHROME).exists():
        sys.exit(f"Chrome not found at {CHROME}")
    print("[build] rendering PDF via headless Chrome ...")
    html_to_pdf(html_path, pdf_path)
    size = pdf_path.stat().st_size // 1024
    print(f"[build] wrote {pdf_path} ({size} KB)")
    return 0


TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>{title}</title>
<script>
window.MathJax = {{
  tex: {{ inlineMath: [['\\(','\\)']], displayMath: [['\\[','\\]']] }},
  options: {{ ignoreHtmlClass: '.*', processHtmlClass: 'arithmatex' }},
  svg: {{ fontCache: 'global' }},
  startup: {{ typeset: true }}
}};
</script>
<script>{mathjax}</script>
<style>
:root {{ --ink:#1a1a1a; --muted:#666; --rule:#ddd; --accent:#2a5db0; --codebg:#f4f5f7; }}
* {{ box-sizing: border-box; }}
html {{ -webkit-print-color-adjust: exact; print-color-adjust: exact; }}
body {{
  font-family: "Charter","Georgia",serif; color: var(--ink);
  font-size: 10.5pt; line-height: 1.5; margin: 0;
}}
@page {{ size: A4; margin: 16mm 18mm; }}
h1,h2,h3,h4 {{ font-family: "Helvetica Neue",Arial,sans-serif; line-height:1.25; }}
h1 {{ font-size: 19pt; color: var(--accent); margin: 0 0 .2em; }}
h2 {{ font-size: 14pt; border-bottom: 1px solid var(--rule); padding-bottom:.15em; margin-top:1.4em; }}
h3 {{ font-size: 12pt; }}
h4 {{ font-size: 10.5pt; color: var(--muted); }}
p, li {{ orphans: 2; widows: 2; }}
code, pre {{ font-family: "SF Mono","Menlo",monospace; font-size: 9pt; }}
code {{ background: var(--codebg); padding: .08em .3em; border-radius: 3px; }}
pre {{ background: var(--codebg); padding: .7em .9em; border-radius: 6px; overflow-x:auto;
       line-height:1.4; page-break-inside: avoid; }}
pre code {{ background: none; padding: 0; }}
blockquote {{ border-left: 3px solid var(--accent); margin: .8em 0; padding: .1em 1em;
              color: #333; background: #fafbfc; }}
table {{ border-collapse: collapse; margin: .8em 0; font-size: 9.5pt; }}
th, td {{ border: 1px solid var(--rule); padding: .3em .55em; text-align: left; vertical-align: top; }}
th {{ background: #f0f2f5; }}
img {{ max-width: 100%; }}
mjx-container[display="true"] {{ overflow-x: auto; overflow-y: hidden; }}
a {{ color: var(--accent); text-decoration: none; }}

/* code highlighting (codehilite) — muted, print-friendly */
.codehilite .k, .codehilite .kd, .codehilite .kn {{ color:#a626a4; }}
.codehilite .s, .codehilite .s1, .codehilite .s2 {{ color:#50a14f; }}
.codehilite .c, .codehilite .c1, .codehilite .cm {{ color:#a0a1a7; font-style:italic; }}
.codehilite .n, .codehilite .nf {{ color:#1a1a1a; }}
.codehilite .mi, .codehilite .mf {{ color:#986801; }}

/* title page */
.titlepage {{ height: 251mm; display:flex; flex-direction:column; justify-content:center;
  page-break-after: always; text-align:center; }}
.titlepage .big {{ font-family:"Helvetica Neue",Arial,sans-serif; font-size:40pt; font-weight:700;
  color:var(--accent); letter-spacing:-.5px; }}
.titlepage .sub {{ font-family:"Helvetica Neue",Arial,sans-serif; font-size:13pt; color:var(--muted);
  margin-top:.6em; }}
.titlepage .meta {{ margin-top:2.5em; font-size:10pt; color:var(--muted); }}

/* toc */
.toc {{ page-break-after: always; }}
.toc h2 {{ border:0; }}
.toc ol {{ list-style:none; padding:0; counter-reset: toc; }}
.toc li {{ counter-increment: toc; margin:.28em 0; }}
.toc a {{ display:flex; align-items:baseline; gap:.6em; color:var(--ink); }}
.toc a::before {{ content: counter(toc) "."; color:var(--muted); font-variant-numeric:tabular-nums;
  min-width:1.8em; text-align:right; font-family:"Helvetica Neue",Arial,sans-serif; }}
.toc-t {{ font-weight:600; }}
.toc-f {{ margin-left:auto; color:var(--muted); font-size:8.5pt; font-family:"SF Mono",monospace; }}

.chapter {{ page-break-before: always; }}
.chapter-file {{ font-family:"SF Mono",monospace; font-size:8pt; color:#aaa; margin-bottom:.4em; }}
</style>
</head>
<body>
<div class="titlepage">
  <div class="big">{title}</div>
  <div class="sub">{subtitle}</div>
  <div class="meta">{n} topic notes &middot; compiled study copy</div>
</div>
<nav class="toc">
  <h2>Contents</h2>
  <ol>{toc}</ol>
</nav>
{chapters}
</body>
</html>
"""

if __name__ == "__main__":
    raise SystemExit(main())
