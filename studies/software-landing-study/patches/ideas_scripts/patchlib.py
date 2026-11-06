"""Exact-match file edits for the sw/ideas workspace (keeps CRLF endings)."""
import os
import sys

SCR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = os.path.join(SCR, "sw", "ideas")


def edit(rel, old, new, count=1):
    p = os.path.join(WS, rel)
    raw = open(p, "rb").read().decode("utf-8")
    crlf = "\r\n" in raw
    txt = raw.replace("\r\n", "\n")
    n = txt.count(old)
    if n != count:
        sys.exit("%s: expected %d match(es), found %d for:\n%s" % (rel, count, n, old[:200]))
    txt = txt.replace(old, new)
    if crlf:
        txt = txt.replace("\n", "\r\n")
    open(p, "wb").write(txt.encode("utf-8"))
    print("patched", rel)
