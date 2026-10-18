#!/usr/bin/env python3
"""Automated scan for the Barr-C:2018 rules that a tool can check.

Usage: python3 tools/barr_check.py [paths...]   (default: src tests config)
Exit status 0 = no findings. Rules that need a human (comment quality,
naming sense, casts explained) stay with the code review.
"""
import os
import re
import sys

MAX_WIDTH = 80
SKIP_DIRS = {"soup", "build", ".git"}
SKIP_FILES = set()
NAME_RE = re.compile(r"^[a-z0-9_]+\.(c|h)$")
KEYWORDS = re.compile(r"\b(goto|continue|auto|register|short|long)\b")
RAW_INT = re.compile(r"\b(int|char|unsigned|signed)\b(?!\s*\*\s*\w+\s*\[)")
CONTROL = re.compile(r"^(if|else if|for|while|switch)\b.*\)\s*$")
CONTROL_SAME_LINE = re.compile(
    r"^(if|else if|for|while|switch)\b.*\)\s*[^\s{;].*;\s*$")
EQ_TEST = re.compile(r"([\w\.\->\[\]]+)\)?\s*(==|!=)\s*\(?\s*([\w\.\->]+)")
CONST_TOKEN = re.compile(
    r"^(\d+[uU]?|0x[0-9A-Fa-f]+[uU]?|NULL|true|false|[A-Z][A-Z0-9_]{2,})$")
FUNC_DEF = re.compile(r"^([a-z][a-z0-9_]*) \(")

findings = []


def module_of(root):
    """Map ld2450_cfg / ld2450_lcfg to module ld2450 (D-002).

    ld2450_irq is a private part of the ld2450 driver but has its own header
    and its own function prefix, so it stays a module of its own.
    """
    return re.sub(r"_(cfg|lcfg)$", "", root)


def report(path, line_no, rule, text):
    findings.append("%s:%d: [%s] %s" % (path, line_no, rule, text))


def strip_code(line):
    """Remove strings and comments so keyword scans see only code."""
    line = re.sub(r'"(\\.|[^"\\])*"', '""', line)
    line = re.sub(r"//.*$", "", line)
    line = re.sub(r"/\*.*?\*/", "", line)
    return line


def check_file(path):
    name = os.path.basename(path)
    if name in SKIP_FILES:
        return
    root = os.path.splitext(name)[0]
    if not NAME_RE.match(name):
        report(path, 1, "4.1.a", "file name must be lowercase/digits/_")
    with open(path, "rb") as handle:
        raw = handle.read()
    if b"\r" in raw:
        report(path, 1, "3.6.a", "CR character found")
    if b"\t" in raw:
        report(path, 1, "3.5.a", "tab character found")
    lines = raw.decode("utf-8").split("\n")
    if lines[-1] != "" or not lines[-2].startswith("/*** end of file ***/"):
        if not (lines[-1] == "" and lines[-2] == "/*** end of file ***/"):
            report(path, len(lines), "3.3.c", "missing end-of-file marker")
    in_block = False
    code_lines = []
    for idx, line in enumerate(lines, 1):
        if len(line) > MAX_WIDTH:
            report(path, idx, "1.2.a", "line is %d chars" % len(line))
        if line.rstrip() != line:
            report(path, idx, "3.x", "trailing white space")
        code = line
        if in_block:
            if "*/" in code:
                in_block = False
                code = code.split("*/", 1)[1]
            else:
                code = ""
        if "/*" in code and "*/" not in code.split("/*", 1)[1]:
            in_block = True
            code = code.split("/*", 1)[0]
        code_lines.append(strip_code(code))
    if path.endswith(".h"):
        guard = root.upper() + "_H"
        text = "\n".join(lines)
        if ("#ifndef %s" % guard) not in text or \
           ("#define %s" % guard) not in text:
            report(path, 1, "4.2.b", "header guard %s missing" % guard)
    prefix_ok = root
    for idx, code in enumerate(code_lines, 1):
        stripped = code.strip()
        if KEYWORDS.search(code):
            report(path, idx, "1.7/5.2", "forbidden keyword: " + stripped)
        next_is_main = idx < len(code_lines) and \
            code_lines[idx].strip().startswith(("main", "usermain"))
        if RAW_INT.search(code) and not (stripped == "int" and next_is_main) \
                and not stripped.startswith("#"):
            report(path, idx, "5.2.a", "raw C type: " + stripped)
        if CONTROL_SAME_LINE.match(stripped):
            report(path, idx, "1.3.a", "statement on control line")
        if CONTROL.match(stripped) or stripped in ("else", "do"):
            nxt = idx
            while nxt < len(code_lines) and code_lines[nxt].strip() == "":
                nxt += 1
            if nxt >= len(code_lines) or code_lines[nxt].strip() != "{":
                report(path, idx, "1.3.a/b", "brace not on next line")
        if stripped.endswith("{") and stripped != "{" and \
                re.match(r"^(if|else|for|while|switch|do)\b", stripped):
            report(path, idx, "1.3.b", "left brace not on its own line")
        if not stripped.startswith("#"):
            for left, _op, right in EQ_TEST.findall(code):
                if CONST_TOKEN.match(right) and not CONST_TOKEN.match(left):
                    report(path, idx, "8.6.a",
                           "constant right of ==/!=: " + stripped)
        if re.search(r"\bcase\b.*:\s*\S", stripped) and \
                not stripped.endswith(":"):
            pass
        if code.count(";") > 1 and not re.match(r"^\s*for\b", code):
            report(path, idx, "3.3.a", "more than one statement per line")
        match = FUNC_DEF.match(code)
        if match and idx >= 2 and path.endswith(".c"):
            func = match.group(1)
            prev = code_lines[idx - 2].strip()
            is_static = prev.startswith("static")
            in_tests = "/tests/" in path
            if not is_static and not func.startswith(module_of(root)) \
                    and func not in ("main", "usermain", "setUp", "tearDown") \
                    and not in_tests:
                report(path, idx, "6.1.i", "public %s lacks prefix %s_" %
                       (func, module_of(root)))
            if (len(func) > 31 and not in_tests) or func.startswith("_"):
                report(path, idx, "6.1.c/d", "bad function name " + func)
    # Function length (6.2.a): from a column-0 "{" to the matching "}".
    start = None
    for idx, line in enumerate(lines, 1):
        if line == "{" and start is None:
            start = idx
        elif line == "}" and start is not None:
            if idx - start > 100:
                report(path, start, "6.2.a", "function longer than 100 lines")
            start = None


def walk(paths):
    for base in paths:
        if os.path.isfile(base):
            yield base
            continue
        for folder, dirs, files in os.walk(base):
            dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
            for fname in sorted(files):
                if fname.endswith((".c", ".h")):
                    yield os.path.join(folder, fname)


def check_unique_modules(paths):
    seen = {}
    for path in paths:
        root = os.path.splitext(os.path.basename(path))[0]
        if os.path.basename(path) in SKIP_FILES:
            continue
        root = module_of(root)
        key = root[:8]
        seen.setdefault(key, set()).add(root)
    for key, roots in sorted(seen.items()):
        if len(roots) > 1:
            findings.append("module names share first 8 chars '%s': %s "
                            "[4.1.b]" %
                            (key, ", ".join(sorted(roots))))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    top = os.path.dirname(here)
    args = sys.argv[1:] or [os.path.join(top, d)
                            for d in ("src", "tests", "config")]
    files = list(walk(args))
    for path in files:
        check_file(path)
    check_unique_modules(files)
    for line in findings:
        print(line)
    print("%d files checked, %d findings" % (len(files), len(findings)))
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main())
