"""Modelica 结构自检：括号配平、class/end 名字配对、引号配平、常见笔误。"""
import re, sys
from pathlib import Path

KW = r"(?:package|model|block|record|connector|function|type)"

def strip_noncode(src):
    """去掉字符串字面量和注释，避免它们里的括号干扰计数。"""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if c == '"':
            i += 1
            while i < n:
                if src[i] == '\\':
                    i += 2; continue
                if src[i] == '"':
                    i += 1; break
                i += 1
            out.append(' ')
            continue
        if src.startswith("//", i):
            j = src.find("\n", i); i = n if j < 0 else j; continue
        if src.startswith("/*", i):
            j = src.find("*/", i); i = n if j < 0 else j + 2; continue
        out.append(c); i += 1
    return "".join(out)

def check(path):
    src = Path(path).read_text(encoding="utf-8")
    code = strip_noncode(src)
    errs, warns = [], []

    # 引号配平
    q = 0; i = 0
    while i < len(src):
        if src[i] == '\\': i += 2; continue
        if src[i] == '"': q += 1
        i += 1
    if q % 2: errs.append(f"双引号数量为奇数 ({q})，字符串未闭合")

    for name, o, c in [("圆括号", "(", ")"), ("方括号", "[", "]"), ("花括号", "{", "}")]:
        a, b = code.count(o), code.count(c)
        if a != b: errs.append(f"{name}不配平: {o}={a} {c}={b}")

    # class / end 配对
    stack, lineno = [], 0
    decl = re.compile(rf"^\s*(?:partial\s+|replaceable\s+)*({KW})\s+([A-Za-z_]\w*)")
    endre = re.compile(r"^\s*end\s+([A-Za-z_]\w*)\s*;")
    CTRL = {"for", "if", "when", "while"}   # 控制结构的 end，不是类的 end
    for line in code.splitlines():
        lineno += 1
        m = decl.match(line)
        if m and not re.search(r"\bextends\b", line):
            stack.append((m.group(2), lineno)); continue
        m = endre.match(line)
        if m:
            if m.group(1) in CTRL:
                continue
            if not stack:
                errs.append(f"第{lineno}行 end {m.group(1)}; 没有对应的声明")
            else:
                nm, ln = stack.pop()
                if nm != m.group(1):
                    errs.append(f"第{lineno}行 end {m.group(1)}; 与第{ln}行的 {nm} 不匹配")
    for nm, ln in stack:
        errs.append(f"第{ln}行声明的 {nm} 没有 end")

    # 常见笔误
    for i, line in enumerate(src.splitlines(), 1):
        s = line.strip()
        if re.match(r"^(parameter|output|input|protected\s+)?\s*Real\s+\w+.*[^;{,(]$", s) \
           and not s.endswith((";", ",", "(", "{", '"')) and "annotation" not in s:
            warns.append(f"第{i}行可能缺分号: {s[:70]}")

    ndecl = len(re.findall(r"^\s*(?:partial\s+|replaceable\s+)*" + KW + r"\s+", code, re.M))
    print(f"\n===== {Path(path).name} =====")
    print(f"  行数 {len(src.splitlines())}  类声明 {ndecl}")
    if errs:
        print("  [错误]"); [print("   -", e) for e in errs]
    else:
        print("  [OK] 括号/引号配平，class-end 全部配对")
    if warns:
        print("  [提示]"); [print("   -", w) for w in warns[:8]]
    return len(errs)

bad = sum(check(p) for p in sys.argv[1:])
sys.exit(1 if bad else 0)
