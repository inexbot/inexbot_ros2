import re, glob, os, csv

INC = "/opt/nrc_sdk/include/cpp/interface"
rows = []

for path in sorted(glob.glob(f"{INC}/*.h")):
    fname = os.path.basename(path)
    text = open(path, encoding="utf-8", errors="ignore").read()
    # 匹配: 可选的 /** ... */ 注释 + EXPORT_API 声明
    pattern = re.compile(r"(/\*\*(?:(?!\*/).)*?\*/\s*)?EXPORT_API\s+([\w:<>,\s\*&]+?)\s+(\w+)\s*\(([^;]*?)\)\s*;", re.S)
    for m in pattern.finditer(text):
        comment, ret, name, params = m.groups()
        if name.endswith("_robot"):
            continue  # 多机器人变体，与基础版合并统计
        brief = ""
        if comment:
            b = re.search(r"@brief\s+(.+)", comment)
            brief = b.group(1).strip() if b else ""
        params = " ".join(params.split())
        blocking = "是" if re.search(r"阻塞|同步", comment or "") else ""
        rows.append([fname, name, ret.strip(), params, brief, blocking])

out = os.path.expanduser("~/inexbot_ws/docs/sdk_api_list.csv")
with open(out, "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["头文件", "函数", "返回值", "参数", "说明", "注释提及阻塞"])
    w.writerows(rows)

# 按头文件统计
from collections import Counter
c = Counter(r[0] for r in rows)
print(f"共 {len(rows)} 个函数（不含 _robot 变体）\n")
for k, v in sorted(c.items(), key=lambda x: -x[1]):
    print(f"{v:4d}  {k}")
print(f"\n已输出: {out}")
