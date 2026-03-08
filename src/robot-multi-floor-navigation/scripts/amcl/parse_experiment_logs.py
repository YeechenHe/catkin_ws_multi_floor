#!/usr/bin/env python3
# 解析初值扰动实验日志：提取成功率、收敛时间、收敛时误差、重试次数
# 用法: python3 parse_experiment_logs.py <log_dir>
# 输出: CSV 到 stdout 及 summary 文本（含收敛时间与误差的 mean±std）

import os
import re
import sys
from collections import defaultdict

def parse_log(path):
    with open(path, "r", errors="replace") as f:
        lines = f.readlines()
    text = "".join(lines)
    reloc_ok = len(re.findall(r"Robot at correct pose", text))
    task_ok = "Robot arrived at Goal" in text
    retries = len(re.findall(r"Setting initial pose failed", text))

    # 从日志行解析 sim_time 与 Linear/Angular Error（格式: [ INFO] [wall, sim_time]: ...）
    sim_times = []
    linear_errors = []
    angular_errors = []
    for i, line in enumerate(lines):
        m_ts = re.search(r"\[\s*INFO\]\s*\[\d+\.\d+,\s*([\d.]+)\]", line)
        m_err = re.search(r"Linear Error:\s*([\d.]+),\s*Angular Error:\s*([\d.]+)", line)
        if m_ts and "Robot at correct pose" in line:
            sim_times.append(float(m_ts.group(1)))
        if m_err:
            linear_errors.append(float(m_err.group(1)))
            angular_errors.append(float(m_err.group(2)))

    # 收敛时间：从 "Successfully loaded map" 的 sim_time 到紧接着的 "Robot at correct pose" 的 sim_time 做差
    t_load_L0 = t_load_L1 = None
    for line in lines:
        if "Successfully loaded map for level: 0" in line:
            m = re.search(r",\s*([\d.]+)\]:", line)
            if m:
                t_load_L0 = float(m.group(1))
        if "Successfully loaded map for level: 1" in line:
            m = re.search(r",\s*([\d.]+)\]:", line)
            if m:
                t_load_L1 = float(m.group(1))
    dt_L0 = dt_L1 = None
    if t_load_L0 is not None and len(sim_times) >= 1:
        dt_L0 = round(sim_times[0] - t_load_L0, 3)
    if t_load_L1 is not None and len(sim_times) >= 2:
        dt_L1 = round(sim_times[1] - t_load_L1, 3)
    lin_L0 = lin_L1 = ang_L0 = ang_L1 = None
    if len(linear_errors) >= 1:
        lin_L0, ang_L0 = linear_errors[0], angular_errors[0]
    if len(linear_errors) >= 2 and len(angular_errors) >= 2:
        lin_L1, ang_L1 = linear_errors[1], angular_errors[1]

    return {
        "reloc_ok": reloc_ok, "task_ok": task_ok, "retries": retries,
        "dt_L0": dt_L0, "dt_L1": dt_L1,
        "linear_L0": lin_L0, "linear_L1": lin_L1, "angular_L0": ang_L0, "angular_L1": ang_L1,
    }

def main():
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    if not os.path.isdir(log_dir):
        print("Usage: parse_experiment_logs.py <log_dir>", file=sys.stderr)
        sys.exit(1)

    # 按 method, pert, run 收集
    rows = []
    by_key = defaultdict(list)
    for fn in sorted(os.listdir(log_dir)):
        if not fn.endswith(".log"):
            continue
        # baseline_pert_* / spamcl_pert_* / apamcl_pert_*（模块 B）/ relocc_pert_*（模块 C）
        m = re.match(r"(baseline|spamcl|apamcl|relocc)_pert_([\d.]+)_([\d.]+)_([\d.]+)_run(\d+)\.log", fn)
        if not m:
            continue
        method, ox, oy, ot, run_id = m.group(1), m.group(2), m.group(3), m.group(4), m.group(5)
        pert_name = f"{ox}_{oy}_{ot}"
        if (ox, oy, ot) == ("0", "0", "0"):
            pert_name = "0"
        path = os.path.join(log_dir, fn)
        r = parse_log(path)
        r["method"] = method
        r["pert"] = pert_name
        r["run_id"] = run_id
        rows.append(r)
        by_key[(method, pert_name)].append(r)

    # CSV 表头（含收敛时间与误差，空为缺失）
    print("method,pert,run_id,reloc_ok,task_ok,retries,dt_L0_s,dt_L1_s,linear_L0_m,linear_L1_m,angular_L0_rad,angular_L1_rad")
    for r in rows:
        dt0 = r.get("dt_L0") if r.get("dt_L0") is not None else ""
        dt1 = r.get("dt_L1") if r.get("dt_L1") is not None else ""
        l0 = r.get("linear_L0") if r.get("linear_L0") is not None else ""
        l1 = r.get("linear_L1") if r.get("linear_L1") is not None else ""
        a0 = r.get("angular_L0") if r.get("angular_L0") is not None else ""
        a1 = r.get("angular_L1") if r.get("angular_L1") is not None else ""
        print(f"{r['method']},{r['pert']},{r['run_id']},{r['reloc_ok']},{r['task_ok']},{r['retries']},{dt0},{dt1},{l0},{l1},{a0},{a1}")

    # Summary: 成功率
    print("\n# Summary (reloc_ok: 2 = both L0&L1 passed)")
    for (method, pert), list_r in sorted(by_key.items()):
        n = len(list_r)
        task_success = sum(1 for r in list_r if r["task_ok"])
        reloc_full = sum(1 for r in list_r if r["reloc_ok"] >= 2)
        total_retries = sum(r["retries"] for r in list_r)
        print(f"  {method} pert={pert}: n={n} task_ok={task_success}/{n} reloc_2ok={reloc_full}/{n} total_retries={total_retries}")

    # Summary: 收敛时间与误差（mean±std，仅对有效值）
    def mean_std(vals):
        vals = [x for x in vals if x is not None]
        if not vals:
            return None, None
        m = sum(vals) / len(vals)
        v = sum((x - m) ** 2 for x in vals) / len(vals)
        s = v ** 0.5 if v > 0 else 0
        return round(m, 4), round(s, 4)

    print("\n# Convergence time (s) and error (m/rad) mean±std")
    for (method, pert), list_r in sorted(by_key.items()):
        dt_L0 = [r.get("dt_L0") for r in list_r]
        dt_L1 = [r.get("dt_L1") for r in list_r]
        lin_L0 = [r.get("linear_L0") for r in list_r]
        lin_L1 = [r.get("linear_L1") for r in list_r]
        ang_L0 = [r.get("angular_L0") for r in list_r]
        ang_L1 = [r.get("angular_L1") for r in list_r]
        m_dt0, s_dt0 = mean_std(dt_L0)
        m_dt1, s_dt1 = mean_std(dt_L1)
        m_lin0, s_lin0 = mean_std(lin_L0)
        m_lin1, s_lin1 = mean_std(lin_L1)
        m_ang0, s_ang0 = mean_std(ang_L0)
        m_ang1, s_ang1 = mean_std(ang_L1)
        parts = [f"  {method} pert={pert}:"]
        if m_dt0 is not None:
            parts.append(f" dt_L0={m_dt0}±{s_dt0}s dt_L1={m_dt1}±{s_dt1}s")
        if m_lin0 is not None:
            parts.append(f" linear_L0={m_lin0}±{s_lin0}m linear_L1={m_lin1}±{s_lin1}m angular_L0={m_ang0}±{s_ang0}rad angular_L1={m_ang1}±{s_ang1}rad")
        print("".join(parts))

if __name__ == "__main__":
    main()
