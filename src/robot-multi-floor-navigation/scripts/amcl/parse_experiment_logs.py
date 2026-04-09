#!/usr/bin/env python3
# 解析初值扰动实验日志（v3）：提取成功率、收敛时间、通过时误差、通过后稳定性、误通过率
# 用法: python3 parse_experiment_logs.py <log_dir>
# 输出: CSV 到 stdout 及 summary 文本
#
# 误通过 / FAR（启发式，可论文中并列报告）：
#   false_accept（FA_task）：两段均宣告到位（reloc_ok>=2）但最终任务失败（与 POST_PASS 窗无关）。
#   fa_post_spike：reloc_ok>=2 且任一层 max_linear_after > e_pass_linear + FA_POST_SPIKE_DELTA_M
#                 （通过后短窗内线误差相对通过时刻「明显」增大；可能与起步运动耦合，解释需谨慎）。
#   fa_any：上述二者之一。
FA_POST_SPIKE_DELTA_M = 0.15
# 无平面扰动时 e_pass 可极小，POST_PASS 窗内起步易使 max_linear ≫ e_pass，误标 spike；仅当通过时刻线误差已具尺度时才判
FA_POST_SPIKE_MIN_EPASS_M = 0.05

import os
import re
import sys
import math
from collections import defaultdict


def pert_planar_offset(pert):
    """Parse perturbation name from log filename into (dx, dy, planar_norm).

    pert is e.g. "0", "0.2_0.2_0.1", "0.3_0.3_0.15".
    Returns (None, None, None) if no planar offset (none perturbation or parse fail).
    """
    if not pert or pert == "0":
        return None, None, None
    parts = str(pert).split("_")
    if len(parts) < 2:
        return None, None, None
    try:
        dx = float(parts[0])
        dy = float(parts[1])
        off = math.hypot(dx, dy)
        return dx, dy, off
    except ValueError:
        return None, None, None


def corr_lin_ratio(e_pass_linear, planar_offset):
    """corr_lin = 1 - e_pass_linear / ||(dx,dy)||; None if undefined."""
    if e_pass_linear is None or planar_offset is None or planar_offset < 1e-9:
        return None
    return round(1.0 - float(e_pass_linear) / planar_offset, 4)


def mean_std(vals):
    vals = [x for x in vals if x is not None]
    if not vals:
        return None, None
    m = sum(vals) / len(vals)
    v = sum((x - m) ** 2 for x in vals) / len(vals)
    s = v ** 0.5 if v > 0 else 0
    return round(m, 4), round(s, 4)


def parse_log(path):
    with open(path, "r", errors="replace") as f:
        lines = f.readlines()
    text = "".join(lines)
    reloc_ok = len(re.findall(r"Robot at correct pose", text))
    task_ok = "Robot arrived at Goal" in text
    retries = len(re.findall(r"Setting initial pose failed", text))

    sim_times = []
    linear_errors = []
    angular_errors = []
    for line in lines:
        m_ts = re.search(r"\[\s*INFO\]\s*\[\d+\.\d+,\s*([\d.]+)\]", line)
        m_err = re.search(r"Linear Error:\s*([\d.]+),\s*Angular Error:\s*([\d.]+)", line)
        if m_ts and "Robot at correct pose" in line:
            sim_times.append(float(m_ts.group(1)))
        if m_err:
            linear_errors.append(float(m_err.group(1)))
            angular_errors.append(float(m_err.group(2)))

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

    # --- v3 新指标 ---
    # RELOC_PASS:
    # new format:
    #   linear_error=..., angular_error=..., linear_error_init=..., angular_error_init=..., tr_sigma=..., C_t=...
    # old format (backward compatible):
    #   linear_error=..., angular_error=..., tr_sigma=..., C_t=...
    reloc_pass_entries = []
    for line in lines:
        mp_new = re.search(
            r"RELOC_PASS: linear_error=([\d.]+), angular_error=([\d.]+), linear_error_init=([\d.]+), angular_error_init=([\d.]+), tr_sigma=(-?[\d.]+), C_t=([\d.]+)",
            line)
        if mp_new:
            reloc_pass_entries.append({
                "linear": float(mp_new.group(1)),
                "angular": float(mp_new.group(2)),
                "linear_init": float(mp_new.group(3)),
                "angular_init": float(mp_new.group(4)),
                "tr_sigma": float(mp_new.group(5)),
                "C_t": float(mp_new.group(6)),
            })
            continue

        mp_old = re.search(
            r"RELOC_PASS: linear_error=([\d.]+), angular_error=([\d.]+), tr_sigma=(-?[\d.]+), C_t=([\d.]+)",
            line)
        if mp_old:
            reloc_pass_entries.append({
                "linear": float(mp_old.group(1)),
                "angular": float(mp_old.group(2)),
                "linear_init": None,
                "angular_init": None,
                "tr_sigma": float(mp_old.group(3)),
                "C_t": float(mp_old.group(4)),
            })

    e_pass_linear_L0 = reloc_pass_entries[0]["linear"] if len(reloc_pass_entries) >= 1 else None
    e_pass_yaw_L0 = reloc_pass_entries[0]["angular"] if len(reloc_pass_entries) >= 1 else None
    e_pass_linear_L1 = reloc_pass_entries[1]["linear"] if len(reloc_pass_entries) >= 2 else None
    e_pass_yaw_L1 = reloc_pass_entries[1]["angular"] if len(reloc_pass_entries) >= 2 else None
    e_pass_linear_init_L0 = reloc_pass_entries[0]["linear_init"] if len(reloc_pass_entries) >= 1 else None
    e_pass_yaw_init_L0 = reloc_pass_entries[0]["angular_init"] if len(reloc_pass_entries) >= 1 else None
    e_pass_linear_init_L1 = reloc_pass_entries[1]["linear_init"] if len(reloc_pass_entries) >= 2 else None
    e_pass_yaw_init_L1 = reloc_pass_entries[1]["angular_init"] if len(reloc_pass_entries) >= 2 else None
    e_tr_sigma_L0 = reloc_pass_entries[0]["tr_sigma"] if len(reloc_pass_entries) >= 1 else None
    e_C_t_L0 = reloc_pass_entries[0]["C_t"] if len(reloc_pass_entries) >= 1 else None
    e_tr_sigma_L1 = reloc_pass_entries[1]["tr_sigma"] if len(reloc_pass_entries) >= 2 else None
    e_C_t_L1 = reloc_pass_entries[1]["C_t"] if len(reloc_pass_entries) >= 2 else None

    # POST_PASS: dt=X.XXXs, linear_error=X.XXXX, angular_error=X.XXXX
    # Group POST_PASS lines by which reloc pass they belong to (L0 or L1).
    # Strategy: collect all POST_PASS lines; split into groups by RELOC_PASS markers.
    post_pass_groups = []
    current_group = []
    for line in lines:
        if re.search(r"RELOC_PASS:", line):
            if current_group:
                post_pass_groups.append(current_group)
            current_group = []
        mpp = re.search(
            r"POST_PASS: dt=([\d.]+)s, linear_error=([\d.]+), angular_error=([\d.]+)",
            line)
        if mpp:
            current_group.append({
                "dt": float(mpp.group(1)),
                "linear": float(mpp.group(2)),
                "angular": float(mpp.group(3)),
            })
    if current_group:
        post_pass_groups.append(current_group)

    def compute_after_pass_stats(group):
        if not group:
            return {}
        lins = [e["linear"] for e in group]
        angs = [e["angular"] for e in group]
        m_l, s_l = mean_std(lins)
        m_a, s_a = mean_std(angs)
        return {
            "mean_linear": m_l,
            "max_linear": round(max(lins), 4) if lins else None,
            "std_linear": s_l,
            "mean_yaw": m_a,
            "max_yaw": round(max(angs), 4) if angs else None,
        }

    after_L0 = compute_after_pass_stats(post_pass_groups[0]) if len(post_pass_groups) >= 1 else {}
    after_L1 = compute_after_pass_stats(post_pass_groups[1]) if len(post_pass_groups) >= 2 else {}

    # false_accept: reloc passed (both L0 & L1) but task ultimately failed.
    # POST_PASS window is intentionally excluded: the robot starts moving immediately
    # after reloc, so position drift in that window reflects navigation motion, not
    # localization instability.
    false_accept = 1 if (reloc_ok >= 2 and not task_ok) else 0

    fa_post_spike = 0
    if reloc_ok >= 2:
        if (
            e_pass_linear_L0 is not None
            and e_pass_linear_L0 >= FA_POST_SPIKE_MIN_EPASS_M
            and after_L0.get("max_linear") is not None
            and after_L0["max_linear"] > e_pass_linear_L0 + FA_POST_SPIKE_DELTA_M
        ):
            fa_post_spike = 1
        if (
            e_pass_linear_L1 is not None
            and e_pass_linear_L1 >= FA_POST_SPIKE_MIN_EPASS_M
            and after_L1.get("max_linear") is not None
            and after_L1["max_linear"] > e_pass_linear_L1 + FA_POST_SPIKE_DELTA_M
        ):
            fa_post_spike = 1
    fa_any = 1 if (false_accept or fa_post_spike) else 0

    # Hard Localization Failure (HLF): algorithm declares RELOC_PASS but true-value
    # pose error already exceeds safety thresholds at that moment.
    # Thresholds: linear > 0.45 m OR yaw > 0.15 rad.
    # Applied per layer independently; only layers with a RELOC_PASS event are scored.
    HLF_LIN = 0.45   # m
    HLF_YAW = 0.15   # rad

    def _hlf(lin, yaw):
        """Return 1 if this RELOC_PASS event is a hard localization failure, else 0.
        Returns None if no pass event exists for this layer."""
        if lin is None:
            return None
        return 1 if (lin > HLF_LIN or (yaw is not None and yaw > HLF_YAW)) else 0

    hlf_L0 = _hlf(e_pass_linear_L0, e_pass_yaw_L0)
    hlf_L1 = _hlf(e_pass_linear_L1, e_pass_yaw_L1)
    # Combined: 1 if ANY passed layer is HLF; 0 if all passed layers are safe; None if no pass at all
    passed_layers_hlf = [v for v in [hlf_L0, hlf_L1] if v is not None]
    hlf_any = max(passed_layers_hlf) if passed_layers_hlf else None

    return {
        "reloc_ok": reloc_ok, "task_ok": task_ok, "retries": retries,
        "dt_L0": dt_L0, "dt_L1": dt_L1,
        "linear_L0": lin_L0, "linear_L1": lin_L1, "angular_L0": ang_L0, "angular_L1": ang_L1,
        "e_pass_linear_L0": e_pass_linear_L0, "e_pass_yaw_L0": e_pass_yaw_L0,
        "e_tr_sigma_L0": e_tr_sigma_L0, "e_C_t_L0": e_C_t_L0,
        "e_pass_linear_L1": e_pass_linear_L1, "e_pass_yaw_L1": e_pass_yaw_L1,
        "e_tr_sigma_L1": e_tr_sigma_L1, "e_C_t_L1": e_C_t_L1,
        "e_pass_linear_init_L0": e_pass_linear_init_L0, "e_pass_yaw_init_L0": e_pass_yaw_init_L0,
        "e_pass_linear_init_L1": e_pass_linear_init_L1, "e_pass_yaw_init_L1": e_pass_yaw_init_L1,
        "mean_linear_after_L0": after_L0.get("mean_linear"),
        "max_linear_after_L0": after_L0.get("max_linear"),
        "std_linear_after_L0": after_L0.get("std_linear"),
        "mean_yaw_after_L0": after_L0.get("mean_yaw"),
        "max_yaw_after_L0": after_L0.get("max_yaw"),
        "mean_linear_after_L1": after_L1.get("mean_linear"),
        "max_linear_after_L1": after_L1.get("max_linear"),
        "std_linear_after_L1": after_L1.get("std_linear"),
        "mean_yaw_after_L1": after_L1.get("mean_yaw"),
        "max_yaw_after_L1": after_L1.get("max_yaw"),
        "false_accept": false_accept,
        "fa_post_spike": fa_post_spike,
        "fa_any": fa_any,
        "hlf_L0": hlf_L0,
        "hlf_L1": hlf_L1,
        "hlf_any": hlf_any,
    }


def fmt(v):
    return "" if v is None else str(v)


def main():
    log_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    if not os.path.isdir(log_dir):
        print("Usage: parse_experiment_logs.py <log_dir>", file=sys.stderr)
        sys.exit(1)

    rows = []
    by_key = defaultdict(list)
    for fn in sorted(os.listdir(log_dir)):
        if not fn.endswith(".log"):
            continue
        # Accept both "spamcl" and "sp_amcl" log prefixes (historical naming inconsistency).
        # Also accept combined method "sp_amcl_c" for convenience.
        m = re.match(r"(baseline|spamcl|sp_amcl|apamcl|relocc|sp_amcl_c)_pert_([\d.]+)_([\d.]+)_([\d.]+)_run(\d+)\.log", fn)
        if not m:
            continue
        method, ox, oy, ot, run_id = m.group(1), m.group(2), m.group(3), m.group(4), m.group(5)
        if method == "sp_amcl":
            method = "spamcl"
        pert_name = f"{ox}_{oy}_{ot}"
        if (ox, oy, ot) == ("0", "0", "0"):
            pert_name = "0"
        path = os.path.join(log_dir, fn)
        r = parse_log(path)
        r["method"] = method
        r["pert"] = pert_name
        r["run_id"] = run_id
        _dx, _dy, poff = pert_planar_offset(pert_name)
        r["pert_offset_xy_m"] = round(poff, 6) if poff is not None else None
        r["corr_lin_L0"] = corr_lin_ratio(r.get("e_pass_linear_L0"), poff)
        r["corr_lin_L1"] = corr_lin_ratio(r.get("e_pass_linear_L1"), poff)
        rows.append(r)
        by_key[(method, pert_name)].append(r)

    # CSV header
    cols = [
        "method", "pert", "run_id", "reloc_ok", "task_ok", "retries",
        "dt_L0_s", "dt_L1_s", "linear_L0_m", "linear_L1_m", "angular_L0_rad", "angular_L1_rad",
        "e_pass_linear_L0", "e_pass_yaw_L0", "e_tr_sigma_L0", "e_C_t_L0",
        "e_pass_linear_L1", "e_pass_yaw_L1", "e_tr_sigma_L1", "e_C_t_L1",
        "e_pass_linear_init_L0", "e_pass_yaw_init_L0", "e_pass_linear_init_L1", "e_pass_yaw_init_L1",
        "mean_linear_after_L0", "max_linear_after_L0", "std_linear_after_L0",
        "mean_linear_after_L1", "max_linear_after_L1", "std_linear_after_L1",
        "false_accept", "fa_post_spike", "fa_any",
        "hlf_L0", "hlf_L1", "hlf_any",
        "pert_offset_xy_m", "corr_lin_L0", "corr_lin_L1",
    ]
    print(",".join(cols))
    for r in rows:
        vals = [
            r["method"], r["pert"], r["run_id"], str(r["reloc_ok"]), str(r["task_ok"]), str(r["retries"]),
            fmt(r.get("dt_L0")), fmt(r.get("dt_L1")),
            fmt(r.get("linear_L0")), fmt(r.get("linear_L1")),
            fmt(r.get("angular_L0")), fmt(r.get("angular_L1")),
            fmt(r.get("e_pass_linear_L0")), fmt(r.get("e_pass_yaw_L0")),
            fmt(r.get("e_tr_sigma_L0")), fmt(r.get("e_C_t_L0")),
            fmt(r.get("e_pass_linear_L1")), fmt(r.get("e_pass_yaw_L1")),
            fmt(r.get("e_tr_sigma_L1")), fmt(r.get("e_C_t_L1")),
            fmt(r.get("e_pass_linear_init_L0")), fmt(r.get("e_pass_yaw_init_L0")),
            fmt(r.get("e_pass_linear_init_L1")), fmt(r.get("e_pass_yaw_init_L1")),
            fmt(r.get("mean_linear_after_L0")), fmt(r.get("max_linear_after_L0")), fmt(r.get("std_linear_after_L0")),
            fmt(r.get("mean_linear_after_L1")), fmt(r.get("max_linear_after_L1")), fmt(r.get("std_linear_after_L1")),
            str(r.get("false_accept", 0)),
            str(r.get("fa_post_spike", 0)),
            str(r.get("fa_any", 0)),
            fmt(r.get("hlf_L0")), fmt(r.get("hlf_L1")), fmt(r.get("hlf_any")),
            fmt(r.get("pert_offset_xy_m")), fmt(r.get("corr_lin_L0")), fmt(r.get("corr_lin_L1")),
        ]
        print(",".join(vals))

    # ===== Summary =====
    print("\n# Summary (reloc_ok: 2 = both L0&L1 passed)")
    for (method, pert), list_r in sorted(by_key.items()):
        n = len(list_r)
        task_success = sum(1 for r in list_r if r["task_ok"])
        reloc_full = sum(1 for r in list_r if r["reloc_ok"] >= 2)
        total_retries = sum(r["retries"] for r in list_r)
        fa = sum(r.get("false_accept", 0) for r in list_r)
        fps = sum(r.get("fa_post_spike", 0) for r in list_r)
        fany = sum(r.get("fa_any", 0) for r in list_r)
        fa_rate = f"{fa}/{n}" if n > 0 else "N/A"
        fps_rate = f"{fps}/{n}" if n > 0 else "N/A"
        fany_rate = f"{fany}/{n}" if n > 0 else "N/A"
        print(f"  {method} pert={pert}: n={n} task_ok={task_success}/{n} reloc_2ok={reloc_full}/{n} "
              f"FA_task={fa_rate} FA_post_spike(Δ>{FA_POST_SPIKE_DELTA_M}m)={fps_rate} FA_any={fany_rate} "
              f"total_retries={total_retries}")

    # Convergence time
    print("\n# Convergence time (s) mean±std")
    for (method, pert), list_r in sorted(by_key.items()):
        m_dt0, s_dt0 = mean_std([r.get("dt_L0") for r in list_r])
        m_dt1, s_dt1 = mean_std([r.get("dt_L1") for r in list_r])
        parts = [f"  {method} pert={pert}:"]
        if m_dt0 is not None:
            parts.append(f" dt_L0={m_dt0}±{s_dt0}s dt_L1={m_dt1}±{s_dt1}s")
        else:
            parts.append(" (no pass)")
        print("".join(parts))

    # Pass-time error
    print("\n# Pass-time error (e_pass) mean±std")
    for (method, pert), list_r in sorted(by_key.items()):
        ml0, sl0 = mean_std([r.get("e_pass_linear_L0") for r in list_r])
        my0, sy0 = mean_std([r.get("e_pass_yaw_L0") for r in list_r])
        ml1, sl1 = mean_std([r.get("e_pass_linear_L1") for r in list_r])
        my1, sy1 = mean_std([r.get("e_pass_yaw_L1") for r in list_r])
        parts = [f"  {method} pert={pert}:"]
        if ml0 is not None:
            parts.append(f" L0: lin={ml0}±{sl0}m yaw={my0}±{sy0}rad")
        if ml1 is not None:
            parts.append(f" L1: lin={ml1}±{sl1}m yaw={my1}±{sy1}rad")
        if ml0 is None and ml1 is None:
            parts.append(" (no pass)")
        print("".join(parts))

    # Confidence at RELOC_PASS (same tr_sigma / C_t logged for baseline and relocc)
    print("\n# Pass-time confidence tr(Σ) and C_t=exp(-λ·tr) mean±std")
    for (method, pert), list_r in sorted(by_key.items()):
        ts0, _ = mean_std([r.get("e_tr_sigma_L0") for r in list_r])
        ct0, _ = mean_std([r.get("e_C_t_L0") for r in list_r])
        ts1, _ = mean_std([r.get("e_tr_sigma_L1") for r in list_r])
        ct1, _ = mean_std([r.get("e_C_t_L1") for r in list_r])
        parts = [f"  {method} pert={pert}:"]
        if ts0 is not None:
            parts.append(f" L0: tr={ts0} C_t={ct0}")
        if ts1 is not None:
            parts.append(f" L1: tr={ts1} C_t={ct1}")
        if ts0 is None and ts1 is None:
            parts.append(" (no RELOC_PASS)")
        print("".join(parts))

    # Subset: only runs with reloc_ok>=2 (both levels passed)
    print("\n# Same metrics restricted to runs with reloc_ok>=2 (mean±std)")
    for (method, pert), list_r in sorted(by_key.items()):
        sub = [r for r in list_r if r.get("reloc_ok", 0) >= 2]
        if not sub:
            continue
        ts0, _ = mean_std([r.get("e_tr_sigma_L0") for r in sub])
        ct0, _ = mean_std([r.get("e_C_t_L0") for r in sub])
        ts1, _ = mean_std([r.get("e_tr_sigma_L1") for r in sub])
        ct1, _ = mean_std([r.get("e_C_t_L1") for r in sub])
        ma0, _ = mean_std([r.get("mean_linear_after_L0") for r in sub])
        mx0, _ = mean_std([r.get("max_linear_after_L0") for r in sub])
        ma1, _ = mean_std([r.get("mean_linear_after_L1") for r in sub])
        mx1, _ = mean_std([r.get("max_linear_after_L1") for r in sub])
        dt0, _ = mean_std([r.get("dt_L0") for r in sub])
        dt1, _ = mean_std([r.get("dt_L1") for r in sub])
        print(
            f"  {method} pert={pert} n={len(sub)}: "
            f"L0 tr={ts0} C_t={ct0} dt_L0={dt0}s | after mean_lin={ma0} max_lin~{mx0} | "
            f"L1 tr={ts1} C_t={ct1} dt_L1={dt1}s | after mean_lin={ma1} max_lin~{mx1}"
        )

    # After-pass stability
    print("\n# After-pass stability (2s window) mean±std")
    for (method, pert), list_r in sorted(by_key.items()):
        ml0, sl0 = mean_std([r.get("mean_linear_after_L0") for r in list_r])
        mx0, _ = mean_std([r.get("max_linear_after_L0") for r in list_r])
        ml1, sl1 = mean_std([r.get("mean_linear_after_L1") for r in list_r])
        mx1, _ = mean_std([r.get("max_linear_after_L1") for r in list_r])
        parts = [f"  {method} pert={pert}:"]
        if ml0 is not None:
            parts.append(f" L0: mean_lin={ml0}±{sl0}m max_lin~{mx0}m")
        if ml1 is not None:
            parts.append(f" L1: mean_lin={ml1}±{sl1}m max_lin~{mx1}m")
        if ml0 is None and ml1 is None:
            parts.append(" (no pass)")
        print("".join(parts))

    # Normalized linear correction vs planar perturbation (Module A fairness / auxiliary)
    # corr_lin = 1 - e_pass_linear / ||(dx,dy)||; see docs/amcl/03_region_init_experiment_plan.md §2.2
    print("\n# Normalized linear correction corr_lin = 1 - e_pass_linear / ||(δx,δy)|| (mean±std)")
    print("# Undefined for pert=0; negative means pass-time error exceeds perturbation norm (e.g. mid-convergence).")
    for (method, pert), list_r in sorted(by_key.items()):
        _, _, poff = pert_planar_offset(pert)
        if poff is None or poff < 1e-9:
            print(f"  {method} pert={pert}: (no planar perturbation)")
            continue
        c0, s0 = mean_std([r.get("corr_lin_L0") for r in list_r])
        c1, s1 = mean_std([r.get("corr_lin_L1") for r in list_r])
        parts = [f"  {method} pert={pert} ||(dx,dy)||={round(poff, 4)}m:"]
        if c0 is not None:
            parts.append(f" L0 corr_lin={c0}±{s0}")
        if c1 is not None:
            parts.append(f" L1 corr_lin={c1}±{s1}")
        if c0 is None and c1 is None:
            parts.append(" (no RELOC_PASS)")
        print("".join(parts))

    # Hard Localization Failure (HLF) summary
    # HLF: RELOC_PASS declared but e_linear > 0.45 m OR e_yaw > 0.15 rad at that moment.
    print("\n# Hard Localization Failure (HLF) — thresholds: lin>0.45m OR yaw>0.15rad")
    print("# hlf_L0/L1: per-layer (1=HLF, 0=safe, blank=no pass event)")
    print("# hlf_rate: HLF events / RELOC_PASS events across all scored layers")
    for (method, pert), list_r in sorted(by_key.items()):
        hlf0_vals = [r["hlf_L0"] for r in list_r if r.get("hlf_L0") is not None]
        hlf1_vals = [r["hlf_L1"] for r in list_r if r.get("hlf_L1") is not None]
        all_hlf = hlf0_vals + hlf1_vals
        n_hlf = sum(all_hlf)
        n_scored = len(all_hlf)
        rate = f"{n_hlf}/{n_scored}" if n_scored > 0 else "N/A"
        pct = f"({100*n_hlf/n_scored:.0f}%)" if n_scored > 0 else ""
        print(f"  {method} pert={pert}: HLF={rate} {pct}"
              f"  [L0: {sum(hlf0_vals)}/{len(hlf0_vals)}  L1: {sum(hlf1_vals)}/{len(hlf1_vals)}]")


if __name__ == "__main__":
    main()
