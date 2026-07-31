#!/usr/bin/env python3
"""Check the correspondence between LiDAR PCD filename timestamps and the GNSS CSV.

LiDAR frames are named "<ms epoch>.pcd" (parsed as `stod(stem) * 1e-3` in
lidar_utils/io_impl.cpp), while the GNSS CSV first column is an epoch in
seconds (gnss_processor.cpp). This tool loads both, mirrors the matching rule
used by synchronizeFrames() in src/time_sync.cpp, and reports which GNSS epochs
have a corresponding LiDAR frame and which ones are loose (no frame inside the
1/lidar_hz window), plus anomalies in the folder itself.

Usage:
  ./check_file.py <gnss_csv> <pcd_folder> [more_pcd_folders ...]

Examples:
  ./check_file.py .../VTS/20260723133522_gnss_full.csv .../PCD/top/
  ./check_file.py .../VTS/20260723133522_gnss_full.csv .../PCD/*/
  ./check_file.py gnss.csv .../PCD/top/ --lidar-hz 10 --csv-out /tmp/sync.csv
"""

from __future__ import annotations

import argparse
import bisect
import csv
import os
import statistics
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone

# Filename timestamps further than this from the GNSS/median epoch are treated
# as corrupted names rather than valid frames (e.g. "459800.pcd").
OUTLIER_WINDOW_S = 24 * 3600.0


def ts_str(ts: float | None) -> str:
    """Format an epoch (seconds) as UTC + raw value, or '-' when missing."""
    if ts is None:
        return "-"
    try:
        stamp = datetime.fromtimestamp(ts, timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    except (ValueError, OverflowError, OSError):
        # Epoch outside the representable range: wrong unit or corrupt value.
        stamp = "<not a valid date>"
    return "%s (%.3f)" % (stamp, ts)


# Epoch units the GNSS CSV may use in its time column.
TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}


def detect_time_unit(values: list[float]) -> str:
    """Guess whether the epoch column is in s / ms / us / ns.

    A present-day epoch is ~1.8e9 s, so the magnitude alone identifies the
    unit. Anything below the 1970s-in-seconds range is left as seconds.
    """
    if not values:
        return "s"
    mag = abs(statistics.median(values))
    if mag >= 1e17:
        return "ns"
    if mag >= 1e14:
        return "us"
    if mag >= 1e11:
        return "ms"
    return "s"


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
@dataclass
class Folder:
    """Timestamps parsed from one LiDAR frame folder."""

    name: str
    path: str
    ts: list[float] = field(default_factory=list)  # sorted, seconds
    files: list[str] = field(default_factory=list)  # aligned with ts
    unparsable: list[str] = field(default_factory=list)
    outliers: list[tuple[str, float]] = field(default_factory=list)
    duplicates: list[tuple[float, int]] = field(default_factory=list)
    n_scanned: int = 0

    @property
    def nominal_dt(self) -> float | None:
        """Median inter-frame interval, i.e. the observed frame period."""
        if len(self.ts) < 2:
            return None
        diffs = [b - a for a, b in zip(self.ts, self.ts[1:])]
        return statistics.median(diffs)

    def gaps(self, factor: float = 1.5) -> list[tuple[float, float, float]]:
        """Consecutive frames whose spacing exceeds factor * nominal_dt."""
        dt = self.nominal_dt
        if dt is None or dt <= 0:
            return []
        out = []
        for a, b in zip(self.ts, self.ts[1:]):
            if b - a > factor * dt:
                out.append((a, b, b - a))
        return out


def load_folder(name: str, path: str, ref_epoch: float | None) -> Folder:
    """Parse every .pcd/.las filename in `path` into an epoch in seconds.

    Mirrors executeReadContent(): the stem is read as a number of milliseconds.
    Names that do not parse, or that land absurdly far from `ref_epoch` (the
    GNSS span, when known), are collected as anomalies instead of frames.
    """
    fld = Folder(name=name, path=path)
    pairs: list[tuple[float, str]] = []
    with os.scandir(path) as it:
        for entry in it:
            if not entry.is_file():
                continue
            stem, ext = os.path.splitext(entry.name)
            if ext.lower() not in (".pcd", ".las"):
                continue
            fld.n_scanned += 1
            try:
                ts = float(stem) * 1e-3
            except ValueError:
                fld.unparsable.append(entry.name)
                continue
            pairs.append((ts, entry.name))

    if not pairs:
        return fld

    center = ref_epoch if ref_epoch is not None else statistics.median(
        t for t, _ in pairs
    )
    kept = []
    for ts, fname in pairs:
        if abs(ts - center) > OUTLIER_WINDOW_S:
            fld.outliers.append((fname, ts))
        else:
            kept.append((ts, fname))

    kept.sort()
    prev = None
    run = 0
    for ts, fname in kept:
        fld.ts.append(ts)
        fld.files.append(fname)
        if ts == prev:
            run += 1
        else:
            if run:
                fld.duplicates.append((prev, run + 1))
            prev, run = ts, 0
    if run:
        fld.duplicates.append((prev, run + 1))
    return fld


@dataclass
class Gnss:
    path: str
    ts: list[float] = field(default_factory=list)  # seconds, file order
    std: list[float] = field(default_factory=list)  # position std norm, meters
    line: list[int] = field(default_factory=list)  # 1-based file line of each ts
    n_rows: int = 0
    n_bad_rows: int = 0
    n_unsorted: int = 0
    duplicates: int = 0
    unit: str = "s"  # unit found in the time column
    has_std: bool = True  # False when the std columns are absent


def load_gnss(
    path: str, time_col: int = 0, std_cols: tuple[int, int, int] | None = (4, 5, 6),
    unit: str = "auto",
) -> Gnss:
    """Read the epoch column and, when present, the lat/lon/alt std columns.

    The pipeline's *_gnss_full.csv keeps the epoch in seconds, but other
    exports (e.g. *_gnss_full_trajectory.csv) use milliseconds, so the unit is
    detected from the magnitude unless it is given explicitly.
    """
    g = Gnss(path=path)
    raw: list[float] = []
    stds: list[float] = []
    lines: list[int] = []
    with open(path, newline="") as fh:
        # The line number is tracked so --prune-gnss can drop exactly the rows
        # reported here (these CSVs have one record per line, no quoted
        # newlines).
        for lineno, row in enumerate(csv.reader(fh), start=1):
            if not row:
                continue
            g.n_rows += 1
            try:
                ts = float(row[time_col])
            except (ValueError, IndexError):
                g.n_bad_rows += 1  # header or malformed row, as the C++ does
                continue
            std = 0.0
            if std_cols is not None:
                try:
                    std = sum(float(row[c]) ** 2 for c in std_cols) ** 0.5
                except (ValueError, IndexError):
                    g.has_std = False
            raw.append(ts)
            stds.append(std)
            lines.append(lineno)

    g.unit = detect_time_unit(raw) if unit == "auto" else unit
    scale = TIME_UNITS[g.unit]
    for ts, std, lineno in zip(raw, stds, lines):
        ts *= scale
        if g.ts:
            if ts < g.ts[-1]:
                g.n_unsorted += 1
            elif ts == g.ts[-1]:
                g.duplicates += 1
        g.ts.append(ts)
        g.std.append(std)
        g.line.append(lineno)
    if not g.has_std:
        g.std = [0.0] * len(g.ts)
    return g


def prune_gnss_lines(src: str, dst: str, drop_lines: set[int]) -> tuple[int, int]:
    """Copy `src` to `dst` verbatim minus the given 1-based line numbers.

    Copied line by line rather than through the csv writer so surviving rows
    keep their exact original text (precision, quoting, line endings).
    """
    kept = dropped = 0
    with open(src, newline="") as fi, open(dst, "w", newline="") as fo:
        for lineno, line in enumerate(fi, start=1):
            if lineno in drop_lines:
                dropped += 1
                continue
            fo.write(line)
            kept += 1
    return kept, dropped


# --------------------------------------------------------------------------- #
# Config / path handling
# --------------------------------------------------------------------------- #
def folder_label(path: str) -> str:
    """Short display name for a folder, e.g. ".../PCD/top/" -> "top"."""
    return os.path.basename(os.path.normpath(path)) or path


# --------------------------------------------------------------------------- #
# Correspondence check
# --------------------------------------------------------------------------- #
@dataclass
class Match:
    """Result of matching one GNSS epoch against one LiDAR folder."""

    fwd_diff: float | None  # first frame at/after GNSS, as time_sync.cpp does
    fwd_file: str | None
    near_diff: float | None  # signed diff of the closest frame either side
    near_file: str | None


def match_one(fld: Folder, target: float) -> Match:
    idx = bisect.bisect_left(fld.ts, target)
    fwd_diff = fwd_file = None
    if idx < len(fld.ts):
        fwd_diff = fld.ts[idx] - target
        fwd_file = fld.files[idx]

    cands = []
    if idx < len(fld.ts):
        cands.append(idx)
    if idx > 0:
        cands.append(idx - 1)
    near_diff = near_file = None
    for i in cands:
        d = fld.ts[i] - target
        if near_diff is None or abs(d) < abs(near_diff):
            near_diff, near_file = d, fld.files[i]
    return Match(fwd_diff, fwd_file, near_diff, near_file)


def pair_timestamps(
    a: list[float], b: list[float], tol: float
) -> tuple[int, list[int], list[int]]:
    """Greedily pair two sorted timestamp lists one-to-one within `tol`.

    Returns (n_paired, indices only in `a`, indices only in `b`). Because both
    lists are monotonic, a two-pointer sweep is enough: whichever side is
    behind by more than `tol` cannot have a partner, so it is an extra entry.
    """
    i = j = 0
    n_paired = 0
    only_a: list[int] = []
    only_b: list[int] = []
    while i < len(a) and j < len(b):
        d = b[j] - a[i]
        if abs(d) <= tol:
            n_paired += 1
            i += 1
            j += 1
        elif d > 0:  # a[i] is earlier and has no partner
            only_a.append(i)
            i += 1
        else:
            only_b.append(j)
            j += 1
    only_a.extend(range(i, len(a)))
    only_b.extend(range(j, len(b)))
    return n_paired, only_a, only_b


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Check LiDAR PCD filename timestamps against a GNSS CSV.",
        usage="%(prog)s <gnss_csv> <pcd_folder> [pcd_folder ...] [options]",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("gnss_csv", help="GNSS CSV; first column is the epoch (s or ms)")
    ap.add_argument(
        "pcd_folder",
        nargs="+",
        help="Folder(s) of <ms epoch>.pcd frames (one per LiDAR)",
    )
    ap.add_argument(
        "--gnss-time-unit",
        choices=["auto", "s", "ms", "us", "ns"],
        default="auto",
        help="Unit of the GNSS epoch column; auto-detected from its magnitude",
    )
    ap.add_argument("--gnss-time-col", type=int, default=0, help="0-based index of the GNSS epoch column")
    ap.add_argument(
        "--gnss-std-cols",
        default="4,5,6",
        help='0-based indices of the lat/lon/alt std columns, or "none" to skip the std check',
    )
    ap.add_argument("--lidar-hz", type=float, default=10.0, help="LiDAR rate; the sync window is 1/lidar_hz")
    ap.add_argument("--gnss-freq", type=float, default=10.0, help="GNSS processing rate used for throttling")
    ap.add_argument("--std-thres", type=float, default=500.0, help="Reject GNSS rows with position std norm above this (m)")
    ap.add_argument(
        "--no-throttle",
        action="store_true",
        help="Check every GNSS row instead of only the ones gnss_freq keeps",
    )
    ap.add_argument("--max-list", type=int, default=10, help="Examples printed per issue")
    ap.add_argument(
        "--pair-tol",
        type=float,
        default=0.0,
        help="Tolerance for the one-to-one pairing; 0 means half the GNSS interval",
    )
    ap.add_argument("--csv-out", help="Write the per-epoch diffs to this CSV")
    ap.add_argument("--extra-out", help="Write the FULL list of unpaired entries to this CSV")
    ap.add_argument(
        "--prune-gnss",
        nargs="?",
        const="",
        metavar="OUT_CSV",
        help="Delete the GNSS rows that have no PCD frame. Writes "
        "<input>_pruned.csv unless a path is given; never touches the input "
        "without --in-place",
    )
    ap.add_argument(
        "--in-place",
        action="store_true",
        help="With --prune-gnss, overwrite the input CSV (a .bak copy is kept first)",
    )
    args = ap.parse_args()

    gnss_path = args.gnss_csv
    lidar_hz = args.lidar_hz
    gnss_freq = args.gnss_freq
    std_thres = args.std_thres

    # Label each folder by its own basename, disambiguating repeats.
    lidars: list[tuple[str, str]] = []
    seen: dict[str, int] = {}
    for path in args.pcd_folder:
        label = folder_label(path)
        seen[label] = seen.get(label, 0) + 1
        if seen[label] > 1:
            label = "%s#%d" % (label, seen[label])
        lidars.append((label, path))

    print("=" * 78)
    print("LiDAR filename <-> GNSS timestamp correspondence check")
    print("=" * 78)
    print("GNSS CSV      : %s" % gnss_path)
    print("sync window   : %.4f s (1 / lidar_hz=%g)" % (1.0 / lidar_hz, lidar_hz))
    print("gnss_freq     : %g Hz%s" % (gnss_freq, "  (throttling disabled)" if args.no_throttle else ""))
    print("gnss_std_thres: %g m" % std_thres)

    if not os.path.isfile(gnss_path):
        print("\nFATAL: GNSS CSV not found: %s" % gnss_path)
        return 2

    std_cols: tuple[int, int, int] | None = None
    if args.gnss_std_cols.strip().lower() not in ("none", ""):
        try:
            parts = tuple(int(c) for c in args.gnss_std_cols.split(","))
        except ValueError:
            ap.error("--gnss-std-cols must be three comma-separated indices or 'none'")
        if len(parts) != 3:
            ap.error("--gnss-std-cols must be three comma-separated indices or 'none'")
        std_cols = parts

    g = load_gnss(gnss_path, args.gnss_time_col, std_cols, args.gnss_time_unit)
    if not g.ts:
        print("\nFATAL: no parsable GNSS rows in %s" % gnss_path)
        return 2

    problems = 0

    # ---------------- GNSS summary ---------------- #
    print("\n--- GNSS ---")
    print("rows              : %d (parsed %d, skipped %d)" % (g.n_rows, len(g.ts), g.n_bad_rows))
    print(
        "time column       : col %d in %s%s"
        % (args.gnss_time_col, g.unit, " (auto-detected)" if args.gnss_time_unit == "auto" else "")
    )
    if std_cols is None:
        print("std check         : disabled")
    elif not g.has_std:
        print(
            "std check         : DISABLED - columns %s are missing or non-numeric"
            % (",".join(str(c) for c in std_cols))
        )
    print("first / last      : %s  ->  %s" % (ts_str(g.ts[0]), ts_str(g.ts[-1])))
    print("span              : %.3f s" % (g.ts[-1] - g.ts[0]))
    gnss_gaps: list[tuple[float, float]] = []
    gnss_dt = 0.0
    if len(g.ts) > 1:
        d = [b - a for a, b in zip(g.ts, g.ts[1:])]
        dt = gnss_dt = statistics.median(d)
        print("interval (median) : %.4f s  -> %.2f Hz" % (dt, 1.0 / dt))
        print(
            "average rate      : %.2f Hz (%d rows over %.1f s)"
            % (len(g.ts) / (g.ts[-1] - g.ts[0]), len(g.ts), g.ts[-1] - g.ts[0])
        )
        gnss_gaps = [(a, b - a) for a, b in zip(g.ts, g.ts[1:]) if b - a > 1.5 * dt]
        if gnss_gaps:
            missing = sum(gap - dt for _, gap in gnss_gaps)
            print(
                "GAPS              : %d gap(s) > 1.5x the interval, %.1f s of GNSS missing"
                % (len(gnss_gaps), missing)
            )
            for a, gap in sorted(gnss_gaps, key=lambda x: -x[1])[: args.max_list]:
                print("                    %.3f s missing after %s" % (gap, ts_str(a)))
            problems += 1
    n_high_std = sum(1 for s in g.std if s > std_thres)
    if n_high_std > 0.5 * len(g.ts):
        # More than half the file rejected means those columns almost certainly
        # are not standard deviations (e.g. a trajectory export with a
        # different layout). Ignore them instead of checking nothing.
        print(
            "std check         : DISABLED - %d of %d rows exceed %g m, so columns %s"
            % (n_high_std, len(g.ts), std_thres,
               ",".join(str(c) for c in std_cols) if std_cols else "-")
        )
        print("                    are probably not std; pass --gnss-std-cols to set them")
        g.std = [0.0] * len(g.ts)
        n_high_std = 0
        problems += 1
    elif n_high_std:
        print("rejected by std   : %d rows above %g m" % (n_high_std, std_thres))
    if g.n_unsorted:
        print("WARNING: %d GNSS rows are not in increasing time order" % g.n_unsorted)
        problems += 1
    if g.duplicates:
        print("WARNING: %d duplicated GNSS timestamps" % g.duplicates)
        problems += 1

    gnss_mid = 0.5 * (g.ts[0] + g.ts[-1])

    # ---------------- Folder summaries ---------------- #
    folders: list[Folder] = []
    for name, path in lidars:
        print("\n--- %s: %s ---" % (name, path))
        if not os.path.isdir(path):
            print("ERROR: folder does not exist")
            problems += 1
            continue
        fld = load_folder(name, path, gnss_mid)
        folders.append(fld)
        print("frames            : %d valid of %d files" % (len(fld.ts), fld.n_scanned))
        if fld.ts:
            print("first / last      : %s  ->  %s" % (ts_str(fld.ts[0]), ts_str(fld.ts[-1])))
            dt = fld.nominal_dt
            if dt:
                print("interval (median) : %.4f s  -> %.2f Hz" % (dt, 1.0 / dt))
        if fld.unparsable:
            print("BAD NAMES         : %d file(s) whose stem is not a number" % len(fld.unparsable))
            for n in fld.unparsable[: args.max_list]:
                print("                    %s" % n)
            problems += 1
        if fld.outliers:
            print(
                "OUT-OF-RANGE      : %d file(s) more than %.0f h from the GNSS span"
                % (len(fld.outliers), OUTLIER_WINDOW_S / 3600.0)
            )
            for n, ts in sorted(fld.outliers, key=lambda x: x[1])[: args.max_list]:
                print("                    %-24s -> %s" % (n, ts_str(ts)))
            if not fld.ts:
                # Every name is out of range: most likely the filenames are not
                # in milliseconds, or the GNSS epoch column was read wrong.
                print(
                    "                    -> NO valid frame at all; filenames may not be in ms"
                )
            problems += 1
        if fld.duplicates:
            print("DUPLICATE STAMPS  : %d timestamp(s) used by >1 file" % len(fld.duplicates))
            for ts, cnt in fld.duplicates[: args.max_list]:
                print("                    %s x%d" % (ts_str(ts), cnt))
            problems += 1
        gaps = fld.gaps()
        if gaps:
            print("GAPS              : %d gap(s) > 1.5x the frame interval" % len(gaps))
            for a, _, d in sorted(gaps, key=lambda x: -x[2])[: args.max_list]:
                print("                    %.3f s missing after %s" % (d, ts_str(a)))
            problems += 1
        # Coverage against the GNSS span
        if fld.ts:
            lead = fld.ts[0] - g.ts[0]
            trail = g.ts[-1] - fld.ts[-1]
            print(
                "vs GNSS span      : starts %+.3f s, ends %+.3f s relative to GNSS"
                % (lead, -trail)
            )
            if lead > 1.0 / lidar_hz:
                print("                    -> GNSS starts %.3f s BEFORE any frame" % lead)
            if trail > 1.0 / lidar_hz:
                print("                    -> GNSS ends %.3f s AFTER the last frame" % trail)

    if not folders:
        print("\nFATAL: no readable LiDAR folder")
        return 2

    # ---------------- Correspondence ---------------- #
    window = 1.0 / lidar_hz
    process_interval = 1.0 / gnss_freq * 0.6  # as in time_sync.cpp
    stats = {
        f.name: {"ok": 0, "loose": 0, "none": 0, "diffs": [], "near": [], "worst": None}
        for f in folders
    }
    n_checked = 0
    n_all_ok = 0
    loose_epochs: list[tuple[float, list[str]]] = []
    used_files = {f.name: set() for f in folders}
    last_processed = -1.0
    rows_out = []

    for ts, std in zip(g.ts, g.std):
        if std > std_thres:
            continue
        if not args.no_throttle and last_processed > 0 and (ts - last_processed) < process_interval:
            continue
        last_processed = ts
        n_checked += 1
        all_ok = True
        reasons = []
        row = {"gnss_ts": "%.4f" % ts}
        for fld in folders:
            st = stats[fld.name]
            m = match_one(fld, ts)
            row["%s_fwd_diff" % fld.name] = "" if m.fwd_diff is None else "%.4f" % m.fwd_diff
            row["%s_near_diff" % fld.name] = "" if m.near_diff is None else "%.4f" % m.near_diff
            row["%s_file" % fld.name] = m.fwd_file or ""
            if m.near_diff is not None:
                st["near"].append(m.near_diff)
            if m.fwd_diff is None:
                st["none"] += 1
                all_ok = False
                reasons.append("%s: no frame at/after" % fld.name)
            elif m.fwd_diff > window:
                st["loose"] += 1
                all_ok = False
                reasons.append("%s: +%.3fs > %.3fs" % (fld.name, m.fwd_diff, window))
                if st["worst"] is None or m.fwd_diff > st["worst"][0]:
                    st["worst"] = (m.fwd_diff, ts, m.fwd_file)
            else:
                st["ok"] += 1
                st["diffs"].append(m.fwd_diff)
                used_files[fld.name].add(m.fwd_file)
        if all_ok:
            n_all_ok += 1
        else:
            loose_epochs.append((ts, reasons))
        rows_out.append(row)

    print("\n" + "=" * 78)
    print("CORRESPONDENCE (rule from time_sync.cpp: first frame at or after the")
    print("GNSS epoch, accepted when diff <= %.4f s)" % window)
    print("=" * 78)
    print("GNSS epochs checked          : %d" % n_checked)
    print("epochs matched by ALL LiDARs : %d (%.2f%%)" % (n_all_ok, 100.0 * n_all_ok / max(n_checked, 1)))
    print("epochs loose (>=1 LiDAR off) : %d" % (n_checked - n_all_ok))
    print("expected merge output frames : %d" % max(n_all_ok - 1, 0))
    if n_all_ok < n_checked:
        problems += 1

    print("\n%-12s %8s %8s %8s   %-28s %s" % ("lidar", "matched", "loose", "no-frm", "matched diff (s)", "nearest diff (s)"))
    for fld in folders:
        st = stats[fld.name]
        if st["diffs"]:
            dsum = "min %.4f med %.4f max %.4f" % (
                min(st["diffs"]),
                statistics.median(st["diffs"]),
                max(st["diffs"]),
            )
        else:
            dsum = "-"
        if st["near"]:
            nsum = "min %+.4f med %+.4f max %+.4f" % (
                min(st["near"]),
                statistics.median(st["near"]),
                max(st["near"]),
            )
        else:
            nsum = "-"
        print(
            "%-12s %8d %8d %8d   %-28s %s"
            % (fld.name, st["ok"], st["loose"], st["none"], dsum, nsum)
        )

    for fld in folders:
        st = stats[fld.name]
        if st["worst"]:
            d, ts, f = st["worst"]
            print(
                "worst loose match on %s: +%.3f s at GNSS %s -> %s"
                % (fld.name, d, ts_str(ts), f)
            )

    if loose_epochs:
        print("\nLoose GNSS epochs (first %d of %d):" % (min(args.max_list, len(loose_epochs)), len(loose_epochs)))
        for ts, reasons in loose_epochs[: args.max_list]:
            print("  %s  %s" % (ts_str(ts), "; ".join(reasons)))

    # Frames inside the GNSS span that no epoch consumed.
    print("\n--- unused frames inside the GNSS time span ---")
    for fld in folders:
        inside = [t for t in fld.ts if g.ts[0] <= t <= g.ts[-1]]
        unused = len(inside) - len(used_files[fld.name])
        print(
            "%-12s %d frames in span, %d matched, %d unused (%.1f%%)"
            % (
                fld.name,
                len(inside),
                len(used_files[fld.name]),
                unused,
                100.0 * unused / max(len(inside), 1),
            )
        )

    # ---------------- Which side has more, and exactly which entries ------- #
    # One-to-one pairing, so leftovers on either side are named individually.
    tol = args.pair_tol if args.pair_tol else 0.5 * (gnss_dt or window)
    print("\n" + "=" * 78)
    print("WHICH SIDE HAS MORE (one-to-one pairing, tolerance %.4f s)" % tol)
    print("=" * 78)
    drop_lines: set[int] = set()  # GNSS lines with no frame, for --prune-gnss
    for fld in folders:
        n_paired, only_gnss, only_frames = pair_timestamps(g.ts, fld.ts, tol)
        drop_lines.update(g.line[i] for i in only_gnss)
        print("\n%s  vs  %s" % (os.path.basename(gnss_path), fld.path))
        print("  GNSS rows        : %d" % len(g.ts))
        print("  frames           : %d" % len(fld.ts))
        print("  paired 1:1       : %d" % n_paired)
        if len(only_gnss) == len(only_frames) == 0:
            print("  -> identical timestamp sets, neither side has extras")
            continue
        detail = "%d GNSS-only, %d frame-only" % (len(only_gnss), len(only_frames))
        if len(g.ts) > len(fld.ts):
            print("  -> GNSS CSV has %d MORE entries (%s)" % (len(g.ts) - len(fld.ts), detail))
        elif len(fld.ts) > len(g.ts):
            print("  -> the FOLDER has %d MORE entries (%s)" % (len(fld.ts) - len(g.ts), detail))
        else:
            print("  -> same count, but timestamps do not all line up (%s)" % detail)
        problems += 1

        if only_gnss:
            shown = min(args.max_list, len(only_gnss))
            print(
                "\n  GNSS rows with NO frame (%d, showing %d) - csv row / time:"
                % (len(only_gnss), shown)
            )
            for i in only_gnss[: args.max_list]:
                m = match_one(fld, g.ts[i])
                print(
                    "    line %-7d %s   nearest frame %s (%s)"
                    % (
                        g.line[i],
                        ts_str(g.ts[i]),
                        m.near_file or "-",
                        "%+.3f s" % m.near_diff if m.near_diff is not None else "-",
                    )
                )
        if only_frames:
            shown = min(args.max_list, len(only_frames))
            print(
                "\n  frames with NO GNSS row (%d, showing %d) - filename / time:"
                % (len(only_frames), shown)
            )
            for j in only_frames[: args.max_list]:
                k = bisect.bisect_left(g.ts, fld.ts[j])
                near = None
                for c in (k, k - 1):
                    if 0 <= c < len(g.ts):
                        d = g.ts[c] - fld.ts[j]
                        if near is None or abs(d) < abs(near):
                            near = d
                print(
                    "    %-24s %s   nearest GNSS %s"
                    % (
                        fld.files[j],
                        ts_str(fld.ts[j]),
                        "%+.3f s" % near if near is not None else "-",
                    )
                )
        if args.extra_out:
            path = args.extra_out
            if len(folders) > 1:
                stem, ext = os.path.splitext(path)
                path = "%s_%s%s" % (stem, fld.name, ext or ".csv")
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["side", "csv_line_or_filename", "epoch_s", "utc"])
                for i in only_gnss:
                    w.writerow(["gnss_only", g.line[i], "%.4f" % g.ts[i], ts_str(g.ts[i]).split(" (")[0]])
                for j in only_frames:
                    w.writerow(["frame_only", fld.files[j], "%.4f" % fld.ts[j], ts_str(fld.ts[j]).split(" (")[0]])
            print("\n  Full extra list written to %s" % path)

    # ---------------- Delete the GNSS rows that have no frame -------------- #
    if args.prune_gnss is not None:
        print("\n" + "=" * 78)
        print("PRUNE GNSS ROWS WITH NO PCD FRAME")
        print("=" * 78)
        if not drop_lines:
            print("Nothing to delete: every GNSS row has a matching frame.")
        else:
            if len(folders) > 1:
                print(
                    "Note: %d folders given, so a row is deleted when it lacks a frame\n"
                    "      in ANY of them (the pipeline needs all LiDARs matched)."
                    % len(folders)
                )
            if args.in_place:
                out_path = gnss_path
                backup = gnss_path + ".bak"
                if os.path.exists(backup):
                    print("REFUSING to overwrite: %s already exists" % backup)
                    print("Move or delete that backup first.")
                    return 2
                with open(gnss_path, "rb") as fi, open(backup, "wb") as fo:
                    fo.write(fi.read())
                print("Backup written to %s" % backup)
                tmp_path = gnss_path + ".tmp"
                kept, dropped = prune_gnss_lines(backup, tmp_path, drop_lines)
                os.replace(tmp_path, gnss_path)
            else:
                out_path = args.prune_gnss
                if not out_path:
                    stem, ext = os.path.splitext(gnss_path)
                    out_path = "%s_pruned%s" % (stem, ext or ".csv")
                if os.path.abspath(out_path) == os.path.abspath(gnss_path):
                    print("REFUSING to write over the input; use --in-place instead.")
                    return 2
                kept, dropped = prune_gnss_lines(gnss_path, out_path, drop_lines)
            print(
                "Deleted %d row(s) with no PCD frame, kept %d line(s)."
                % (dropped, kept)
            )
            print("Written to %s" % out_path)
            if not args.in_place:
                print("The input file was NOT modified (pass --in-place to overwrite).")

    if args.csv_out:
        cols = ["gnss_ts"]
        for fld in folders:
            cols += ["%s_fwd_diff" % fld.name, "%s_near_diff" % fld.name, "%s_file" % fld.name]
        with open(args.csv_out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            w.writerows(rows_out)
        print("\nPer-epoch diffs written to %s (%d rows)" % (args.csv_out, len(rows_out)))

    print("\n" + "=" * 78)
    if problems:
        print("RESULT: %d issue group(s) reported above." % problems)
    else:
        print("RESULT: filenames and GNSS timestamps are fully in correspondence.")
    print("=" * 78)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
