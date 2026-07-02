#!/usr/bin/env python3
"""
traccar_baseline.py — query Traccar's Postgres backend to re-baseline
VorsprungLogger telemetry quality.

Companion to SPDD.md §16 / §16.1. The same method (fix availability, OBD-speed
as independent motion reference, gap distribution) is used by every subcommand
so post-change runs are directly comparable to the pre-/post-antenna numbers
already in the SPDD.

Connection params come from env vars (so passwords don't live in git):

    TRACCAR_DB_HOST  default 127.0.0.1
    TRACCAR_DB_PORT  default 5432
    TRACCAR_DB_NAME  default traccar
    TRACCAR_DB_USER  default postgres
    TRACCAR_DB_PASS  required

Subcommands:

    devices                                list known devices, newest first
    schema                                 columns of tc_positions
    recent     <devid> [n=10]              dump most-recent N rows
    stats      <devid> [hours=36]          fix availability + gap distribution
    coverage   <devid> [hours=36]          per-attribute % of rows w/ each metric
    outages    <devid> [hours=36] [min_s=30]
                                           devicetime gaps > min_s (cell dropouts
                                           while the car was driving — SD has
                                           these rows; PG does not, see
                                           [[no-sd-backfill]])
    drive      <devid> <from_utc> <to_utc> full per-row + path summary for a
                                           specified window (UTC ISO 8601)
"""
from __future__ import annotations

import json
import math
import os
import statistics
import sys
from datetime import datetime, timedelta

import psycopg2
import psycopg2.extras


def connect():
    pw = os.environ.get("TRACCAR_DB_PASS")
    if not pw:
        sys.exit(
            "ERROR: set TRACCAR_DB_PASS in env (password is in the Traccar "
            "server config under database.password)."
        )
    return psycopg2.connect(
        host=os.environ.get("TRACCAR_DB_HOST", "127.0.0.1"),
        port=int(os.environ.get("TRACCAR_DB_PORT", "5432")),
        dbname=os.environ.get("TRACCAR_DB_NAME", "traccar"),
        user=os.environ.get("TRACCAR_DB_USER", "postgres"),
        password=pw,
        connect_timeout=8,
    )


def attrs(row) -> dict:
    a = row["attributes"]
    if a is None:
        return {}
    if isinstance(a, str):
        try:
            return json.loads(a)
        except Exception:
            return {}
    return a


def pct(n, d):
    return f"{100 * n / d:.1f}%" if d else "n/a"


def fetch_window(cur, devid, hours):
    cur.execute(
        """
        SELECT devicetime, fixtime, servertime, valid,
               latitude, longitude, altitude, speed, course, attributes
        FROM tc_positions
        WHERE deviceid = %s
          AND devicetime > NOW() - (%s || ' hours')::interval
        ORDER BY devicetime
        """,
        (devid, hours),
    )
    return cur.fetchall()


def fetch_between(cur, devid, t_from, t_to):
    cur.execute(
        """
        SELECT devicetime, fixtime, servertime, valid,
               latitude, longitude, altitude, speed, course, attributes
        FROM tc_positions
        WHERE deviceid = %s
          AND devicetime BETWEEN %s AND %s
        ORDER BY devicetime
        """,
        (devid, t_from, t_to),
    )
    return cur.fetchall()


# -------- subcommands --------------------------------------------------------


def cmd_devices(cur, _argv):
    cur.execute(
        """
        SELECT id, name, uniqueid, lastupdate, status, positionid
        FROM tc_devices ORDER BY lastupdate DESC NULLS LAST
        """
    )
    for r in cur.fetchall():
        print(dict(r))


def cmd_schema(cur, _argv):
    cur.execute(
        """
        SELECT column_name, data_type FROM information_schema.columns
        WHERE table_name = 'tc_positions' ORDER BY ordinal_position
        """
    )
    for r in cur.fetchall():
        print(r["column_name"], r["data_type"])


def cmd_recent(cur, argv):
    devid = int(argv[0])
    n = int(argv[1]) if len(argv) > 1 else 10
    cur.execute(
        """
        SELECT id, devicetime, fixtime, servertime, valid, latitude, longitude,
               altitude, speed, course, attributes
        FROM tc_positions
        WHERE deviceid = %s
        ORDER BY devicetime DESC
        LIMIT %s
        """,
        (devid, n),
    )
    for r in cur.fetchall():
        print(dict(r))


def cmd_stats(cur, argv):
    devid = int(argv[0])
    hours = int(argv[1]) if len(argv) > 1 else 36
    rows = fetch_window(cur, devid, hours)
    if not rows:
        print("(no rows in window)")
        return

    total = len(rows)
    with_fix = 0
    moving = 0
    moving_with_fix = 0
    sats, hdops, batts = [], [], []
    gaps = []
    prev_t = None
    SESS_GAP = 300
    sessions = []
    cur_s = []

    for r in rows:
        a = attrs(r)
        lat, lon = r["latitude"], r["longitude"]
        has = bool(r["valid"]) and lat is not None and lon is not None and \
            not (float(lat) == 0 and float(lon) == 0)
        if has:
            with_fix += 1
        osp = a.get("obdSpeed") or a.get("obd_speed")
        if osp is not None and osp > 5:
            moving += 1
            if has:
                moving_with_fix += 1
        if "sat" in a:
            sats.append(a["sat"])
        if "hdop" in a:
            hdops.append(a["hdop"])
        b = a.get("battery") or a.get("power")
        if isinstance(b, (int, float)):
            batts.append(b)

        if prev_t is not None:
            g = (r["devicetime"] - prev_t).total_seconds()
            gaps.append(g)
            if g > SESS_GAP:
                if cur_s:
                    sessions.append(cur_s)
                cur_s = []
        cur_s.append((r["devicetime"], has, osp))
        prev_t = r["devicetime"]
    if cur_s:
        sessions.append(cur_s)

    print(f"=== Window: device {devid}, last {hours}h ===")
    print(f"  span: {rows[0]['devicetime']} -> {rows[-1]['devicetime']}")
    print(f"  rows: {total}")
    print()
    print("=== Fix availability ===")
    print(f"  with fix:        {with_fix} ({pct(with_fix, total)})")
    print(f"  moving rows:     {moving}")
    print(f"    of which fix:  {moving_with_fix} ({pct(moving_with_fix, moving)})")
    print()
    if sats:
        print(f"=== Signal: sat n={len(sats)} min={min(sats)} median={statistics.median(sats):.0f} max={max(sats)}")
    if hdops:
        print(f"          hdop n={len(hdops)} min={min(hdops)} median={statistics.median(hdops):.0f} max={max(hdops)}")
    if batts:
        print(f"          batt min={min(batts):.2f} median={statistics.median(batts):.2f} max={max(batts):.2f}")
    print()
    if gaps:
        gs = sorted(gaps)
        def q(p):
            return gs[int(p * (len(gs) - 1))]
        print("=== Gap distribution between consecutive rows ===")
        print(f"  n={len(gs)} min={gs[0]:.2f}s p50={q(0.5):.2f}s p90={q(0.9):.2f}s p99={q(0.99):.2f}s max={gs[-1]:.1f}s")
        print(f"  gaps >10s: {sum(1 for g in gs if g > 10)}   >60s: {sum(1 for g in gs if g > 60)}   >5min: {sum(1 for g in gs if g > 300)}")
    print()
    print(f"=== Drive sessions (split on >{SESS_GAP}s silence) ===")
    for i, s in enumerate(sessions, 1):
        dur = (s[-1][0] - s[0][0]).total_seconds()
        fix = sum(1 for x in s if x[1])
        mv = sum(1 for x in s if x[2] is not None and x[2] > 5)
        mv_fix = sum(1 for x in s if x[2] is not None and x[2] > 5 and x[1])
        print(f"  #{i:2d}  {s[0][0]} -> {s[-1][0]}  ({dur/60:5.1f} min, {len(s):5d} rows)")
        print(f"        fix {fix}/{len(s)} ({pct(fix, len(s))})  moving {mv}  moving-fix {mv_fix} ({pct(mv_fix, mv)})")


def cmd_coverage(cur, argv):
    """% of rows carrying each attribute key, and within-driving-session gap
    between successive occurrences of each metric."""
    devid = int(argv[0])
    hours = int(argv[1]) if len(argv) > 1 else 36
    rows = fetch_window(cur, devid, hours)
    if not rows:
        print("(no rows)")
        return

    total = len(rows)
    keys = {}            # key -> count of rows containing it
    last_seen = {}       # key -> last devicetime seen
    inter = {}           # key -> list of gaps between occurrences (s)
    SESS_GAP = 300
    prev_t = None

    for r in rows:
        a = attrs(r)
        t = r["devicetime"]
        # session boundary: drop the per-key "last seen" so we don't sum
        # cross-session sleep gaps into the inter-occurrence distribution
        if prev_t is not None and (t - prev_t).total_seconds() > SESS_GAP:
            last_seen.clear()
        for k, v in a.items():
            if v is None:
                continue
            keys[k] = keys.get(k, 0) + 1
            if k in last_seen:
                inter.setdefault(k, []).append((t - last_seen[k]).total_seconds())
            last_seen[k] = t
        prev_t = t

    print(f"=== Attribute coverage over last {hours}h ({total} rows) ===")
    print(f"{'key':28s} {'rows':>8s} {'%':>7s} {'gap p50':>9s} {'gap p90':>9s} {'gap p99':>9s} {'gap max':>9s}")
    for k in sorted(keys, key=lambda x: -keys[x]):
        gs = sorted(inter.get(k, []))
        def q(p):
            return gs[int(p * (len(gs) - 1))] if gs else float("nan")
        if gs:
            row = f"{k:28s} {keys[k]:8d} {100*keys[k]/total:6.1f}% {q(0.5):8.1f}s {q(0.9):8.1f}s {q(0.99):8.1f}s {gs[-1]:8.1f}s"
        else:
            row = f"{k:28s} {keys[k]:8d} {100*keys[k]/total:6.1f}%   (single occurrence)"
        print(row)


def cmd_outages(cur, argv):
    """Find gaps in devicetime > min_s while the car was driving. Each is a
    likely cellular dropout: SD has those cycles but PG doesn't (firmware
    never replays SD, see [[no-sd-backfill]]).

    Heuristic for "while driving": OBD speed > 5 km/h in the row before AND
    after the gap, AND gap < 30 min (longer = ignition off, not an outage).
    """
    devid = int(argv[0])
    hours = int(argv[1]) if len(argv) > 1 else 36
    min_s = float(argv[2]) if len(argv) > 2 else 30.0

    rows = fetch_window(cur, devid, hours)
    if len(rows) < 2:
        print("(not enough rows)")
        return

    OUTAGE_MAX = 30 * 60  # treat longer than this as ignition-off, not outage

    outages = []
    for prev, cur_r in zip(rows[:-1], rows[1:]):
        gap = (cur_r["devicetime"] - prev["devicetime"]).total_seconds()
        if gap < min_s or gap > OUTAGE_MAX:
            continue
        p_a, c_a = attrs(prev), attrs(cur_r)
        p_sp = p_a.get("obdSpeed") or p_a.get("obd_speed")
        c_sp = c_a.get("obdSpeed") or c_a.get("obd_speed")
        # Many rows lack obdSpeed (other UDS slots) — be lenient: require at
        # least one side to show motion.
        moving_either = (p_sp is not None and p_sp > 5) or (c_sp is not None and c_sp > 5)
        if not moving_either:
            continue
        outages.append((prev["devicetime"], cur_r["devicetime"], gap, p_sp, c_sp))

    if not outages:
        print(f"No outages > {min_s:.0f}s while driving in the last {hours}h. "
              "Cell coverage held throughout the drive(s).")
        return

    total_lost_s = sum(o[2] for o in outages)
    print(f"=== {len(outages)} probable cellular outages in last {hours}h "
          f"(gap > {min_s:.0f}s, < 30 min, car moving) ===")
    print(f"Total time inside outages: {total_lost_s:.0f} s ({total_lost_s/60:.1f} min)")
    print()
    print(f"{'from (UTC)':27s} {'to (UTC)':27s} {'gap_s':>8s} {'sp_before':>10s} {'sp_after':>10s}")
    for t_from, t_to, g, sp_b, sp_a in outages:
        print(f"{str(t_from):27s} {str(t_to):27s} {g:8.1f} {str(sp_b):>10s} {str(sp_a):>10s}")


def cmd_drive(cur, argv):
    devid = int(argv[0])
    t_from = argv[1]
    t_to = argv[2]
    rows = fetch_between(cur, devid, t_from, t_to)
    if not rows:
        print("(no rows in window)")
        return

    def hav(a, b):
        R = 6371000.0
        la1, lo1 = math.radians(float(a[0])), math.radians(float(a[1]))
        la2, lo2 = math.radians(float(b[0])), math.radians(float(b[1]))
        dla, dlo = la2 - la1, lo2 - lo1
        h = math.sin(dla / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin(dlo / 2) ** 2
        return 2 * R * math.asin(math.sqrt(h))

    lats = [float(r["latitude"]) for r in rows]
    lons = [float(r["longitude"]) for r in rows]
    # path: only sum hops where OBD or GPS speed > 3 km/h, suppresses
    # stationary jitter without losing real movement
    total_m = 0.0
    prev = None
    for r in rows:
        a = attrs(r)
        osp = a.get("obdSpeed") or a.get("obd_speed")
        gsp_kmh = (float(r["speed"]) * 1.852) if r["speed"] is not None else 0.0
        moving = (osp is not None and osp > 3) or gsp_kmh > 3
        if prev is not None and moving:
            total_m += hav(prev, (float(r["latitude"]), float(r["longitude"])))
        prev = (float(r["latitude"]), float(r["longitude"]))

    print(f"window: {rows[0]['devicetime']} -> {rows[-1]['devicetime']}  ({len(rows)} rows)")
    print(f"lat:    {min(lats):.6f} .. {max(lats):.6f}  span ~ {(max(lats)-min(lats))*111:.1f} km")
    print(f"lon:    {min(lons):.6f} .. {max(lons):.6f}")
    print(f"path:   {total_m/1000:.1f} km (hops while moving > 3 km/h)")
    print()
    step = max(1, len(rows) // 12)
    print("waypoints (t, lat, lon, gps_kmh, obd_kmh):")
    for r in rows[::step]:
        a = attrs(r)
        osp = a.get("obdSpeed") or a.get("obd_speed")
        gsp_kmh = (float(r["speed"]) * 1.852) if r["speed"] is not None else 0.0
        print(f"  {r['devicetime']}  {float(r['latitude']):.5f},{float(r['longitude']):.5f}  "
              f"gps={gsp_kmh:5.1f}  obd={osp}")


COMMANDS = {
    "devices": cmd_devices,
    "schema": cmd_schema,
    "recent": cmd_recent,
    "stats": cmd_stats,
    "coverage": cmd_coverage,
    "outages": cmd_outages,
    "drive": cmd_drive,
}


def main(argv):
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        return
    cmd = argv[0]
    if cmd not in COMMANDS:
        sys.exit(f"unknown command: {cmd}\n{__doc__}")
    conn = connect()
    try:
        cur = conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
        COMMANDS[cmd](cur, argv[1:])
        cur.close()
    finally:
        conn.close()


if __name__ == "__main__":
    main(sys.argv[1:])
