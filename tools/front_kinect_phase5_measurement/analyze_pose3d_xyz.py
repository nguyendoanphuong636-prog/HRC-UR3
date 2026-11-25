#!/usr/bin/env python3
"""Analyze joint validity, time gaps and static-position variability from recorder CSV.

Python 3 standard library only. Variability includes human sway, tracker jitter and
camera noise; it is NOT an absolute accuracy measurement or a safety certificate.
"""
import argparse
import csv
import math
import os
import statistics as st
from collections import Counter, defaultdict

JOINT_ORDER = (0, 11, 12, 13, 14, 15, 16, 23, 24)


def qtile(data, prob):
    data = sorted(data)
    if not data:
        return math.nan
    u = (len(data) - 1) * prob
    lo = int(math.floor(u))
    hi = int(math.ceil(u))
    return data[lo] * (hi-u) + data[hi] * (u-lo) if lo != hi else data[lo]


def summarize(path):
    frames = defaultdict(dict)
    names = {}
    invalid_rows = 0
    frame_ids = Counter()
    meta_counts = Counter()
    with open(path, newline='', encoding='utf-8') as fh:
        reader = csv.DictReader(fh)
        expected = {'stamp_sec', 'stamp_nanosec', 'joint_id', 'joint_name',
                    'valid_3d', 'x_m', 'y_m', 'z_m', 'frame_id'}
        if not expected.issubset(set(reader.fieldnames or [])):
            raise ValueError('Unrecognized CSV format: missing recorder columns')
        for row in reader:
            key = int(row['stamp_sec']) * 1_000_000_000 + int(row['stamp_nanosec'])
            jid = int(row['joint_id'])
            names[jid] = row['joint_name']
            if key not in frames:
                frame_ids[row['frame_id']] += 1
            xyz = None
            if row['valid_3d'] == '1':
                try:
                    xyz0 = (float(row['x_m']), float(row['y_m']), float(row['z_m']))
                    if all(map(math.isfinite, xyz0)):
                        xyz = xyz0
                except ValueError:
                    pass
            if xyz is None:
                invalid_rows += 1
            frames[key][jid] = xyz
            if row.get('matched_2d_metadata') == '1':
                meta_counts[key] = 1
    if not frames:
        raise ValueError('No Pose3D data in CSV')
    stamps = sorted(frames)
    gaps_ms = [(stamps[i] - stamps[i - 1]) / 1e6 for i in range(1, len(stamps))]
    counts = Counter(sum(frames[stamp].get(jid) is not None for jid in JOINT_ORDER) for stamp in stamps)
    rows = []
    for jid in JOINT_ORDER:
        samples = [frame[jid] for frame in frames.values() if frame.get(jid) is not None]
        n = len(samples)
        if n:
            mx, my, mz = (st.median(v[i] for v in samples) for i in range(3))
            deviations_mm = [1000 * math.dist(v, (mx, my, mz)) for v in samples]
            rms_mm = math.sqrt(sum(x*x for x in deviations_mm)/n)
            sx, sy, sz = [1000 * st.stdev([v[i] for v in samples]) if n > 1 else 0
                          for i in range(3)]
            medz = mz
            p95 = qtile(deviations_mm, .95)
        else:
            mx = my = medz = sx = sy = sz = rms_mm = p95 = math.nan
        rows.append({'joint_id': jid, 'joint_name': names.get(jid, str(jid)),
                     'samples_valid': n, 'samples_total': len(stamps),
                     'valid_pct': 100 * n / len(stamps),
                     'median_x_m': mx, 'median_y_m': my, 'median_z_m': medz,
                     'std_x_mm': sx, 'std_y_mm': sy, 'std_z_mm': sz,
                     'radial_rms_from_median_mm': rms_mm,
                     'radial_p95_from_median_mm': p95})
    return stamps, gaps_ms, counts, rows, frame_ids, len(meta_counts), invalid_rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv', help='CSV produced by record_pose3d_xyz.py')
    ap.add_argument('--summary', default=None, help='optional output CSV')
    args = ap.parse_args()
    stamps, gaps, counts, rows, frame_ids, matched, invalid = summarize(args.csv)
    total = len(stamps)
    duration = (stamps[-1]-stamps[0])/1e9 if total > 1 else 0
    out = args.summary or os.path.splitext(os.path.abspath(args.csv))[0] + '_summary.csv'
    with open(out, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print('\n=== FRONT KINECT 3D XYZ ANALYSIS ===')
    print(f'Frames: {total}; stamp span: {duration:.2f}s; rate over stamp span: '
          f'{(total-1)/duration:.2f}Hz' if duration > 0 else 'Frames: 1')
    if gaps:
        print(f'Stamp gap median={st.median(gaps):.1f}ms, p95={qtile(gaps,.95):.1f}ms, '
              f'max={max(gaps):.1f}ms; >500ms={sum(g>500 for g in gaps)}')
    print(f'Valid joint count histogram: {dict(sorted(counts.items()))}; '
          f'average={sum(k*v for k,v in counts.items())/total:.2f}/9')
    print(f'3D frames with 2D metadata: {matched}/{total}; invalid joint rows: {invalid}')
    print('Frame IDs:', dict(frame_ids))
    print('Joint                valid%   medZ(m)  stdX(mm) stdY(mm) stdZ(mm) radial p95(mm)')
    for r in rows:
        print(f"{r['joint_name']:<20} {r['valid_pct']:5.1f}  "
              f"{r['median_z_m']:7.3f}   {r['std_x_mm']:7.1f}  "
              f"{r['std_y_mm']:7.1f}  {r['std_z_mm']:7.1f}    "
              f"{r['radial_p95_from_median_mm']:7.1f}")
    print('\nInterpret ONLY for a stationary subject. Nonzero variation includes body sway.')
    print('Large deviations, even with high valid%, require examination; no safety certification.')
    print('Joint summary CSV:', out)


if __name__ == '__main__':
    main()
