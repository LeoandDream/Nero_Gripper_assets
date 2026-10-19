"""Issue 16: saved robot/cube contact patterns, with no simulator imports.

Rows and event indexes are zero based; JSONL line and recorded step are row+1.
Adjacent unique contacts of the same unordered geom pair are compared as samples,
never identified as the same material feature. Multiple contacts are ambiguous and
excluded from sample-to-sample angle statistics rather than matched by list index.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


LIMIT_M = 0.001  # Original criterion; angle thresholds are descriptive only.
ANGLE_THRESHOLDS = (10, 30, 60)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def angle(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    denominator = np.linalg.norm(a) * np.linalg.norm(b)
    if not np.isfinite(denominator) or denominator <= 0:
        raise ValueError('Invalid saved contact normal')
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / denominator, -1, 1))))


def intervals(records):
    """Consecutive physical rows, preserving multiplicity separately from rows."""
    by_row = defaultdict(list)
    for record in records:
        by_row[record['row']].append(record)
    chunks = []
    for row in sorted(by_row):
        if not chunks or row != chunks[-1][-1] + 1:
            chunks.append([])
        chunks[-1].append(row)
    result = []
    for index, rows in enumerate(chunks):
        members = [record for row in rows for record in by_row[row]]
        maximum = max(members, key=lambda record: -record['distance_m'])
        result.append(dict(interval_index=index, start_row=rows[0], end_row=rows[-1],
                           physical_rows=len(rows), contact_records=len(members),
                           start_time_s=members[0]['time_s'], end_time_s=members[-1]['time_s'],
                           sampled_span_s=members[-1]['time_s'] - members[0]['time_s'],
                           maximum_penetration_mm=-maximum['distance_m'] * 1000,
                           maximum_row=maximum['row'], maximum_event_index=maximum['event_index']))
    return result


def csv_write(path, records, fields):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(records)


def analyze_rows(rows, times, phases, cube_rotations, highlight_rows=()):
    """Analyze already decoded rows; only arrays and saved JSON fields are used."""
    if len(rows) != len(times) or len(times) != len(phases):
        raise ValueError('Saved archive/contact lengths differ')
    frames = []
    over = []
    multiplicity = Counter()
    total_contacts = 0
    for i, row in enumerate(rows):
        if abs(float(row['time']) - times[i]) > 1e-10 or row['phase'] != phases[i]:
            raise ValueError(f'Contact/trajectory alignment mismatch at row {i}')
        frame = defaultdict(list)
        for j, event in enumerate(row['events']):
            bodies = event['bodies']
            if 'cube' not in bodies or 'world' in bodies:
                continue
            pair = '|'.join(sorted(event['geoms']))
            # Saved frame normal points geom1 -> geom2; orient toward cube.
            sign = 1.0 if bodies[1] == 'cube' else -1.0
            normal = sign * np.asarray(event['normal'], dtype=float)
            local_normal = cube_rotations[i].T @ normal
            record = dict(row=i, contacts_jsonl_line=i + 1, recorded_step=i + 1,
                          event_index=j, time_s=float(times[i]), phase=str(phases[i]),
                          geom_pair=pair, geoms_saved_order='|'.join(event['geoms']),
                          bodies_saved_order='|'.join(bodies), distance_m=float(event['distance_m']),
                          penetration_mm=max(0, -float(event['distance_m']) * 1000),
                          normal_toward_cube_world=normal.tolist(),
                          normal_toward_cube_local=local_normal.tolist(),
                          position_world_m=event['position_m'], feature_id='UNKNOWN')
            if not np.isfinite(record['distance_m']):
                raise ValueError(f'Nonfinite distance at row {i}, event {j}')
            frame[pair].append(record)
            total_contacts += 1
            if record['distance_m'] < -LIMIT_M:
                over.append(record)
        for pair, members in frame.items():
            multiplicity[(pair, len(members))] += 1
        frames.append(frame)

    transitions = []
    ambiguous = []
    incoming = {}
    outgoing = {}
    for i in range(1, len(frames)):
        for pair in sorted(frames[i - 1].keys() & frames[i].keys()):
            previous = frames[i - 1][pair]
            current = frames[i][pair]
            if len(previous) != 1 or len(current) != 1:
                ambiguous.append(dict(previous_row=i - 1, row=i, geom_pair=pair,
                                      previous_contact_count=len(previous), contact_count=len(current),
                                      status='UNKNOWN_CORRESPONDENCE_NO_FEATURE_ID'))
                continue
            a, b = previous[0], current[0]
            transition = dict(previous_row=i - 1, row=i, previous_event_index=a['event_index'],
                              event_index=b['event_index'], geom_pair=pair, phase=b['phase'],
                              time_s=b['time_s'], previous_distance_m=a['distance_m'],
                              distance_m=b['distance_m'],
                              previous_over_1mm=a['distance_m'] < -LIMIT_M,
                              current_over_1mm=b['distance_m'] < -LIMIT_M,
                              world_angle_deg=angle(a['normal_toward_cube_world'], b['normal_toward_cube_world']),
                              cube_local_angle_deg=angle(a['normal_toward_cube_local'], b['normal_toward_cube_local']),
                              position_world_delta_mm=float(np.linalg.norm(np.asarray(b['position_world_m']) - a['position_world_m']) * 1000),
                              correspondence='UNIQUE_PAIR_SAMPLES_FEATURE_UNKNOWN')
            transitions.append(transition)
            incoming[(i, pair)] = transition
            outgoing[(i - 1, pair)] = transition

    bounces = []
    for i in range(1, len(frames) - 1):
        for pair in sorted(frames[i].keys()):
            if (i, pair) not in incoming or (i, pair) not in outgoing:
                continue
            a, b, c = (frames[k][pair][0] for k in (i - 1, i, i + 1))
            entry = incoming[(i, pair)]
            exit_ = outgoing[(i, pair)]
            bounces.append(dict(row=i, event_index=b['event_index'], geom_pair=pair,
                                phase=b['phase'], time_s=b['time_s'], distance_m=b['distance_m'],
                                current_over_1mm=b['distance_m'] < -LIMIT_M,
                                incoming_angle_deg=entry['world_angle_deg'],
                                outgoing_angle_deg=exit_['world_angle_deg'],
                                endpoint_angle_deg=angle(a['normal_toward_cube_world'], c['normal_toward_cube_world'])))

    for record in over:
        key = (record['row'], record['geom_pair'])
        record['incoming_world_angle_deg'] = incoming.get(key, {}).get('world_angle_deg')
        record['outgoing_world_angle_deg'] = outgoing.get(key, {}).get('world_angle_deg')
    groups = defaultdict(list)
    for record in over:
        groups[(record['geom_pair'], record['phase'])].append(record)
    over_intervals = intervals(over)
    threshold_stats = {}
    for threshold in ANGLE_THRESHOLDS:
        jumps = [t for t in transitions if t['world_angle_deg'] >= threshold]
        bounce = [b for b in bounces if min(b['incoming_angle_deg'], b['outgoing_angle_deg']) >= threshold
                  and b['endpoint_angle_deg'] < threshold]
        threshold_stats[str(threshold)] = dict(
            jump_transitions=len(jumps), current_row_over_1mm=sum(t['current_over_1mm'] for t in jumps),
            either_endpoint_over_1mm=sum(t['current_over_1mm'] or t['previous_over_1mm'] for t in jumps),
            both_endpoints_within_1mm=sum(not t['current_over_1mm'] and not t['previous_over_1mm'] for t in jumps),
            over_records_with_incoming_jump=sum(r['incoming_world_angle_deg'] is not None and r['incoming_world_angle_deg'] >= threshold for r in over),
            over_records_with_incoming_or_outgoing_jump=sum(any(r[k] is not None and r[k] >= threshold for k in ('incoming_world_angle_deg', 'outgoing_world_angle_deg')) for r in over),
            bounce_centers=len(bounce), bounce_centers_over_1mm=sum(b['current_over_1mm'] for b in bounce),
            bounce_definition=f'both adjacent angles >= {threshold}deg; previous-to-next angle < {threshold}deg')
    angle_groups = defaultdict(list)
    for transition in transitions:
        angle_groups[(transition['geom_pair'], transition['phase'])].append(transition)
    summary = dict(physical_rows=len(rows), robot_cube_contact_records=total_contacts,
                   over_1mm_contact_records=len(over), over_1mm_physical_rows=len({r['row'] for r in over}),
                   first_over_1mm=over[0] if over else None,
                   maximum_penetration=max(over, key=lambda r: -r['distance_m']) if over else None,
                   contiguous_intervals=over_intervals,
                   singleton_intervals=sum(i['physical_rows'] == 1 for i in over_intervals),
                   sustained_intervals=sum(i['physical_rows'] > 1 for i in over_intervals),
                   longest_interval_rows=max((i['physical_rows'] for i in over_intervals), default=0),
                   groups=[dict(geom_pair=pair, phase=phase, contact_records=len(records),
                                physical_rows=len({r['row'] for r in records}),
                                intervals=intervals(records)) for (pair, phase), records in sorted(groups.items())],
                   multiplicities=[dict(geom_pair=p, contacts_per_row=n, physical_rows=count)
                                   for (p, n), count in sorted(multiplicity.items())],
                   adjacent_unique_pair_comparisons=len(transitions),
                   ambiguous_adjacent_pair_comparisons=len(ambiguous),
                   descriptive_angle_thresholds_deg=threshold_stats,
                   angle_groups=[dict(geom_pair=pair, phase=phase, comparisons=len(members),
                                      maximum_angle_deg=max(t['world_angle_deg'] for t in members),
                                      jumps_at_least_deg={str(threshold): sum(t['world_angle_deg'] >= threshold for t in members)
                                                          for threshold in ANGLE_THRESHOLDS})
                                 for (pair, phase), members in sorted(angle_groups.items())],
                   highlight_records=[record for i in highlight_rows for members in frames[i].values() for record in members],
                   material_contact_feature_id='UNKNOWN_NOT_RECORDED')
    return summary, over, transitions, bounces, ambiguous


def run(campaign, manifest_path, output):
    output = Path(output)
    # Refuse to put derived output anywhere inside the archived campaign.
    if Path(campaign).resolve() in output.resolve().parents or output.resolve() == Path(campaign).resolve():
        raise ValueError('Output must be outside archived campaign')
    output.mkdir(parents=True, exist_ok=False)
    inputs = [Path(manifest_path)] + [Path(campaign) / label / name for label in ('R0', 'R1')
                                        for name in ('trajectory.npz', 'contacts.jsonl', 'observations.npz')]
    before = {str(path.resolve()): sha256(path) for path in inputs}
    manifest = json.loads(Path(manifest_path).read_text())
    expected = {}

    def collect(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if isinstance(child, dict) and 'sha256' in child:
                    expected[key] = child['sha256']
                collect(child)
        elif isinstance(value, list):
            for child in value:
                collect(child)
    collect(manifest)
    for path in inputs[1:]:
        key = str(path.resolve())
        if expected.get(key) != before[key]:
            raise ValueError(f'Original manifest missing/mismatched input: {key}')
    summaries = {}
    for label in ('R0', 'R1'):
        episode = Path(campaign) / label
        with np.load(episode / 'trajectory.npz', allow_pickle=False) as saved:
            times, phases = saved['time'], saved['phase']
        with np.load(episode / 'observations.npz', allow_pickle=False) as saved:
            if not np.array_equal(saved['time'], times):
                raise ValueError('Observation/trajectory times differ')
            rotations = saved['gripper_rotation_world'] @ saved['cube_rotation_in_gripper']
        with (episode / 'contacts.jsonl').open() as stream:
            rows = [json.loads(line) for line in stream]
        highlight = (33317, 33318, 33319) if label == 'R0' else (23622, 23623, 23624)
        summary, over, transitions, bounces, ambiguous = analyze_rows(rows, times, phases, rotations, highlight)
        summaries[label] = summary
        fields = ['row', 'contacts_jsonl_line', 'recorded_step', 'event_index', 'time_s', 'phase', 'geom_pair',
                  'geoms_saved_order', 'bodies_saved_order', 'distance_m', 'penetration_mm',
                  'incoming_world_angle_deg', 'outgoing_world_angle_deg', 'normal_toward_cube_world',
                  'normal_toward_cube_local', 'position_world_m', 'feature_id']
        csv_write(output / f'{label}-over-1mm.csv', over, fields)
        csv_write(output / f'{label}-critical-window.csv', summary.pop('highlight_records'), fields)
        summary['highlight_rows'] = highlight
        summary['over_1mm_interval_count'] = len(summary['contiguous_intervals'])
        interval_records = [dict(scope='all_robot_cube', geom_pair='', phase='', **interval)
                            for interval in summary.pop('contiguous_intervals')]
        for group in summary['groups']:
            chunks = group.pop('intervals')
            group['interval_count'] = len(chunks)
            group['singleton_intervals'] = sum(chunk['physical_rows'] == 1 for chunk in chunks)
            group['longest_interval_rows'] = max(chunk['physical_rows'] for chunk in chunks)
            interval_records.extend(dict(scope='geom_pair_phase', geom_pair=group['geom_pair'], phase=group['phase'], **interval)
                                    for interval in chunks)
        csv_write(output / f'{label}-over-1mm-intervals.csv', interval_records,
                  ['scope', 'geom_pair', 'phase', 'interval_index', 'start_row', 'end_row', 'physical_rows',
                   'contact_records', 'start_time_s', 'end_time_s', 'sampled_span_s',
                   'maximum_penetration_mm', 'maximum_row', 'maximum_event_index'])
        jumps = [t for t in transitions if t['world_angle_deg'] >= min(ANGLE_THRESHOLDS)]
        csv_write(output / f'{label}-normal-jumps.csv', jumps,
                  ['previous_row', 'row', 'previous_event_index', 'event_index', 'geom_pair', 'phase', 'time_s',
                   'previous_distance_m', 'distance_m', 'previous_over_1mm', 'current_over_1mm',
                   'world_angle_deg', 'cube_local_angle_deg', 'position_world_delta_mm', 'correspondence'])
        bounce = [b for b in bounces if min(b['incoming_angle_deg'], b['outgoing_angle_deg']) >= 10 and b['endpoint_angle_deg'] < 60]
        csv_write(output / f'{label}-bounce-candidates.csv', bounce,
                  ['row', 'event_index', 'geom_pair', 'phase', 'time_s', 'distance_m', 'current_over_1mm',
                   'incoming_angle_deg', 'outgoing_angle_deg', 'endpoint_angle_deg'])
        csv_write(output / f'{label}-ambiguous-comparisons.csv', ambiguous,
                  ['previous_row', 'row', 'geom_pair', 'previous_contact_count', 'contact_count', 'status'])
    after = {str(path.resolve()): sha256(path) for path in inputs}
    if before != after:
        raise ValueError('Archived inputs changed during analysis')
    report = dict(scope='Pure saved-array/JSON analysis; no integration, contact generation or force evaluation',
                  argv=sys.argv, analyzer_sha256=sha256(__file__), python=sys.executable, numpy=np.__version__, inputs_before_sha256=before,
                  inputs_after_sha256=after, original_manifest_matches=True, input_hashes_unchanged=True,
                  index_convention='row and event_index zero based; JSONL line/recorded step=row+1',
                  original_penetration_limit_m=LIMIT_M,
                  angle_thresholds='10/30/60 degrees descriptive only; no smoothing/filtering/re-evaluation of acceptance',
                  correspondence='Consecutive physical rows, same unordered geom pair, exactly one contact on both rows. Contact index is only an archive locator; multi-contact correspondence skipped as UNKNOWN. Unique pair samples do not establish material feature identity.',
                  limitations='Observed temporal association cannot identify collision feature or establish a physical root cause; no face/feature IDs are present in contacts.jsonl.',
                  episodes=summaries)
    with (output / 'summary.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({label: {key: value for key, value in summary.items() if key in
                            ('over_1mm_contact_records', 'over_1mm_physical_rows', 'singleton_intervals',
                             'sustained_intervals', 'longest_interval_rows', 'adjacent_unique_pair_comparisons',
                             'ambiguous_adjacent_pair_comparisons')} for label, summary in summaries.items()}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', required=True, type=Path)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    arguments = parser.parse_args()
    run(arguments.campaign, arguments.manifest, arguments.output_dir)
