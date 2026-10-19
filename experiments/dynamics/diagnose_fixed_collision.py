"""Reduce completed Issue18 saved collision geometry; never import a simulator.

Reads only sealed saved records. Geometry measurement FAIL is an observed A/B
change, not failure of the experiment. Contact feature identities are unknown.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path

from collision_geometry_match import compare_geometry

AB_SCALES = dict(distance_m=.001, position_m=.01, normal_deg=30.)
GEOMETRY_FIELDS = ('geoms', 'bodies', 'distance_m', 'position_m', 'normal')
KEYS = {'R1': [23622, 23623, 23624], 'R0': [33317, 33318, 33319]}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def read_rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream]


def canonical(event, index):
    """Geometry only, retaining the original sequence index as provenance."""
    value = {k: event[k] for k in GEOMETRY_FIELDS}
    geoms, bodies, normal = list(value['geoms']), list(value['bodies']), list(value['normal'])
    if geoms[0] > geoms[1]:
        geoms.reverse(); bodies.reverse(); normal = [-x for x in normal]
    length = math.sqrt(sum(x*x for x in normal))
    return dict(original_event_index=index, geoms=geoms, bodies=bodies,
                distance_m=float(value['distance_m']), position_m=list(value['position_m']),
                normal=[x / length for x in normal])


def angle(a, b):
    cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    return math.degrees(math.atan2(math.sqrt(sum(x*x for x in cross)), sum(x*y for x, y in zip(a, b))))


def robot_cube(event):
    return 'cube' in event['bodies'] and 'world' not in event['bodies']


def over(events, threshold=.001):
    return any(robot_cube(e) and e['distance_m'] < -threshold for e in events)


def unique_pair(events, pair):
    found = [canonical(e, i) for i, e in enumerate(events) if sorted(e['geoms']) == sorted(pair)]
    if len(found) != 1:
        raise ValueError('Missing or non-unique preregistered contact')
    return found[0]


def evaluate_h1(rows, prediction):
    h = prediction['H1']; found = {(r['label'], r['row']): r for r in rows}
    result = dict(status='INVALID', conjunction_required=True, geom_pair=h['geom_pair'],
        row=h['row'], preceding_row=h['preceding_row'], reference_MTD_m=h['independent_MTD_reference_m'])
    try:
        current, previous = [found[('R1', h[k])] for k in ('row', 'preceding_row')]
        for row in (current, previous):
            pair = next(p for p in row['A_B_comparison']['pairs'] if p['geoms'] == sorted(h['geom_pair']))
            if pair['correspondence_status'] != 'PASS' or pair['reference_count'] != 1 or pair['query_count'] != 1:
                raise ValueError('Missing, extra or ambiguous preregistered correspondence')
        archived, a, b = [unique_pair(current[k], h['geom_pair']) for k in ('archive', 'A', 'B')]
        prior_b = unique_pair(previous['B'], h['geom_pair'])
        target = h['independent_MTD_reference_m']
        errors = {k: abs(max(0., -e['distance_m']) - target) for k, e in [('archive', archived), ('A', a), ('B', b)]}
        if errors['A'] <= 0:
            raise ValueError('Zero baseline error makes fractional improvement undefined')
        reduction = 1. - errors['B'] / errors['A']
        jump = angle(prior_b['normal'], b['normal'])
        improvement = errors['B'] <= errors['A'] * (1. - h['error_reduction_fraction_at_least'])
        normal_pass = jump < h['native_incoming_normal_jump_strictly_below_deg']
        result.update(status='PASS' if improvement and normal_pass else 'FAIL',
            penetration_m={k: max(0., -e['distance_m']) for k, e in [('archive', archived), ('A', a), ('B', b)]},
            absolute_MTD_error_m=errors, relative_error_reduction_fraction=reduction,
            error_reduction_fraction_at_least=h['error_reduction_fraction_at_least'],
            error_reduction_condition=improvement, B_incoming_canonical_normal_jump_deg=jump,
            normal_jump_strictly_below_deg=h['native_incoming_normal_jump_strictly_below_deg'],
            normal_condition=normal_pass, B_previous=prior_b, B_current=b,
            interpretation='Fixed-pose collision-query sensitivity only; not physical grasp or unique root-cause acceptance')
    except (KeyError, StopIteration, ValueError) as exc:
        result['reason'] = str(exc)
    return result


def evaluate_h2(rows, prediction):
    h = prediction['H2']; found = {(r['label'], r['row']): r for r in rows}; observations = []
    for index in h['rows']:
        row = found.get(('R1', index))
        if row is None:
            observations.append(dict(row=index, status='UNKNOWN', contacts=[], B_any_robot_cube_over_1mm=None))
            continue
        contacts = [dict(**canonical(e, i), over_1mm=e['distance_m'] < -h['predicted_any_native_robot_cube_penetration_over_m'])
                    for i, e in enumerate(row['B']) if robot_cube(e)]
        observations.append(dict(row=index, status='OBSERVED', contacts=contacts,
            B_any_robot_cube_over_1mm=any(e['over_1mm'] for e in contacts),
            A_B_contact_correspondence_status='UNKNOWN' if any(p['correspondence_status'] != 'PASS' or p['count_status'] != 'PASS'
                for p in row['A_B_comparison']['pairs']) else 'PASS'))
    return dict(status='INVALID' if any(r['status'] == 'UNKNOWN' for r in observations) else 'OBSERVED', rows=observations,
        threshold_m=h['predicted_any_native_robot_cube_penetration_over_m'], strict_greater_than=True,
        B_rows_with_any_robot_cube_over_1mm=sum(r['B_any_robot_cube_over_1mm'] is True for r in observations),
        interpretation='Boundary observation only; neither B nor historical SAT is automatically ground truth')


def _counts(rows, algorithm):
    events = [e for r in rows for e in r[algorithm]]
    robot = [e for e in events if robot_cube(e)]
    return dict(all_contacts=len(events), robot_cube_contacts=len(robot),
        robot_cube_over_1mm_contacts=sum(e['distance_m'] < -.001 for e in robot),
        robot_cube_over_1mm_pose_rows=sum(over(r[algorithm]) for r in rows),
        robot_cube_max_penetration_m=max([max(0., -e['distance_m']) for e in robot], default=0.))


def _contact_rows(row):
    """Join saved matcher indices; never match contacts by list position."""
    cmp = row['A_B_comparison']; archive_cmp = compare_geometry(row['archive'], row['A'])
    archive_map = {m['query_event_index']: m['reference_event_index'] for p in archive_cmp['pairs']
                   if p['correspondence_status'] == 'PASS' for m in p['matches']}
    result = []
    for pair in cmp['pairs']:
        def append(ai, bi, status):
            archive_index = archive_map.get(ai)
            result.append(dict(label=row['label'], row=row['row'], phase=row['phase'],
                contacts_jsonl_line=row['row']+1, pair=pair['geoms'], correspondence_status=status,
                pair_presence_status=pair['presence_status'], pair_measurement_status=pair['measurement_status'],
                A_pair_contact_count=pair['reference_count'], B_pair_contact_count=pair['query_count'],
                archive=None if archive_index is None else canonical(row['archive'][archive_index], archive_index),
                A=None if ai is None else canonical(row['A'][ai], ai),
                B=None if bi is None else canonical(row['B'][bi], bi), contact_feature_id='UNKNOWN'))
        if pair['correspondence_status'] == 'UNKNOWN':
            for i, e in enumerate(row['A']):
                if sorted(e['geoms']) == pair['geoms']: append(i, None, 'AMBIGUOUS_A_UNKNOWN')
            for i, e in enumerate(row['B']):
                if sorted(e['geoms']) == pair['geoms']: append(None, i, 'AMBIGUOUS_B_UNKNOWN')
        else:
            for match in pair['matches']: append(match['reference_event_index'], match['query_event_index'], 'MATCHED')
            for index in pair['missing_reference_event_indices']: append(index, None, 'MISSING_B_UNKNOWN')
            for index in pair['extra_query_event_indices']: append(None, index, 'EXTRA_B_UNKNOWN')
    # A failed archive reproduction must not hide any archive-only event.
    represented = {r['archive']['original_event_index'] for r in result if r['archive'] is not None}
    for index, event in enumerate(row['archive']):
        if index not in represented:
            result.append(dict(label=row['label'], row=row['row'], phase=row['phase'], contacts_jsonl_line=row['row']+1,
                pair=sorted(event['geoms']), correspondence_status='ARCHIVE_UNMATCHED_UNKNOWN',
                pair_presence_status='UNKNOWN', pair_measurement_status='UNKNOWN', A_pair_contact_count=None,
                B_pair_contact_count=None, archive=canonical(event, index), A=None, B=None, contact_feature_id='UNKNOWN'))
    return result


def reduce_geometry(rows):
    labels = {}; contacts = []
    for label in sorted({r['label'] for r in rows}):
        selected = [r for r in rows if r['label'] == label]
        pair_records = [(r['row'], p) for r in selected for p in r['A_B_comparison']['pairs']]
        matches = [m for _, p in pair_records for m in p['matches']]
        label_contacts = [c for r in selected for c in _contact_rows(r)]; contacts.extend(label_contacts)
        unknown = [c for c in label_contacts if c['correspondence_status'] != 'MATCHED']
        changes = [dict(row=index, geoms=p['geoms'], presence_status=p['presence_status'], A_count=p['reference_count'],
                        B_count=p['query_count'], contact_correspondence='UNKNOWN')
                   for index, p in pair_records if p['count_status'] != 'PASS' or p['correspondence_status'] != 'PASS']
        labels[label] = dict(pose_rows=len(selected), archive=_counts(selected, 'archive'), A=_counts(selected, 'A'), B=_counts(selected, 'B'),
            reported_over_1mm_pose_changes=dict(eliminated_rows=[r['row'] for r in selected if over(r['A']) and not over(r['B'])],
                new_rows=[r['row'] for r in selected if not over(r['A']) and over(r['B'])],
                scope='Frozen selected pose rows only; no claim about any unsampled state or physical acceptance'),
            matched_contact_changes=dict(matched_contacts=len(matches),
                distance_changed_contacts=sum(m['distance_m'] > 0 for m in matches),
                point_changed_contacts=sum(m['position_m'] > 0 for m in matches), normal_changed_contacts=sum(m['normal_deg'] > 0 for m in matches),
                distance_changes_over_reproduction_tolerance=sum(not m['within_tolerance']['distance_m'] for m in matches),
                point_changes_over_reproduction_tolerance=sum(not m['within_tolerance']['position_m'] for m in matches),
                normal_changes_over_reproduction_tolerance=sum(not m['within_tolerance']['normal_deg'] for m in matches),
                max_abs_distance_change_m=max([m['distance_m'] for m in matches], default=0.),
                max_point_displacement_m=max([m['position_m'] for m in matches], default=0.),
                max_canonical_normal_change_deg=max([m['normal_deg'] for m in matches], default=0.),
                eliminated_robot_cube_over_1mm_contacts=sum(c['correspondence_status']=='MATCHED' and robot_cube(c['A']) and
                    c['A']['distance_m'] < -.001 and c['B']['distance_m'] >= -.001 for c in label_contacts),
                new_robot_cube_over_1mm_contacts=sum(c['correspondence_status']=='MATCHED' and robot_cube(c['B']) and
                    c['A']['distance_m'] >= -.001 and c['B']['distance_m'] < -.001 for c in label_contacts)),
            A_B_measurement_status_counts=dict(Counter(r['A_B_comparison']['status'] for r in selected)),
            contact_count_or_pair_changes=changes, contact_correspondence_status='UNKNOWN' if unknown else 'PASS',
            contact_correspondence_status_counts=dict(Counter(c['correspondence_status'] for c in label_contacts)))
    return labels, contacts


def maximum_errors(comparisons):
    pairs = [p for r in comparisons for p in r['comparison']['pairs']]
    matches = [m for p in pairs for m in p['matches']]
    return dict(status='PASS' if all(r['comparison']['status']=='PASS' for r in comparisons) else 'UNKNOWN',
        pose_rows=len(comparisons), matched_contacts=len(matches),
        max_abs_distance_error_m=max([m['distance_m'] for m in matches], default=0.),
        max_point_error_m=max([m['position_m'] for m in matches], default=0.),
        max_oriented_normal_error_deg=max([m['normal_deg'] for m in matches], default=0.),
        count_changed_pairs=sum(p['count_status'] != 'PASS' for p in pairs),
        ambiguous_pairs=sum(p['correspondence_status'] != 'PASS' for p in pairs))


def _index(records, expected, label):
    result = {(r['label'], r['row']): r for r in records}
    if len(result) != len(records) or set(result) != expected:
        raise ValueError(label+' row set differs from sealed selection')
    return result


def _verify_audit(result, rows, algorithms):
    if result['status'] != 'PASS' or result['query_reproduction'] != 'PASS' or not result['input_hashes_unchanged']:
        raise ValueError('Requires completed PASS query stages')
    if result['new_steps'] != 0 or result['new_force_solves'] != 0:
        raise ValueError('Nonzero dynamics/force count')
    audit = result['api_calls']; calls = audit['calls']
    if audit['forbidden_attempts'] or set(calls)-{'mj_kinematics','mj_collision','mj_id2name','mj_name2id'}:
        raise ValueError('Unauthorized saved API audit')
    if calls.get('mj_collision') != rows*algorithms or calls.get('mj_kinematics') != rows*algorithms:
        raise ValueError('Saved API query count mismatch')
    return dict(pose_rows=rows, collision_queries=calls['mj_collision'], calls=calls, forbidden_attempts=[])


def _csv(path, contacts, sat_rows):
    fields = ['label','row','contacts_jsonl_line','phase','pair','correspondence_status','pair_presence_status',
              'pair_measurement_status','A_pair_contact_count','B_pair_contact_count','contact_feature_id','historical_SAT_reference']
    fields += [stage+'_'+key for stage in ('archive','A','B') for key in ('original_event_index',)+GEOMETRY_FIELDS]
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n'); writer.writeheader()
        for contact in contacts:
            value = {k: contact[k] for k in fields if k in contact}
            value['historical_SAT_reference'] = [r for r in sat_rows.get((contact['label'], str(contact['row'])), [])
                if r['geom_pair'].split('|') == contact['pair']]
            for stage in ('archive','A','B'):
                for key, item in (contact[stage] or {}).items(): value[stage+'_'+key] = item
            writer.writerow({k: json.dumps(v, separators=(',',':'), allow_nan=False) if isinstance(v,(dict,list)) else v for k,v in value.items()})


def analyze(campaign, output):
    campaign, output = Path(campaign).resolve(), Path(output).resolve()
    if output.exists(): raise FileExistsError(output)
    # Check completion before reading either algorithm's saved query records.
    if not all((campaign/p).is_file() for p in ('gate0/result.json','ab/result.json')) or (campaign/'STOPPED.json').exists():
        raise ValueError('Requires a completed, non-stopped A/B campaign')
    gate_result, ab_result = read(campaign/'gate0/result.json'), read(campaign/'ab/result.json')
    if gate_result.get('status') != 'PASS' or ab_result.get('status') != 'PASS':
        raise ValueError('Requires completed PASS query stages')
    plan = read(campaign/'plan.json'); prediction = read(campaign/'predictions.json')
    sources = [campaign, Path(plan['source_root']).resolve(), Path(plan['history']).resolve()]
    if any(output == p or p in output.parents for p in sources):
        raise ValueError('Output must be outside campaign and original sources')
    paths = [campaign/p for p in ('plan.json','plan.sha256','predictions.json','predictions.sha256',
        'gate0/result.json','gate0/STARTED.json','gate0/A.jsonl','gate0/reproduction.json',
        'ab/result.json','ab/STARTED.json','ab/A.jsonl','ab/B.jsonl','ab/comparison.jsonl','ab/reproduction.json','ab/model-difference.json')]
    paths += [Path(p) for p in plan['inputs_sha256']]
    source = Path(plan['source_root'])/'reports/issue16-evidence'
    paths += [source/p for p in ('manifest.json','summary.json','exceedances.csv')]
    paths += [Path(__file__).resolve(), Path(__file__).with_name('collision_geometry_match.py').resolve()]
    paths += [campaign/stage/name for stage in ('gate0','ab') for name in plan['code_sha256']]
    before = {str(p): digest(p) for p in paths}
    if digest(campaign/'plan.json') != (campaign/'plan.sha256').read_text().strip() or digest(campaign/'predictions.json') != (campaign/'predictions.sha256').read_text().strip():
        raise ValueError('Plan or prediction seal changed')
    for path, expected in plan['inputs_sha256'].items():
        if before[path] != expected: raise ValueError('Frozen source SHA changed: '+path)
    for stage in ('gate0','ab'):
        started = read(campaign/stage/'STARTED.json')
        if started['code_sha256'] != plan['code_sha256'] or started['plan_sha256'] != digest(campaign/'plan.json'):
            raise ValueError('Executed plan/code mismatch')
        for name, expected in plan['code_sha256'].items():
            if digest(campaign/stage/name) != expected: raise ValueError('Executed source snapshot changed')
    if (prediction['plan_sha256'] != digest(campaign/'plan.json') or prediction['gate0_result_sha256'] != digest(campaign/'gate0/result.json')
            or prediction['code_sha256'] != plan['code_sha256'] or ab_result['preregistration_sha256'] != digest(campaign/'predictions.json')
            or not prediction['first_native_query_not_yet_executed']):
        raise ValueError('Preregistration dependency mismatch')
    expected = {(label, row) for label, rows in plan['selected_rows'].items() for row in rows}
    gate_expected = {(label,row) for label,rows in plan['gate0_rows'].items() for row in rows}
    a = _index(read_rows(campaign/'ab/A.jsonl'), expected, 'A')
    b = _index(read_rows(campaign/'ab/B.jsonl'), expected, 'B')
    rows = read_rows(campaign/'ab/comparison.jsonl'); indexed = _index(rows, expected, 'comparison')
    gate = _index(read_rows(campaign/'gate0/A.jsonl'), gate_expected, 'Gate0')
    for stage, index in [('A',a),('B',b),('A',gate)]:
        for key, value in index.items():
            if (value['algorithm'] != stage or value['contact_count'] != len(value['events']) or value['time'] != 0 or value['nefc'] != 0
                    or not value['fresh_MjData'] or not value['scratch_qvel_and_ctrl_zero']
                    or value['qpos_sha256'] != plan['qpos_sha256'][key[0]][str(key[1])]):
                raise ValueError('Saved query invariant mismatch')
    comparisons = []; gate_comparisons = []
    for key, row in indexed.items():
        if row['A'] != a[key]['events'] or row['B'] != b[key]['events']:
            raise ValueError('Saved comparison/query records differ')
        comparison = compare_geometry(row['A'],row['B'],cost_scales=plan['ab_matching_cost_scales'])
        if comparison != row['A_B_comparison']: raise ValueError('Saved matcher evidence changed')
        reproduced = compare_geometry(row['archive'],row['A'],plan['gate0_tolerances'])
        comparisons.append(dict(label=key[0],row=key[1],comparison=reproduced))
        if key in gate:
            gate_comparisons.append(dict(label=key[0],row=key[1],comparison=compare_geometry(row['archive'],gate[key]['events'],plan['gate0_tolerances'])))
    full_errors, gate_errors = maximum_errors(comparisons), maximum_errors(gate_comparisons)
    if full_errors['status'] != 'PASS' or gate_errors['status'] != 'PASS': raise ValueError('A reproduction not established')
    if (_index(comparisons,expected,'computed A reproduction') != _index(read(campaign/'ab/reproduction.json'),expected,'saved A reproduction')
            or _index(gate_comparisons,gate_expected,'computed Gate0 reproduction') != _index(read(campaign/'gate0/reproduction.json'),gate_expected,'saved Gate0 reproduction')):
        raise ValueError('Saved reproduction evidence mismatch')
    for result, selection in [(gate_result,plan['gate0_rows']),(ab_result,plan['selected_rows'])]:
        if result['rows'] != {k:len(v) for k,v in selection.items()}: raise ValueError('Saved stage row counts mismatch')
    audits = dict(gate0=_verify_audit(gate_result,len(gate_expected),1), ab=_verify_audit(ab_result,len(expected),2))
    for result, count in [(gate_result,0),(ab_result,len(expected))]:
        if result['native_queries'] != count or result['plan_sha256'] != digest(campaign/'plan.json'):
            raise ValueError('Saved native query count/plan mismatch')
    difference = read(campaign/'ab/model-difference.json')
    if difference['option_differences'] != ['enableflags'] or difference['enableflags_A'] != 0 or difference['enableflags_B'] <= 0:
        raise ValueError('Unexpected model difference')
    issue16 = read(source/'summary.json')
    frame = next(r for r in issue16['episodes']['R1']['key_frames'] if r['row'] == prediction['H1']['row'])
    target = next(c['sat']['depth_m'] for c in frame['contacts'] if sorted(c['geom_pair']) == sorted(prediction['H1']['geom_pair']))
    if target != prediction['H1']['independent_MTD_reference_m']:
        raise ValueError('Preregistered independent MTD differs from frozen Issue16 reference')
    sat_reference = {}
    with (source/'exceedances.csv').open(newline='') as stream:
        sat_saved = list(csv.DictReader(stream))
    for row in sat_saved:
        label = row.get('label', row.get('episode'))
        sat_reference.setdefault((label,str(row['row'])),[]).append(row)
    labels, contacts = reduce_geometry(rows)
    focus = {(label,index) for label,indices in KEYS.items() for index in indices}
    focus.update(('R1',index) for index in prediction['H2']['rows'])
    focus.update((control['label'],control['row']) for control in plan['normal_controls'])
    key_geometry = [dict(label=r['label'],row=r['row'],phase=r['phase'],
        archive=[canonical(e,i) for i,e in enumerate(r['archive'])], A=[canonical(e,i) for i,e in enumerate(r['A'])],
        B=[canonical(e,i) for i,e in enumerate(r['B'])], A_B_comparison=r['A_B_comparison'],
        historical_SAT_reference=sat_reference.get((r['label'],str(r['row'])),[]))
        for r in rows if (r['label'],r['row']) in focus]
    summary = dict(issue=18,status='COMPLETE',diagnostic_only=True,query_reproduction=dict(status='PASS',gate0=gate_errors,all_A=full_errors),
        H1=evaluate_h1(rows,prediction),H2=evaluate_h2(rows,prediction),physical_grasp='FAIL',root_cause='UNKNOWN',model_change_authorization='NONE',
        actual=dict(selected_pose_rows=len(expected),selected_pose_rows_per_label={k:len(v) for k,v in plan['selected_rows'].items()},
            gate0_pose_rows=len(gate_expected),A_queries=len(a),B_queries=len(b),total_collision_queries=sum(v['collision_queries'] for v in audits.values()),
            api_audit=audits,new_steps=0,new_force_solves=0,new_constraint_solves=0,new_control_executions=0,reducer_simulator_calls=0), per_label=labels,
        normal_orientation='lexically first geom -> second geom',contact_feature_id='UNKNOWN',
        source_commit=plan['source_commit'],plan_sha256=digest(campaign/'plan.json'),prediction_sha256=digest(campaign/'predictions.json'),
        SAT_scope='Only stored historical exceedance rows; no new SAT computation; independent nonstandard geometry reference, not official mjContact.dist criterion',
        limitations=['Frozen pose set only; no new physics or grasp attempt.',
            'Missing/extra contacts and ambiguous matching have UNKNOWN contact correspondence.',
            'A/B measurement FAIL records changed geometry, not experiment failure.'])
    summary['key_geometry_pose_rows'] = len(key_geometry)
    summary['normal_control_rows'] = [{'label':c['label'],'row':c['row']} for c in plan['normal_controls']]
    summary['historical_original_over_1mm_pose_rows'] = {k:len(v) for k,v in plan['original_exceedance_rows'].items()}
    after = {str(p): digest(p) for p in paths}
    if before != after: raise ValueError('Input SHA changed during saved-data reduction')
    summary['input_immutability'] = 'PASS'
    # Create output only after all source reads, calculations and integrity checks.
    output.mkdir(parents=True,exist_ok=False)
    _csv(output/'contacts.csv',contacts,sat_reference)
    for name,value in [('key-geometry.json',key_geometry),('summary.json',summary),('manifest.json',dict(
            input_sha256_before=before,input_sha256_after=after,input_immutability='PASS',
            reducer_sha256=digest(__file__),new_steps=0,new_force_solves=0,new_collision_queries=0))]:
        with (output/name).open('x') as stream: json.dump(value,stream,indent=2,allow_nan=False); stream.write('\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args = parser.parse_args(); result = analyze(args.campaign,args.output_dir)
    print(json.dumps(dict(status=result['status'],H1=result['H1']['status'],actual=result['actual'])))
