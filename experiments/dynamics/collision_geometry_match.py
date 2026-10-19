"""Compare saved contact geometry only; this module never queries a simulator.

Normals point from canonical (lexically first) geom to the second geom. Input
sequence indices locate events and never contribute to correspondence costs.
"""
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


GATE0_TOLERANCES = {'distance_m': 1e-9, 'position_m': 1e-8, 'normal_deg': 1e-5}


def _limits(overrides):
    result = dict(GATE0_TOLERANCES)
    if overrides:
        if set(overrides) - set(result):
            raise ValueError('unknown geometry limit key')
        result.update(overrides)
    if any(not math.isfinite(float(v)) or float(v) <= 0 for v in result.values()):
        raise ValueError('geometry limits must be finite and positive')
    return {key: float(value) for key, value in result.items()}


def _canonical(events):
    groups = {}
    for index, event in enumerate(events):
        geoms, bodies = tuple(event['geoms']), tuple(event['bodies'])
        if (len(geoms) != 2 or len(bodies) != 2 or geoms[0] == geoms[1]
                or any(not isinstance(s, str) or not s for s in geoms + bodies)):
            raise ValueError(f'event {index}: expected distinct named geoms and two bodies')
        position = np.asarray(event['position_m'], dtype=float)
        normal = np.asarray(event['normal'], dtype=float)
        distance = float(event['distance_m'])
        if (position.shape != (3,) or normal.shape != (3,) or not np.isfinite(distance)
                or not np.isfinite(position).all() or not np.isfinite(normal).all()
                or np.linalg.norm(normal) == 0):
            raise ValueError(f'event {index}: invalid contact geometry')
        if geoms[0] > geoms[1]:
            geoms, bodies, normal = geoms[::-1], bodies[::-1], -normal
        groups.setdefault(geoms, []).append(dict(event_index=index, bodies=bodies,
            distance_m=distance, position_m=position, normal=normal / np.linalg.norm(normal)))
    return groups


def _errors(reference, query):
    # atan2 preserves tiny angles that acos(dot) can round to zero or inflate.
    normal_deg = math.degrees(math.atan2(float(np.linalg.norm(np.cross(
        reference['normal'], query['normal']))),
        float(np.dot(reference['normal'], query['normal']))))
    return dict(distance_m=abs(query['distance_m'] - reference['distance_m']),
                position_m=float(np.linalg.norm(query['position_m'] - reference['position_m'])),
                normal_deg=normal_deg)


def _match(reference, query, errors, tolerances):
    within = {key: errors[key] <= tolerances[key] for key in tolerances}
    body_match = reference['bodies'] == query['bodies']
    return dict(reference_event_index=reference['event_index'],
                query_event_index=query['event_index'], **errors,
                reference_bodies=list(reference['bodies']), query_bodies=list(query['bodies']),
                bodies_match=body_match, within_tolerance=within,
                status='PASS' if body_match and all(within.values()) else 'FAIL')


def _assignment(cost, ambiguity_rtol, ambiguity_atol):
    rows, columns = linear_sum_assignment(cost)
    best = float(cost[rows, columns].sum())
    alternative, alternative_indices = math.inf, None
    # Every distinct assignment excludes at least one selected edge. Solving
    # with each selected edge forbidden therefore finds the true second best,
    # including rectangular assignments where an unmatched contact can change.
    if cost.shape != (1, 1):
        for row, column in zip(rows, columns):
            forbidden = cost.copy()
            forbidden[row, column] = math.inf
            try:
                other_rows, other_columns = linear_sum_assignment(forbidden)
            except ValueError:  # no alternative feasible assignment
                continue
            total = float(forbidden[other_rows, other_columns].sum())
            if total < alternative:
                alternative = total
                alternative_indices = list(zip(other_rows.tolist(), other_columns.tolist()))
    threshold = ambiguity_atol + ambiguity_rtol * max(abs(best), abs(alternative))
    ambiguous = math.isfinite(alternative) and alternative - best <= threshold
    evidence = dict(best_cost=best, alternative_cost=alternative if math.isfinite(alternative) else None,
                    ambiguity_rtol=ambiguity_rtol, ambiguity_atol=ambiguity_atol,
                    ambiguous=ambiguous, alternative_indices=alternative_indices)
    return list(zip(rows.tolist(), columns.tolist())), evidence


def compare_geometry(reference_events, query_events, tolerances=None, *, cost_scales=None,
                     ambiguity_rtol=1e-6, ambiguity_atol=1e-9):
    """Return JSON-serializable correspondence and measurement evidence.

    ``tolerances`` defaults to the frozen Gate 0 limits. ``cost_scales`` only
    changes correspondence costs, never measurement acceptance. Missing pairs
    and count changes are FAIL; ambiguous correspondence is UNKNOWN. A definite
    count/measurement FAIL takes precedence in the aggregate. Each pair also
    exposes correspondence_status independently for A/B analyses.
    """
    tolerances, scales = _limits(tolerances), _limits(cost_scales)
    if any(not math.isfinite(x) or x < 0 for x in (ambiguity_rtol, ambiguity_atol)):
        raise ValueError('ambiguity limits must be finite and nonnegative')
    reference, query = _canonical(reference_events), _canonical(query_events)
    pairs = []
    for geoms in sorted(reference.keys() | query.keys()):
        left, right = reference.get(geoms, []), query.get(geoms, [])
        n, m = len(left), len(right)
        presence = ('ADDED_PAIR' if not n else 'MISSING_PAIR' if not m else
                    'COUNT_CHANGED' if n != m else 'SAME_COUNT')
        pair = dict(geoms=list(geoms), reference_count=n, query_count=m,
                    presence_status=presence, count_status='PASS' if n == m else 'FAIL',
                    correspondence_status='PASS', matches=[], best_candidate_matches=[],
                    missing_reference_event_indices=[], extra_query_event_indices=[])
        if n and m:
            errors = [[_errors(a, b) for b in right] for a in left]
            cost = np.array([[math.sqrt(sum((e[k] / scales[k]) ** 2 for k in scales))
                              for e in row] for row in errors])
            indices, evidence = _assignment(cost, ambiguity_rtol, ambiguity_atol)
            evidence['alternative_event_pairs'] = (None if evidence['alternative_indices'] is None else
                [[left[i]['event_index'], right[j]['event_index']]
                 for i, j in evidence['alternative_indices']])
            pair['assignment'] = evidence
            matches = [_match(left[i], right[j], errors[i][j], tolerances) for i, j in indices]
            if evidence['ambiguous']:
                pair['correspondence_status'] = 'UNKNOWN'
                pair['best_candidate_matches'] = matches
                # Counts remain known, but specific unmatched events are not
                # established when more than one correspondence is plausible.
                pair['missing_reference_event_indices'] = None
                pair['extra_query_event_indices'] = None
            else:
                pair['matches'] = matches
                used_left, used_right = {i for i, _ in indices}, {j for _, j in indices}
                pair['missing_reference_event_indices'] = [a['event_index'] for i, a in enumerate(left)
                                                           if i not in used_left]
                pair['extra_query_event_indices'] = [b['event_index'] for j, b in enumerate(right)
                                                     if j not in used_right]
        else:
            pair['missing_reference_event_indices'] = [a['event_index'] for a in left]
            pair['extra_query_event_indices'] = [b['event_index'] for b in right]
        pair['measurement_status'] = ('UNKNOWN' if pair['correspondence_status'] == 'UNKNOWN'
                                      else 'FAIL' if any(x['status'] == 'FAIL' for x in pair['matches'])
                                      else 'PASS')
        pair['status'] = ('FAIL' if pair['count_status'] == 'FAIL' or pair['measurement_status'] == 'FAIL'
                          else pair['correspondence_status'])
        pairs.append(pair)
    statuses = {pair['status'] for pair in pairs}
    return dict(status='FAIL' if 'FAIL' in statuses else 'UNKNOWN' if 'UNKNOWN' in statuses else 'PASS',
                tolerances=tolerances, cost_scales=scales,
                normal_orientation='lexically first geom -> second geom', pairs=pairs)
