from PyKDL import Vector
from utils import dist, dist_from_line

def collision_risk(psm1, state1, psm2, state2, as_list=False):
    if state1.target is None or state2.target is None:
        return [False] if as_list else False

    # Extract positions and targets
    pos1, tgt1 = psm1.measured_cp().p, Vector(*state1.target)
    pos2, tgt2 = psm2.measured_cp().p, Vector(*state2.target)
     
    nearby = lambda p1, p2: dist(p1, p2, xy=True) < 0.03
    on_route = lambda P, linepoints: dist_from_line(P, *linepoints) < 0.01

    conditions = [
        any([
            #target1[1] < target2[1],
            nearby(tgt1, tgt2),
            nearby(pos1, tgt2),
            nearby(pos2, tgt1),
            on_route(pos1, (pos2, tgt2)),
            on_route(pos2, (pos1, tgt1)),
        ]),
        any([
            "Move" in state1.s.name,
            "Move" in state2.s.name
        ])
    ]
    return conditions if as_list else all(conditions)


