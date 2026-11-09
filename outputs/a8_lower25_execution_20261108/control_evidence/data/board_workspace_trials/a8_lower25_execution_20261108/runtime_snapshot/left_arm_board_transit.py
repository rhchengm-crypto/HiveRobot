"""Deterministic bounded joint-space search, checked by the board/body scene.

Targets are geometric IK results. This module contains no stored joint poses,
square labels, offsets, motor interfaces, or learned-controller updates.
"""
import numpy as np


def connect_path(start, goal, limits, edge_is_clear, *, max_extensions=400, seed=7301):
    start, goal, limits = np.asarray(start), np.asarray(goal), np.asarray(limits)
    span = limits[:, 1]-limits[:, 0]
    if not edge_is_clear(start, start) or not edge_is_clear(goal, goal):
        return None
    if edge_is_clear(start, goal):
        return [start.copy(), goal.copy()]
    trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]
    rng = np.random.default_rng(seed)

    def extend(tree, target):
        nodes, parents = tree
        nearest = int(np.argmin([np.linalg.norm((q-target)/span) for q in nodes]))
        delta = target-nodes[nearest]
        q = nodes[nearest]+delta*min(1., .18/max(np.max(np.abs(delta)), 1e-12))
        if not edge_is_clear(nodes[nearest], q):
            return None
        nodes.append(q)
        parents.append(nearest)
        return len(nodes)-1

    def branch(tree, index):
        result = []
        while index >= 0:
            result.append(tree[0][index])
            index = tree[1][index]
        return result[::-1]

    for iteration in range(max_extensions):
        active, other = iteration % 2, 1-iteration % 2
        # Bias toward connecting the trees, with full bound coverage remaining.
        target = (trees[other][0][-1] if iteration % 5 == 0 else
                  rng.uniform(limits[:, 0], limits[:, 1]))
        a = extend(trees[active], target)
        if a is None:
            continue
        goal_point = trees[active][0][a]
        for _ in range(40):
            b = extend(trees[other], goal_point)
            if b is None:
                break
            if np.max(np.abs(trees[other][0][b]-goal_point)) < 1e-10:
                left, right = branch(trees[active], a), branch(trees[other], b)
                route = left+right[-2::-1]
                if active == 1:
                    route = route[::-1]
                # Shortcut only along fully rechecked joint interpolation.
                compact, i = [route[0]], 0
                while i < len(route)-1:
                    j = len(route)-1
                    while j > i+1 and not edge_is_clear(route[i], route[j]):
                        j -= 1
                    compact.append(route[j])
                    i = j
                return compact
    return None
