"""Task 8 worked example: left-hand rule vs DFS + path reduction vs shortest-path replay.
Run: python3 maze_demo.py        (prints every step used in docs/08_maze_algorithms.md)
Maze: junction graph with a loop and an END stub INSIDE the loop (see maze_example.png).

Hardware Context:
- Tasks 1-8 target the Toshiba TB6612FNG motor driver + 2S 7.4V LiPo + 6V N20 motors.
- Task 9 uses the Appendix A hardware configuration (L298N + 3S 11.1V LiPo).
"""
import heapq
from typing import Dict, List, Set, Tuple

# Node coordinates (x right, y up) -- also used for heading / turn computation
POS: Dict[str, Tuple[int, int]] = {
    "S": (5, 0), "A": (3, 0), "B": (1, 0), "H": (0, 0),
    "C": (3, 2), "D": (1, 2), "N": (2, 2), "E": (4, 2), "G": (2, 1),
}
EDGES: List[Tuple[str, str]] = [
    ("S", "A"), ("A", "B"), ("A", "C"), ("B", "D"), ("D", "N"),
    ("N", "C"), ("N", "G"), ("B", "H"), ("C", "E"),
]
START, END = "S", "G"

ADJ: Dict[str, List[str]] = {n: [] for n in POS}
for a, b in EDGES:
    ADJ[a].append(b)
    ADJ[b].append(a)


def dist(u: str, v: str) -> int:
    return abs(POS[u][0] - POS[v][0]) + abs(POS[u][1] - POS[v][1])


ORDER = "NESW"


def heading(u: str, v: str) -> str:
    dx, dy = POS[v][0] - POS[u][0], POS[v][1] - POS[u][1]
    step = ((dx > 0) - (dx < 0), (dy > 0) - (dy < 0))
    return {(0, 1): "N", (1, 0): "E", (0, -1): "S", (-1, 0): "W"}[step]


def turn(h_from: str, h_to: str) -> str:
    d = (ORDER.index(h_to) - ORDER.index(h_from)) % 4
    return {0: "S", 1: "R", 2: "B", 3: "L"}[d]  # Straight / Right / Back / Left


def options(node: str, h: str) -> Dict[str, str]:
    """Neighbours of `node` keyed by relative turn ('L', 'S', 'R', 'B')."""
    return {turn(h, heading(node, n)): n for n in ADJ[node]}


# ---------------------------------------------------------------- left-hand rule
def left_hand(max_steps: int = 60) -> Tuple[List[str], bool]:
    node, h = "A", heading(START, "A")
    trace = [START, "A"]  # First segment: S -> A
    visited_states: Set[Tuple[str, str]] = set()

    for _ in range(max_steps):
        if node == END:
            return trace, True
        st = (node, h)
        if st in visited_states:
            return trace, False  # Re-entered exact same (node, heading) => infinite loop
        visited_states.add(st)

        opt = options(node, h)
        chosen_turn = None
        for t in "LSRB":  # Priority: Left > Straight > Right > Back
            if t in opt:
                chosen_turn = t
                break
        assert chosen_turn is not None
        nxt = opt[chosen_turn]
        h = heading(node, nxt)
        node = nxt
        trace.append(node)

    return trace, False


# ------------------------------------------------------------------ DFS + recording
def dfs_explore() -> Tuple[List[str], List[Tuple[str, str, str, str]], List[str], Set[Tuple[str, str]]]:
    """DFS over junction nodes with a visited set.
    Also records all sensed edges at visited junctions so Dijkstra can use the learned map.
    """
    visited: Set[str] = {START, "A"}
    path_stack: List[str] = [START, "A"]
    turns: List[str] = []
    log: List[Tuple[str, str, str, str]] = []
    discovered_edges: Set[Tuple[str, str]] = {(min(START, "A"), max(START, "A"))}

    node, h = "A", heading(START, "A")

    def go(frm: str, to: str, why: str) -> None:
        nonlocal h
        hn = heading(frm, to)
        t = turn(h, hn)
        turns.append(t)
        log.append((frm, t, to, why))
        h = hn

    while node != END:
        opt = options(node, h)
        # Record all branches visible from this junction into the learned map
        for nbr in opt.values():
            discovered_edges.add((min(node, nbr), max(node, nbr)))

        nxt = None
        for t in "LSR":  # Exploration priority: Left > Straight > Right
            if t in opt and opt[t] not in visited:
                nxt = opt[t]
                break

        if nxt is not None:  # Descend into unvisited neighbour
            go(node, nxt, "explore")
            visited.add(nxt)
            path_stack.append(nxt)
            node = nxt
        else:  # Dead end or all neighbours already visited: backtrack
            if len(path_stack) <= 1:
                raise RuntimeError("Maze is disconnected: END is unreachable")
            path_stack.pop()
            back = path_stack[-1]
            go(node, back, "backtrack" if len(opt) > 1 else "dead end")
            node = back

    return turns, log, path_stack, discovered_edges


# -------------------------------------------------------------- path reduction
# Note: The course README table lists "L+B+R -> S", which is a typo; the true Pololu 3pi rule is
# L+B+R -> B because turn angles add modulo 360 deg (L=270, B=180, R=90 => 270+180+90 = 540 == 180 => B).
ANG = {"S": 0, "R": 90, "B": 180, "L": 270}
INV = {v: k for k, v in ANG.items()}


def reduce_turns(seq: List[str]) -> Tuple[List[str], List[str]]:
    """Eliminate dead-end U-turns ('B') by replacing any interior (a, 'B', c) triple with its net turn."""
    seq = list(seq)
    steps: List[str] = []
    while True:
        # Find the first interior 'B' (index 1 .. len(seq)-2) so a leading 'B' never blocks reduction
        b_idx = next((i for i in range(1, len(seq) - 1) if seq[i] == "B"), None)
        if b_idx is None:
            break
        a, b, c = seq[b_idx - 1], seq[b_idx], seq[b_idx + 1]
        r = INV[(ANG[a] + ANG[b] + ANG[c]) % 360]
        steps.append(f"{a}{b}{c} -> {r}")
        seq[b_idx - 1 : b_idx + 2] = [r]
    return seq, steps


# ---------------------------------------------------------- shortest path (Dijkstra) + replay
def dijkstra(src: str, dst: str, allowed_edges: Set[Tuple[str, str]] = None) -> Tuple[List[str], int]:
    pq = [(0, src)]
    best = {src: 0}
    prev: Dict[str, str] = {}
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            break
        if d > best[u]:
            continue
        for v in ADJ[u]:
            if allowed_edges is not None and (min(u, v), max(u, v)) not in allowed_edges:
                continue
            nd = d + dist(u, v)
            if nd < best.get(v, 10**9):
                best[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    path, u = [dst], dst
    while u != src:
        u = prev[u]
        path.append(u)
    return path[::-1], best[dst]


def turns_along(path: List[str]) -> List[str]:
    h = heading(path[0], path[1])
    out: List[str] = []
    for i in range(1, len(path) - 1):
        hn = heading(path[i], path[i + 1])
        out.append(turn(h, hn))
        h = hn
    return out


if __name__ == "__main__":
    print("1) LEFT-HAND RULE")
    tr, ok = left_hand()
    print("   visit order :", " ".join(tr[:24]), "..." if len(tr) > 24 else "")
    print("   reached END :", ok, "(stuck in a cycle)" if not ok else "")

    print("\n2) DFS exploration (priority L > S > R, visited set)")
    turns, log, stack, learned_edges = dfs_explore()
    for frm, t, to, why in log:
        print("   at %s turn %s -> %s   [%s]" % (frm, t, to, why))
    print("   raw turn string     :", "".join(turns))
    stack_cost = sum(dist(a, b) for a, b in zip(stack, stack[1:]))
    print("   DFS stack (route found) :", "->".join(stack), " (%d length units)" % stack_cost)
    red, steps = reduce_turns(turns)
    print("   reduction steps     :", "; ".join(steps))
    print("   reduced turn string :", "".join(red), " (dead ends removed, loop NOT removed)")

    print("\n3) Optimal replay from the stored map (Dijkstra on discovered edge lengths)")
    path, cost = dijkstra(START, END, learned_edges)
    print("   shortest path       :", "->".join(path), " cost =", cost)
    print("   turn string at junctions A..N:", "".join(turns_along(path)))

    # Self-checks
    assert not ok, "Left-hand rule should get stuck in the outer loop on this maze"
    assert "".join(turns) == "SSBLRSSBSL"
    assert "".join(red) == "SRRR"
    assert path == ["S", "A", "C", "N", "G"] and cost == 6
    assert "".join(turns_along(path)) == "RLL"
    assert reduce_turns(["B", "L", "B", "R"])[0] == ["B", "B"]  # Verifies leading-'B' fix
    print("\nall Task 8 maze checks passed")
