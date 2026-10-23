"""
webots_dfs_dijkstra_controller.py  --  Meshmerize: DFS map -> Dijkstra -> "LSR..." replay   (Python, Webots R2025a)

Run:  put this file in a Webots controller folder as <folder>/<folder>.py and select it as the robot controller.

Three layers; each one only talks to the one below through a tiny interface:

  MISSION   (class Mission)     explore everything -> return to (0,0) -> Dijkstra -> replay the LSR string
  MAP       (class MazeMap)     pure Python, no Webots. Coordinates start at (0,0), in metres, any heading,
                                any grid pitch. Nodes are identified by dead-reckoned position; the DFS tree is used
                                for backtracking (so returning never depends on re-recognising a position).
  MOVEMENT  (class WebotsMotion) sensors, PD line following, junction / dead-end / END-box detection, turning.
                                Knows nothing about coordinates. Output = NodeEvent
                                ("drove X m, found a node with exits L/S/R, or a dead end, or the END box").

DFS rules  : at every node prefer LEFT, then STRAIGHT, then RIGHT among exits not yet travelled;
             nothing left -> go back to the DFS parent.
Cycles     : arriving at an already-known node through a NEW edge records that edge and turns straight back
             (each physical edge is driven exactly twice, never more). END is stored in a separate list and is
             treated like a dead end, so finding it never disturbs the traversal.
Replay     : Dijkstra over the measured edge lengths gives the shortest route; it is turned into a string such as
             "LSRLSSRL" (one letter per node event on the route) and executed with the same event detector.

Assumptions: START is the end of a line (as in the Meshmerize drawing); lines meet at 90 degrees; the maze is connected.
The MAP and MISSION layers are tested on a PC (test_logic_random_mazes.py). The MOVEMENT layer can only be tuned in Webots:
see the "TUNE" notes in class Cfg.
"""
import heapq
import math

try:                                    # Webots provides `controller`; the PC test harness does not
    from controller import Robot
    IN_WEBOTS = True
except ImportError:                     # pragma: no cover
    Robot = None
    IN_WEBOTS = False


# =====================================================================================================
#  CONFIG
# =====================================================================================================
class Cfg:
    # ---- devices (names as in your robot) ----
    LEFT_MOTOR, RIGHT_MOTOR = "left_motor", "right_motor"
    LEFT_ENC, RIGHT_ENC = "left_encoder", "right_encoder"
    IR_NAMES = ["ir0", "ir1", "ir2", "ir3", "ir4"]        # ir0 = leftmost corner sensor, ir4 = rightmost
    COMPASS, END_LED = "compass", "end_led"               # compass optional: encoder yaw is used if it is missing

    # ---- geometry (metres) ----
    WHEEL_RADIUS = 0.022
    AXLE_LENGTH = 0.100                                    # TUNE: must equal the real wheel-to-wheel distance if no compass
    SENSOR_OFFSET = 0.050                                  # sensor bar ahead of the axle
    LINE_WIDTH = 0.030
    # outer sensor first touches the branch -> axle stands over the junction centre
    ADV = SENSOR_OFFSET + LINE_WIDTH / 2.0                 # TUNE: watch the first junction, the axle should stop over its centre
    END_MIN_RUN = 0.060                                    # all five sensors white this long = END box (a 30 mm crossing line is shorter)

    # ---- sensor calibration (raw: white ~466, black 1000; recalibrated on the start line) ----
    WHITE_RAW, BLACK_RAW = 466.0, 1000.0

    # ---- thresholds on normalised readings (1000 = white line) ----
    OUTER_ON = 400.0                                       # corner sensors: "I see a branch"
    WHITE_ON = 600.0                                       # all-white test, centre-sensor test
    LOST_SUM = 180.0                                       # sum of 5 below this = no line under the bar
    ARM_RUN = 0.005                                        # corner sensors dark this long before a junction may trigger
    DEAD_END_RUN = 0.010                                   # no line this long = dead end
    MIN_TRAVEL_BEFORE_DEAD_END = 0.020
    TRIG_SAMPLES = 2

    # ---- speeds (wheel rad/s) and PD (your values) ----
    EXPLORE_SPEED, REPLAY_SPEED, ADV_SPEED = 8.0, 11.5, 4.5
    KP, KD = 3.2, 6.5                                      # derivative is per simulation step, as in your code
    KH = 8.0                                               # heading hold across a junction (rad/s per rad)
    ACCEL = 0.6                                            # m/s^2 for the replay speed profile
    MAX_WHEEL = 24.0

    # ---- turning ----
    TURN_MIN, TURN_MAX, TURN_KP = 1.5, 5.0, 4.0
    TURN_TOL = 0.03                                        # rad
    TURN_LINE_SEARCH = 0.35                                # extra rad allowed to find the line after the nominal angle
    TURN_TIMEOUT_S = 6.0

    # ---- map ----
    NODE_TOL = 0.06                                        # m: measured positions closer than this are the same node
    MAX_EVENTS = 4000


# =====================================================================================================
#  Shared small types
# =====================================================================================================
class NodeEvent:
    def __init__(self):
        self.dist = 0.0                                    # metres driven since the previous decision point
        self.left = self.straight = self.right = False
        self.dead_end = self.end_box = self.aborted = False


def rel_turn(h_from, h_to):
    """heading index -> 'S','R','B','L'  (index grows clockwise)."""
    return "SRBL"[(h_to - h_from) % 4]


# =====================================================================================================
#  LAYER 2: MAP (pure logic)
# =====================================================================================================
class _Node:
    __slots__ = ("x", "y", "exits", "nbr", "length")

    def __init__(self, x, y):
        self.x, self.y = x, y
        self.exits = set()                                 # absolute directions where a line leaves this node
        self.nbr = [-1, -1, -1, -1]                        # -1 unexplored, -2 being travelled, >=0 neighbour node id
        self.length = [0.0] * 4


class MazeMap:
    DX = (0, 1, 0, -1)                                     # heading 0 = start direction (+y), 1 = right, 2 = back, 3 = left
    DY = (1, 0, -1, 0)

    def __init__(self, tol=Cfg.NODE_TOL):
        self.tol = tol
        self.reset()

    def reset(self):
        self.nodes = [_Node(0.0, 0.0)]                     # start = (0,0)
        self.nodes[0].exits.add(0)
        self.nodes[0].nbr[0] = -2
        self.parent = [-1]
        self.stack = [0]
        self.cur, self.heading = 0, 0
        self.returning, self.expected = False, None
        self.end_nodes = []                                # END stored separately from the traversal
        self.log = []

    # ---- helpers ----
    def _find(self, x, y):
        best, bd = None, 1e18
        for i, n in enumerate(self.nodes):
            dx, dy = n.x - x, n.y - y
            if abs(dx) <= self.tol and abs(dy) <= self.tol and dx * dx + dy * dy < bd:
                best, bd = i, dx * dx + dy * dy
        return best

    def _dir_to(self, a, b):
        for d in range(4):
            if self.nodes[a].nbr[d] == b:
                return d
        return -1

    # ---- DFS step: the robot reached a node ----
    def on_arrive(self, dist, left, straight, right, is_end):
        """Returns 'L','S','R','B' (turn to execute), 'DONE' (whole maze explored, back at start) or 'FAIL'."""
        frm, h = self.cur, self.heading
        back = (h + 2) % 4
        seen = {back}
        if left:
            seen.add((h + 3) % 4)
        if straight:
            seen.add(h)
        if right:
            seen.add((h + 1) % 4)

        new = False
        if self.returning and self.expected is not None:   # backtracking: identity known from the DFS tree
            m = self.expected
        else:                                              # exploring: identify by dead-reckoned position
            x = self.nodes[frm].x + self.DX[h] * dist
            y = self.nodes[frm].y + self.DY[h] * dist
            m = self._find(x, y)
            if m is None:
                m = len(self.nodes)
                self.nodes.append(_Node(x, y))
                self.parent.append(frm)
                new = True
        node = self.nodes[m]
        node.exits |= seen
        if self.nodes[frm].nbr[h] < 0:
            self.nodes[frm].nbr[h], self.nodes[frm].length[h] = m, dist
        if node.nbr[back] < 0:
            node.nbr[back], node.length[back] = frm, dist

        was_returning = self.returning
        self.returning, self.expected, self.cur = False, None, m

        if is_end:                                         # stored separately; the traversal carries on unchanged
            if m not in self.end_nodes:
                self.end_nodes.append(m)
            self.log.append("E")
        if new:
            self.stack.append(m)
        elif not was_returning:                            # CYCLE: known node reached by a new edge -> go straight back
            self.returning, self.expected, self.heading = True, frm, back
            self.log.append("B")
            return "B"

        for d in ((h + 3) % 4, h, (h + 1) % 4):            # left, straight, right
            if d in node.exits and node.nbr[d] == -1:
                node.nbr[d] = -2
                self.heading = d
                t = rel_turn(h, d)
                self.log.append(t)
                return t

        if self.stack and self.stack[-1] == m:             # nothing left here: back to the DFS parent
            self.stack.pop()
        if not self.stack:
            return "DONE" if self.end_nodes else "FAIL"
        p = self.stack[-1]
        d = self._dir_to(m, p)
        if d < 0:
            return "FAIL"
        self.returning, self.expected, self.heading = True, p, d
        self.log.append(rel_turn(h, d))
        return rel_turn(h, d)

    # ---- Dijkstra over measured edge lengths -> LSR string ----
    def shortest_route(self):
        """Returns (moves, segment_lengths, total_length) or None. moves has one letter per intermediate node."""
        if not self.end_nodes:
            return None
        INF = float("inf")
        dist = [INF] * len(self.nodes)
        prev = [-1] * len(self.nodes)
        dist[0] = 0.0
        pq = [(0.0, 0)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for k in range(4):
                v = self.nodes[u].nbr[k]
                if v >= 0 and d + self.nodes[u].length[k] < dist[v]:
                    dist[v] = d + self.nodes[u].length[k]
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))
        reachable = [e for e in self.end_nodes if dist[e] < INF]
        if not reachable:
            return None
        goal = min(reachable, key=lambda e: dist[e])
        path = [goal]
        while path[-1] != 0:
            path.append(prev[path[-1]])
        path.reverse()
        moves, segs, h = "", [], 0
        for i in range(len(path) - 1):
            d = self._dir_to(path[i], path[i + 1])
            segs.append(self.nodes[path[i]].length[d])
            if i > 0:
                moves += rel_turn(h, d)
            h = d
        return moves, segs, dist[goal], path


# =====================================================================================================
#  LAYER 3: MISSION (talks only to the movement object and MazeMap)
# =====================================================================================================
class Mission:
    """`motion` must provide: move_to_next_node(seg_hint_m, fast)->NodeEvent, turn('L'|'R'|'B'|'S'), stop(), set_led(bool)."""

    def __init__(self, motion, verbose=True):
        self.mo, self.verbose = motion, verbose
        self.map = MazeMap()
        self.moves = ""
        self.cost = None

    def say(self, *a):
        if self.verbose:
            print(*a)

    def run(self):
        r = self.explore()
        return r if r != "OK" else self.replay()

    def explore(self):
        m = self.map
        m.reset()
        led = False
        self.say("=== PHASE 1: DFS exploration from (0,0) (left > straight > right, back when stuck) ===")
        for k in range(Cfg.MAX_EVENTS):
            ev = self.mo.move_to_next_node(-1.0, False)
            if ev.aborted:
                return "ABORTED"
            if ev.end_box and not led:
                self.mo.set_led(True)
                led = True
            act = m.on_arrive(ev.dist, ev.left, ev.straight, ev.right, ev.end_box)
            tag = "END-BOX" if ev.end_box else ("DEAD-END" if ev.dead_end else "")
            x, y = m.nodes[m.cur].x, m.nodes[m.cur].y
            self.say("[%3d] drove %.3f m  exits %s%s%s %-8s node %2d at (%+.3f,%+.3f)  -> %s" % (
                k, ev.dist, "L" if ev.left else "-", "S" if ev.straight else "-", "R" if ev.right else "-", tag, m.cur, x, y, act))
            if act == "DONE":
                self.mo.stop()
                self.mo.set_led(False)
                self.mo.turn(rel_turn(m.heading, 0))        # back at (0,0): face the start direction again
                self.say("=== exploration finished: %d nodes, %d END node(s), back at START ===" % (len(m.nodes), len(m.end_nodes)))
                return "OK"
            if act == "FAIL":
                self.mo.stop()
                return "NO_END_FOUND" if not m.end_nodes else "MAP_FAILED"
            self.mo.turn(act)
        self.mo.stop()
        return "MAP_FAILED"

    def replay(self):
        route = self.map.shortest_route()
        if route is None:
            return "NO_PATH"
        self.moves, segs, self.cost, _ = route
        self.say("\n=== PHASE 2: Dijkstra shortest route %.3f m, commands: %r ===" % (self.cost, self.moves))
        self.say("=== PHASE 3: replay ===")
        for i, seg in enumerate(segs):
            ev = self.mo.move_to_next_node(seg, True)
            if ev.aborted:
                return "ABORTED"
            if i == len(segs) - 1:                          # last segment must end in the END box
                self.mo.stop()
                self.mo.set_led(True)
                self.say(">>> END box reached, LED on, stopped" if ev.end_box else "!!! END box expected but not seen")
                return "OK" if ev.end_box else "REPLAY_MISMATCH"
            if ev.end_box or ev.dead_end:
                self.mo.stop()
                self.say("!!! unexpected event at replay step", i)
                return "REPLAY_MISMATCH"
            self.say("[replay %d/%d] %s" % (i + 1, len(segs) - 1, self.moves[i]))
            self.mo.turn(self.moves[i])
        return "REPLAY_MISMATCH"


# =====================================================================================================
#  LAYER 1: MOVEMENT (Webots only)
# =====================================================================================================
if IN_WEBOTS:

    class WebotsMotion:
        def __init__(self):
            self.robot = Robot()
            self.ts = int(self.robot.getBasicTimeStep())
            dev = self.robot.getDevice
            self.lm, self.rm = dev(Cfg.LEFT_MOTOR), dev(Cfg.RIGHT_MOTOR)
            for m in (self.lm, self.rm):
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
            self.le, self.re = dev(Cfg.LEFT_ENC), dev(Cfg.RIGHT_ENC)
            self.le.enable(self.ts)
            self.re.enable(self.ts)
            self.ir = []
            for n in Cfg.IR_NAMES:
                s = dev(n)
                s.enable(self.ts)
                self.ir.append(s)
            self.led = dev(Cfg.END_LED)
            self.led.set(0)
            try:
                self.compass = dev(Cfg.COMPASS)
            except Exception:
                self.compass = None
            if self.compass:
                self.compass.enable(self.ts)
            self.dead = False
            self.white_raw, self.black_raw = Cfg.WHITE_RAW, Cfg.BLACK_RAW
            self.nv = [0.0] * 5
            self.prev_err = 0.0
            self.last_err = 0.0
            self.step()
            self.step()
            self.calibrate()

        # ---- basics ----
        def step(self):
            if self.robot.step(self.ts) == -1:
                self.dead = True
                return False
            return True

        def alive(self):
            return not self.dead

        def calibrate(self):
            """Robot starts on the line: ir2 on white, ir0/ir4 outside on black."""
            rw = self.ir[2].getValue()
            rb = max(self.ir[0].getValue(), self.ir[4].getValue())
            if rb - rw > 100.0:
                self.white_raw, self.black_raw = rw, rb
            self.read()

        def read(self):
            span = self.black_raw - self.white_raw
            if span < 50.0:
                span = 534.0
            for i in range(5):
                v = (self.black_raw - self.ir[i].getValue()) / span * 1000.0
                self.nv[i] = max(0.0, min(1000.0, v))

        def odo(self):
            return 0.5 * (self.le.getValue() + self.re.getValue()) * Cfg.WHEEL_RADIUS    # turning in place cancels out

        def yaw(self):
            """counter-clockwise positive; only differences are used."""
            if self.compass:
                n = self.compass.getValues()
                return math.atan2(-n[1], n[0])
            return (self.re.getValue() - self.le.getValue()) * Cfg.WHEEL_RADIUS / Cfg.AXLE_LENGTH

        @staticmethod
        def wrap(a):
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            return a

        def drive(self, l, r):
            m = Cfg.MAX_WHEEL
            self.lm.setVelocity(max(-m, min(m, l)))
            self.rm.setVelocity(max(-m, min(m, r)))

        def stop(self):
            self.drive(0.0, 0.0)

        def set_led(self, on):
            self.led.set(1 if on else 0)

        # ---- line following (your PD: the 3 inner sensors steer, the 2 corner sensors are only for junctions) ----
        def follow(self, base):
            nv = self.nv
            num = sum(w * v for w, v in zip((-2, -1, 0, 1, 2), nv))
            den = sum(nv)
            full = num / den if den >= Cfg.LOST_SUM else None
            in_den = nv[1] + nv[2] + nv[3]
            if in_den >= 200.0:
                err = (nv[3] - nv[1]) / in_den
            else:
                err = full if full is not None else self.last_err
            self.last_err = err
            corr = Cfg.KP * err + Cfg.KD * (err - self.prev_err)
            self.prev_err = err
            self.drive(base + corr, base - corr)           # line on the right (err > 0): left wheel faster

        def centred(self):
            """inner sensors see the line and it is within +-1 sensor pitch: a corner-sensor hit is a branch, not our own line edge."""
            nv = self.nv
            s = nv[1] + nv[2] + nv[3]
            return s >= 200.0 and abs((nv[3] - nv[1]) / s) < 1.0

        def speed_for(self, d, seg_hint, fast):
            if not fast:
                return Cfg.EXPLORE_SPEED
            r = Cfg.WHEEL_RADIUS
            vmax, vturn = Cfg.REPLAY_SPEED * r, Cfg.EXPLORE_SPEED * r
            v = min(vmax, math.sqrt(vturn ** 2 + 2 * Cfg.ACCEL * d))                      # ramp up after a turn
            if seg_hint > 0:
                rem = max(0.0, seg_hint - Cfg.ADV - d)
                v = min(v, math.sqrt(vturn ** 2 + 2 * Cfg.ACCEL * rem))                   # brake for the known node
            return v / r

        # ---- layer-1 API ----
        def move_to_next_node(self, seg_hint=-1.0, fast=False):
            ev = NodeEvent()
            o0 = self.odo()
            armed, clear_from, lost_from, trig = False, None, None, 0
            self.prev_err = 0.0
            while self.step():
                self.read()
                d = self.odo() - o0
                nv = self.nv
                oL, oR = nv[0] > Cfg.OUTER_ON, nv[4] > Cfg.OUTER_ON
                lost = sum(nv) < Cfg.LOST_SUM
                if not armed:                               # corner sensors must have been dark for a short run first
                    if not oL and not oR and not lost:
                        clear_from = d if clear_from is None else clear_from
                        armed = (d - clear_from) >= Cfg.ARM_RUN
                    else:
                        clear_from = None
                if lost and d >= Cfg.MIN_TRAVEL_BEFORE_DEAD_END:                         # line ends under the whole bar
                    lost_from = d if lost_from is None else lost_from
                    if d - lost_from >= Cfg.DEAD_END_RUN:
                        self.stop()
                        ev.dead_end, ev.dist = True, d
                        return ev
                elif not lost:
                    lost_from = None
                if armed and (oL or oR) and self.centred():
                    trig += 1
                    if trig >= Cfg.TRIG_SAMPLES:
                        return self._cross(o0)
                else:
                    trig = 0
                self.follow(self.speed_for(d, seg_hint, fast))
            ev.aborted = True
            return ev

        def _cross(self, o0):
            """outer sensor touched a branch -> go straight (heading held) until the axle is over the junction centre, noting exits."""
            ev = NodeEvent()
            t0, yaw0 = self.odo(), self.yaw()
            sL, sR = self.nv[0] > Cfg.OUTER_ON, self.nv[4] > Cfg.OUTER_ON
            white_from = None
            while True:
                d = self.odo() - t0
                all_w = all(v > Cfg.WHITE_ON for v in self.nv)
                white_from = (d if white_from is None else white_from) if all_w else None
                if all_w and d - white_from >= Cfg.END_MIN_RUN:                           # long all-white run = END box
                    self.stop()
                    ev.end_box, ev.dist = True, self.odo() - o0
                    return ev
                if d >= Cfg.ADV and not all_w:
                    break
                e = self.wrap(self.yaw() - yaw0)                                          # >0: drifted CCW -> left wheel faster
                self.drive(Cfg.ADV_SPEED + Cfg.KH * e, Cfg.ADV_SPEED - Cfg.KH * e)
                if not self.step():
                    ev.aborted = True
                    return ev
                self.read()
                sL = sL or self.nv[0] > Cfg.OUTER_ON
                sR = sR or self.nv[4] > Cfg.OUTER_ON
            self.stop()
            ev.left, ev.right = sL, sR
            ev.straight = any(self.nv[i] > Cfg.OUTER_ON for i in (1, 2, 3))
            ev.dist = self.odo() - o0
            return ev

        def turn(self, c):
            """'L' 90 deg CCW, 'R' 90 deg CW, 'B' 180 deg, 'S' nothing. Angle from yaw, then a short search until the line is under the centre sensor."""
            self.prev_err = 0.0
            if c == "S":
                return
            delta = {"L": math.pi / 2, "R": -math.pi / 2, "B": math.pi}[c]
            sign = 1.0 if delta > 0 else -1.0
            total, prev, t0 = 0.0, self.yaw(), self.robot.getTime()
            while self.step():
                y = self.yaw()
                total += self.wrap(y - prev)
                prev = y
                self.read()
                rem = delta - total                                                      # signed angle still to turn
                reached = sign * rem <= Cfg.TURN_TOL
                if reached and (self.nv[2] > Cfg.OUTER_ON or sign * rem < -Cfg.TURN_LINE_SEARCH):
                    break                                                                 # on the line, or searched long enough
                if self.robot.getTime() - t0 > Cfg.TURN_TIMEOUT_S:
                    break
                v = max(Cfg.TURN_MIN, min(Cfg.TURN_MAX, abs(rem) * Cfg.TURN_KP)) if not reached else Cfg.TURN_MIN
                self.drive(-sign * v, sign * v)                                            # keep turning the same way while searching
            self.stop()
            for _ in range(3):
                self.step()
            self.read()
            self.prev_err = 0.0


def main():
    motion = WebotsMotion()
    mission = Mission(motion, verbose=True)
    result = mission.run()
    print("\nMission result:", result, "| route:", mission.moves, "| length: %s m" % (None if mission.cost is None else round(mission.cost, 3)))


if __name__ == "__main__":
    if IN_WEBOTS:
        main()
    else:
        print("Run this file as a Webots controller. For the PC logic test run test_logic_random_mazes.py")
