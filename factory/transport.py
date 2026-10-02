"""
AGVPool: AGV n대가 FIFO로 작업을 실어 나른다.

AGV 한 대는 작업 하나를 싣고 travel_time 뒤에 내려 주고, 다시 travel_time 걸려 돌아온다.
모든 AGV가 바쁘면 작업은 대기열에서 기다린다.
"""
from pypdevs.DEVS import AtomicDEVS

from station import INF, EPS


class Vehicle:
    def __init__(self):
        self.phase = "IDLE"      # IDLE | DELIVER | RETURN
        self.remaining = INF
        self.job = None


class AGVState:
    def __init__(self, n_vehicles):
        self.vehicles = [Vehicle() for _ in range(n_vehicles)]
        self.queue = []
        self.outbox = {}
        self.busy_time = 0.0     # AGV별 바쁜 시간의 합
        self.max_queue = 0
        self.moved = 0

    def __str__(self):
        return "queue=%d %s" % (len(self.queue), [v.phase for v in self.vehicles])


class AGVPool(AtomicDEVS):
    def __init__(self, n_vehicles, travel_time, name="AGV"):
        AtomicDEVS.__init__(self, name)
        self.n_vehicles = n_vehicles
        self.travel_time = travel_time
        self.state = AGVState(n_vehicles)
        self.job_in = self.addInPort("job_in")
        self.job_out = self.addOutPort("job_out")

    def passTime(self, dt):
        for v in self.state.vehicles:
            if v.phase != "IDLE":
                v.remaining -= dt
                self.state.busy_time += dt

    def assign(self):
        s = self.state
        for v in s.vehicles:
            if v.phase == "IDLE" and s.queue:
                v.phase, v.remaining, v.job = "DELIVER", self.travel_time, s.queue.pop(0)

    def timeAdvance(self):
        if self.state.outbox:
            return 0.0
        return min([v.remaining for v in self.state.vehicles if v.phase != "IDLE"] or [INF])

    def extTransition(self, inputs):
        self.passTime(self.elapsed)
        s = self.state
        s.queue.extend(inputs[self.job_in])
        self.assign()
        s.max_queue = max(s.max_queue, len(s.queue))
        return s

    def outputFnc(self):
        return self.state.outbox

    def intTransition(self):
        s = self.state
        if s.outbox:
            s.outbox = {}
            return s
        self.passTime(self.timeAdvance())
        for v in s.vehicles:
            if v.phase == "DELIVER" and v.remaining <= EPS:
                s.outbox.setdefault(self.job_out, []).append(v.job)
                s.moved += 1
                v.phase, v.remaining, v.job = "RETURN", self.travel_time, None
            elif v.phase == "RETURN" and v.remaining <= EPS:
                v.phase, v.remaining = "IDLE", INF
        self.assign()
        return s
