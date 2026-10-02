"""
Coupled DEVS 구조의 가공 Station (버퍼 + 로봇 + 기계 1대).

    ┌────────────────────────── Station ────────────────────────────┐
    │                                                               │
    │  job_in ─▶ [InputBuffer] ──job──▶ [Robot] ──unload──▶ [OutputBuffer] ─▶ job_out
    │              ▲    │count          │  ▲                  │space
    │              └take┘               │  └──────────────────┘
    │                            cmd    ▼  ▲ done
    │                               [Machine]
    └───────────────────────────────────────────────────────────────┘

부품 사이 신호:
    InputBuffer → Robot   count : 대기 중인 작업 수 (바뀔 때마다)
    Robot → InputBuffer   take  : 작업 하나 달라는 요청
    InputBuffer → Robot   job   : 요청에 대한 작업
    Robot → Machine       cmd   : ("load_start" | "load_done" | "unload_start" | "unload_done", 작업)
    Machine → Robot       done  : 가공이 끝난 작업
    Robot → OutputBuffer  unload: 꺼낸 완료품
    OutputBuffer → Robot  space : 남은 빈자리 수 (바뀔 때마다)
"""
from collections import defaultdict

from pypdevs.DEVS import AtomicDEVS, CoupledDEVS

INF = float("inf")
EPS = 1e-9
DEBUG = False


def fmtValue(v):
    if isinstance(v, tuple):
        return "(%s)" % ", ".join(fmtValue(x) for x in v)
    return str(v)


def fmtPorts(port_values):
    return "  ".join("%s=[%s]" % (port.name, ", ".join(fmtValue(v) for v in values))
                     for port, values in port_values.items())


class Job:
    """작업지시. process_times는 공정별 가공 시간 (예: {"CNC": 6.0, "INSPECT": 1.5})"""
    def __init__(self, name, product, process_times, release=0.0):
        self.name = name
        self.product = product
        self.process_times = process_times
        self.release = release

    def __str__(self):
        return "%s(%s)" % (self.name, self.product)


class ComponentState:
    """모든 부품의 공통 상태: 현재 단계, 다음 내부 이벤트까지 남은 시간, 시계, 단계별 누적 시간"""
    def __init__(self, phase):
        self.phase = phase
        self.remaining = INF
        self.clock = 0.0
        self.time_in = defaultdict(float)
        self.history = [(0.0, phase)]

    def __str__(self):
        return self.phase


class Component(AtomicDEVS):
    """시간 흐름과 단계 변화를 기록하는 공통 부모 클래스"""
    def timeAdvance(self):
        return self.state.remaining

    def passTime(self, dt):
        """dt만큼 시간이 흘렀음을 반영한다 (내부 전이에서는 timeAdvance 값, 외부 전이에서는 elapsed)"""
        self.state.time_in[self.state.phase] += dt
        self.state.clock += dt
        if self.state.remaining != INF:
            self.state.remaining -= dt

    def setPhase(self, phase, duration=INF):
        if phase != self.state.phase:
            self.state.history.append((self.state.clock, phase))
        self.state.phase = phase
        self.state.remaining = duration

    def closeStats(self, end_time):
        """시뮬레이션 종료 시각까지 마지막 단계에 머문 시간을 더한다"""
        self.state.time_in[self.state.phase] += end_time - self.state.clock
        self.state.clock = end_time

    # ---- 디버그 출력 ----
    def summary(self):
        """디버그 줄 끝에 붙일 부품별 상태 요약"""
        return ""

    def log(self, time, kind, msg):
        if DEBUG:
            print("%8.2f  %-12s %-3s %s" % (time, self.name, kind, msg))

    def stateText(self):
        return "-> %s (ta=%g) %s" % (self.state.phase, self.timeAdvance(), self.summary())

    def logExt(self, inputs):
        """외부 전이 끝에 호출: 받은 입력과 전이 후 상태"""
        self.log(self.state.clock, "EXT", "%s  %s" % (fmtPorts(inputs), self.stateText()))

    def logInt(self):
        """내부 전이 끝에 호출: 전이 후 상태"""
        self.log(self.state.clock, "INT", self.stateText())

    def logOut(self, outputs):
        """출력 함수에서 호출: 내보내는 값 (출력 함수는 다음 내부 이벤트 시각에 불린다)"""
        if outputs:
            self.log(self.time_next[0], "OUT", fmtPorts(outputs))
        return outputs


class InputBuffer(Component):
    """도착한 작업을 쌓아 두고, 로봇이 요청하면 하나씩 내준다"""
    def __init__(self, name="InputBuffer"):
        AtomicDEVS.__init__(self, name)
        self.state = ComponentState("WAIT")
        self.state.queue = []
        self.state.outbox = {}
        self.state.max_len = 0

        self.job_in = self.addInPort("job_in") #job 도착
        self.take_in = self.addInPort("take_in") #로봇의 작업 요청
        self.job_out = self.addOutPort("job_out") #작업 1개 전달
        self.count_out = self.addOutPort("count_out") #대기 작업 수

    def extTransition(self, inputs):
        self.passTime(self.elapsed)
        s = self.state
        s.queue.extend(inputs.get(self.job_in, []))
        if inputs.get(self.take_in) and s.queue:
            s.outbox[self.job_out] = [s.queue.pop(0)]
        s.outbox[self.count_out] = [len(s.queue)]
        s.max_len = max(s.max_len, len(s.queue))
        s.remaining = 0.0
        self.logExt(inputs)
        return s

    def outputFnc(self):
        return self.logOut(self.state.outbox)

    def intTransition(self):
        self.passTime(self.state.remaining)
        self.state.outbox = {}
        self.state.remaining = INF
        self.logInt()
        return self.state

    def summary(self):
        return "queue=[%s]" % ", ".join(str(j) for j in self.state.queue)


class Robot(Component):
    """기계에 작업물을 넣고(load) 빼는(unload) 로더. 한 번에 한 가지 일만 한다."""
    def __init__(self, load_time, unload_time, out_capacity, name="Robot"):
        AtomicDEVS.__init__(self, name)
        self.load_time = load_time
        self.unload_time = unload_time

        self.state = ComponentState("IDLE")
        self.state.avail = 0                 # 입력 버퍼의 대기 작업 수
        self.state.out_space = out_capacity  # 출력 버퍼의 빈자리 수
        self.state.machine = "EMPTY"         # 로봇이 아는 기계 상태: EMPTY | BUSY | DONE
        self.state.carrying = None           # 기계에 넣고 있는 작업
        self.state.finished = None           # 기계에서 꺼내야 할 완료품

        self.count_in = self.addInPort("count_in")
        self.job_in = self.addInPort("job_in")
        self.done_in = self.addInPort("done_in")
        self.space_in = self.addInPort("space_in")
        self.take_out = self.addOutPort("take_out")
        self.cmd_out = self.addOutPort("cmd_out")
        self.unload_out = self.addOutPort("unload_out")

    def decide(self):
        """다음에 할 일을 고른다. 완료품 꺼내기가 새 작업 넣기보다 우선이다."""
        s = self.state
        if s.machine == "DONE" and s.out_space > 0:
            self.setPhase("START_UNLOAD", 0.0)
        elif s.machine == "EMPTY" and s.avail > 0:
            self.setPhase("REQUEST", 0.0)
        else:
            self.setPhase("IDLE")

    def extTransition(self, inputs):
        self.passTime(self.elapsed)
        s = self.state
        if self.count_in in inputs: #
            s.avail = inputs[self.count_in][-1]
        if self.space_in in inputs:
            s.out_space = inputs[self.space_in][-1]
        if self.done_in in inputs:
            s.machine = "DONE"
            s.finished = inputs[self.done_in][0]
        if self.job_in in inputs:
            s.carrying = inputs[self.job_in][0]
            self.setPhase("START_LOAD", 0.0)
        elif s.phase == "IDLE":
            self.decide()
        self.logExt(inputs)
        return s

    def outputFnc(self):
        s = self.state
        if s.phase == "REQUEST":
            return self.logOut({self.take_out: [True]})
        if s.phase == "START_LOAD":
            return self.logOut({self.cmd_out: [("load_start", None)]})
        if s.phase == "LOADING":
            return self.logOut({self.cmd_out: [("load_done", s.carrying)]})
        if s.phase == "START_UNLOAD":
            return self.logOut({self.cmd_out: [("unload_start", None)]})
        if s.phase == "UNLOADING":
            return self.logOut({self.cmd_out: [("unload_done", None)],
                                self.unload_out: [s.finished]})
        return {}

    def intTransition(self):
        self.passTime(self.state.remaining)
        s = self.state
        if s.phase == "REQUEST":
            self.setPhase("WAIT_JOB")
        elif s.phase == "START_LOAD":
            self.setPhase("LOADING", self.load_time)
        elif s.phase == "LOADING":
            s.machine = "BUSY"
            s.carrying = None
            self.decide()
        elif s.phase == "START_UNLOAD":
            self.setPhase("UNLOADING", self.unload_time)
        elif s.phase == "UNLOADING":
            s.machine = "EMPTY"
            s.finished = None
            s.out_space -= 1
            self.decide()
        self.logInt()
        return s

    def summary(self):
        s = self.state
        return "machine=%s avail=%d out_space=%d" % (s.machine, s.avail, s.out_space)


class Machine(Component):
    """
    가공 설비. 단계:
        IDLE        작업이 없어 놀고 있음 (starved)
        LOADING     로봇이 작업물을 넣는 중
        SETUP       품목이 바뀌어 셋업 중
        PROCESSING  가공 중
        BLOCKED     가공을 마쳤지만 출력 버퍼가 꽉 차서 꺼내지 못함
        UNLOADING   로봇이 완료품을 꺼내는 중
        DOWN        고장 수리 중 (수리 후 남은 가공을 이어서 함)
    고장은 가공 시간 기준으로 mtbf마다 일어나고 mttr 동안 수리한다.
    """
    PHASES = ["IDLE", "LOADING", "SETUP", "PROCESSING", "BLOCKED", "UNLOADING", "DOWN"]

    def __init__(self, operation, setup_time=0.0, mtbf=None, mttr=0.0, name="Machine"):
        AtomicDEVS.__init__(self, name)
        self.operation = operation           # 이 기계가 하는 공정 (작업의 process_times 키)
        self.setup_time = setup_time
        self.mtbf = INF if mtbf is None else mtbf
        self.mttr = mttr

        self.state = ComponentState("IDLE")
        self.state.job = None
        self.state.proc_left = 0.0           # 남은 가공 시간
        self.state.ttf = self.mtbf           # 다음 고장까지 남은 가공 시간
        self.state.last_product = None
        self.state.done_jobs = 0
        self.state.failures = 0

        self.cmd_in = self.addInPort("cmd_in")
        self.done_out = self.addOutPort("done_out")

    def passTime(self, dt):
        if self.state.phase == "PROCESSING":
            self.state.proc_left -= dt
            self.state.ttf -= dt
        Component.passTime(self, dt)

    def startProcessing(self):
        s = self.state
        s.last_product = s.job.product
        self.setPhase("PROCESSING", min(s.proc_left, s.ttf))

    def extTransition(self, inputs):
        self.passTime(self.elapsed)
        s = self.state
        for cmd, job in inputs[self.cmd_in]:
            if cmd == "load_start":
                self.setPhase("LOADING")
            elif cmd == "load_done":
                s.job = job
                s.proc_left = job.process_times[self.operation]
                if self.setup_time > 0 and job.product != s.last_product:
                    self.setPhase("SETUP", self.setup_time)
                else:
                    self.startProcessing()
            elif cmd == "unload_start":
                self.setPhase("UNLOADING")
            elif cmd == "unload_done":
                s.job = None
                self.setPhase("IDLE")
        self.logExt(inputs)
        return s

    def outputFnc(self):
        s = self.state
        # 가공 완료가 고장보다 먼저(또는 동시)면 완료품을 알린다
        if s.phase == "PROCESSING" and s.proc_left <= s.ttf:
            return self.logOut({self.done_out: [s.job]})
        return {}

    def intTransition(self):
        self.passTime(self.state.remaining)
        s = self.state
        if s.phase == "SETUP":
            self.startProcessing()
        elif s.phase == "PROCESSING":
            if s.proc_left <= EPS:
                s.done_jobs += 1
                self.setPhase("BLOCKED")
            else:
                s.failures += 1
                self.setPhase("DOWN", self.mttr)
        elif s.phase == "DOWN":
            s.ttf = self.mtbf
            self.startProcessing()
        self.logInt()
        return s

    def summary(self):
        s = self.state
        return "job=%s proc_left=%g ttf=%g" % (s.job, s.proc_left, s.ttf)


class OutputBuffer(Component):
    """완료품을 보관하고 discharge_time마다 하나씩 다음 공정으로 내보낸다 (컨베이어/AGV 반출)"""
    def __init__(self, capacity, discharge_time, name="OutputBuffer"):
        AtomicDEVS.__init__(self, name)
        self.capacity = capacity
        self.discharge_time = discharge_time

        self.state = ComponentState("WAIT")
        self.state.queue = []
        self.state.discharge_left = INF
        self.state.notify = False
        self.state.max_len = 0

        self.job_in = self.addInPort("job_in")
        self.job_out = self.addOutPort("job_out")
        self.space_out = self.addOutPort("space_out")

    def timeAdvance(self):
        return 0.0 if self.state.notify else self.state.discharge_left

    def passTime(self, dt):
        if self.state.discharge_left != INF:
            self.state.discharge_left -= dt
        self.state.clock += dt

    def extTransition(self, inputs):
        self.passTime(self.elapsed)
        s = self.state
        for job in inputs[self.job_in]:
            if not s.queue:
                s.discharge_left = self.discharge_time
            s.queue.append(job)
        s.max_len = max(s.max_len, len(s.queue))
        s.notify = True
        self.logExt(inputs)
        return s

    def outputFnc(self):
        s = self.state
        if s.notify:
            return self.logOut({self.space_out: [self.capacity - len(s.queue)]})
        return self.logOut({self.job_out: [s.queue[0]],
                            self.space_out: [self.capacity - len(s.queue) + 1]})

    def intTransition(self):
        s = self.state
        if s.notify:
            s.notify = False
        else:
            self.passTime(s.discharge_left)
            s.queue.pop(0)
            s.discharge_left = self.discharge_time if s.queue else INF
        self.logInt()
        return s

    def summary(self):
        s = self.state
        return "queue=[%s] discharge_left=%g" % (", ".join(str(j) for j in s.queue), s.discharge_left)


class Station(CoupledDEVS):
    """
    입력 버퍼 → 로봇 → 기계 → 로봇 → 출력 버퍼로 이루어진 가공 Station

    포트:
        job_in        작업 투입
        job_out       출력 버퍼에서 반출된 작업
        unloaded_out  기계에서 작업을 꺼낸 순간 그 작업을 알림 (상위 배분기가 빈 기계를 알 수 있도록)
    """
    def __init__(self, name, process_params):
        CoupledDEVS.__init__(self, name)
        p = process_params
        self.job_in = self.addInPort("job_in")
        self.job_out = self.addOutPort("job_out")
        self.unloaded_out = self.addOutPort("unloaded_out")

        self.in_buffer = self.addSubModel(InputBuffer())
        self.robot = self.addSubModel(Robot(p["load_time"], p["unload_time"], p["out_capacity"]))
        self.machine = self.addSubModel(Machine(p["operation"], p["setup_time"], p["mtbf"], p["mttr"]))
        self.out_buffer = self.addSubModel(OutputBuffer(p["out_capacity"], p["discharge_time"]))

        self.connectPorts(self.job_in, self.in_buffer.job_in)
        self.connectPorts(self.in_buffer.count_out, self.robot.count_in)
        self.connectPorts(self.robot.take_out, self.in_buffer.take_in)
        self.connectPorts(self.in_buffer.job_out, self.robot.job_in)
        self.connectPorts(self.robot.cmd_out, self.machine.cmd_in)
        self.connectPorts(self.machine.done_out, self.robot.done_in)
        self.connectPorts(self.robot.unload_out, self.out_buffer.job_in)
        self.connectPorts(self.robot.unload_out, self.unloaded_out)
        self.connectPorts(self.out_buffer.space_out, self.robot.space_in)
        self.connectPorts(self.out_buffer.job_out, self.job_out)

    def components(self):
        return [self.in_buffer, self.robot, self.machine, self.out_buffer]
