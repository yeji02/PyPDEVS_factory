"""
Cell: 공정 단계 목록을 받아 Station들을 직렬·병렬로 연결한다.

    stages = [("CNC", cnc_params, 3), ("INSPECT", inspect_params, 1)] 이면

                             ┌─▶ [CNC_1] ─┐
    job_in ─▶ [CNC_Dispatcher] ─▶ [CNC_2] ─┼─▶ [INSPECT_Dispatcher] ─▶ [INSPECT] ─▶ job_out
                             └─▶ [CNC_3] ─┘

각 단계 앞의 Dispatcher는 FIFO 공용 대기열을 가진다. 각 Station에는 max_wip개까지만
작업을 보내고(기본 1개 = 기계에 있는 작업만), 기계에서 작업이 빠지면(unloaded) 다음 작업을 보낸다.
"""
from pypdevs.DEVS import AtomicDEVS, CoupledDEVS

from station import Station, INF


class DispatcherState:
    def __init__(self, n_targets):
        self.queue = []                  # 배정을 기다리는 작업 (FIFO)
        self.load = [0] * n_targets      # Station별로 보냈지만 아직 기계에서 빠지지 않은 작업 수
        self.outbox = {}
        self.max_queue = 0

    def __str__(self):
        return "queue=%d load=%s" % (len(self.queue), self.load)


class Dispatcher(AtomicDEVS):
    """먼저 온 작업을, 여유가 있는 Station 중 번호가 가장 앞선 곳에 보낸다"""
    def __init__(self, n_targets, max_wip=1, name="Dispatcher"):
        AtomicDEVS.__init__(self, name)
        self.max_wip = max_wip
        self.state = DispatcherState(n_targets)

        self.job_in = self.addInPort("job_in")
        self.unloaded_in = [self.addInPort("unloaded_in_%d" % k) for k in range(n_targets)]
        self.job_out = [self.addOutPort("job_out_%d" % k) for k in range(n_targets)]

    def extTransition(self, inputs):
        s = self.state
        s.queue.extend(inputs.get(self.job_in, []))
        for k, port in enumerate(self.unloaded_in):
            s.load[k] -= len(inputs.get(port, []))

        while s.queue:
            free = [k for k, n in enumerate(s.load) if n < self.max_wip]
            if not free:
                break
            k = free[0]
            s.outbox.setdefault(self.job_out[k], []).append(s.queue.pop(0))
            s.load[k] += 1
        s.max_queue = max(s.max_queue, len(s.queue))
        return s

    def timeAdvance(self):
        return 0.0 if self.state.outbox else INF

    def outputFnc(self):
        return self.state.outbox

    def intTransition(self):
        self.state.outbox = {}
        return self.state


class Stage:
    """공정 단계 하나: 배분기 1개와 같은 일을 하는 Station n대"""
    def __init__(self, name, dispatcher, stations):
        self.name = name
        self.dispatcher = dispatcher
        self.stations = stations


class Cell(CoupledDEVS):
    """
    공정 단계들을 순서대로 이은 Cell

    :param name: Cell 이름
    :param stages: (단계 이름, Station 파라미터, 대수) 목록. 작업은 앞 단계부터 차례로 거친다.
                   Station 파라미터는 station.Station 참고 ("operation"이 작업의 공정 시간 키)
    """
    def __init__(self, name, stages):
        CoupledDEVS.__init__(self, name)
        self.job_in = self.addInPort("job_in")
        self.job_out = self.addOutPort("job_out")

        self.stages = []
        for stage_name, params, count in stages: #stage 종류, 개수 따라 각 station 생성
            dispatcher = self.addSubModel(Dispatcher(count, name="%s_Dispatcher" % stage_name))
            names = ["%s_%d" % (stage_name, k + 1) for k in range(count)] if count > 1 else [stage_name]
            stations = [self.addSubModel(Station(n, params)) for n in names]
            for k, station in enumerate(stations):
                self.connectPorts(dispatcher.job_out[k], station.job_in)
                self.connectPorts(station.unloaded_out, dispatcher.unloaded_in[k])
            self.stages.append(Stage(stage_name, dispatcher, stations))

        # 단계 연결: Cell 입구 → 첫 단계, 각 단계의 Station들 → 다음 단계, 마지막 단계 → Cell 출구
        self.connectPorts(self.job_in, self.stages[0].dispatcher.job_in)
        for prev, nxt in zip(self.stages, self.stages[1:]):
            for station in prev.stations:
                self.connectPorts(station.job_out, nxt.dispatcher.job_in)
        for station in self.stages[-1].stations:
            self.connectPorts(station.job_out, self.job_out)

    def stations(self):
        return [st for stage in self.stages for st in stage.stations]
