"""
Cell 하나를 시뮬레이션하고 결과를 출력한다. 공정 구성은 아래 stages에서 정한다.

    PoissonGenerator(또는 JobSource) ─▶ Cell ─▶ Sink
"""
import argparse

from pypdevs.DEVS import CoupledDEVS
from pypdevs.simulator import Simulator

import station
from station import Job, Machine
from cell import Cell
from run_station import JobSource, PoissonGenerator, Sink


class CellExperiment(CoupledDEVS):
    def __init__(self, source, stages):
        """source: 작업을 job_out 포트로 내보내는 생성기 (JobSource 또는 PoissonGenerator)"""
        CoupledDEVS.__init__(self, "Experiment")
        self.source = self.addSubModel(source)
        self.cell = self.addSubModel(Cell("Cell1", stages))
        self.sink = self.addSubModel(Sink())
        self.connectPorts(self.source.job_out, self.cell.job_in)
        self.connectPorts(self.cell.job_out, self.sink.job_in)


def report(model):
    finished = model.sink.state
    makespan = max(t for _, t in finished)
    for station in model.cell.stations():
        for comp in station.components():
            comp.closeStats(makespan)

    print("== 작업별 결과")
    print("작업          투입    반출    체류시간")
    for job, t in finished:
        print("%-12s %5.1f  %6.1f  %8.1f" % (job, job.release, t, t - job.release))
    flow = [t - job.release for job, t in finished]
    print("완료 %d개, makespan %.1f, 평균 체류 %.1f\n" % (len(finished), makespan, sum(flow) / len(flow)))

    print("== Station별 기계 상태 비율 (%)")
    print("%-9s" % "Station" + "".join("%11s" % p for p in Machine.PHASES) + "   처리  고장")
    for station in model.cell.stations():
        m = station.machine.state
        row = "".join("%11.1f" % (100 * m.time_in.get(p, 0.0) / makespan) for p in Machine.PHASES)
        print("%-9s%s   %4d  %4d" % (station.name, row, m.done_jobs, m.failures))

    print("\n== 버퍼 최대 적재")
    for stage in model.cell.stages:
        print("%-20s 대기열 %d개" % (stage.dispatcher.name, stage.dispatcher.state.max_queue))
    for station in model.cell.stations():
        print("%-9s 입력 %d개, 출력 %d개 (용량 %d)" % (
            station.name, station.in_buffer.state.max_len,
            station.out_buffer.state.max_len, station.out_buffer.capacity))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Cell 시뮬레이션")
    parser.add_argument("--debug", action="store_true", help="부품별 전이/출력 과정을 시간순으로 출력")
    args = parser.parse_args()
    station.DEBUG = args.debug

    cnc_params = {
        "operation": "CNC",
        "load_time": 0.5,
        "unload_time": 0.5,
        "setup_time": 2.0,     # 품목이 바뀔 때 셋업 시간
        "mtbf": 20.0,          # 가공 시간 기준 고장 간격
        "mttr": 4.0,           # 수리 시간
        "out_capacity": 2,
        "discharge_time": 1.0, # 검사로 보내는 컨베이어 이송 시간
    }
    inspect_params = {
        "operation": "INSPECT",
        "load_time": 0.2,
        "unload_time": 0.2,
        "setup_time": 0.0,
        "mtbf": None,          # 고장 없음
        "mttr": 0.0,
        "out_capacity": 3,
        "discharge_time": 0.5, # Cell 밖으로 반출하는 시간
    }
    process_times = {
        "A": {"CNC": 6.0, "INSPECT": 1.5},
        "B": {"CNC": 8.0, "INSPECT": 2.0},
    }

    # 고정 스케줄로 투입하려면:
    # products = ["A", "A", "B", "B", "A", "B", "B", "A", "A", "B", "A", "B"]
    # jobs = [Job("Job%d" % (i + 1), p, process_times[p], release=2.0 * i)
    #         for i, p in enumerate(products)]
    # source = JobSource(jobs)

    # 포아송 도착: 평균 2.0분마다 1개 (rate = 0.5)
    source = PoissonGenerator(rate=0.5,
                              product_mix={"A": 0.5, "B": 0.5},
                              process_times=process_times,
                              max_jobs=12,
                              seed=42)

    # 공정 단계: (단계 이름, Station 파라미터, 대수). 작업은 위에서부터 차례로 거친다.
    stages = [
        ("CNC", cnc_params, 3),
        ("INSPECT", inspect_params, 1),
    ]

    model = CellExperiment(source, stages)
    sim = Simulator(model)
    sim.setTerminationTime(1000)
    sim.simulate()
    report(model)
