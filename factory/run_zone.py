"""
Zone 하나(가공 Zone)를 시뮬레이션하고 결과를 출력한다.

                 ┌─ A ─▶ [Cell1: CNC×2 → INSPECT]          ─┐
    Generator ─▶ [Router]                                   ├─▶ Sink
                 └─ B ─▶ [Cell3: CNC×2 → WASH → INSPECT]   ─┘
"""
import argparse

from pypdevs.DEVS import CoupledDEVS
from pypdevs.simulator import Simulator

import station
from station import Job, Machine
from zone import Zone, Router
from run_station import JobSource, PoissonGenerator, Sink


class ZoneExperiment(CoupledDEVS):
    def __init__(self, source, zone):
        """source: 작업을 job_out 포트로 내보내는 생성기 (JobSource 또는 PoissonGenerator)"""
        CoupledDEVS.__init__(self, "Experiment")
        self.source = self.addSubModel(source)
        self.zone = self.addSubModel(zone)
        self.sink = self.addSubModel(Sink())
        self.connectPorts(self.source.job_out, self.zone.job_in)
        self.connectPorts(self.zone.job_out, self.sink.job_in)


def report(model):
    finished = model.sink.state
    makespan = max(t for _, t in finished)
    for cell in model.zone.cells:
        for station in cell.stations():
            for comp in station.components():
                comp.closeStats(makespan)

    print("== 작업별 결과")
    print("작업          투입    반출    체류시간")
    for job, t in finished:
        print("%-12s %5.1f  %6.1f  %8.1f" % (job, job.release, t, t - job.release))
    print("완료 %d개, makespan %.1f\n" % (len(finished), makespan))

    print("== 품목별 평균 체류시간")
    for product in sorted(set(job.product for job, _ in finished)):
        flow = [t - job.release for job, t in finished if job.product == product]
        print("%s: %d개, 평균 %.1f" % (product, len(flow), sum(flow) / len(flow)))

    print("\n== Station별 기계 상태 비율 (%)")
    print("%-16s" % "Station" + "".join("%11s" % p for p in Machine.PHASES) + "   처리")
    for cell in model.zone.cells:
        for station in cell.stations():
            m = station.machine.state
            row = "".join("%11.1f" % (100 * m.time_in.get(p, 0.0) / makespan) for p in Machine.PHASES)
            print("%-16s%s   %4d" % ("%s.%s" % (cell.name, station.name), row, m.done_jobs))

    print("\n== 배분기 최대 대기열")
    for cell in model.zone.cells:
        for stage in cell.stages:
            print("%s.%-20s %d개" % (cell.name, stage.dispatcher.name, stage.dispatcher.state.max_queue))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Zone 시뮬레이션")
    parser.add_argument("--debug", action="store_true", help="부품별 전이/출력 과정을 시간순으로 출력")
    args = parser.parse_args()
    station.DEBUG = args.debug

    cnc = {"operation": "CNC", "load_time": 0.5, "unload_time": 0.5, "setup_time": 2.0,
           "mtbf": 20.0, "mttr": 4.0, "out_capacity": 2, "discharge_time": 1.0}
    wash = {"operation": "WASH", "load_time": 0.5, "unload_time": 0.5, "setup_time": 0.0,
            "mtbf": None, "mttr": 0.0, "out_capacity": 2, "discharge_time": 1.0}
    inspect = {"operation": "INSPECT", "load_time": 0.2, "unload_time": 0.2, "setup_time": 0.0,
               "mtbf": None, "mttr": 0.0, "out_capacity": 3, "discharge_time": 0.5}

    # 가공 Zone: A는 Cell1, B는 Cell3
    zone = Zone("Zone1",
                Router({"A": 0, "B": 1}, n_outputs=2),
                [("Cell1", [("CNC", cnc, 2), ("INSPECT", inspect, 1)]),
                 ("Cell3", [("CNC", cnc, 2), ("WASH", wash, 1), ("INSPECT", inspect, 1)])])

    process_times = {
        "A": {"CNC": 6.0, "INSPECT": 1.5},
        "B": {"CNC": 8.0, "WASH": 1.0, "INSPECT": 2.0},
    }

    # 고정 스케줄로 투입하려면 (A와 B를 번갈아 1분 간격으로 6개씩):
    # products = ["A", "B"] * 6
    # jobs = [Job("Job%d" % (i + 1), p, process_times[p], release=1.0 * i)
    #         for i, p in enumerate(products)]
    # source = JobSource(jobs)

    # 포아송 도착: 평균 1.0분마다 1개 (rate = 1.0), A와 B 반반
    source = PoissonGenerator(rate=1.0,
                              product_mix={"A": 0.5, "B": 0.5},
                              process_times=process_times,
                              max_jobs=12,
                              seed=42)

    model = ZoneExperiment(source, zone)
    sim = Simulator(model)
    sim.setTerminationTime(10000)
    sim.simulate()
    report(model)
