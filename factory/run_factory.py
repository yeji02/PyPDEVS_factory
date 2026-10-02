"""
혼합형 공장: 가공 Zone(병렬 Cell 2개) → AGV → 조립 Zone(키팅 + 조립 Cell)

          Zone1 (가공)                                         Zone2 (조립)
          ┌─ A ─▶ [Cell1: CNC×2 → INSPECT]          ─┐
    작업 ─▶ [Router]                                   ├─▶ [AGV] ─▶ [Kitting A+B] ─▶ [Cell2: ASSEMBLY → INSPECT] ─▶ 완성품
          └─ B ─▶ [Cell3: CNC×2 → WASH → INSPECT]   ─┘
"""
import argparse

from pypdevs.DEVS import CoupledDEVS
from pypdevs.simulator import Simulator

import station
from station import Job, Machine
from zone import Zone, Router, Kitting
from transport import AGVPool
from run_station import JobSource, PoissonGenerator, Sink


class Factory(CoupledDEVS):
    def __init__(self, source, zone1, agv, zone2):
        """source: 작업을 job_out 포트로 내보내는 생성기 (JobSource 또는 PoissonGenerator)"""
        CoupledDEVS.__init__(self, "Factory")
        self.source = self.addSubModel(source)
        self.zone1 = self.addSubModel(zone1)
        self.agv = self.addSubModel(agv)
        self.zone2 = self.addSubModel(zone2)
        self.sink = self.addSubModel(Sink())

        self.connectPorts(self.source.job_out, self.zone1.job_in)
        self.connectPorts(self.zone1.job_out, self.agv.job_in)
        self.connectPorts(self.agv.job_out, self.zone2.job_in)
        self.connectPorts(self.zone2.job_out, self.sink.job_in)

    def zones(self):
        return [self.zone1, self.zone2]


def report(model):
    finished = model.sink.state
    makespan = max(t for _, t in finished)
    for zone in model.zones():
        for cell in zone.cells:
            for station in cell.stations():
                for comp in station.components():
                    comp.closeStats(makespan)

    print("== 완성품")
    print("완성품        부품           첫 부품 투입   완성   리드타임")
    for kit, t in finished:
        print("%-12s %-14s %10.1f  %6.1f  %8.1f" % (
            kit, "+".join(kit.parts), kit.release, t, t - kit.release))
    lead = [t - kit.release for kit, t in finished]
    print("완성 %d개, makespan %.1f, 평균 리드타임 %.1f\n" % (len(finished), makespan, sum(lead) / len(lead)))

    print("== Station별 기계 상태 비율 (%)")
    print("%-22s" % "Station" + "".join("%11s" % p for p in Machine.PHASES) + "   처리")
    for zone in model.zones():
        for cell in zone.cells:
            for station in cell.stations():
                m = station.machine.state
                row = "".join("%11.1f" % (100 * m.time_in.get(p, 0.0) / makespan) for p in Machine.PHASES)
                print("%-22s%s   %4d" % ("%s.%s.%s" % (zone.name, cell.name, station.name), row, m.done_jobs))

    agv = model.agv
    print("\n== AGV %d대: 이송 %d회, 가동률 %.1f%%, 최대 대기 %d개" % (
        agv.n_vehicles, agv.state.moved,
        100 * agv.state.busy_time / (agv.n_vehicles * makespan), agv.state.max_queue))

    kitting = model.zone2.entry.state
    print("== 키팅: 키트 %d개, 짝을 기다린 최대 부품 수 %s" % (kitting.kits_made, kitting.max_waiting))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="공장 전체 시뮬레이션")
    parser.add_argument("--debug", action="store_true", help="부품별 전이/출력 과정을 시간순으로 출력")
    args = parser.parse_args()
    station.DEBUG = args.debug

    cnc = {"operation": "CNC", "load_time": 0.5, "unload_time": 0.5, "setup_time": 2.0,
           "mtbf": 20.0, "mttr": 4.0, "out_capacity": 2, "discharge_time": 1.0}
    wash = {"operation": "WASH", "load_time": 0.5, "unload_time": 0.5, "setup_time": 0.0,
            "mtbf": None, "mttr": 0.0, "out_capacity": 2, "discharge_time": 1.0}
    inspect = {"operation": "INSPECT", "load_time": 0.2, "unload_time": 0.2, "setup_time": 0.0,
               "mtbf": None, "mttr": 0.0, "out_capacity": 3, "discharge_time": 0.5}
    assembly = {"operation": "ASSEMBLY", "load_time": 0.5, "unload_time": 0.5, "setup_time": 0.0,
                "mtbf": 30.0, "mttr": 5.0, "out_capacity": 2, "discharge_time": 1.0}

    # Zone1 (가공): A는 Cell1, B는 Cell3
    zone1 = Zone("Zone1",
                 Router({"A": 0, "B": 1}, n_outputs=2),
                 [("Cell1", [("CNC", cnc, 2), ("INSPECT", inspect, 1)]),
                  ("Cell3", [("CNC", cnc, 2), ("WASH", wash, 1), ("INSPECT", inspect, 1)])])

    # Zone 사이 이송: AGV 2대, 편도 3분
    agv = AGVPool(n_vehicles=2, travel_time=3.0)

    # Zone2 (조립): A 1개 + B 1개 → 키트 AB → 조립 → 검사
    zone2 = Zone("Zone2",
                 Kitting({"A": 1, "B": 1}, "AB", {"ASSEMBLY": 4.0, "INSPECT": 2.0}),
                 [("Cell2", [("ASSEMBLY", assembly, 1), ("INSPECT", inspect, 1)])])

    process_times = {
        "A": {"CNC": 6.0, "INSPECT": 1.5},
        "B": {"CNC": 8.0, "WASH": 1.0, "INSPECT": 2.0},
    }

    # 고정 스케줄로 투입하려면 (A와 B를 번갈아 3분 간격으로 6개씩):
    # products = ["A", "B"] * 6
    # jobs = [Job("Job%d" % (i + 1), p, process_times[p], release=3.0 * i)
    #         for i, p in enumerate(products)]
    # source = JobSource(jobs)

    # 포아송 도착: 평균 3.0분마다 1개 (rate = 1/3), A와 B 반반
    # 품목이 랜덤이라 A와 B 개수가 다를 수 있고, 짝이 없는 부품은 키팅에서 대기한 채로 끝난다
    source = PoissonGenerator(rate=1 / 3.0,
                              product_mix={"A": 0.5, "B": 0.5},
                              process_times=process_times,
                              max_jobs=12,
                              seed=42)

    model = Factory(source, zone1, agv, zone2)
    sim = Simulator(model)
    sim.setTerminationTime(10000)
    sim.simulate()
    report(model)
