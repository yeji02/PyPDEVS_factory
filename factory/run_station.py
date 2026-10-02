"""
가공 Station 하나를 시뮬레이션하고 상태별 시간과 작업별 결과를 출력한다.

    JobSource ─▶ Station(InputBuffer → Robot → Machine → OutputBuffer) ─▶ Sink
"""
import argparse
import random

from pypdevs.DEVS import AtomicDEVS, CoupledDEVS
from pypdevs.simulator import Simulator

import station
from station import Station, Job, Machine, INF


class JobSource(AtomicDEVS):
    """각 작업을 release 시각에 Station으로 투입한다"""
    def __init__(self, jobs):
        AtomicDEVS.__init__(self, "JobSource")
        self.state = {"jobs": sorted(jobs, key=lambda j: j.release), "clock": 0.0}
        self.job_out = self.addOutPort("job_out")

    def timeAdvance(self):
        jobs = self.state["jobs"]
        return jobs[0].release - self.state["clock"] if jobs else INF

    def outputFnc(self):
        jobs = self.state["jobs"]
        return {self.job_out: [j for j in jobs if j.release == jobs[0].release]}

    def intTransition(self):
        jobs = self.state["jobs"]
        now = jobs[0].release
        self.state = {"jobs": [j for j in jobs if j.release != now], "clock": now}
        return self.state


class PoissonGenerator(AtomicDEVS):
    """
    도착률 rate(단위 시간당 평균 작업 수)의 포아송 과정으로 작업을 만든다.
    도착 간격 ~ Exp(rate), 평균 1/rate. 품목은 product_mix 비율로 고른다.

        product_mix    {"A": 0.6, "B": 0.4}
        process_times  {"A": {"CNC": 3.0}, "B": {"CNC": 4.0}}
        max_jobs       생성할 최대 작업 수 (None이면 제한 없음)
        end_time       이 시각 이후로는 생성하지 않음
        seed           난수 시드 (같은 값이면 같은 도착 패턴)
    """
    def __init__(self, rate, product_mix, process_times, max_jobs=None, end_time=INF, seed=None):
        AtomicDEVS.__init__(self, "PoissonGenerator")
        self.rate = rate
        self.products = list(product_mix)
        self.weights = [product_mix[p] for p in self.products]
        self.process_times = process_times
        self.max_jobs = INF if max_jobs is None else max_jobs
        self.end_time = end_time
        self.rng = random.Random(seed)

        # gap/product는 다음에 도착할 작업을 미리 뽑아 둔 값 (outputFnc는 상태를 바꾸면 안 되므로)
        self.state = {"clock": 0.0, "count": 0, "gap": 0.0, "product": None}
        self.sampleNext()
        self.job_out = self.addOutPort("job_out")

    def sampleNext(self):
        self.state["gap"] = self.rng.expovariate(self.rate)
        self.state["product"] = self.rng.choices(self.products, self.weights)[0]

    def timeAdvance(self):
        s = self.state
        if s["count"] >= self.max_jobs or s["clock"] + s["gap"] > self.end_time:
            return INF
        return s["gap"]

    def outputFnc(self):
        s = self.state
        p = s["product"]
        job = Job("Job%d" % (s["count"] + 1), p, self.process_times[p], release=s["clock"] + s["gap"])
        return {self.job_out: [job]}

    def intTransition(self):
        s = self.state
        s["clock"] += s["gap"]
        s["count"] += 1
        self.sampleNext()
        return s


class Sink(AtomicDEVS):
    """Station을 빠져나온 작업과 그 시각을 기록한다"""
    def __init__(self):
        AtomicDEVS.__init__(self, "Sink")
        self.state = []
        self.job_in = self.addInPort("job_in")

    def extTransition(self, inputs):
        now = self.time_last[0] + self.elapsed
        for job in inputs[self.job_in]:
            self.state.append((job, now))
        return self.state

    def timeAdvance(self):
        return INF


class StationExperiment(CoupledDEVS):
    def __init__(self, source, params):
        """source: 작업을 job_out 포트로 내보내는 생성기 (JobSource 또는 PoissonGenerator)"""
        CoupledDEVS.__init__(self, "Experiment")
        self.source = self.addSubModel(source)
        self.station = self.addSubModel(Station("CNC_Station", params))
        self.sink = self.addSubModel(Sink())
        self.connectPorts(self.source.job_out, self.station.job_in)
        self.connectPorts(self.station.job_out, self.sink.job_in)


def report(model):
    finished = model.sink.state
    makespan = max(t for _, t in finished)
    for comp in model.station.components():
        comp.closeStats(makespan)

    print("== 작업별 결과")
    print("작업          투입    반출    체류시간")
    for job, t in finished:
        print("%-12s %5.1f  %6.1f  %8.1f" % (job, job.release, t, t - job.release))
    print("완료 %d개, makespan %.1f\n" % (len(finished), makespan))

    m = model.station.machine.state
    print("== 기계 상태별 시간 (고장 %d회)" % m.failures)
    for phase in Machine.PHASES:
        t = m.time_in.get(phase, 0.0)
        print("%-11s %6.1f  %5.1f%%  %s" % (phase, t, 100 * t / makespan, "#" * int(40 * t / makespan)))

    r = model.station.robot.state
    robot_busy = r.time_in.get("LOADING", 0.0) + r.time_in.get("UNLOADING", 0.0)
    print("\n== 로봇 가동률 %.1f%%" % (100 * robot_busy / makespan))
    print("== 버퍼 최대 적재: 입력 %d개, 출력 %d개 (용량 %d)" % (
        model.station.in_buffer.state.max_len,
        model.station.out_buffer.state.max_len,
        model.station.out_buffer.capacity))

    print("\n== 기계 상태 변화 기록 (머문 시간이 0인 단계는 생략)")
    steps = [(t, p) for (t, p), (t_next, _) in zip(m.history, m.history[1:] + [(INF, None)])
             if t_next > t]
    print("  ".join("%.1f:%s" % (t, p) for t, p in steps))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="가공 Station 시뮬레이션")
    parser.add_argument("--debug", action="store_true", help="부품별 전이/출력 과정을 시간순으로 출력")
    args = parser.parse_args()
    station.DEBUG = args.debug

    params = {
        "operation": "CNC",    # 이 Station이 하는 공정
        "load_time": 0.5,      # 로봇이 작업물을 넣는 시간
        "unload_time": 0.5,    # 로봇이 완료품을 빼는 시간
        "setup_time": 2.0,     # 품목이 바뀔 때 셋업 시간
        "mtbf": 12.0,          # 가공 시간 기준 고장 간격
        "mttr": 3.0,           # 수리 시간
        "out_capacity": 2,     # 출력 버퍼 용량
        "discharge_time": 8.0, # 출력 버퍼에서 하나를 반출하는 데 걸리는 시간
    }
    process_time = {"A": 3.0, "B": 4.0}

    # 고정 스케줄로 투입하려면:
    # products = ["A", "A", "B", "B", "A", "B", "B", "A"]
    # jobs = [Job("Job%d" % (i + 1), p, {"CNC": process_time[p]}, release=2.0 * i)
    #         for i, p in enumerate(products)]
    # source = JobSource(jobs)

    # 포아송 도착: 평균 2.0마다 1개 (rate = 0.5)
    source = PoissonGenerator(rate=0.5,
                              product_mix={"A": 0.5, "B": 0.5},
                              process_times={p: {"CNC": t} for p, t in process_time.items()},
                              max_jobs=8,
                              seed=42)

    model = StationExperiment(source, params)
    sim = Simulator(model)
    sim.setTerminationTime(1000)
    sim.simulate()
    report(model)
