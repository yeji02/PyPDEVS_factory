"""
Zone: 입구 모델(Router 또는 Kitting) 하나와 Cell 여러 개를 묶는다.

    job_in ─▶ [입구 모델] ─▶ Cell들 ─▶ job_out

입구 모델:
    Router   품목에 따라 작업을 정해진 Cell로 보낸다 (가공 Zone)
    Kitting  레시피대로 부품이 모이면 조립 키트 하나를 만들어 보낸다 (조립 Zone)
"""
from pypdevs.DEVS import AtomicDEVS, CoupledDEVS

from station import Job, INF
from cell import Cell


class OutboxModel(AtomicDEVS):
    """입력을 받으면 같은 시각에 바로 내보내는(시간 0) 모델의 공통 부모. state.outbox에 담으면 나간다."""
    def timeAdvance(self):
        return 0.0 if self.state.outbox else INF

    def outputFnc(self):
        return self.state.outbox

    def intTransition(self):
        self.state.outbox = {}
        return self.state


class RouterState:
    def __init__(self):
        self.outbox = {}


class Router(OutboxModel):
    """
    품목별로 작업을 보낼 출구를 정한다

    :param routes: {품목: 출구 번호}
    :param n_outputs: 출구 개수
    """
    def __init__(self, routes, n_outputs, name="Router"):
        AtomicDEVS.__init__(self, name)
        self.routes = routes
        self.state = RouterState()
        self.job_in = self.addInPort("job_in")
        self.job_out = [self.addOutPort("job_out_%d" % k) for k in range(n_outputs)]

    def extTransition(self, inputs):
        for job in inputs[self.job_in]:
            port = self.job_out[self.routes[job.product]]
            self.state.outbox.setdefault(port, []).append(job)
        return self.state


class KittingState:
    def __init__(self, recipe):
        self.waiting = {product: [] for product in recipe}   # 품목별로 짝을 기다리는 부품
        self.max_waiting = {product: 0 for product in recipe}
        self.outbox = {}
        self.kits_made = 0


class Kitting(OutboxModel):
    """
    레시피의 부품이 모두 모이면 먼저 온 부품부터 짝지어 키트(새 작업) 하나를 만든다

    :param recipe: {부품 품목: 필요 개수}  예: {"A": 1, "B": 1}
    :param kit_product: 만들어지는 키트의 품목 이름
    :param kit_process_times: 키트의 공정별 가공 시간
    """
    def __init__(self, recipe, kit_product, kit_process_times, name="Kitting"):
        AtomicDEVS.__init__(self, name)
        self.recipe = recipe
        self.kit_product = kit_product
        self.kit_process_times = kit_process_times
        self.state = KittingState(recipe)
        self.job_in = self.addInPort("job_in")
        self.job_out = [self.addOutPort("job_out_0")]

    def extTransition(self, inputs):
        s = self.state
        for job in inputs[self.job_in]:
            s.waiting[job.product].append(job)
        for product in self.recipe:
            s.max_waiting[product] = max(s.max_waiting[product], len(s.waiting[product]))

        while all(len(s.waiting[p]) >= n for p, n in self.recipe.items()):
            parts = []
            for product, n in self.recipe.items():
                parts.extend(s.waiting[product][:n])
                s.waiting[product] = s.waiting[product][n:]
            s.kits_made += 1
            kit = Job("Kit%d" % s.kits_made, self.kit_product, self.kit_process_times,
                      release=min(p.release for p in parts))
            kit.parts = [p.name for p in parts]
            s.outbox.setdefault(self.job_out[0], []).append(kit)
        return s


class Zone(CoupledDEVS):
    """
    입구 모델 하나와 Cell 여러 개로 이루어진 Zone

    :param name: Zone 이름
    :param entry: 입구 모델 (Router 또는 Kitting). 출구 k번이 k번째 Cell로 연결된다.
    :param cells: (Cell 이름, 공정 단계 목록) 목록. 공정 단계 형식은 cell.Cell 참고
    """
    def __init__(self, name, entry, cells):
        CoupledDEVS.__init__(self, name)
        self.job_in = self.addInPort("job_in")
        self.job_out = self.addOutPort("job_out")

        self.entry = self.addSubModel(entry)
        self.cells = [self.addSubModel(Cell(cell_name, stages)) for cell_name, stages in cells]

        self.connectPorts(self.job_in, self.entry.job_in)
        for k, cell in enumerate(self.cells):
            self.connectPorts(self.entry.job_out[k], cell.job_in)
            self.connectPorts(cell.job_out, self.job_out)
