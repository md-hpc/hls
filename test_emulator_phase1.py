import sys
sys.argv.append('-t')
from hls import *
import hls # so we can set CONFIG_VERBOSE
from test_structures_phase1 import *
import phase1

hls.CONFIG_VERBOSE = False

input_set = set()

for cidx, reference_cache in enumerate(p_caches):
    for r in reference_cache.contents:
        if r is NULL:
            break
        for nidx in neighborhood(cidx):
            neighbor_cache = p_caches[nidx]
            for n in neighbor_cache.contents:
                if n is NULL:
                    break
                input_set.add(pident(r,n))
                
class ControlUnit(Logic):
    def __init__(self):
        super().__init__("control-unit")
        self.double_buffer = Output(self, "ctl-double-buffer")
        self.force_evaluation_ready = Output(self, "ctl-force-evaluation-ready")
        self.force_evaluation_done = Input(self, "ctl-force-evaluation-done")

        self.filter_empty = [Input(self, f"filter-empty-{i}") for i in range(N_FILTER)]

        self.ready = 1
 
        self.verbose = False

    def logic(self):
        done = self.force_evaluation_done.get()
        filter_empty = all([i.get() is NULL for i in self.filter_empty])
         
        if done == 1 and filter_empty:
            self.ready = 0

        self.double_buffer.set(0)
        self.force_evaluation_ready.set(self.ready)

force_evaluation_done = m.add(Register("force-evaluation-done"))
filter_empty = [m.add(Register(f"filter-empty-{i}")) for i in range(N_FILTER)]
control_unit = m.add(ControlUnit())

connect(control_unit.double_buffer, phase1.CTL_DOUBLE_BUFFER)
connect(control_unit.force_evaluation_ready, phase1.CTL_FORCE_EVALUATION_READY)

connect(phase1.CTL_FORCE_EVALUATION_DONE, force_evaluation_done.i)
force_evaluation_done.contents = 0

connect(force_evaluation_done.o, control_unit.force_evaluation_done)
for f, reg, i in zip(filter_bank, filter_empty, control_unit.filter_empty):
    connect(f.empty, reg.i)
    connect(reg.o, i)

t = 0
while control_unit.ready == 1:
    print("======== Next Timestep ========")
    m.clock()
print(len(force_pipeline.input_set))
