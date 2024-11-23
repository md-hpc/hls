from hls import *
from common import *
from numpy.linalg import norm

VERBOSE = False
   
class ParticleFilter(Logic):
    def __init__(self,ident):
        super().__init__(f"particle-filter-{ident}")

        self.reference = Input(self,"reference")
        self.neighbor = Input(self,"neighbor")

        self.o = Output(self,"o")

        self.pipeline(FILTER_PIPELINE_STAGES)

        self.input_set = None
        self.input_expect = None

    def logic(self):
        reference = self.reference.get()
        neighbor = self.neighbor.get()
        
        if reference is NULL or neighbor is NULL:
            self.o.set(NULL)
            return

        pi = pair_ident(reference, neighbor)
        if pi in self.input_set:
            print(f"Filter bank recieved duplicate pair from origin {pi_to_p(pi)}")
            exit()
        self.input_set.add(pi)
        if pi not in self.input_expect:
            print(f"Filter bank recieved particle from unexpected origin {pi_to_p(pi)}")
            exit()
        self.input_expect.remove(pi)
         

        if not n3l(reference.r, neighbor.r):
            self.o.set(NULL)
            return
        if reference == neighbor:
            self.o.set(NULL)
            return
        
        r = norm(modr(reference.r,neighbor.r))
        if r == 0:
            print(f"{self.name} received duplicate position: {reference} and {neighbor}")
            exit()
        
        if r >= CUTOFF:
            self.o.set(NULL)
            return

        self.o.set(
                [reference, neighbor] if r < CUTOFF else NULL
        )


class PairQueue(Logic):
    def __init__(self,i):
        super().__init__(f"pair-queue-{i}")
        self.i = [Input(self, f"i{i}") for i in range(N_FILTER)]
        self.o = Output(self, "o")
        self._queue = []
        self.qempty = Output(self, "qempty")
    
    def logic(self):
        for i in [i.get() for i in self.i]:
            if i is not NULL:
                self._queue.append(i)
        self.qempty.set(len(self._queue) == 0)
        if len(self._queue) != 0:
            self.o.set(self._queue.pop(0))
        else:
            self.o.set(NULL)

class ForcePipeline(Logic):
    def __init__(self, i):
        super().__init__(f"force-pipeline-{i}")
        
        self.i = Input(self,"i")
        self.o = Output(self,"o")
     
        self.pipeline(FORCE_PIPELINE_STAGES)
        
        self.input_set = None
        self.input_expect = None

    def logic(self):
        i = self.i.get()
        if i is NULL:
            self.o.set(NULL)
            return

        reference, neighbor = i
        
        pi = pair_ident(reference,neighbor)
        if pi in self.input_set:
            print(f"duplicate: {reference}, {neighbor}")
            exit(1)
        pi2 = pair_ident(neighbor, reference)
        self.input_set.add(pi)
        self.input_expect.remove(pi)
        self.input_expect.remove(pi2)

        v = lj(reference.r, neighbor.r) * DT

        self.o.set([
            Velocity(cell = reference.cell, addr = reference.addr, v = v),
            Velocity(cell = neighbor.cell, addr = neighbor.addr, v = -1*v),
        ])

class PipelineReader(Logic):
    def __init__(self,i):
        super().__init__(f"pipeline-reader-{i}")
        self.i = Input(self, "i")
        self._reference = None
        self._queue = []
        self.o = Output(self, "o")

        self.almost_done = Input(self, "almost-done")
        self.done = Output(self, "done")

    def logic(self):
        i = self.i.get()

        almost_done = self.almost_done.get()

        if self._reference is not None and almost_done:
            self._queue.append(self._reference)
            self._reference = None
        
        if i is not NULL:
            reference, neighbor = i
            if self._reference is None:
                self._reference = reference
            elif reference.addr != self._reference.addr or reference.cell != self._reference.cell:
                self._queue.append(self._reference)
                self._reference = reference
            else:
                self._reference.v += reference.v
            self._queue.append(neighbor)

        self.done.set(len(self._queue) == 0 and almost_done)

        if len(self._queue) == 0:
            self.o.set(NULL)
        else:
            acceleration = self._queue.pop(0)
            self.o.set(acceleration)

class ComputePipeline:
    def __init__(self, ident, read_controller, m):
        self._reference = m.add(Register(f"reference-{ident}"))
        self.filter_bank = [m.add(ParticleFilter(f"{ident}-{j}")) for j in range(N_FILTER)]
        self.filters_empty = m.add(And(N_FILTER,f"filters-empty-{ident}"))
        self.pair_queue = m.add(PairQueue(ident))
        self.force_pipeline = m.add(ForcePipeline(ident))

        almost_done_signals = [
            read_controller.done,
            self.filters_empty.o,
            self.pair_queue.empty,
            self.force_pipeline.empty
        ]
        self.almost_done = m.add(And(len(almost_done_signals),f"almost-done-{ident}"))
        self.pipeline_reader = m.add(PipelineReader(ident))
        
        # filter_bank inputs
        self.i = [f.neighbor for f in self.filter_bank]
        for f in self.filter_bank:
            connect(self._reference.o, f.reference)
        self.reference = self._reference.i

        # filters_empty inputs
        for f, i in zip(self.filter_bank, self.filters_empty.i):
            connect(f.empty, i)

        # pair_queue inputs
        for f, i in zip(self.filter_bank, self.pair_queue.i):
            connect(f.o, i)

        # force_pipeline inputs
        connect(self.pair_queue.o, self.force_pipeline.i)

        # almost_done inputs
        for signal, i in zip(almost_done_signals, self.almost_done.i):
            connect(signal, i)

        # pipeline_reader inputs
        connect(self.force_pipeline.o, self.pipeline_reader.i)
        connect(self.almost_done.o, self.pipeline_reader.almost_done)

        # external inputs
        self.o = self.pipeline_reader.o
        self.done = self.pipeline_reader.done



