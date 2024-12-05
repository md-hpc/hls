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
         

        if reference.cell == neighbor.cell and not n3l(reference.r, neighbor.r):
            self.o.set(NULL)
            return
        if reference == neighbor:
            self.o.set(NULL)
            return
        
        r = norm(modr(reference.r,neighbor.r))
        if r == 0:
            print(f"{self.name} received duplicate position: {reference} and {neighbor}")
            exit()
        
        if r < CUTOFF:
            self.o.set([reference, neighbor])
        else:
            self.o.set(NULL)


class PairQueue(Logic):
    def __init__(self,i):
        super().__init__(f"pair-queue-{i}")
        self.i = [Input(self, f"i{i}") for i in range(NSIZE)]
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
        pi2 = pair_ident(neighbor, reference)
        p = pi_to_p(pi)
        if pi in self.input_set:
            print(f"duplicate in force pipeline: {p}")
            exit(1)
        if pi not in self.input_expect:
            print(f"unexpected in force pipeline: {p}")
            exit(1)
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
        self._neighbor = None

        self.reference = Output(self,"reference")
        self.neighbor = Output(self,"neighbor")

        self.done = Input(self, "almost-done")

    def logic(self):
        i = self.i.get()

        done = self.done.get()

        if done:
            if self._reference is not None:
                self.reference.set(self._reference)
                self._reference = None
            else:
                self.reference.set(NULL)

            if self._neighbor is not None:
                self.neighbor.set(self._neighbor)
                self._neighbor = None
            else:
                self.neighbor.set(NULL)
            return
        
        nul_n = True
        nul_r = True
        if i is not NULL:
            reference, neighbor = i
            
            if self._reference is None:
                self._reference = reference
            elif reference != self._reference:
                nul_r = False
                self.reference.set(self._reference)
                self._reference = reference
            else:
                self._reference.v += reference.v

            if self._neighbor is None:
                self._neighbor = neighbor
            elif neighbor != self._neighbor:
                nul_n = False
                self.neighbor.set(self._neighbor)
                self._neighbor = neighbor
            else:
                self._neighbor.v += neighbor.v

        if nul_r:
            nul(self.reference)
        if nul_n:
            nul(self.neighbor)

class Noop(Logic):
    def __init__(self, ident):
        super().__init__(ident)
        self.i = Input(self,f"{ident}-i")
        self.o = Output(self,f"{ident}-o")

    def logic(self):
        self.o.set(self.i.get())

class ComputePipeline:
    def __init__(self, ident, read_controller, m):
        self._reference = m.add(Noop(f"reference-{ident}"))
        self._neighbors = [m.add(Register(f"neighbor-{ident}-{i}")) for i in range(NSIZE)]
        self.filter_bank = [m.add(ParticleFilter(f"{ident}-{j}")) for j in range(NSIZE)]
        self.filters_empty = m.add(And(NSIZE,f"filters-empty-{ident}"))
        self.pair_queue = m.add(PairQueue(ident))
        self.force_pipeline = m.add(ForcePipeline(ident))

        done_signals = [
            read_controller.done,
            self.filters_empty.o,
            self.pair_queue.empty,
            self.force_pipeline.empty
        ]
        self.done_AND = m.add(And(len(done_signals),f"almost-done-{ident}"))
        self.pipeline_reader = m.add(PipelineReader(ident))
        
        # inputs
        self.reference = self._reference.i
        self.neighbors = [neighbor.i for neighbor in self._neighbors]

        # outputs
        self.reference_out = self.pipeline_reader.reference
        self.neighbor_out = self.pipeline_reader.neighbor
        self.done = self.done_AND.o

        # filter_bank inputs
        for neighbor, f in zip(self._neighbors, self.filter_bank):
            connect(self._reference.o, f.reference)
            connect(neighbor.o, f.neighbor)
            
        # filters_empty inputs
        for f, i in zip(self.filter_bank, self.filters_empty.i):
            connect(f.empty, i)

        # pair_queue inputs
        for f, i in zip(self.filter_bank, self.pair_queue.i):
            connect(f.o, i)

        # force_pipeline inputs
        connect(self.pair_queue.o, self.force_pipeline.i)

        # done inputs
        for signal, i in zip(done_signals, self.done_AND.i):
            connect(signal, i)

        # pipeline_reader inputs
        connect(self.force_pipeline.o, self.pipeline_reader.i)
        connect(self.done_AND.o, self.pipeline_reader.done)




