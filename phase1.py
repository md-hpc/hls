import sys
from hls import *
from structures import *
from numpy.linalg import norm
import numpy
# PHASE 1: position cache read/filter write AND filter read/force evaluator write

# an important note for phase 1: we use periodic boundary conditions in this simulation, meaning that
# if your reference cell is, say, (3, 3, 3) in a 4x4x4 universe (indexing starts at 0), then you will need
# to pair with particles from (3,0,0), (3,3,0), (0,0,3), etc. because they are considered adjacent by our
# model


# we use double buffering (every other phase we use the top half of the caches instead because of
# particle transfers that happen in the motion update phase). This will provide a bit (0 or 1) that will give
# an offset to add to all addresses used. 0 if it reads 0 and 256 if it reads 1
#
# e.g. if you're accessing address 13 of a givne BRAM, and the bit is 0, you will just access address 13
# but if bit is 1, you will access address 13 + 256 = 269 instead.
CTL_DOUBLE_BUFFER = Input 

# Force evaluation control
CTL_READY = Input # reads as 1 when force evaluation should begin/continue, otherwise 0
CTL_DONE = Output # write 1 when your units are done evaluating forces, otherwise write 0

class PositionReadController(Logic):
    def __init__(self):
        super().__init__("position-read-controller")
        
        self.ready = Input(self,"ctl-force-evaluation-ready")
        self.ctl_double_buffer = Input(self, "ctl-dobule-buffer")
        
        self.done = Output(self, "ctl-force-evaluation-done")

        self._next_timestep = True

        self._cell_r = 0
        self._addr_r = 0
        self._addr_n = 0

        # read from register
        self.stale_reference = Input(self, "stale-reference") # if all particles read last cycle were NULL

        self.oaddr = Output(self, "oaddr")
        self.cell_r = Output(self, "cell-r")

        self._new_reference = NULL
        self._stale_reference = True
        self.new_reference = Output(self, "new-reference") # if PositionReader should write a new value to the reference register
        
    def halt(self):
        self._next_timestep = True
        self.done.set(True)
        self.oaddr.set(NULL)
        self.cell_r.set(NULL)
        self.new_reference.set(NULL)
        
    def logic(self):
        ready = self.ready.get() 

        if ready and self._cell_r >= N_CELL:
            self.halt()
            return
        
        if not ready:
            # we only reset ourselves once the control unit has acknowledged that we're done
            self._next_timestep = True
            self._cell_r = 0
            self.halt()
            return

        ctl_double_buffer = self.ctl_double_buffer.get()
        stale_reference = self.stale_reference.get()
        addr_offset = 0 if not ctl_double_buffer else 256

        if self._next_timestep:
            stale_reference = True
            self._new_reference = 0

                        
        if self._new_reference is not NULL:
            # if we loaded a new reference last cycle
            if self._next_timestep:
                # this is the first cycle of a new timestep
                self._next_timestep = False
            else:
                if stale_reference == False:
                    self._stale_reference = False
                self._new_reference += 1

            if self._new_reference == N_PIPELINE:
                # all references have been loaded
                if self._stale_reference:
                    # all references read were NULL, 
                    self._cell_r += 1
                    if self._cell_r == N_CELL:
                        # we've iterated over all cells. Terminate timestep
                        self.halt()
                        return
                    
                    # go to next cell
                    self._new_reference = 0
                    self._addr_r = addr_offset
                    self._stale_reference = True
                    addr = self._addr_r
                else:
                    # some references are not NULL, we can iterate over neighbors
                    self._new_reference = NULL
                    self._stale_reference = False
                    addr = self._addr_n
            else:
                addr = self._addr_r + self._new_reference

        elif stale_reference:
            # if we only read NULL neighbors last cycle
            self._new_reference = 0
            self._addr_n = addr_offset
            self._addr_r += N_PIPELINE
            addr = self._addr_r
            self._stale_reference = True
        else:
            # we've still got neighbors to go
            self._addr_n += 1
            addr = self._addr_n


        if self._new_reference is NULL:
            print(f"neighbor: {self._cell_r} {addr}")
        else:
            print(f"reference: {self._cell_r} {addr}")
        
        

        self.new_reference.set(self._new_reference)
        self.cell_r.set(self._cell_r)
        self.oaddr.set(addr)
        self.done.set(False)

class PositionReader(Logic):
    def __init__(self):
        super().__init__("position-reader")
        self.i = [Input(self, f"i{i}") for i in range(N_CELL)]
        self.o = [Output(self, f"o{i}") for i in range(N_FILTER)]
        
        self.references = [Output(self, f"reference-{i}") for i in range(N_PIPELINE)]

        self.cell_r = Input(self, "cell-r")
        self.addr = Input(self, "addr")
        self.new_reference = Input(self, "new-reference")
        self.stale_reference = Output(self, "stale-reference")


    def logic(self):
        cell_r = self.cell_r.get()
        new_reference = self.new_reference.get()
        addr = self.addr.get()

        if cell_r is NULL:
            [o.set(NULL) for o in self.o]
            [r.set(NULL) for r in self.references]
            self.stale_reference.set(NULL)
            return

        if new_reference is not NULL:
            _reference = self.i[cell_r].get()
            _stale_reference = _reference is NULL
            if _stale_reference:
                _reference = RESET
            else:
                _reference = Position(cell = cell_r, addr = addr, r=_reference)
            for pidx, reference in enumerate(self.references):
                if pidx == new_reference:
                    reference.set(_reference)
                else:
                    reference.set(NULL)
            self.stale_reference.set(_stale_reference)
            for o in self.o:
                o.set(NULL)
            return

        _stale_reference = True 
        for cidx, o in zip(neighborhood(cell_r), self.o):
            i = self.i[cidx].get()
            if i is NULL:
                o.set(NULL)
            else:
                _stale_reference = False
                o.set(Position(cell=cidx, addr=addr, r=i))
            
        for r in self.references:
            r.set(NULL)
        self.stale_reference.set(_stale_reference)

class ParticleFilter(Logic):
    def __init__(self,pidx,i):
        super().__init__(f"particle-filter-{pidx}-{i}")

        self.reference = Input(self,"reference")
        self.neighbor = Input(self,"neighbor")

        self.o = Output(self,"o")

        self.pipeline(FILTER_PIPELINE_STAGES)

    def logic(self):
        reference = self.reference.get()
        neighbor = self.neighbor.get()

        if reference is NULL or neighbor is NULL or reference == neighbor:
            self.o.set(NULL)
            return

        r = norm(reference.r - neighbor.r)
        assert r != 0, f"{reference} {neighbor}"

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
        
    def logic(self):
        i = self.i.get()
        if i is NULL:
            self.o.set(NULL)
            return

        reference, neighbor = i
        
        r = norm(reference.r - neighbor.r)
        f = 4.0*EPSILON*(6.0*SIGMA**6.0/r**7.0-12.0*SIGMA**12/r**13.0)*(reference.r - neighbor.r)/r

        self.o.set([
            Acceleration(cell = reference.cell, addr = reference.addr, a = f),
            Acceleration(cell = neighbor.cell, addr = neighbor.addr, a = -1*f),
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
                self._reference.a += reference.a
            self._queue.append(neighbor)

        self.done.set(len(self._queue) == 0 and almost_done)

        if len(self._queue) == 0:
            self.o.set(NULL)
        else:
            acceleration = self._queue.pop(0)
            self.o.set(acceleration)

class AccelerationUpdateController(Logic):
    def __init__(self):
        super().__init__("acceleration-update-controller")

        self.i = [Input(self,f"i{i}") for i in range(N_PIPELINE)]
        self._queues = [[] for _ in range(N_CELL)]
        self.o = [Output(self,f"o{i}") for i in range(N_CELL)]
        self.oaddr = [Output(self,f"oaddr-{i}") for i in range(N_CELL)]
        self.qempty = Output(self,"qempty")

    def logic(self):
        for a in [i.get() for i in self.i]:
            if a is NULL:
                continue
            self._queues[a.cell].append(a)
        
        _qempty = True
        for q, oaddr, o in zip(self._queues, self.oaddr, self.o):
            if len(q) == 0:
                oaddr.set(NULL)
                o.set(NULL)
            else:
                _qempty = False
                a = q.pop(0)
                oaddr.set(a.addr)
                o.set(a)

        self.qempty.set(_qempty)

class AccelerationUpdater(Logic):
    def __init__(self):
        super().__init__("acceleration-updater")

        self.fragments = [Input(self,f"fragment{i}") for i in range(N_CELL)]
        self.ai = [Input(self,f"ai{i}") for i in range(N_CELL)]
        self.ao = [Output(self,f"ao{i}") for i in range(N_CELL)]

    def logic(self):
        for fragment, ai, ao in zip(self.fragments, self.ai, self.ao):
            _fragment = fragment.get() 
            _ai = ai.get()

            if _fragment is NULL:
                ao.set(NULL)
                continue

            if _ai is NULL:
                _ao = _fragment.a
            else:
                _ao = _fragment.a + _ai
            ao.set(_ao)
        
class ComputePipeline:
    def __init__(self,i, m):
        self._reference = m.add(Register(f"reference-{i}"))
        self.filter_bank = [m.add(ParticleFilter(i,j)) for j in range(N_FILTER)]
        self.filters_empty = m.add(And(N_FILTER,f"filters-empty-{i}"))
        self.pair_queue = m.add(PairQueue(i))
        self.force_pipeline = m.add(ForcePipeline(i))

        almost_done_signals = [
            position_read_controller.done,
            self.filters_empty.o,
            self.pair_queue.empty,
            self.force_pipeline.empty
        ]
        self.almost_done = m.add(And(len(almost_done_signals),f"almost-done-{i}"))
        self.pipeline_reader = m.add(PipelineReader(i))
        
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

position_read_controller = m.add(PositionReadController())
position_reader = m.add(PositionReader())

compute_pipelines = [ComputePipeline(pidx,m) for pidx in range(N_PIPELINE)]

acceleration_update_controller = m.add(AccelerationUpdateController())
acceleration_updater = m.add(AccelerationUpdater())

# each compute pipeline plus the acceleration_update_controller
phase1_signaler = m.add(And(N_PIPELINE+1,"phase1-signaler"))

# is True when
# all neighbor particles read last cycle were NULL
# - indicates that current reference particle is stale
# positionReadController.new_reference was True last cycle and reference particle read last cycle was NULL
# - indicates that current reference cell is stale
stale_reference = m.add(Register("stale-reference")) 

# ==== POSITION READ ====
# position_read_controller inputs
connect(stale_reference.o, position_read_controller.stale_reference)
CTL_DOUBLE_BUFFER = position_read_controller.ctl_double_buffer
CTL_READY = position_read_controller.ready

# position_reader inputs
for p_cache, i in zip(p_caches, position_reader.i):
    connect(p_cache.o, i)
connect(position_read_controller.cell_r, position_reader.cell_r)
connect(position_read_controller.new_reference, position_reader.new_reference)
connect(position_read_controller.oaddr, position_reader.addr)

# ==== FORCE EVALUATION ====
# compute_pipeline inputs
for pidx in range(N_PIPELINE):
    outputs = position_reader.o
    inputs = compute_pipelines[pidx].i
    for o, i in zip(outputs, inputs):
        connect(o,i)
    connect(position_reader.references[pidx], compute_pipelines[pidx].reference) 

# ==== ACCELERATION UPDATE ====
# acceleration_update_controller inputs
for pipeline, i in zip(compute_pipelines, acceleration_update_controller.i):
    connect(pipeline.o, i)

# acceleration_updater inputs
for o, i in zip(acceleration_update_controller.o, acceleration_updater.fragments):
    connect(o, i)
for a_cache, i in zip(a_caches, acceleration_updater.ai):
    connect(a_cache.o, i)

# phase1_signaler inputs
for i, _ in enumerate(compute_pipelines):
    connect(compute_pipelines[i].done, phase1_signaler.i[i])
connect(acceleration_update_controller.qempty, phase1_signaler.i[-1])

# ==== STATE ====
# stale_reference input
connect(position_reader.stale_reference, stale_reference.i)

# p_muxes inputs
for imux, omux in zip(p_imuxes, p_omuxes):
    connect(position_read_controller.oaddr, omux.oaddr_phase1)
    connect(null_const.o, imux.iaddr_phase1)
    connect(null_const.o, imux.i_phase1)

# a_muxes inputs
for i in range(N_CELL):
    imux, omux = a_imuxes[i], a_omuxes[i]
    connect(acceleration_update_controller.oaddr[i], omux.oaddr_phase1)
    connect(acceleration_update_controller.oaddr[i], imux.iaddr_phase1)
    connect(acceleration_updater.ao[i], imux.i_phase1)

# ==== CONTROL ====
CTL_DONE = phase1_signaler.o

# catch 
o = None
