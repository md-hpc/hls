from numpy.linalg import norm
import numpy
import sys

from hls import *
from common import *
from compute_pipeline import ComputePipeline

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
        self.done.set(True)
        self.oaddr.set(NULL)
        self.cell_r.set(NULL)
        self.new_reference.set(NULL)
        
    def logic(self):
        ready = self.ready.get() 

        if ready and self._cell_r >= N_CELL:
            print("waiting for pipelines and queues to flush")
            self.halt()
            return
        
        if not ready:
            # we only reset ourselves once the control unit has acknowledged that we're done
            self._next_timestep = True
            self._cell_r = 0
            self._addr_r = 0
            self.halt()
            return

        ctl_double_buffer = self.ctl_double_buffer.get()
        stale_reference = self.stale_reference.get()
        addr_offset = 0 if not ctl_double_buffer else DBSIZE

        if self._next_timestep:
            stale_reference = True
            self._new_reference = 0
            self._addr_r = addr_offset

        if self._new_reference is not NULL:
            # if we loaded a new reference last cycle
            if self._next_timestep:
                # this is the first cycle of a new timestep
                self._next_timestep = False
            else:
                if stale_reference == False:
                    self._stale_reference = False
                self._new_reference += 1

            if self._new_reference == N_PPAR:
                # all references have been loaded
                if self._stale_reference:
                    # all references read were NULL, 
                    self._cell_r += N_CPAR
                    if self._cell_r >= N_CELL:
                        # we've iterated over all cells. Terminate timestep
                        print("done")
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
                    self._addr_n = addr_offset
                    addr = self._addr_n
            else:
                addr = self._addr_r + self._new_reference

        elif stale_reference:
            # if we only read NULL neighbors last cycle
            self._new_reference = 0
            self._addr_n = addr_offset
            self._addr_r += N_PPAR
            addr = self._addr_r
            self._stale_reference = True
        else:
            # we've still got neighbors to go
            self._addr_n += 1
            addr = self._addr_n


        if self._new_reference is NULL:
            state = "reading neighbors"
        else:
            state = "loading references"
        print(f"{state}, cell_r=={self._cell_r} addr=={addr} addr_r=={self._addr_r} addr_n=={self._addr_n}")
        
        

        self.new_reference.set(self._new_reference)
        self.cell_r.set(self._cell_r)
        self.oaddr.set(addr)
        self.done.set(False)

class PositionReader(Logic):
    def __init__(self):
        super().__init__("position-reader")
        self.i = [Input(self, f"i{i}") for i in range(N_CELL)]
        self.o = [
            [Output(self, f"o{j}-{i}") for i in range(N_FILTER)]
            for j in range(N_CPAR)
        ]

        self.references = [
            [Output(self, f"reference{j}-{i}") for i in range(N_PPAR)]
            for j in range(N_CPAR)
        ]

        self.cell_r = Input(self, "cell-r")
        self.addr = Input(self, "addr")
        self.new_reference = Input(self, "new-reference")
        self.stale_reference = Output(self, "stale-reference")


    def logic(self):
        cell_r = self.cell_r.get()
        new_reference = self.new_reference.get()
        addr = self.addr.get()

        if cell_r is NULL:
            [nul(o) for o in self.o]
            [nul(r) for r in self.references]
            self.stale_reference.set(NULL)
            return

        if new_reference is not NULL:
            _stale_reference = True
            for bidx in range(N_CPAR):
                if cell_r + bidx >= N_CELL:
                    # our bank window is past the last cell
                    _reference = RESET
                else:
                    _reference = self.i[cell_r + bidx].get()
                
                if type(_reference) is not numpy.ndarray:
                    _reference = RESET
                else:
                    _stale_reference = False # at least one reference still needs to be processed
                    _reference = Position(cell = cell_r + bidx, addr = addr, r=_reference)
                
                for pidx in range(N_PPAR):
                    if pidx == new_reference:
                        self.references[bidx][pidx].set(_reference)
                    else:
                        self.references[bidx][pidx].set(NULL)
        
            self.stale_reference.set(_stale_reference)
            [nul(outputs) for outputs in (self.o)]

        else:
            # read the next set of neighbor particles
            _stale_reference = True 
            for bidx, o_bank in enumerate(self.o):
                cell_b = cell_r + bidx
                if cell_b >= N_CELL:
                    nul(o_bank)
                    continue
                
                for cell_n, o in zip(neighborhood(cell_b), o_bank):
                    i = self.i[cell_n].get()
                    if i is NULL:
                        o.set(NULL)
                    else:
                        _stale_reference = False
                        o.set(Position(cell=cell_n, addr=addr, r=i))
                
            [nul(references) for references in self.references]
            self.stale_reference.set(_stale_reference)

class VelocityUpdateController(Logic):
    def __init__(self):
        super().__init__("velocity-update-controller")

        self.i = [
            [Input(self,f"i{j}-{i}") for i in range(N_PPAR)]
            for j in range(N_CPAR)
        ]
        self._queues = [[] for _ in range(N_CELL)]
        self.o = [Output(self,f"o{i}") for i in range(N_CELL)]
        self.oaddr = [Output(self,f"oaddr-{i}") for i in range(N_CELL)]
        self.qempty = Output(self,"qempty")


    def logic(self):
        for inputs in self.i:
            for v in [i.get() for i in inputs]:
                if v is NULL:
                    continue
                self._queues[v.cell].append(v)
            
        _qempty = True
        for q, oaddr, o in zip(self._queues, self.oaddr, self.o):
            if len(q) == 0:
                oaddr.set(NULL)
                o.set(NULL)
            else:
                _qempty = False
                v = q.pop(0)
                oaddr.set(v.addr)
                o.set(v)

        self.qempty.set(_qempty)

class VelocityUpdater(Logic):
    def __init__(self):
        super().__init__("velocity-updater")

        self.fragments = [Input(self,f"fragment{i}") for i in range(N_CELL)]
        self.vi = [Input(self,f"vi{i}") for i in range(N_CELL)]
        self.vo = [Output(self,f"vo{i}") for i in range(N_CELL)]

    def logic(self):
        for fragment, vi, vo in zip(self.fragments, self.vi, self.vo):
            _fragment = fragment.get() 
            _vi = vi.get()

            if _fragment is NULL:
                vo.set(NULL)
                continue

            if _vi is NULL:
                _vo = _fragment.v
            else:
                _vo = _fragment.v + _vi
            vo.set(_vo)
        
position_read_controller = m.add(PositionReadController())
position_reader = m.add(PositionReader())

compute_pipelines = [
    [ComputePipeline(f"{bidx}-{pidx}", position_read_controller, m) for pidx in range(N_PPAR)]
    for bidx in range(N_CPAR)
]

velocity_update_controller = m.add(VelocityUpdateController())
velocity_updater = m.add(VelocityUpdater())

# each compute pipeline plus the acceleration_update_controller
phase1_signaler = m.add(And(N_CPAR*N_PPAR+1,"phase1-signaler"))

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
for bidx in range(N_CPAR):
    for pidx in range(N_PPAR):
        outputs = position_reader.o[bidx]
        reference = position_reader.references[bidx][pidx]
        cp = compute_pipelines[bidx][pidx]
        for o, i in zip(outputs, cp.i):
            connect(o,i)
        connect(reference, cp.reference)
        
# ==== ACCELERATION UPDATE ====
# acceleration_update_controller inputs
for bidx in range(N_CPAR):
    for pidx in range(N_PPAR):
        connect(compute_pipelines[bidx][pidx].o, velocity_update_controller.i[bidx][pidx])

# acceleration_updater inputs
for o, i in zip(velocity_update_controller.o, velocity_updater.fragments):
    connect(o, i)
for v_cache, i in zip(v_caches, velocity_updater.vi):
    connect(v_cache.o, i)

# phase1_signaler inputs
for bidx in range(N_CPAR):
    for pidx in range(N_PPAR):
        connect(compute_pipelines[bidx][pidx].done, phase1_signaler.i[bidx*N_PPAR+pidx])
connect(velocity_update_controller.qempty, phase1_signaler.i[-1])

# ==== STATE ====
# stale_reference input
connect(position_reader.stale_reference, stale_reference.i)

# p_muxes inputs
for imux, omux in zip(p_imuxes, p_omuxes):
    connect(position_read_controller.oaddr, omux.oaddr_phase1)
    connect(null_const.o, imux.iaddr_phase1)
    connect(null_const.o, imux.i_phase1)

# v_muxes inputs
for i in range(N_CELL):
    imux, omux = v_imuxes[i], v_omuxes[i]
    connect(velocity_update_controller.oaddr[i], omux.oaddr_phase1)
    connect(velocity_update_controller.oaddr[i], imux.iaddr_phase1)
    connect(velocity_updater.vo[i], imux.i_phase1)

# ==== CONTROL ====
CTL_DONE = phase1_signaler.o

compute_pipelines = [cp for cp in concat(*compute_pipelines)]

# catch 
o = None
