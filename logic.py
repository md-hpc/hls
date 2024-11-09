import sys
from hls import *

if "-t" in sys.argv:
    from test_structures_phase1 import *
else:
    from structures import *

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
CTL_FORCE_EVALUATION_READY = Input # reads as 1 when force evaluation should begin/continue, otherwise 0
CTL_FORCE_EVALUATION_DONE = Output # write 1 when your units are done evaluating forces, otherwise write 0

# ==== PHASE 1.1: Force evaluation ====
class PositionReadController(Logic):
    def __init__(self):
        super().__init__("position-read-controller")
        
        self.ctl_force_evaluation_ready = Input(self,"ctl-force-evaluation-ready")
        self.ctl_double_buffer = Input(self, "ctl-dobule-buffer")
        self.ctl_force_evaluation_done = Output(self, "ctl-force-evaluation-done")

        self._next_timestep = True

        self._cell_r = 0
        self._addr_r = 0
        self._addr_n = 0

        # read from register that PositionReader writes
        self.stale_reference = Input(self, "stale-reference") # if all particles read last cycle were NULL

        self.oaddr = Output(self, "oaddr")
        self.cell_r = Output(self, "cell-r")

        self._new_reference = False
        self.new_reference = Output(self, "new-reference") # if PositionReader should write a new value to the reference register
        
        # self.verbose = True
    def halt(self):
        self._next_timestep = True
        self.ctl_force_evaluation_done.set(1)
        self.oaddr.set(NULL)
        self.cell_r.set(NULL)
        self.new_reference.set(NULL)
        
    def logic(self):
        ctl_force_evaluation_ready = self.ctl_force_evaluation_ready.get() 
        if ctl_force_evaluation_ready and self._cell_r == N_CELL:
            self.halt()
            return

        if not ctl_force_evaluation_ready:
            self._next_timestep = True
            self.halt()
            return

        ctl_double_buffer = self.ctl_double_buffer.get()
        stale_reference = self.stale_reference.get()
        offset = 256 if ctl_double_buffer else 0

        if self._next_timestep:
            stale_reference = True
            self._new_reference = True

        if stale_reference:
            if self._new_reference: 
                # We read a new reference particle last cycle, but
                # we still got NULL. Time for a new reference cell
                if self._next_timestep:
                    # Special case: stale_reference && self._new_reference && self._next_timestep
                    # means that we've begun the next timestep and need to start at (0,0)
                    self._next_timestep = False
                    self._cell_r = 0
                    self._addr_r = 0
                else:
                    self._cell_r += 1
                    if self._cell_r == N_CELL:
                        # If we just incremented past the last cell, halt
                        self.halt()
                        return
                    self._addr_r = 0
            else:
                # We need the reader to load a new reference particle
                self._addr_r += 1
            self._new_reference = True
            self._addr_n = 0
            addr = self._addr_r
        else:
            if self._new_reference:
                self._new_reference = False
            else:
                self._addr_n += 1
            addr = self._addr_n
        print(self._cell_r, self._addr_r, self._addr_n)
        self.new_reference.set(self._new_reference)
        self.cell_r.set(self._cell_r)
        self.oaddr.set(addr)
        self.ctl_force_evaluation_done.set(0)

class PositionReader(Logic):
    def __init__(self):
        super().__init__("position-reader")
        self.i = [Input(self, f"i{i}") for i in range(N_CELL)]
        self.o = [Output(self, f"o{i}") for i in range(N_FILTER)]
        self.reference = Output(self, f"reference") # writes to referenceParticle register

        self.cell_r = Input(self, "cell-r")
        self.addr = Input(self, "addr")
        self.new_reference = Input(self, "new-reference")
        self.stale_reference = Output(self, "stale-reference")

        # self.verbose = True

    def logic(self):
        cell_r = self.cell_r.get()
        new_reference = self.new_reference.get()
        addr = self.addr.get()

        if cell_r is NULL:
            [o.set(NULL) for o in self.o]
            self.reference.set(NULL)
            self.stale_reference.set(NULL)
            return

        if new_reference:
            r = self.i[cell_r].get()
            p = Position(cell = cell_r, addr = addr, r = r)
            self.reference.set(p)
            self.stale_reference.set(r is NULL)
            for o in self.o:
                o.set(NULL)
            return

        stale_reference = True
        idx = 0
        for cidx in neighborhood(cell_r):
            r = self.i[cidx].get()
            if r is NULL:
                self.o[idx].set(NULL)
            else:
                stale_reference = False
                p = Position(cell = cidx, addr = addr, r = r)
                self.o[idx].set(p)
            idx += 1

        self.reference.set(NULL)
        self.stale_reference.set(stale_reference)

class PairQueue(Logic):
    def __init__(self):
        super().__init__("pair-queue")
        self.i = [Input(self, f"i{i}") for i in range(N_FILTER)]
        self.o = Output(self, "o")
        self.queue = []

    def logic(self):
        for i in self.i:
            self.queue.append(i.get())
        self.o.set(
                self.queue.pop(0)
        )

class AccelerationUpdateController(Logic):
    def __init__(self):
        super().__init__("reference-accumulator")
        self.reference = Input(self, "reference")
        self.neighbor = Input(self, "neighbor")
        self._r = None
        self._queue = []
        self.o = Output(self, "o")
        self.oaddr = Output(self, "oaddr")

        self.force_pipeline_empty = Input(self, "force-pipeline-empty")
        self.position_writer_done = Input(self, "position-writer-done")

    def logic(self):
        reference = self.reference.get()
        neighbor = self.neighbor.get()

        if self._r is not None and self.position_writer_done.get() and self.force_pipeline_empty.get():
            self._queue.append(self._r)
            self._r = None
        if reference is not NULL:    
            if reference.addr != self._r.addr or reference.cell != self._r.cell:
                self._queue.append(self._r)
                self._r = reference
            else:
                self._r.a = [a1+a2 for a1, a2 in zip(self._r.a, reference.a)]

        if len(self._queue) == 0:
            self.o.set(NULL)
            for o in self.oaddr:
                o.set(NULL)
            return

        acceleration = self._queue.pop(0)

        self.oaddr.set(acceleration.addr)
        self.o.set(acceleration)

class AccelerationUpdater(Logic):
    def __init__(self):
        super().__init__("acceleration-updater")
        self.fragment = Input(self,"i")
        self.ai = [Input(self,"i{i}") for i in range(N_CELL)]
        self.ao = [Output(self,"o{i}") for i in range(N_CELL)]
    
    def logic(self):
        fragment = self.fragment.get()
        if fragment is NULL:
            for o in self.ao:
                o.set(NULL)
            return

        a = [a1+a2 for a1, a2 in zip(fragment.a, self.ai[fragment.cell])]
        for idx, o in self.ao:
            if idx == fragment.cell:
                o.set(a)
            else:
                o.set(NULL)

# ==== PHASE 2: Velocity Update ====
# ==== PHASE 1/2: Acceleration Cache Muxing ====

class AccelerationCacheMux(Logic):
    def __init__(self,i):
        super().__init__(f"acceleration-cache-mux-{i}")

        self.ctl_force_evaulation_ready = Input(self,"force-evaluation-ready")
        self.ctl_velocity_update_ready = Input(self, "velocity-update-ready")

        self.i_p1 = Input(self,"i-p1")
        self.iaddr_p1 = Input(self, "iaddr-p1")
        self.oaddr_p1 = Input(self, "oaddr-p1")

        self.i_p2 = Input(self,"i-p2")
        self.iaddr_p1 = Input(self, "iaddr-p2")
        self.oaddr_p2 = Input(self, "oaddr-p2")

        self.i = Output(self,"i")
        self.iaddr = Output(self, "iaddr")
        self.oaddr = Output(self, "oaddr")

    def logic(self):
        if self.ctl_force_evaluation_ready.get():
            self.i.set(self.i_p1.get())
            self.iaddr.set(self.iaddr_p1.get())
            self.oaddr.set(self.oaddr_p1.get())
        elif self.ctl_velocity_update_ready.get():
            self.i.set(self.i_p2.get())
            self.iaddr.set(self.iaddr_p2.get())
            self.oaddr.set(self.oaddr_p2.get())
        else:
            self.i.set(NULL)
            self.iaddr.set(NULL)
            self.oaddr.set(NULL)
                 

position_read_controller = m.add(PositionReadController())
position_reader = m.add(PositionReader())
pair_queue = m.add(PairQueue())

acceleration_update_controller = m.add(AccelerationUpdateController())
acceleration_updater = m.add(AccelerationUpdater())

velocity_update_controller = m.add(VelocityUpdateController())
velocity_updater = m.add(VelocityUpdater())

acceleration_cache_mux = [m.add(AccelerationCacheMux(i)) for i in range(N_CELL)]

# is True when
# all neighbor particles read last cycle were NULL
# - indicates that current reference particle is stale
# positionReadController.new_reference was True last cycle and reference particle read last cycle was NULL
# - indicates that current reference cell is stale
stale_reference = m.add(Register("stale-reference")) 

# holds reference particle for filter bank
reference = m.add(Register("reference"))

# position_read_controller inputs (ommiting control signals from emulator.py)
connect(stale_reference.o, position_read_controller.stale_reference)

# position_reader inputs
for p_cache, i in zip(p_caches, position_reader.i):
    connect(p_cache.o, i)
connect(position_read_controller.cell_r, position_reader.cell_r)
connect(position_read_controller.new_reference, position_reader.new_reference)
connect(position_read_controller.oaddr, position_reader.addr)

# stale_reference input
connect(position_reader.stale_reference, stale_reference.i)

# reference input
connect(position_reader.reference, reference.i)

# address p_caches
for p_cache in p_caches:
    connect(position_read_controller.oaddr, p_cache.oaddr)

# filter_bank inputs
for o, f in zip(position_reader.o, filter_bank):
    connect(reference.o, f.reference)
    connect(o, f.neighbor)

# pair_queue inputs
for f, i in zip(filter_bank, pair_queue.i):
    connect(f.pair, i)

# force_pipeline inputs
connect(pair_queue.o, force_pipeline.i)

# acceleration_update_controller inputs
connect(force_pipeline.reference, acceleration_update_controller.reference)
connect(force_pipeline.neighbor, acceleration_update_controller.neighbor)
connect(force_pipeline.empty, acceleration_update_controller.force_pipeline_empty)
connect(position_read_controller.ctl_force_evaluation_done, acceleration_update_controller.position_writer_done)

# acceleration_updater inputs
connect(acceleration_update_controller.o, acceleration_updater.fragment)
for a_cache, i in zip(a_caches, acceleration_updater.ai):
    connect(a_cache.o, i)
