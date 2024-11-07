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
        
    def halt(self):
        print(self.name, "halting")
        self.next_timestep = True
        self.ctl_force_evaluation_done.set(1)
        self.oaddr.set(NULL)
        self.cell_r.set(NULL)
        self.new_reference.set(NULL)
        
    def logic(self):
        ctl_force_evaluation_ready = self.ctl_force_evaluation_ready.get() 
        print(self.name, "logic")
        if ctl_force_evaluation_ready and self._cell_r == N_CELL:
            self.halt()

        if not ctl_force_evaluation_ready:
            self._cell_r = 0
            self._addr_r = 0
            self._addr_n = 0
            self.next_timestep = True
            self.halt()
            return
        print(self.name, "continuing")
        ctl_double_buffer = self.ctl_double_buffer.get()
        stale_reference = self.stale_reference.get()

        offset = 256 if ctl_double_buffer else 0

        if self.next_timestep:
            self.next_timestep = False
            self._new_reference = False
            stale_reference = True

        if stale_reference:
            if self._new_reference:
                self._cell_r += 1
                self._addr_r = 0
            else:
                self._addr_r += 1
                self._new_reference = True
            if self._cell_r == N_CELL:
                self.halt()
                return
            addr = self._addr_r
        else:
            addr = self._addr_n
        
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

        self.cell_r = Input(self, "cell_r")
        self.addr = Input(self, "addr")
        self.new_reference = Input(self, "new-reference")
        self.stale_reference = Output(self, "stale-reference")

    def logic(self):
        cell_r = self.cell_r.get()
        assert cell_r is not None
        new_reference = self.new_reference.get()
        addr = self.addr.get()

        if new_reference:
            r = self.i[cell_r].get()
            p = Position(cell = cell_r, addr = addr, r = r)
            self.reference.set(p)
            self.stale_reference.set(r is NULL)
            for o in self.o:
                self.o.set(NULL)
            return

        stale_reference = True
        i, j, k = cell_idx(cell_r)
        idx = 0
        for di in range(-1,2):
            for dk in range(-1,2):
                for dj in range(-1,2):
                    if di < 0 or di == 0 and dj < 0 or di == 0 and dj == 0 and dk < 0:
                        continue
                    cidx = linear_idx(i+di, j+dj, k+dk)
                    r = self.i[cidx].get()
                    if r is not NULL:
                        stale_reference = False
                    p = Position(cell = cidx, addr = addr, r = r)
                    self.o[idx].set(p)
        self.reference.set(NULL)
        self.stale_reference.set(stale_reference)

class PairQueue(Logic):
    def __init__(self):
        super().__init__("pq")
        self.i = [Input(self, "i{i}") for i in range(N_FILTER)]
        self.o = Output(self, "o")
        self.queue = []

    def logic(self):
        for i in self.i:
            self.queue.append(i.get())
        self.o.set(
                self.queue.pop(0)
        )

position_read_controller = m.add(PositionReadController())
position_reader = m.add(PositionReader())
pair_queue = m.add(PairQueue())

# is True when
# all neighbor particles read last cycle were NULL
# - indicates that current reference particle is stale
# positionReadController.new_reference was True last cycle and reference particle read last cycle was NULL
# - indicates that current reference cell is stale
stale_reference = m.add(Register("stale-reference")) 

# holds reference particle
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

# IO for emulator.py
CTL_DOUBLE_BUFFER = position_read_controller.ctl_double_buffer
CTL_FORCE_EVALUATION_READY = position_read_controller.ctl_force_evaluation_ready 
CTL_FORCE_EVALUATION_DONE = position_read_controller.ctl_force_evaluation_done
