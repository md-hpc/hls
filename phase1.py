from hls import *
from structures import *

# PHASE 1: position cache read/filter write AND filter read/force evaluator write

# an important note for phase 1: we use periodic boundary conditions in this simulation, meaning that
# if your reference cell is, say, (3, 3, 3) in a 4x4x4 universe (indexing starts at 0), then you will need
# to pair with particles from (3,0,0), (3,3,0), (0,0,3), etc. because they are considered adjacent by our
# model

'''
Required IO. Used for communication with emulator.ControlUnit

Override these values at the end of the script with references to your control units IO attributes, e.g.

class ReadController(Logic):
    def __init__(self):
        self.ctl_force_evaluation_ready = Input(self)
        self.ctl_force_evaluation_done = Output(self)
        ...

readController = m.add(ReadController())
connect(CTL_FORCE_EVALUATION_READY, readController.ctl_force_evaluation_ready)
CTL_FORCE_EVALUATION_READY = readController.ctl_force_evaluation_done
'''

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

def PositionReadController(Logic):
    def __init__(self):
        super().__init__()
        
        self.ctl_force_evaluation_ready = Input(self)
        self.ctl_double_buffer = Input(self)

        self.ctl_force_evaluation_done = Output(self)

        self._next_timestep = True

        self._cell_r = 0
        self._addr_r = 0
        self._addr_n = 0

        # read from register that PositionReader writes
        self.stale_reference = Input(self) # if all particles read last cycle were NULL

        self.oaddr = Output(self)
        self.cell_r = Output(self)

        self._new_reference = False
        self.new_reference = Output(self) # if PositionReader should write a new value to the reference register
        

    def halt(self):
        self.next_timestep = True
        self.ctl_force_evaluation_done.set(1)
        self.oaddr.set(NULL)
        self.cell_r.set(NULL)
        self.new_reference.set(NULL)

    def logic(self):
        ctl_force_evaluation_ready = self.ctl_force_evaluation_ready.get() 
        if ctl_force_evaluation_ready and self._cell_r == N_CELL:
            self.halt()

        if not ctl_force_evaluation_ready:
            self._cell_r = 0
            self._addr_r = 0
            self._addr_n = 0
            self.next_timestep = True
            self.halt()

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


