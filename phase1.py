from hls import *
from structures import *

# PHASE 1: position cache read/filter write AND filter read/force evaluator write

# an important note for phase 1: we use periodic boundary conditions in this simulation, meaning that
# if your reference cell is, say, (3, 3, 3) in a 4x4x4 universe (indexing starts at 0), then you will need
# to pair with particles from (3,0,0), (3,3,0), (0,0,3), etc. because they are considered adjacent by our
# model

'''
Required IO. Used for communication with emulator.ControlUnit

Override these values at the end of the script with references to your control units' IO attributes, e.g.

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

