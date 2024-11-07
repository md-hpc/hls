from hls import *
from structures import *
import random
from math import floor
# PHASE 3: velocity cache read & position cache read/position cache write

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
CTL_DOUBLE_BUFFER = Input 

# Position update control
CTL_FORCE_POSITION_UPDATE_READY = Input # reads as 1 when force evaluation should begin/continue, otherwise 0
CTL_POSITION_UPDATE_DONE = Output # write 1 when your units are done evaluating forces, otherwise write 0


dT = 1
class PositionController(Logic):
    def __init__(self):
        super().__init__()
        self.ctl_force_position_update_ready = Input(self,"")
        self.ctl_double_buffer = Input(self,"")
        self.ctl_position_update_done = Output(self,"")
		
        self.current_position = [Input(self,"") for _ in range(N_CELL)]
        self.current_velocity = [Input(self,"") for _ in range(N_CELL)]
        self.next_position = [Output(self,"") for _ in range(N_CELL)]
        self.next_velocity = [Output(self,"") for _ in range(N_CELL)]
        self.nextAddr = Output(self,"")
        self.update_address = [Output(self,"") for _ in range(N_CELL)]
        self.current_address = Input(self,"")
		
        self.cidx = Input(self,"")
		
        self.current_cell = Input(self,"")
        self.next_cell = Output(self,"")
		
        self.hasCleared = Input(self,"")
        self.Cleared = Output(self,"")
        
        self.currentClear = Input(self,"")
        self.nextClear = Output(self,"")
    def logic(self):
		
        if(self.ctl_force_position_update_ready.get()):
			
            if(hasCleared.get() == 0):
                print("HELLO")
                for i in range(N_CELL):
                    if(self.currentClear.get() == 255):
                        self.Cleared.set(1)
                        
                    self.update_address[i].set((1 - self.ctl_double_buffer)  * 256 + self.currentClear.get())
                    self.nextClear(self.currentClear.get()+1)
                    

                    self.next_position[i].set(NULL)
                    self.next_velocity[i].set(NULL)

            else:
                r = self.current_position[self.current_cell.get()].get()
                #if(r != NULL and v != NULL)
                v = self.current_velocity[self.current_cell.get()].get()

                if(r == NULL or v == NULL or self.current_address.get() == 256):
                    if(self.current_address.get() == 256):
                        raise("There are 256 particles in this cell")
                    if(current_cell.get() == N_CELL - 1):
                        self.ctl_position_update_done.set(1)
                    else:
                        self.nextAddr.set(( self.ctl_double_buffer) * 256 )
                        self.next_cell.set(self.current_cell.get() + 1)
                else:
                    self.nextAddr.set((self.current_address.get() + 1)%256 + (self.ctl_double_buffer) * 256 )
                    r_o = r + v * dT

                    idx = linear_idx( # find which p_cache this particle must go into
                        [floor(x/CUTOFF) for x in r]
                    )
                    self.update_address[idx].set((1 - self.ctl_double_buffer)  * 256 + self.cidx[idx].get())
                    
                    self.next_velocity[idx].set(v)
                    self.next_position[idx].set(r_o)
                    self.cidx[idx].set(self.cidx[idx].get() + 1)
		
readController = m.add(PositionController())
CTL_FORCE_POSITION_UPDATE_READY = readController.ctl_force_position_update_ready
CTL_DOUBLE_BUFFER = readController.ctl_double_buffer
CTL_POSITION_UPDATE_DONE = readController.ctl_position_update_done



cidx = [m.add(Register("")) for _ in range(N_CELL)]
for i in range(len(cidx)):
	cidx[i].contents = 0

memClear = m.add(Register(""))
memClear.contents = 0

clearCounter = m.add(Register(""))
clearCounter.contents = 0

current_cell = m.add(Register(""))
current_cell.contents = 0
ptrVel = m.add(Register(""))
ptrPos = m.add(Register(""))
addr = m.add(Register(""))
addr.contents = 0
# addr+1 -> ptr -> addr
connect(readController.nextAddr, ptrVel.i)
connect(readController.nextAddr, ptrPos.i)
connect(readController.nextAddr, addr.i)


connect(addr.o, readController.current_address)


connect(readController.Cleared, memClear.i)
connect(memClear.o,readController.hasCleared)

connect(clearCounter.o,readController.currentClear)
connect(readController.nextClear,clearCounter.i)

for i in range(len(p_caches)):
	connect(ptrPos.o, p_caches[i].oaddr)
	connect(ptrVel.o, v_caches[i].oaddr)
	connect(p_caches[i].o,readController.current_position[i])
	connect(v_caches[i].o,readController.current_velocity[i])
	
	connect(readController.update_address[i], p_caches[i].iaddr)
	connect(readController.next_position[i], p_caches[i].i)
	connect(readController.update_address[i], v_caches[i].iaddr)
	connect(readController.next_velocity[i], v_caches[i].i)


L = CUTOFF * UNIVERSE_SIZE # length of one dimension of the simulation box
N = 80 * N_CELL # number of particles in the simulation

init_cidx = [0 for _ in range(N_CELL)] # index into contents of each p_cache
for _ in range(N):
    r = (L*random.random(), L*random.random(), L*random.random())
    v = (random.random(), random.random(), random.random())
    idx = linear_idx( # find which p_cache this particle must go into
            floor(r[0]/CUTOFF),floor(r[1]/CUTOFF),floor(r[2]/CUTOFF)
        )
    p_caches[idx].contents[init_cidx[idx]] = r
    v_caches[idx].contents[init_cidx[idx]] = v
    init_cidx[idx] += 1

for i in range(256):
	m.clock()
#print(p_caches[0].contents)
#print(v_caches[0].contents)
# we use double buffering (every other phase we use the top half of the caches instead because of
# particle transfers that happen in the motion update phase). This will provide a bit (0 or 1) that will give
# an offset to add to all addresses used. 0 if it reads 0 and 256 if it reads 1
#
# e.g. if you're accessing address 13 of a givne BRAM, and the bit is 0, you will just access address 13
# but if bit is 1, you will access address 13 + 256 = 269 instead.
