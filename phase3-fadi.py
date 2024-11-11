from hls import *
from common import *
import random
from math import floor
# PHASE 3: velocity cache read & position cache read/position cache write

CTL_DOUBLE_BUFFER = Input 

# Position update control
CTL_FORCE_POSITION_UPDATE_READY = Input # reads as 1 when force evaluation should begin/continue, otherwise 0
CTL_POSITION_UPDATE_DONE = Output # write 1 when your units are done evaluating forces, otherwise write 0


class PositionController(Logic):
    def __init__(self):
        super().__init__("PositionController")

        self.ctl_force_position_update_ready = Input(self,"FPUR")
        self.ctl_double_buffer = Input(self,"DB")
        self.ctl_position_update_done = Output(self,"PUD")

        self.current_position = [Input(self,"current_position_"+str(i)) for i in range(N_CELL)]
        self.current_velocity = [Input(self,"current_velocity_"+str(i)) for i in range(N_CELL)]
        self.next_position = [Output(self,"next_position_"+str(i)) for i in range(N_CELL)]
        self.next_velocity = [Output(self,"next_velocity_"+str(i)) for i in range(N_CELL)]
        self.nextAddr = Output(self,"nextAddr")
        self.update_address = [Output(self,"update_address_"+str(i)) for i in range(N_CELL)]
        self.current_address = Input(self,"current_address_")
		
        self.cidx = [Input(self,"cidx_"+str(i)) for i in range(N_CELL)]
        self.cidx_o = [Output(self,"cidx_o_"+str(i)) for i in range(N_CELL)]
        self.current_cell = Input(self,"current_cell")
        self.next_cell = Output(self,"next_cell")
		
        self.hasCleared = Input(self,"hasCleared")
        self.Cleared = Output(self,"Cleared")
        
        self.currentClear = Input(self,"currentClear")
        self.nextClear = Output(self,"nextClear")
    def logic(self):
		
        if(self.ctl_force_position_update_ready.get()): 
            if(self.hasCleared.get() == 0):
                #print("HELLO")
                self.ctl_position_update_done.set(0)
                if(self.currentClear.get() == 256):
                    
                    self.Cleared.set(1)
                    self.nextClear.set(self.currentClear.get())
                else:
                    self.nextClear.set(self.currentClear.get()+1)
                    self.Cleared.set(0)
                #print((1 - self.ctl_double_buffer)  * 256 + self.currentClear.get())
                for i in range(N_CELL): 
                    
                    self.update_address[i].set((1 - self.ctl_double_buffer.get())  * 256 + self.currentClear.get())
                    self.next_position[i].set(RESET)
                    self.next_velocity[i].set(RESET)
                self.nextAddr.set( (self.ctl_double_buffer.get()) * 256 )
                for i in range(N_CELL):
                    self.cidx_o[i].set(0)
                self.next_cell.set(0)

            else:
                self.Cleared.set(1)
                self.nextClear.set(self.currentClear.get())
                r = self.current_position[self.current_cell.get()].get()
                #if(r != NULL and v != NULL)
                v = self.current_velocity[self.current_cell.get()].get()
                #print("Something")
                if(r is NULL or v is NULL or self.current_address.get() == 255):
                    for i in range(N_CELL):
                        self.next_velocity[i].set(self.current_velocity[i].get())
                        self.next_position[i].set(self.current_position[i].get())
                        self.cidx_o[i].set(self.cidx[i].get())
                        self.update_address[i].set((1 - self.ctl_double_buffer.get())  * 256 + self.cidx[i].get())
                    if(self.current_address.get() == 256):
                        raise("There are 256 particles in this cell")
                    if(self.current_cell.get() == N_CELL - 1):
                        #self.ctl_position_update_done.set(1)
                        self.nextAddr.set( self.current_address.get())
                        self.next_cell.set(self.current_cell.get())
                        self.ctl_position_update_done.set(1)
                        #print("DONE")
                    else:
                        self.nextAddr.set(( self.ctl_double_buffer.get()) * 256 )
                        self.next_cell.set(self.current_cell.get() + 1)
                        self.ctl_position_update_done.set(0)
                        #print("NEXT: " + str(self.current_cell.get()  + 1))
                else:
                    self.ctl_position_update_done.set(0)
                    self.next_cell.set(self.current_cell.get())
                    self.nextAddr.set((self.current_address.get() + 1)%256 + (self.ctl_double_buffer.get()) * 256 )
                    r = v*DT
                    #print(r)
                    #print(r_o)
                    idx = cell_from_position(r)
                    #if(idx != self.current_cell.get()):
                    #    print(str(idx)+ " -----> " + str(self.current_cell.get()))
                    #else:
                    #    print("NO TRANSITION")
                    self.update_address[idx].set((1 - self.ctl_double_buffer.get())  * 256 + self.cidx[idx].get())
                    
                    self.next_velocity[idx].set(v)
                    self.next_position[idx].set(r)
                    self.cidx_o[idx].set(self.cidx[idx].get() + 1)
                    #if(idx == 0):
                    #    print(str(self.current_cell.get())+"  |  " +str(self.current_address.get() + 1)+ " ------> "+str(idx) + "  |  "+ str(self.cidx[idx].get()))
                    for i in range(N_CELL):
                        if(i != idx):
                            self.next_velocity[i].set(self.current_velocity[i].get())
                            self.next_position[i].set(self.current_position[i].get())
                            self.cidx_o[i].set(self.cidx[i].get())
                            self.update_address[i].set((1 - self.ctl_double_buffer.get())  * 256 + self.cidx[i].get())
        else:
            nul(self.ctl_position_update_done)
            nul(self.next_position)
            nul(self.next_velocity)
            nul(self.nextAddr)
            nul(self.update_address)
            nul(self.cidx_o)
            nul(self.next_cell)
            nul(self.Cleared)
            nul(self.nextClear)

readController = m.add(PositionController())
CTL_READY = [readController.ctl_force_position_update_ready]
CTL_DOUBLE_BUFFER = [readController.ctl_double_buffer]
CTL_DONE = [readController.ctl_position_update_done]



cidx = [m.add(Register("cidx_" + str(i))) for i in range(N_CELL)]
for i in range(len(cidx)):
	cidx[i].contents = 0

memClear = m.add(Register("memClear"))
memClear.contents = 0

clearCounter = m.add(Register("clearCounter"))
clearCounter.contents = 0

current_cell = m.add(Register("ccell"))
current_cell.contents = 0
ptrVel = m.add(Register("ptrVel"))
ptrPos = m.add(Register("ptrPos"))
addr = m.add(Register("addr"))
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
connect(current_cell.o,readController.current_cell)
connect(readController.next_cell,current_cell.i)
for i in range(len(p_caches)):
    
    connect(ptrPos.o, p_omuxes[i].oaddr_phase3)
    connect(ptrVel.o, v_omuxes[i].oaddr_phase3)
    connect(p_caches[i].o,readController.current_position[i])
    connect(v_caches[i].o,readController.current_velocity[i])
	
    connect(readController.update_address[i], p_imuxes[i].iaddr_phase3)
    connect(readController.next_position[i], p_imuxes[i].i_phase3)
    connect(readController.update_address[i], v_imuxes[i].iaddr_phase3)
    connect(readController.next_velocity[i], v_imuxes[i].i_phase3)

    connect(cidx[i].o,readController.cidx[i])
    connect(readController.cidx_o[i],cidx[i].i)
    

