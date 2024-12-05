from hls import *
from common import *

CTL_DOUBLE_BUFFER = []

# Position update control
CTL_READY = [] # reads as 1 when force evaluation should begin/continue, otherwise 0
CTL_DONE = [] # write 1 when your units are done evaluating forces, otherwise write 0

class PositionUpdateController(Logic):
    def __init__(self,cell):
        super().__init__("position-update-controller-"+str(cell))
        self.ready = Input(self,"ready")
        self.done = Output(self,"done")

        self.double_buffer = Input(self,"double-buffer")

        self._raddr = 0
        self._overwrite_addr = 0

        self.raddr = Output(self,"oaddr")
        self.overwrite_addr = Output(self,"overwrite-addr")
        
        self.block = Input(self,"block")
    def logic(self):
        ready = self.ready.get()
        double_buffer = self.double_buffer.get()

        if not ready:
            # wait for ctl to ack our done before resetting
            self._raddr = db(double_buffer)
            self._overwrite_addr = ndb(double_buffer)
            self.raddr.set(NULL)
            self.overwrite_addr.set(NULL)
            self.done.set(NULL)
            return
        

        if self._raddr == db(double_buffer) + DBSIZE and self.ready.get():
            self.raddr.set(NULL)
            self.overwrite_addr.set(NULL)
            self.done.set(True)
            return
         
        self.raddr.set(self._raddr)
        self.overwrite_addr.set(self._overwrite_addr)
        self.done.set(False)

        if self._overwrite_addr is NULL and self.block != False:
            print(f"reading addrs {self._raddr}")
            self._raddr += 1
        else:
            print(f"resetting addrs {self._overwrite_addr}")
            self._overwrite_addr += 1
            if self._overwrite_addr == ndb(double_buffer) + DBSIZE:
                self._overwrite_addr = NULL
                self._raddr = db(double_buffer)
        

class PositionUpdater(Logic):
    def __init__(self,cell):
        super().__init__("position-updater-" + str(cell))
        self.ready = Input(self, "ready")
        self.done = Output(self, "done")
        self.double_buffer = Input(self, "double_buffer")

        self.overwrite_addr = Input(self, "overwrite-addr")

        self._new_addr = None

        self.vi = Input(self,f"vi")
        self.pi = Input(self,f"pi")
       
        self.iaddr = Output(self,f"iaddr")
        self.vo = Output(self,f"vo")
        self.po = Output(self,f"po")
        
        self.nodePosIn = Input(self,"nodePosIn")
        self.nodeVelIn = Input(self,"nodeVelIn")
        self.nodeCellIn = Input(self,"nodeCellIn")
        
        
        self.nodePos = NULL
        self.nodeVel = NULL
        self.nodeCell = NULL
        
        
        
        self.nodePosOut = Output(self,"nodePosOut")
        self.nodeVelOut = Output(self,"nodeVelOut")
        self.nodeCellOut = Output(self,"nodeCellOut")
        
        self.block = Output(self,"block")
        
        self.cell = cell
    def logic(self):
        if not self.ready.get():
            self._new_addr = None
            self.nodeVelOut.set(NULL)
            self.nodePosOut.set(NULL)
            self.nodeCellOut.set(NULL)
            for o in [self.vo, self.po, self.iaddr]:
                nul(o)
            self.done.set(NULL)
            self.block.set(NULL)
            #print("here")
            return


        double_buffer = self.double_buffer.get()
        overwrite_addr = self.overwrite_addr.get()

        if overwrite_addr is not NULL:
            self.iaddr.set(overwrite_addr)
            self.vo.set(RESET)
            self.po.set(RESET)
            self.done.set(False)
            
            self.nodeVelOut.set(NULL)
            self.nodePosOut.set(NULL)
            self.block.set(NULL)
            self.nodeCellOut.set(NULL)
            #print("there")
            return

        if self._new_addr is None:
            addr = 0 if double_buffer else DBSIZE
            self._new_addr = addr 
        
        if self.nodePos is not NULL and self.nodeVel is not NULL and self.nodeCell is not NULL:
            self.done.set(False)
            self.block.set(True)
            if(self.nodeCell != self.cell):
                self.nodeVelOut.set(self.nodeVel)
                self.nodePosOut.set(self.nodePos)
                self.nodeCellOut.set(self.nodeCell)
            else:
                self.nodeVelOut.set(NULL)
                self.nodePosOut.set(NULL)
                self.nodeCellOut.set(NULL)
                self.po.set(self.nodePos)
                self.vo.set(self.nodeVel)
            #print("andthere")
            return
        self.nodePos = self.nodePosIn.get()
        self.nodeVel = self.nodeVelIn.get()
        self.nodeCell = self.nodeCellIn.get()
        _pi = self.pi.get()
        _vi = self.vi.get()
        if _pi is NULL and _vi is NULL and self.nodePos is NULL and self.nodeVel is NULL and self.nodeCell is NULL:
            self.done.set(True)
            #print("WHAT???")
            
            self.iaddr.set(NULL)
            self.po.set(NULL)
            self.vo.set(NULL)
            self.block.set(False)
            self.nodePosOut.set(NULL)
            self.nodeVelOut.set(NULL)
            self.nodeCellOut.set(NULL)
            return
        
        self.done.set(False)
        new_p = (_pi + _vi*DT) % L
        new_cell = cell_from_position(new_p)
        if(self.cell == new_cell):
            self.po.set(new_p)
            self.vo.set(_vi)
            self.iaddr.set(self._new_addr)
            self._new_addr += 1
            
            self.block.set(False)
            self.nodePosOut.set(NULL)
            self.nodeVelOut.set(NULL)
            self.nodeCellOut.set(NULL)
        else:
            self.block.set(True)
            self.nodePosOut.set(new_p)
            self.nodeVelOut.set(_vi)
            self.iaddr.set(NULL)
            self.po.set(NULL)
            self.vo.set(NULL)
            self.nodeCellOut.set(new_cell)
position_update_controller = [m.add(PositionUpdateController(cell)) for cell in range(N_CELL)]
position_updater = [m.add(PositionUpdater(cell)) for cell in range(N_CELL)]

ring_pos_reg =  [m.add(Register("ring_pos_"+str(cell))) for cell in range(N_CELL)]
ring_vel_reg =  [m.add(Register("ring_vel_"+str(cell))) for cell in range(N_CELL)]
ring_cell_reg =  [m.add(Register("ring_cell_"+str(cell))) for cell in range(N_CELL)]
# position_update_controller inputs
for i in range(N_CELL):
    CTL_READY.append(position_update_controller[i].ready)
    CTL_DOUBLE_BUFFER.append(position_update_controller[i].double_buffer)
    CTL_READY.append(position_updater[i].ready)
    CTL_DOUBLE_BUFFER.append(position_updater[i].double_buffer)
    connect(position_update_controller[i].overwrite_addr, position_updater[i].overwrite_addr)
    connect(v_caches[i].o, position_updater[i].vi)
    connect(p_caches[i].o, position_updater[i].pi)
    connect(position_update_controller[i].raddr, p_omuxes[i].oaddr_phase3)
    connect(position_update_controller[i].raddr, v_omuxes[i].oaddr_phase3)
    
    
    connect(position_updater[i].nodePosOut,ring_pos_reg[i].i)
    connect(position_updater[i].nodeVelOut,ring_vel_reg[i].i)
    connect(position_updater[i].nodeCellOut,ring_cell_reg[i].i)
    
    
    
    connect(ring_pos_reg[i].o,position_updater[(i+1)%N_CELL].nodePosIn)
    connect(ring_vel_reg[i].o,position_updater[(i+1)%N_CELL].nodeVelIn)
    connect(ring_cell_reg[i].o,position_updater[(i+1)%N_CELL].nodeCellIn)
    
    
    connect(position_updater[i].block,position_update_controller[i].block)
    
    
for imux, updater in zip(p_imuxes, position_updater):
    connect(updater.iaddr, imux.iaddr_phase3)
    connect(updater.po, imux.i_phase3)
# v_cache inputs
for imux, updater in zip(v_imuxes, position_updater):
    connect(updater.iaddr, imux.iaddr_phase3)
    connect(updater.vo, imux.i_phase3)

# control_unit inputs
CTL_DONE = [controller.done for controller in position_update_controller] + [updater.done for updater in position_updater]
