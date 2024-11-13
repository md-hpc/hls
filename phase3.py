from hls import *
from common import *

CTL_DOUBLE_BUFFER = []

# Position update control
CTL_READY = [] # reads as 1 when force evaluation should begin/continue, otherwise 0
CTL_DONE = [] # write 1 when your units are done evaluating forces, otherwise write 0

class PositionUpdateController(Logic):
    def __init__(self):
        super().__init__("position-update-controller")
        self.ready = Input(self,"ready")
        self.done = Output(self,"done")

        self.double_buffer = Input(self,"double-buffer")

        self._raddr = 0
        self._overwrite_addr = 0

        self.raddr = Output(self,"oaddr")
        self.overwrite_addr = Output(self,"overwrite-addr")
       

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

        if self._overwrite_addr is NULL:
            print(f"reading addrs {self._raddr}")
            self._raddr += 1
        else:
            print(f"resetting addrs {self._overwrite_addr}")
            self._overwrite_addr += 1
            if self._overwrite_addr == ndb(double_buffer) + DBSIZE:
                self._overwrite_addr = NULL
                self._raddr = db(double_buffer)
        

class PositionUpdater(Logic):
    def __init__(self):
        super().__init__("position-updater")
        self.ready = Input(self, "ready")
        self.done = Output(self, "done")
        self.double_buffer = Input(self, "double_buffer")

        self.overwrite_addr = Input(self, "overwrite-addr")

        self._queues = [[] for _ in range(N_CELL)]
        self._new_addr = None

        self.vi = [Input(self,f"vi{i}") for i in range(N_CELL)]
        self.pi = [Input(self,f"pi{i}") for i in range(N_CELL)]
       
        self.iaddr = [Output(self,f"iaddr{i}") for i in range(N_CELL)]
        self.vo = [Output(self,f"vo{i}") for i in range(N_CELL)]
        self.po = [Output(self,f"p{i}") for i in range(N_CELL)]
        

    def logic(self):
        if not self.ready.get():
            self._new_addr = None
            for o in [self.vo, self.po, self.iaddr]:
                nul(o)
            self.done.set(NULL)
            return


        double_buffer = self.double_buffer.get()
        overwrite_addr = self.overwrite_addr.get()

        if overwrite_addr is not NULL:
            for iaddr, vo, po in zip(self.iaddr, self.vo, self.po):
                iaddr.set(overwrite_addr)
                vo.set(RESET)
                po.set(RESET)
            self.done.set(False)
            return

        if self._new_addr is None:
            addr = 0 if double_buffer else DBSIZE
            self._new_addr = [addr for _ in range(N_CELL)]            

        for pi, vi in zip(self.pi, self.vi):
            _pi = pi.get()
            _vi = vi.get()
            
            if _pi is NULL and _vi is NULL:
                continue
        
            new_p = (_pi + _vi*DT) % L
            new_cell = cell_from_position(new_p)
            self._queues[new_cell].append((new_p, _vi))

        _done = all([len(q) == 0 for q in self._queues])
        self.done.set(_done)
        if _done:
            for o in [self.vo, self.po, self.iaddr]:
                nul(o)
            return
       
        for cell in range(N_CELL):
            q = self._queues[cell]
            if len(q) == 0:
                for o in [self.vo, self.po, self.iaddr]:
                    o[cell].set(NULL)
                continue
            _po, _vo = q.pop(0)
            self.po[cell].set(_po)
            self.vo[cell].set(_vo)
            self.iaddr[cell].set(self._new_addr[cell])
            self._new_addr[cell] += 1
            
        
position_update_controller = m.add(PositionUpdateController())
position_updater = m.add(PositionUpdater())

# position_update_controller inputs
CTL_READY.append(position_update_controller.ready)
CTL_DOUBLE_BUFFER.append(position_update_controller.double_buffer)

# position_updater inputs
CTL_READY.append(position_updater.ready)
CTL_DOUBLE_BUFFER.append(position_updater.double_buffer)
connect(position_update_controller.overwrite_addr, position_updater.overwrite_addr)
for v_cache, vi in zip(v_caches, position_updater.vi):
    connect(v_cache.o, vi)
for p_cache, pi in zip(p_caches, position_updater.pi):
    connect(p_cache.o, pi)

# p_cache inputs
for omux in p_omuxes:
    connect(position_update_controller.raddr, omux.oaddr_phase3)
for imux, iaddr, i in zip(p_imuxes, position_updater.iaddr, position_updater.po):
    connect(iaddr, imux.iaddr_phase3)
    connect(i, imux.i_phase3)

# v_cache inputs
for omux in v_omuxes:
    connect(position_update_controller.raddr, omux.oaddr_phase3)
for imux, iaddr, i in zip(v_imuxes, position_updater.iaddr, position_updater.vo):
    connect(iaddr, imux.iaddr_phase3)
    connect(i, imux.i_phase3)

# control_unit inputs
CTL_DONE = [position_update_controller.done, position_updater.done]
