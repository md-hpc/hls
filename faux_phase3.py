from hls import *
from structures import *


CTL_DOUBLE_BUFFER = Input
CTL_READY = Input
CTL_DONE = Input 

class FauxPositionUpdater(Logic):
    def __init__(self, p_caches, v_caches):
        super().__init__("faux-position-updater")
        
        self.ctl_double_buffer = Input(self,"double-buffer")
        self.ctl_position_update_ready = Input(self,"position-update-ready")
        self.ctl_position_update_done = Output(self,"position-update-done")
        
        self.p_caches = p_caches
        self.v_caches = v_caches

    def logic(self):
        double_buffer = self.ctl_double_buffer.get()
        ready = self.ctl_position_update_ready.get()

        if not ready:
            self.ctl_position_update_done.set(NULL)
            return

        roffset = 256 if double_buffer else 0
        woffset = (roffset+256)%512
        waddrs = [woffset for _ in self.p_caches] # next free address

        for cell in range(N_CELL):
            for waddr in range(woffset, woffset+256):
                self.p_caches[cell].contents[waddr] = NULL
                self.v_caches[cell].contents[waddr] = NULL
        
        for cell in range(N_CELL):
            for raddr in range(roffset, roffset + 256):
                v = self.v_caches[cell].contents[raddr]
                r = self.p_caches[cell].contents[raddr]
                

                if r is NULL and v is NULL:
                    continue

                if r is NULL:
                    print(cell, raddr)
                else:
                    r = (r + v*DT) % L
                
                new_cell = cell_from_position(r)
                waddr = waddrs[new_cell]
                self.p_caches[new_cell].contents[waddr] = r
                self.v_caches[new_cell].contents[waddr] = v
                waddrs[new_cell] += 1
        self.ctl_position_update_done.set(True)        


faux_position_updater = m.add(FauxPositionUpdater(p_caches, v_caches))
# faux_position_updater inputs
CTL_DOUBLE_BUFFER = faux_position_updater.ctl_double_buffer
CTL_READY = faux_position_updater.ctl_position_update_ready

# control_unit inputs
CTL_DONE = faux_position_updater.ctl_position_update_done

# p_mux inputs
for i in range(N_CELL):
    connect(null_const.o, p_omuxes[i].oaddr_phase3)
    connect(null_const.o, p_imuxes[i].iaddr_phase3)
    connect(null_const.o, p_imuxes[i].i_phase3)

# v_mux inputs
for i in range(N_CELL):
    connect(null_const.o, v_omuxes[i].oaddr_phase3)
    connect(null_const.o, v_imuxes[i].iaddr_phase3)
    connect(null_const.o, v_imuxes[i].i_phase3)
