from hls import *
from structures import *


CTL_DOUBLE_BUFFER = Input
CTL_POSITION_UPDATE_READY = Input
CTL_POSITION_UPDATE_DONE = Input 

class FauxPositionUpdater(Logic):
    def __init__(self, p_caches, v_caches):
        super().__init__("faux-position-updater")
        
        self.ctl_double_buffer = Input(self,"double-buffer")
        self.ctl_position_update_ready = Input(self,"position-update-ready")
        self.ctl_position_update_done = Output(self,"position-update-done")
        
        self.caches = p_caches.copy() + v_caches.copy()

    def logic(self):
        double_buffer = self.ctl_double_buffer.get()
        ready = self.ctl_position_update_ready.get()

        if not ready:
            self.ctl_position_update_done.set(NULL)
            return

        for cache in self.caches:
            new_contents = cache.contents.copy()
            for i, x in enumerate(cache.contents):
                new_contents[(i+256)%512] = cache.contents[i]
            cache.contents = new_contents
        self.ctl_position_update_done.set(True)


faux_position_updater = m.add(FauxPositionUpdater(p_caches, v_caches))
# faux_position_updater inputs
CTL_DOUBLE_BUFFER = faux_position_updater.ctl_double_buffer
CTL_POSITION_UPDATE_READY = faux_position_updater.ctl_position_update_ready

# control_unit inputs
CTL_POSITION_UPDATE_DONE = faux_position_updater.ctl_position_update_done

# p_mux inputs
for i in range(N_CELL):
    connect(null_const.o, p_omuxes[i].oaddr_p3)
    connect(null_const.o, p_imuxes[i].iaddr_p3)
    connect(null_const.o, p_imuxes[i].i_p3)

# v_mux inputs
for i in range(N_CELL):
    connect(null_const.o, v_omuxes[i].oaddr_p3)
    connect(null_const.o, v_imuxes[i].iaddr_p3)
    connect(null_const.o, v_imuxes[i].i_p3)
