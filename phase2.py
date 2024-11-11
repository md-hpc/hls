from hls import *
from structures import *
import numpy

# PHASE 2: force pipeline read/acceleration cache write AND acceleration & velocity cache read AND velocity cache write

# we use double buffering (every other phase we use the top half of the caches instead because of
# particle transfers that happen in the motion update phase). This will provide a bit (0 or 1) that will give
# an offset to add to all addresses used. 0 if it reads 0 and 256 if it reads 1
#
# e.g. if you're accessing address 13 of a givne BRAM, and the bit is 0, you will just access address 13
# but if bit is 1, you will access address 13 + 256 = 269 instead.

CTL_READY = [] # read as 1 when velocity update should begin/continue, otherwise 0
CTL_DONE = Output # write 1 when your units are done evaluating velocity, otherwise write 0

class VelocityUpdateController(Logic):
    def __init__(self):
        super().__init__("velocity-update-controller")

        self.ctl_velocity_update_ready = Input(self,"velocity-update-ready")
        self.ctl_velocity_update_done = Output(self,"velocity-update-done")
        
        self._next_timestep = True
        self.updater_done = Input(self, "writer-done")

        self._addr = 0
        self.addr = Output(self, "addr")

    def logic(self):
        ctl_velocity_update_ready = self.ctl_velocity_update_ready.get()

        if not ctl_velocity_update_ready:
            self.addr.set(NULL)
            self.ctl_velocity_update_done.set(NULL)
            return

        updater_done = self.updater_done.get()

        if updater_done and not self._next_timestep:
            self._addr = 0
            self.addr.set(NULL)
            self.ctl_velocity_update_done.set(True)
            self._next_timestep = True
            return
 
        print(f"phase2: {self._addr}")
       
        self._next_timestep = False
        self.addr.set(self._addr)
        self._addr += 1
        self.ctl_velocity_update_done.set(False)

class VelocityUpdater(Logic):
    def __init__(self):
        super().__init__("velocity-updater")
        
        self.ready = Input(self, "ready")

        self.a = [Input(self, f"a{i}") for i in range(N_CELL)]
        self.vi = [Input(self, f"vi{i}") for i in range(N_CELL)]
        self.vo = [Output(self, f"vo{i}") for i in range(N_CELL)]
        
        self.updater_done = Output(self,"writer-done")

    def logic(self):
        if not self.ready.get():
            nul(self.vo)
            self.updater_done.set(NULL)
            return

        _updater_done = True
        for a, vi, vo in zip(self.a, self.vi, self.vo):
            _a = a.get()
            _vi = vi.get()
        
            if _a is NULL:
                vo.set(NULL)
            else:
                vo.set(_vi + DT * _a)
                _updater_done = False
        self.updater_done.set(_updater_done)

velocity_update_controller = m.add(VelocityUpdateController())
velocity_updater = m.add(VelocityUpdater())

# is True when updater read NULL from every cache last cycle
updater_done = m.add(Register("velocity-updater-done"))

# velocity_update_controller inputs
CTL_READY.append(velocity_update_controller.ctl_velocity_update_ready)
connect(updater_done.o, velocity_update_controller.updater_done)

# velocity_updater inputs
CTL_READY.append(velocity_updater.ready)
for i in range(N_CELL):
    connect(a_caches[i].o, velocity_updater.a[i])
    connect(v_caches[i].o, velocity_updater.vi[i])

# updater_done input
connect(velocity_updater.updater_done, updater_done.i)

# a_muxes input
for i in range(N_CELL):
    connect(velocity_update_controller.addr, a_omuxes[i].oaddr_phase2)
    # reset each acceleration as we read it so phase 1 doesn't have to
    # worry about stale contents when it writes
    connect(velocity_update_controller.addr, a_imuxes[i].iaddr_phase2)
    connect(reset_const.o, a_imuxes[i].i_phase2)

# v_muxes input
for i in range(N_CELL):
    connect(velocity_update_controller.addr, v_omuxes[i].oaddr_phase2)
    connect(velocity_update_controller.addr, v_imuxes[i].iaddr_phase2)
    connect(velocity_updater.vo[i], v_imuxes[i].i_phase2)

# control_unit inputs
CTL_DONE = velocity_updater.updater_done
