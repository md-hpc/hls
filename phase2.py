from hls import *
import sys

if "-t" in sys.argv:
    from t12_structures import *
else:
    from structures import *

# PHASE 2: force pipeline read/acceleration cache write AND acceleration & velocity cache read AND velocity cache write

# we use double buffering (every other phase we use the top half of the caches instead because of
# particle transfers that happen in the motion update phase). This will provide a bit (0 or 1) that will give
# an offset to add to all addresses used. 0 if it reads 0 and 256 if it reads 1
#
# e.g. if you're accessing address 13 of a givne BRAM, and the bit is 0, you will just access address 13
# but if bit is 1, you will access address 13 + 256 = 269 instead.

CTL_VELOCITY_UPDATE_READY = Input # read as 1 when velocity update should begin/continue, otherwise 0
CTL_VELOCITY_UPDATE_DONE = Output # write 1 when your units are done evaluating velocity, otherwise write 0

class VelocityUpdateController(Logic):
    def __init__(self):
        super().__init__("velocity-update-controller")

        self.ctl_velocity_update_ready = Input(self,"velocity-update-ready")
        self.ctl_velocity_update_done = Output(self,"velocity-update-done")

        self.updater_done = Input(self, "writer-done")

        self._oaddr = 0
        self.oaddr = Output(self, "oaddr")

    def logic(self):
        ctl_velocity_update_ready = self.ctl_velocity_update_ready.get()

        if not ctl_velocity_update_ready:
            self.oaddr.set(NULL)
            self.ctl_velocity_update_done.set(NULL)

        updater_done = self.updater_done.get()

        if updater_done:
            self._oaddr = 0
            self.oaddr.set(NULL)
            self.ctl_velocity_update_done.set(True)
        
        self.oaddr.set(self._oaddr)
        self._oaddr += 1

class VelocityUpdater(Logic):
    def __init__(self):
        super().__init__("velocity-updater")
        
        self.a = [Input(self, f"a{i}") for i in range(N_CELL)]
        self.vi = [Input(self, f"vi{i}") for i in range(N_CELL)]
        self.vo = [Output(self, f"vo{i}") for i in range(N_CELL)]

        self.updater_done = Output(self,"writer-done")

    def logic(self):
        _updater_done = True
        for a, vi, vo in zip(self.a, self.vi, self.vo):
            _a = a.get()
            _vi = vi.get()
            _vo = vo.get()
            
            if _a is NULL:
                vo.set(NULL)
            else:
                vo.set([vn+an*DT for vn, an in zip(_v, _a)])
                _updater_done = False
        self.updater_done.set(_updater_done)


velocity_update_controller = m.add(VelocityUpdateController())
velocity_updater = m.add(VelocityUpdater())

# is True when updater read NULL from every cache last cycle
updater_done = m.add(Register("velocity-updater-done"))

# velocity_update_controller inputs
CTL_VELOCITY_UPDATE_READY = velocity_update_controller.ctl_velocity_update_ready
connect(updater_done.o, velocity_update_controller.updater_done)

# velocity_updater inputs
for i in range(N_CELL):
    connect(a_caches[i].o, velocity_updater.a[i])
    connect(v_caches[i].o, velocity_updater.vi[i])

# updater_done input
connect(velocity_updater.updater_done, updater_done.i)

# a_muxes input
for i in range(N_CELL):
    connect(velocity_update_controller.oaddr, a_muxes[i].oaddr_p2)
    connect(null_const.o, a_muxes[i].iaddr_p2)
    connect(null_const.o, a_muxes[i].i_p2)

# v_muxes input
for i in range(N_CELL):
    connect(velocity_update_controller.oaddr, v_muxes[i].oaddr_p2)
    connect(velocity_update_controller.oaddr, v_muxes[i].iaddr_p2)
    connect(velocity_updater.vo[i], v_muxes[i].i_p2)

# control_unit inputs
CTL_VELOCITY_UPDATE_DONE = velocity_updater.updater_done
