from hls import *
from structures import *

# PHASE 2: force pipeline read/acceleration cache write AND acceleration & velocity cache read AND velocity cache write

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

# Motion update control
CTL_VELOCITY_UPDATE_READY = Input # read as 1 when velocity update should begin/continue, otherwise 0
CTL_VELOCITY_UPDATE_DONE = Output # write 1 when your units are done evaluating velocity, otherwise write 0



class ReferenceAccumulator(Logic):
    def __init__(self):
        super().__init__("reference-accumulator")
        self.i = Input(self, "i")
        self.reset = Input(self, "reset")
        self._r = None
        self.o = Output(self, "o")

    def logic(self):
        i = self.i.get()
        reset = self.reset.get()
        
        if i is NULL or reset is NULL:
            return
        elif reset:
            self._r = i
        elif i.addr != self._r.addr or i.cell != self._r.cell:
            self.o.set(self._r)
            self._r = i
        else:
            self.o.set(NULL)
            self._r.a = [a1+a2 for a1, a2 in zip(self._r.a, i.a)]

class AccelerationQueue(Logic):
    def __init__(self):
        super().__init__("acceleration-queue")
        self.i = [Input(self,f"i{i}") for i in range(2)]
        self._queue = []
        self.o = Output(self, "o")
        self.qempty = Output(self,"qempty") # This will be our CLT_FORCE_EVALUATION_DONE

    def logic(self):
        for i in [i.get() for i in self.i]:
            if i is not NULL:
                self._queue.append(i)

        self.qempty.set(len(self._queue == 0))
        if len(self._queue):
            self.o.set(self._queue.pop(0))
        else:
            self.o.set(NULL)
        
class AccelerationCacheAdresser(Logic):
    def __init__(self):
        super().__init__("acceleraiton-cache-addresser")

        self.i = Input(self,"i")
        self.oaddr = Output(self, "oaddr")

    def logic(self):
        i = self.i.get()
        self.oaddr.set(i.addr)

class AccelerationCacheWriter(Logic):
    def __init__(self):
        super().__init__("acceleration-cache-writer")

        self.fragment = Input(self,"fragment")
        self.i = [Input(self, f"i{i}") for i in range(N_CELL)]
        self.o = [Output(self, f"o{i}") for i in range(N_CELL)]

    def logic(self):
        fragment = self.fragment.get()
        current_sum = self.i[fragment.cell].get()

        if fragment is NULL or current_sum is NULL:
            for o in self.o:
                o.set(NULL)
            return

        for idx, o in enumerate(self.o):
            if idx == fragment.cell:
                o.set([a1+a2 for a1, a2 in zip(fragment.a, current_sum)])
            else:
                o.set(NULL)

class VelocityUpdateController(Logic):
    def __init__(self):
        super().__init__("velocity-cache-controller")

        self.ctl_velocity_update_ready = Input(self,"velocity-update-ready")
        self.ctl_velocity_update_done = Output(self,"velocity-update-done")

        self.writer_done = Input(self, "writer-done")

        self._oaddr = 0
        self.oaddr = Output(self, "oaddr")

    def logic(self):
        ctl_velocity_update_ready = self.ctl_velocity_update_ready.get()

        if not ctl_velocity_update_ready:

