from hls import hls
import random
from collections import deque

UNIVERSE_SIZE = 4
PARTICLES_PER_CACHE = 120
CUTOFF = 8.0
FORCE_PIPELINE_STAGES = 70
FILTER_PIPELINE_STAGES = 9
FILTER_BANK_SIZE = 13

NULL = -1

N_CELLS = UNIVERSE_SIZE ** 3

'''
I'm going to call a particle's (cell, addr) tuple it's "origin"
'''

class Position:
    def __init__(self, r, addr, cell):
        self.cell = cell
        self.addr = addr
        self.r = r

class Velocity:
    def __init__(self, v, addr, cell):
        self.cell = cell
        self.addr = addr
        self.v = v

class Acceleration:
    def __init__(self, a, addr, cell):
        self.cell = cell
        self.addr = addr
        self.a = a

def linear_idx(i,j,k):
    return (i%UNIVERSE_SIZE) + (j%UNIVERSE_SIZE)*UNIVERSE_SIZE + (k%UNIVERSE_SIZE)*UNIVERSE_SIZE**2

def cell_idx(i):
    return i%UNIVERSE_SIZE, i//UNIVERSE_SIZE%UNIVERSE_SIZE, i//UNIVERSE_SIZE**2%UNIVERSE_SIZE

class CellIndex:
    def __init__(self,start=0,stop=UNIVERSE_SIZE):
        self.start = start
        self.stop = stop

    def __iter__(self):
        coordinate = [self.start for _ in range(3)]
        while not all([i == self.stop-1 for i in coordinate]):
            for i in range(len(coordinate)):
                coordinate[i]=(coordinate[i]+1)%self.stop
                if coordinate[i]:
                    yield coordinate

class Queue(hls.Logic):
    def __init__(self, fan_in): 
        self.i = [hls.Input(self) for _ in range(fan_in)]
        self.q = []
        self.o = hls.Output(self)

    def logic(self):
        for i in self.i:
            a = i.get()
            if a != NULL:
                self.q.append(a)
        
        if len(self.q) == 0:
            self.o.set(NULL)
        else:
            self.o.set(self.q.pop(0))

class Adder(hls.Logic):
    def __init___(self, fan_in):
        self.i = [hls.Input(self) for _ in range(fan_in)]
        self.o = hls.Output(self)

    def logic(self):
        s = 0
        for i in self.i:
            s += i.get()
        self.o.set(s)
        
class Mux(hls.Logic):
    def __init__(self, fan_in):
        self.i = [hls.Input(self) for _ in range(fan_in)]
        self.select = hls.Input(self)
        self.o = hls.Output(self)

    def logic(self):
        idx = self.select.get()
        self.o.set(
                self.i[idx].get()
        )



class ParticleFilter(hls.Logic):
    def __init__(self):
        super().__init__()

        self.reference = hls.Input(self)
        self.neighbor = hls.Input(self)

        # pauses computation. Used when switching reference particles
        self.halt = hls.Input(self) 

        self.pair = hls.Output(self)

        self.pipeline(FILTER_PIPELINE_STAGES)

    def logic(self):
        r = self.reference.get()
        n = self.neighbor.get()
        halt = self.halt.get()

        if halt:
            self.pair.set(NULL)
            return

        r = math.sqrt(sum([
            (r1 - r2)**2 for r1, r2 in zip(r.r, n.r)
        ]))

        self.pair.set(
                (r, n) if r < CUTOFF else NULL
        )
 
 class ForcePipeline(hls.Logic):
    def __init__(self):
        super().__init__()

        self.i = hls.Input(self)

        self.reference = hls.Output(self)
        self.neighbor = hls.Output(self)
        
        self.pipeline(FORCE_PIPELINE_STAGES)

    def logic(self):
        i = self.i.get()
        if i == 0:
            self.reference.set(NULL)
            self.neighbor.set(NULL)
            return

        ref, neighbor = self.i.get()
        f = random.random() - 0.5
        


        self.reference.set(
            Acceleration(cell = ref_cell, addr = ref_addr, a = f)
        )

        self.neighbor.set(
            Acceleration(cell = neighbor_cell, addr = neighbor_addr, a = -f)
        )

class PositionCacheReadAddresser(hls.Logic):
    def __init__(self):
        self.cell_r = Input(self)
        self.addr_r = Input(self)
        self.addr_n = Input(self)
       

        self.ctl_next_reference = Input(self) # set when idx_r should be used. Used when pairings for current reference are done

        self.oaddr = [Output(self) for _ in range(N_CELLS)]
        self.addr = Output(self)

    def logic(self):
        cell_r = self.cell_r.get()
        addr_r = self.addr_r.get()
        addr_n = self.addr_n.get()

        ctl_next_reference = self.ctl_next_reference.get()

        addr = addr_r if ctl_next_reference else addr_n
        
        for oaddr in self.oaddr:
            oaddr.set(addr)
        self.addr.set(addr)
        
class PositionCacheReader(hls.Logic):
    def __init__(self):
        self.i = [Input(self) for _ in range(N_CELLS)]

        # Reader needs these to create Position objects
        self.cell_r = Input(self) # reference particle cell
        self.addr = Input(self) # address used by PostionCacheReadAddresser for this cycle

        self.o = [Ouptut(self) for _ in range(N_PIPELINE)]
        self.r = self.o[0] # output of the reference cell, for convenience

    def logic(self):
        cell_r = self.cell_r.get()
        addr = self.addr.get()

        i, j, k = cell_idx(cell_r)
        
        idx = 0
        for di in range(-1,2):
            for dj in range(-1,2):
                for dk in range(-1,2):
                    if di < 0 or di == 0 and dj < 0 or di == 0 and dj == 0 and dk < 0:
                        continue
                    cell = linear_idx(i+di, j+dj, k+dk)
                    r = self.i[cell].o.get()
                    p = Particle(r = r, addr = addr, cell = cell)

                    self.o[idx].set(p)

class PositionCacheReadController(hls.Logic):
    def __init__(self):
        self.i = [Input(self) for _ in range(N_PIPELINE)]
        
        self.addr_ri = Input(self)
        self.addr_ro = Output(self)
        
        self.cell_ri = Input(self)
        self.cell_ro = Output(self)

        self.addr_ni = Input(self)
        self.addr_no = Output(self)

        self.ctli_next_reference = Input(self) # hold value of below
        self.ctlo_next_reference = Output(self) # set when current reference particle pairings are complete 
        
        self.ctli_motion_update = Input(self) # hold value of below
        self.ctlo_motion_update = Output(self) # set when pairings for this timestep have completed

        self.ctli_next_timestep = Input(self) # set by MUU when motion update is complete

    def logic(self):
        ctl_next_reference = self.ctli_next_reference.get()
        ctl_motion_update = self.ctli_motion_update.get()
        ctl_next_timestep = self.ctli_next_timestep.get()

        addr_r = self.addr_ri.get()
        cell_r = self.cell_ri.get()
        addr_n = self.addr_ni.get()

        # if we are in the middle of a motion update
        if ctl_motion_update and not ctl_next_timestep:
            self.addr_ro.set(0)
            self.cell_ro.set(0)
            self.addr_no.set(0)
            self.ctlo_next_reference.set(1) # this must be set to true so that the next read cycle will be read into reference registers
            self.ctlo_motion_update.set(1)

        # if we tried to access the next reference particle, but there are none left in the cell
        elif ctl_next_particle and self.i[cell_r].get() == 0:

            # if there are more reference cells left for this timestep
            if cell_r < N_CELLS:
                # select a new reference cell
                self.addr_ro.set(0)
                self.cell_ro.set(cell_r)
                self.addr_no.set(0)

                self.ctlo_next_reference.set(1)
                self.ctlo_next_timestep.set(0)
            else:
                # signal motion update
                self.addr_ro.set(0)
                self.cell_ro.set(0)
                self.addr_no.set(0)

                self.ctlo_next_reference.set(1)
                self.ctlo_next_timestep.set(1)
        # if there are no more particles left to pair
        elif all([i.get() == 0 for i in self.i]):
            # get a new reference particle
            self.addr_ro.set(addr_r + 1)
            self.cell_ro.set(cell_r)
            self.addr_no.set(0)

            self.ctlo_next_reference.set(1)
            self.ctlo_next_timestep.set(0)
        else:
            # get the next batch of neighbor particles
            self.addr_ro.set(addr_r)
            self.cell_ro.set(cell_r)
            self.addr_no.set(addr_n + 1)

            self.ctlo_next_reference.set(0)
            self.ctlo_next_reference.set(0)

m = hls.MockFPGA()

init_caches = lambda: [m.add(hls.BRAM(size = PARTICLES_PER_CACHE)) for _ in range(N_CELLS)]

p_caches = init_caches()
v_caches = init_caches()
a_caches = init_caches()

'''
Something to initialize everything here
'''

positionCacheReadAddresser = m.add(PositionCacheReadAddresser())
positionCacheReader = m.add(PositionCacheReader())
positionCacheReadController = m.add(PositionCacheReadController())

addr_r = m.add(hls.Register())
cell_r = m.add(hls.Register())
addr_n = m.add(hls.Register())

ctl_next_reference = m.add(hls.Register())
ctl_motion_update = m.add(hls.Register())
ctl_next_timestep = m.add(hls.Register())

# positionCacheReadController controls all position cache read registers
hls.connect(positionCacheReadController.addr_ro, addr_r.i)
hls.connect(positionCacheReadController.cell_ro, cell_r.i)
hls.connect(positionCacheReadController.addr_no, addr_n.o)
hls.connect(positionCacheReadController.ctlo_next_reference, ctl_next_reference.i)
hls.connect(positionCacheReadController.ctlo_motion_update, ctl_motion_update.i)

# p_cache read inputs
for oaddr, p_cache in zip(positionCacheReadAddresser.oaddr, p_caches):
    hls.connect(oaddr, p_cache.oaddr)

# positionCacheReadAddresser inputs
hls.connect(cell_r.o, positionCacheReadAddresser.cell_r)
hls.connect(addr_r.o, positionCacheReadAddresser.addr_r)
hls.connect(addr_n.o, positionCacheReadAddresser.addr_n)
hls.connect(ctl_next_reference.o, positionCacheReadAddresser.ctl_next_reference)

# positonCacheReader inputs
for p_cache, i in zip(p_caches, positionCacheReader.i):
    hls.connect(p_cache.o, i)
hls.connect(cell_r.o, positionCacheReader.cell_r)
hls.connect(positionCacheReadAddresser.addr, positionCacheReader.addr)
hls.connect(cell_r.o, positionCacheReader.cell_r)

# positionCacheReadController inputs
for p_cache, i in zip(p_caches, positionCacheReadController.i):
    hls.connect(p_cache.o, i)
hls.connect(addr_r.o, positionCacheReadController.addr_ri)
hls.connect(cell_r.o, positionCacheReadController.cell_ri)
hls.connect(addr_n.o, positionCacheReadController.addr_ni)
hls.connect(ctl_next_reference.o, positionCacheReadController.ctli_next_reference)
hls.connect(ctl_motion_update.o, positionCacheReadController.ctli_motion_update)
hls.connect(ctl_next_timestep.o, positionCacheReadController.ctli_next_timestep)

# FORCE COMPUTATION
def PairQueue(hls.Logic):
    def __init__(self):
        super().__init__()

        self.i = [hls.Input(self) for _ in range(N_FILTERS)]

        self.queue = []

        self.o = hls.Output()

    def logic(self):
        for i in self.i:
            pair = self.i.get()
            if pair == 0:
                continue
            self.queue.append(pair)
        
        self.o.set(
                self.queue.pop(0)
        )

forcePipeline = m.add(ForcePipeline())
filters = [m.add(ParticleFilter()) for _ in range(N_FILTERS)]
pairQueue = m.add(PairQueue())

reference = m.add(hls.Register(write_enable_port = True))

# Reference particle register inputs
hls.connect(positionCacheReader.r, reference_particle.i)
hls.connect(positionCacheReadController.ctlo_next_reference, reference_particle.write_enable)

# Filter inputs
for o, f in zip(positionCacheReader.o, filters):
    hls.connect(o, f.reference)
    hls.connect(reference.o, f.reference)
    hls.connect(positionCacheReadController.ctlo_next_reference, f.halt)

# pairQueue inputs
for f, i in zip(filters, pairQueue.i):
    hls.connect(f.pair, i)

# forcePipeline inputs
hls.connect(pairQueue.o, forcePipeline.i)

# FORCE WRITEBACK
class ReferenceAccelerationAccumulator(hls.Logic):
    def __init__(self):
        self.f = hls.Input(self)
        self.ref_regi = hls.Input(self)

        self.ref_rego = hls.Output(self)
        self.ref_writeback = hls.Output(self)

    def logic(self):
        ref_regi = self.ref_regi.get()
        f = self.f.get()
        
        if f.addr != ref_regi.addr or f.cell != ref_regi.cell:
            self.o.set(f)
            self.ref_writeback.set(ref_regi)
        else:
            self.o.set(
                    Acceleration(addr = a1.addr, cell = a1.cell, a = a1.a + a2.a)
            )
            self.ref_writeback.set(0)

class AccelerationWriteAddresser(hls.Logic):
    def __init__(self):
        self.a = hls.Input(self)

        self.addr = hls.Output(self)
        self.cell = hls.Output(self)

        self.write_enable = [hls.Output(self) for _ in range(N_CELL)] # wire once to each a_cache

    def logic(self):
        a = self.a.get()

        if a == NULL:
            for o in self.write_enable:
                o.set(0)
            self.addr.set(0)
            return

        cell = a.cell
        addr = a.addr

        self.addr.set(addr)
        self.cell.set(cell)

        for idx, o in enumerate(self.write_enable):
            if idx == cell:
                o.set(1)
            else:
                o.set(0)

class AccelerationWriter(hls.Logic):
    def __init__(self):
        self.a = hls.Input(self)
        self.o = [Output(self) for _ in range(N_CELL)]

    def logic(self):
        accel_r_reg = self.force_r.regi.get()
        accel_r_pipeline = self.force_r_pipeline.get()
        accel_n_pipeline = self.force_n_pipeline.get()

        if not ctl_next_reference:
            for o in self.o:
                o.set(accel_n_pipeline.a)
            self.accel_r_rego.set(accel_r_reg)
        else:
            for o in self.o:
                o.set(accel_r_reg.a)
            self.accel_r_rego.set(Acceleration(cell = self.cell_r.get(), addr = self.addr_r.get(), a = 0))

class ReferenceForceWriter(hls.Logic):
    def __init__(self):
        self.reg_i = hls.Input(self) # reference force from accumulator register
        self.pipeline_i = hls.Input(self) # reference force from pipeline

        self.ctli_next_timestep = hls.Input(self)
        self.next_timestep = False

        self.reg_o = hls.Output(self)
        self.bram_o = hls.Output(self)

    def logic(self):
        reg = self.reg_i.get()
        pipeline = self.pipeline_i.get()
        ctl_next_timestep = self.ctli_next_timestep.get()

        if ctl_next_timestep:
            self.next_timestep = True
            self.reg_o.set(NULL)
            self.bram_o.set(NULL)
        elif pipeline == NULL:
            self.reg_o.set(reg)
            self.bram_o.set(NULL)
        elif self.next_timestep:
            self.next_timestep = False
            self.reg_o.set(pipeline)
            self.bram_o.set(NULL)
        elif pipeline.addr != reg.addr or pipeline.cell != reg.cell:
            self.reg_o.set(pipeline)
            self.bram_o.set(reg)
        else:
            reg.a += pipeline.a
            self.reg_o.set(reg)
            self.bram_o.set(NULL)

referenceForceWriter = m.add(ReferenceForceWriter())
forceQueue = m.add(Queue(2))
accelerationWriteAddresser = m.add(AccelerationWriteAddresser())
forceAccumulatorMux = m.add(Mux(N_CELL))
forceAccumulator = m.add(Adder(2))

reference_force = m.add(hls.Register())
hls.connect(referenceForceWriter.reg_o, reference_force.i)

# accelerationWriteAddresser inputs
hls.connect(reference_force.o, referenceForceWriter.reg_i)
hls.connect(forcePipeline.reference, referenceForceWriter.pipeline_i)
hls.connect(ctl_next_timestep.o, referenceForceWriter.ctli_next_timestep)

# forceQueue inputs
hls.connect(referenceForceWriter.bram_o, forceQueue.i[0])
hls.connect(forcePipeline.neighbor, forceQueue.i[1])

# accelerationWriteAddresser inputs
hls.connect(forceQueue.o, accelerationWriteAddresser.a)

# forceAccumulatorMux inputs
for idx in range(N_CELL):
    hls.connect(f_caches[idx].o, forceAccumulatorMux.i[idx])
hls.connect(accelerationWriteAddresser.cell, forceAccumulatorMux.select)

# forceAccumulator inputs
hls.connect(forceAccumulatorMux.o, forceAccumulator.i[0])
hls.connect(forceQueue.i, forceAccumulator.i[1])


# f_cache inputs
for idx in range(N_CELL):
    cache = f_caches[idx]

    hls.connect(accelerationWriteAddresser.addr, cache.oaddr)
    hls.connect(accelerationWriteAddresser.addr, cache.iaddr)
    hls.connect(accelerationWriteAddresser.write_enable[idx], cache.write_enable)

    hls.connect(forceAccumulator.o, cache.i)
    

