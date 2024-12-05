from numpy.linalg import norm
import numpy
import sys

from hls import *
from common import *
from compute_pipeline import ComputePipeline

# PHASE 1: position cache read/filter write AND filter read/force evaluator write

# an important note for phase 1: we use periodic boundary conditions in this simulation, meaning that
# if your reference cell is, say, (3, 3, 3) in a 4x4x4 universe (indexing starts at 0), then you will need
# to pair with particles from (3,0,0), (3,3,0), (0,0,3), etc. because they are considered adjacent by our
# model


# we use double buffering (every other phase we use the top half of the caches instead because of
# particle transfers that happen in the motion update phase). This will provide a bit (0 or 1) that will give
# an offset to add to all addresses used. 0 if it reads 0 and 256 if it reads 1
#
# e.g. if you're accessing address 13 of a givne BRAM, and the bit is 0, you will just access address 13
# but if bit is 1, you will access address 13 + 256 = 269 instead.
CTL_DOUBLE_BUFFER = Input 

# Force evaluation control
CTL_READY = Input # reads as 1 when force evaluation should begin/continue, otherwise 0
CTL_DONE = Output # write 1 when your units are done evaluating forces, otherwise write 0

class PositionReadController(Logic):
    def __init__(self):
        super().__init__("position-read-controller")
        self.ready = Input(self,"ready")
        self.done = Output(self,"done")

        self.finished_batch = Input(self,"finished-batch")
        self.finished_all = Input(self,"finished-all")
        self.in_flight = Input(self,"in-flight")

        self.dispatch = Output(self,"dispatch")
        self._startup = 0
    
    def logic(self):
        ready = self.ready.get()
        finished_all = self.finished_all.get()
        finished_batch = self.finished_batch.get()
        in_flight = self.in_flight.get()

        if not ready:
            self._startup = 0
            self.dispatch.set(NULL)
            self.done.set(False)
        else:
            if self._startup == 0:
                self.dispatch.set(True)
                self.done.set(False)
                self._startup = 1
            elif self._startup == 1:
                if not in_flight:
                    self.dispatch.set(True)
                    self.done.set(False)
                    self._startup = 2
                else:
                    self.dispatch.set(True)
                    self.done.set(False)
            elif finished_batch and finished_all:
                self.dispatch.set(False)
                self.done.set(True)
            else:
                if finished_batch and not in_flight:
                    self.dispatch.set(True)
                    self.done.set(False)
                else:
                    self.dispatch.set(False)
                    self.done.set(False)
                
class PositionRingNode(Logic):
    def __init__(self,cell):
        super().__init__(f"p-ring-node-{cell}")
        self._cell = cell

        # if the buffered neighbor particles should be dispatched
        self.double_buffer = Input(self,"double-buffer")
        self.dispatch = Input(self,"dispatch")

        self.prev = Input(self,"prev")
        self.next = Output(self,"next")
        self.bram_in = Input(self,"bram-in")

        self._neighbor_buffer = [RESET for _ in range(NSIZE)]
        self._i = 0

        self.neighbors = [Output(self,f"neighbor-{i}") for i in range(NSIZE)]
        self.reference = Output(self,"reference")

        self.addr = Output(self,"addr")
        self._addr_n = 0
        self._addr_r = 0
        self._ptype = "" # the type of particle that was fetched from the BRAM

        # if the current batch of neighbor cells have been processed
        self._done_batch = False
        self.done_batch = Output(self,"done-batch")
        # if this cell has no more neighbor cells to provide
        self._done_all = False
        self.done_all = Output(self,"done-all")
        # if this node transmitted a particle through the ring this cycle
        self.in_flight = Output(self,"in-flight")

    def logic(self):
        dispatch = self.dispatch.get()
        offset = db(self.double_buffer.get())

        prev = self.prev.get()
        bram_in = self.bram_in.get()
        
        if dispatch is NULL:
            self._addr_r = 0
            self._addr_n = 0
            self._done_batch = False
            self._done_all = False
            
            nul(self.next)
            nul(self.neighbors)
            nul(self.reference)
            nul(self.addr)
            nul(self.done_batch)
            nul(self.done_all)
            nul(self.in_flight)
            return
        
        if dispatch:
            if prev is not NULL:
                # there is no queue for the ring nodes, so
                # this would be catastrophic
                print("dispatch is true while prev is not null")
                exit(1)

            # all PEs are done with their current
            # batch of neighbor cells
            # time to dispatch a new batch
            for _n, n in zip(self._neighbor_buffer, self.neighbors):
                n.set(_n)

            # default each entry in our buffer to RESET
            # so when we write these to the neighbor registers,
            # we overwrite neighbors from last batch even if 
            # their cell didn't send anything
            self._neighbor_buffer = [RESET for _ in range(NSIZE)]
            self.reference.set(NULL)
            
            # fetch a new neighbor to be broadcasted
            if not self._done_all:
                self.addr.set(self._addr_n + offset)
            else:
                self.addr.set(RESET)

            # reset our batch state
            self._ptype = "n"
            self._i = 0
            self._addr_r = 0
            self._done_batch = False
        
            # nothin' to transmit here
            next_ = RESET

        elif self._ptype == "n":
            if prev is not NULL:
                # see same clause above
                print("uh oh!!")
                exit(1)

            if bram_in is NULL:
                self._done_all = True
                p = NULL
                next_ = RESET
            else:
                p = Position(cell = self._cell, addr = self._addr_n + offset, r = bram_in)
                self._addr_n += 1
                
                self._neighbor_buffer[self._i] = p
                self._i += 1
            
                next_ = p

            # fetch the first reference particle
            self.reference.set(NULL)
            self._ptype = "r"
            self.addr.set(self._addr_r)

        elif self._ptype == "r":
            if bram_in is NULL:
                self._done_batch = True
                p = NULL
            else:
                p = Position(cell = self._cell, addr = self._addr_r + offset, r = bram_in)
                self._addr_r += 1

            self.reference.set(p)
            self.addr.set(self._addr_r + offset)

            next_ = RESET
            if prev is not NULL and self._cell != prev.cell:
                if n3l_cell(self._cell, prev.cell):
                    if self._i == NSIZE:
                        print(f"neighbor buffer overflow in node {self._cell}")
                        exit(1)
                    
                    self._neighbor_buffer[self._i] = prev
                    self._i += 1
                next_ = prev
        
        if not dispatch:
            nul(self.neighbors)

        assert next_ is not NULL
        self.in_flight.set(next_ is not RESET)
        self.next.set(next_)
        self.done_all.set(self._done_all)
        self.done_batch.set(self._done_batch)
                
class VelocityRingNode(Logic):
    def __init__(self, cell):
        super().__init__(f"velocity-ring-node-{cell}")
        self._cell = cell

        self.reference = Input(self, "reference")
        self.neighbor = Input(self, "neighbor")
        self.fragment_out = Output(self,"fragment-out")
        self._queue_out = []
        self.addr = Output(self,"oaddr")

        self.rempty = Output(self,"rempty")
        self.prev = Input(self,"prev")
        self.next = Output(self,"next")
        self._queue_next = []

    def logic(self):
        fragments = [self.reference.get(), self.neighbor.get(), self.prev.get()]
        
        for fragment in fragments:
            if fragment is NULL:
                continue
            if fragment.cell == self._cell:
                self._queue_out.append(fragment)
            else:
                self._queue_next.append(fragment)

        self.rempty.set(
            all([len(q) == 0 for q in [self._queue_out, self._queue_next]])
        )
        if len(self._queue_out) != 0:
            v = self._queue_out.pop(0)
            self.fragment_out.set(v.v)
            self.addr.set(v.addr)
        else:
            self.fragment_out.set(NULL)
            self.addr.set(NULL)

        if len(self._queue_next) != 0:
            self.next.set(
                self._queue_next.pop(0)
            )
        else:
            self.next.set(RESET)


class Adder(Logic):
    def __init__(self,i):
        super().__init__(f"adder-{i}")
        
        self.a = Input(self,"a")
        self.b = Input(self,"b")
        self.o = Output(self,"o")
    
    def logic(self):
        a = self.a.get()
        b = self.b.get()
        if a is NULL or b is NULL:
            self.o.set(NULL)
        else:
            self.o.set(
                self.a.get() + self.b.get()
            ) 

position_read_controller = m.add(PositionReadController())

p_ring_nodes = [m.add(PositionRingNode(i)) for i in range(N_CELL)]
p_ring_regs = [m.add(Register(f"p-ring-reg-{i}")) for i in range(N_CELL)]
p_ring_addrs = [m.add(Register(f"p-ring-addr-{i}")) for i in range(N_CELL)]

finished_batch = m.add(Register("finished-batch"))
finished_batch_AND = m.add(And(N_CELL,"finished-batch-AND"))

finished_all = m.add(Register("finished-all"))
finished_all_AND = m.add(And(N_CELL,"finished-all-AND"))

in_flight = m.add(Register("in-flight"))
in_flight_OR = m.add(Or(N_CELL,"in-flight-AND"))

compute_pipelines = [ComputePipeline(i, position_read_controller, m) for i in range(N_CELL)]

v_ring_nodes = [m.add(VelocityRingNode(i)) for i in range(N_CELL)]
v_ring_regs = [m.add(Register(f"v-ring-reg-{i}")) for i in range(N_CELL)]

v_adders = [m.add(Adder(f"v-{i}")) for i in range(N_CELL)]

# 1 signal from position read, N_CELL signals from compute pipelines,
# and N_CELL signals from velocity ring nodes
done = m.add(And(1 + N_CELL + N_CELL, "phase1-done-AND"))

# position_read_controller inputs
CTL_READY = position_read_controller.ready
connect(finished_batch.o, position_read_controller.finished_batch)
connect(finished_all.o, position_read_controller.finished_all)
connect(in_flight.o, position_read_controller.in_flight)

# p_ring_nodes inputs
CTL_DOUBLE_BUFFER = [node.double_buffer for node in p_ring_nodes]

for cell, cache, imux, omux, addr, node in zip(range(N_CELL), p_caches, p_imuxes, p_omuxes, p_ring_addrs, p_ring_nodes):
    # p_ring_nodes inputs
    connect(position_read_controller.dispatch, node.dispatch) 
    connect(cache.o, node.bram_in)
    connect(p_ring_regs[cell-1].o, node.prev)

    # p_ring_regs inputs
    connect(node.next, p_ring_regs[cell].i)

    # p_ring_addrs inputs
    connect(node.addr, addr.i)

    # p_caches inputs
    connect(null_const.o, imux.iaddr_phase1)
    connect(null_const.o, imux.i_phase1)
    connect(addr.o, omux.oaddr_phase1)

# finished_{batch,all}_AND inputs
for node, i_batch, i_all, i_in_flight in zip(p_ring_nodes, finished_batch_AND.i, finished_all_AND.i, in_flight_OR.i):
    connect(node.done_batch, i_batch)
    connect(node.done_all, i_all)
    connect(node.in_flight, i_in_flight)

# finished_{batch,all} inputs
connect(finished_batch_AND.o, finished_batch.i)
connect(finished_all_AND.o, finished_all.i)
connect(in_flight_OR.o, in_flight.i)

# compute_pipelines input
for node, pipeline in zip(p_ring_nodes, compute_pipelines):
    connect(node.reference, pipeline.reference)
    for o, i in zip(node.neighbors, pipeline.neighbors):
        connect(o,i)

for cell, pipeline, node, adder, omux, cache, imux in zip(range(N_CELL), compute_pipelines, v_ring_nodes, v_adders, v_omuxes, v_caches, v_imuxes):
    # v_ring_nodes inputs
    connect(pipeline.neighbor_out, node.neighbor)
    connect(pipeline.reference_out, node.reference)
    connect(v_ring_regs[cell-1].o, node.prev)

    # v_ring_regs inputs
    connect(node.next, v_ring_regs[cell].i)

    # v_adders inputs
    connect(node.fragment_out, adder.a)
    connect(cache.o, adder.b)

    # v_caches inputs
    connect(node.addr, omux.oaddr_phase1)
    connect(node.addr, imux.iaddr_phase1)
    connect(adder.o, imux.i_phase1)

# done inputs
i = 0
connect(position_read_controller.done, done.i[i])
i += 1
for di in range(N_CELL):
    connect(compute_pipelines[di].done, done.i[i+di])
i += N_CELL
for di in range(N_CELL):
    connect(v_ring_nodes[di].rempty, done.i[i+di])

# control_unit inputs
CTL_DONE = done.o
