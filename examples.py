from hls import Logic, Register, BRAM
from hls import Input, Output, connect
from hls import MockFPGA

class Accumulator(Logic):
    def __init__(self):
        super().__init__()

        self.last = Input(self)
        self.addr = Input(self)
        self.this = Input(self)
        
        self.sum = Output(self)
        self.nextAddr = Output(self)

    def logic(self):
        last = self.last.get()
        addr = self.addr.get()
        this = self.this.get() 

        self.sum.set(last + this)
        self.nextAddr.set(addr + 1)

class Incrementor(Logic):
    def __init__(self, pipeline_stages = 0):
        super().__init__()

        self.last = Input(self)
        self.next = Output(self)
        self.pipeline(pipeline_stages)

    def logic(self):
        self.next.set(
                self.last.get() + 1
        )

'''
Some examples of how to use this library
'''

# incrementor. Just increments the value in the register every cycle
# if pipeline_stages is set to a non-zero value, the increment will
# experience that many cycles of delay
m = MockFPGA()
reg = m.add(Register())
adder = m.add(Incrementor(pipeline_stages = 0))
connect(reg.o,adder.last)
connect(adder.next, reg.i)

for i in range(100):
    m.clock()
    print(reg.contents)

# adder
# sums over the memory such that mem[i] is the sum of mem[0] to mem[i]

m = MockFPGA()

mem = m.add(BRAM(size=100, ports=1))
mem.contents = [1 for _ in range(100)]

last = m.add(Register())
ptr = m.add(Register())
adder = m.add(Accumulator())

# sum -> last (reg) -> last
connect(adder.sum, last.i)
connect(last.o, adder.last)

# addr+1 -> ptr -> addr
connect(adder.nextAddr, ptr.i)
connect(ptr.o, adder.addr)

# mem[ptr] -> this
connect(ptr.o, mem.oaddr[0])
connect(mem.o[0], adder.this)

# sum -> mem[ptr]
connect(ptr.o, mem.iaddr[0])
connect(adder.sum, mem.i[0])

m.verify_connections() # checks that no Inputs or Outputs are left dangling
for i in range(100):
    print(mem.contents)
    m.clock()
