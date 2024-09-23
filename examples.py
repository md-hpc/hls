from hls import Logic, Register, BRAM
from hls import Input, Output, connect
from hls import MockFPGA

class Accumulator(Logic):
    def __init__(self):
        super().__init__()

        self.last = Input(self)
        self.this = Input(self)
        self.addr = Input(self)
        
        self.sum = Output(self)
        self.nextAddr = Output(self)

    def logic(self):
        last = self.last.get()
        this = self.this.get()
        addr = self.addr.get()

        self.sum.set(last + this)
        self.nextAddr.set(addr + 1)

class Incrementor(Logic):
    def __init__(self):
        super().__init__()

        self.last = Input(self)
        self.next = Output(self)
        self.pipeline(8)

    def logic(self):
        self.next.set(
                self.last.get() + 1
        )

if __name__ == "__main__":
    '''
    Some examples of how to use this library
    '''

    # incrementor
    m = MockFPGA()
    reg = m.add(Register())
    adder = m.add(Incrementor())
    connect(reg.o,adder.last)
    connect(adder.next, reg.i)

    for i in range(100):
        m.clock()
        print(reg.content)

    # adder
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
        m.clock()
        print(mem.contents)
