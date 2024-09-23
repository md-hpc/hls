from abc import ABC, abstractmethod
from collections import deque

class Input:
    def __init__(self, parent):
        self.val = None
        self.parent = parent
        self.output = None

    def __call__(self):
        if self.val is None:
            self.val = self.output()
        return self.val
    
    def connected(self):
        return self.output is not None

    def reset(self):
        self.val = None

    def get(self):
        return self()
    
class Output:
    def __init__(self, parent):
        self.val = None
        self.parent = parent
        self.inputs = []

    def __call__(self):
        if self.val is None:
            self.parent()
        return self.val

    def set(self,val):
        if self.val is not None:
            raise RecursionError(f"The output of {type(self.parent)} is being set twice")
        self.val = val

    def connected(self):
        return len(self.inputs) != 0

    def reset(self):
        self.val = None

class Register:
    def __init__(self, write_enable_port = False):
        self._called = False
        self.content = 0
        self.i = Input(self)
        self.o = Output(self)
        self.write_enable = Input(self) if write_enable_port else None

    def write(self):
        if (self.write_enable is None or self.write_enable()):
            self.content = self.i()

    def __call__(self):
        if self._called:
            return
        self._called = True

        self.o.set(self.content)
        if self.write_enable is not None:
            self.write_enable()

    def connected(self):
        msg = ""
        
        if not self.i.connected():
            msg += f"i"
        
        if not self.o.connected():
            msg += f"o"
        
        return "register: " + msg if len(msg) else None

    def reset(self):
        self.i.reset()
        self.o.reset()
        self._called = False

class BRAM:
    def __init__(self, size, ports=None, iports=None, oports=None):
        self._called = False

        self.contents = [0 for _ in range(size)]
        
        if ports is None and (iports is None or oports is None):
            raise Exception("BRAM.__init__: if ports is not specified, must specify both iports and oports")

        if iports is None:
            iports = ports

        if oports is None:
            oports = ports

        self.i = [Input(self) for _ in range(iports)]
        self.iaddr = [Input(self) for _ in range(iports)]

        self.o = [Output(self) for _ in range(oports)]
        self.oaddr = [Input(self) for _ in range(oports)]

    def write(self):
        for i, addr in zip(self.i, self.iaddr):
            self.contents[addr()] = i()

    def __call__(self):
        if self._called == True:
            return
        self._called = True

        for o, addr in zip(self.o, self.oaddr):
            o.set(self.contents[addr()])

    def connected(self):
        msg = ""
        for idx, i in enumerate(self.i):
            if not i.connected():
                msg += f"i[{idx}] "
        
        for idx, o in enumerate(self.o):
            if not o.connected():
                msg += f"o[{idx}] "

        for idx, i in enumerate(self.iaddr):
            if not i.connected():
                msg += f"iaddr[{idx}] "
        
        for idx, o in enumerate(self.oaddr):
            if not o.connected():
                msg += f"oaddr[{idx}] "
        
        return "BRAM: "+msg if len(msg) else None

    def reset(self):
        for i in self.i:
            i.reset()

        for o in self.o:
            o.reset()

        for i in self.iaddr:
            i.reset()

        for i in self.oaddr:
            i.reset()
        
        self._called = False

class Logic(ABC):
    def __init__(self):
        self._init = True
        self._called = False
        self._inputs = []
        self._outputs = []
        self._pipeline = deque([])

    def __setattr__(self, k, v):
        if k != "_init" and not hasattr(self,"_init"):
            raise Exception("Must call super().__init__() on hls.Logic before setting any attributes")

        if type(v) is Input:
            self._inputs.append(v)
        if type(v) is Output:
            self._outputs.append(v)
            self._pipeline = deque([[0 for _ in self._outputs] for _ in self._pipeline])

        super(Logic, self).__setattr__(k, v)

    @abstractmethod
    def logic(self):
        pass

    def pipeline(self, n):
        self._pipeline = deque([[0 for _ in self._outputs] for _ in range(n)])

    def __call__(self):
        if self._called == True:
            return
        self._called = True

        self.logic()
        
        for o in self._outputs:
            if o.val == None:
                raise Exception(f"Output not set after calling logic unit for {type(self)}")
       
        self._pipeline.append(
                [o.val for o in self._outputs]
        )
        for o, val in zip(self._outputs, self._pipeline.popleft()):
            o.val = val

    def connected(self):
        msg = ""
        for idx, port in enumerate(self._inputs):
            if not port.connected():
                msg += f"i[{idx}] "
        for idx, port in enumerate(self._outputs):
            if not port.connected():
                msg += f"o[{idx}] "
        return f"{type(self)}: " + msg if len(msg) else None
            
    def reset(self):
        for i in self._inputs:
            i.reset()

        for o in self._outputs:
            o.reset()

        self._called = False

class MockFPGA:
    def __init__(self):
        self.units = []

    def add(self, obj):
        t = type(obj)
        if not isinstance(obj, Logic) and t is not Register and t is not BRAM:
            raise TypeError(f"MockFPGA units must be either Logic, Register, or BRAM (got {t})")
        self.units.append(
                obj
        )
        return self.units[-1]
    
       
    def verify_connections(self):
        succ = True
        msgs = [unit.connected() for unit in self.units]
        for msg in msgs:
            if msg is not None:
                print(msg)
                succ = False
        
        if not succ:
            raise Exception("verify_connections: circuit not fully connected")
            
    def clock(self):
        for unit in self.units:
            unit()

        for unit in self.units:
            if type(unit) is BRAM or type(unit) is Register:
                unit.write()

        for unit in self.units:
            unit.reset()

def connect(o, i):
    if type(o) is not Output:
        raise TypeError("Connected output is not type Output()")
    if type(i) is not Input:
        raise TypeError("Connected input is not type Input()")
    if i.output is not None:
        raise ValueError(f"input belonging to {type(i.parent)} already has output belonging to {type(o.parent)}")
    o.inputs.append(i)
    i.output = o
