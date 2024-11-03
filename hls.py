from abc import ABC, abstractmethod
from collections import deque

NULL = []

class Input:
    def __init__(self, parent, name):
        self.val = None
        self.parent = parent
        self.name = name
        self.output = None

    def __call__(self):
        if self.val is None:
            self.val = self.output()
        assert self.val is not None, f"{self.name} read None from Output {self.output.name}"
        return self.val
    
    def connected(self):
        return self.output is not None

    def reset(self):
        self.val = None

    def get(self):
        return self()
    
class Output:
    def __init__(self, parent, name):
        self.val = None
        self.parent = parent
        self.name = name
        self.inputs = []

    def __call__(self):
        if self.val is None:
            self.parent()
        assert self.val is not None, f"{self.name}.set() was not invoked with non-None value by {type(self.parent)}"
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
    def __init__(self, name):
        self._called = False
        self.contents = NULL
        self.i = Input(self, f"{name} i")
        self.o = Output(self, f"{name} o")

    def write(self):
        x = self.i()
        if x is not NULL:
            self.contents = self.i()

    def __call__(self):
        if self._called:
            return
        self._called = True

        self.o.set(self.contents)

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
    def __init__(self, size, name):
        self._called = False

        self.contents = [NULL for _ in range(size)]

        self.i = Input(self, f"{name} i")
        self.iaddr = Input(self, f"{name} iaddr")

        self.o = Output(self, f"{name} o")
        self.oaddr = Input(self, f"{name} oaddr")

    def write(self):
        i = self.i()
        iaddr = self.iaddr()
        if x is not NULL and addr is not NULL:
            self.contents[iaddr] = i

    def __call__(self):
        if self._called == True:
            return
        self._called = True
        oaddr = self.oaddr()
        if oaddr is not NULL:
            self.o.set(self.contents[oaddr])
        else:
            self.o.set(NULL)

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
        self.i.reset()
        self.o.reset()
        self.iaddr.reset()
        self.oaddr.reset()
        
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
            self._pipeline = deque([[NULL for _ in self._outputs] for _ in self._pipeline])

        super(Logic, self).__setattr__(k, v)

    @abstractmethod
    def logic(self):
        pass

    def pipeline(self, n):
        self._pipeline = deque([[NULL for _ in self._outputs] for _ in range(n)])
        if not hasattr(self,"empty"):
            self.empty = Output(self, f"{type(self)} empty")

    def __call__(self):
        if self._called == True:
            return
        self._called = True

        self.logic()
        self.empty.val = 0 # satisfy assertion below
        for o in self._outputs:
            if o.val == None:
                raise Exception(f"Output not set after calling logic unit for {type(self)}")
       
        self._pipeline.append(
                [o.val for o in self._outputs]
        )
        self.empty.val = 1 if len(self._pipeline) == 0 else 0
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
