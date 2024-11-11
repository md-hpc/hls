from abc import ABC, abstractmethod
from collections import deque
import re

NULL = "NULL"
RESET = "RESET"

empty_ident = re.compile(".*/empty")

CONFIG_VERBOSE = False
dioi = 0
di = lambda: "."*dioi
def debug(*args, **kwargs):
    if CONFIG_VERBOSE:
        print(*args,**kwargs)

class Input:
    def __init__(self, parent, name):
        self.val = None
        self.parent = parent
        self.name = f"{parent.name}/{name}"
        self.output = None

        if isinstance(self.parent, Logic):
            self.parent._inputs.append(self)

    def __call__(self):
        global dioi
        dioi += 1
        debug(di() + "reading input", self.name)
        if self.val is None:
            self.val = self.output() 
        dioi -= 1
        assert self.val is not None, f"{self.name} read None from Output {self.output.name}"
        return self.val
    
    def connected(self):
        return self.output is not None

    def reset(self):
        self.val = None

    def get(self):
        return self()

    def adjacencies(self):
        return [self.output.name, self.name]

class Output:
    def __init__(self, parent, name):
        self.val = None
        self.parent = parent
        self.name = f"{parent.name}/{name}"
        self.inputs = []
        if isinstance(self.parent, Logic):
            self.parent._outputs.append(self)
            self.parent._pipeline = deque([[NULL for _ in self.parent._outputs] for _ in self.parent._pipeline])
    
    def __call__(self):
        debug(di() + "reading output", self.name)
        if self.val is None:
            self.parent()
        assert self.val is not None, f"{type(self.parent)} failed to set non-None value for {self.name}. Could be failure to invoke set() on {self.name} or cycle in graph"
        return self.val

    def set(self,val):
        debug("writing", self.name)
        assert self.val is None, f"{self.name} is being set twice"
        self.val = val

    def connected(self):
        return len(self.inputs) != 0

    def reset(self):
        debug("reset", self.name)
        self.val = None

class Register:
    def __init__(self, name):
        self._called = False
        self.name = name
        self.contents = NULL
        self.i = Input(self, f"i")
        self.o = Output(self, f"o")

    def write(self):
        i = self.i()
        if i is not NULL:
            if i is RESET:
                i = NULL
            self.contents = i

    def __call__(self):
        if self._called:
            return self.contents
        self._called = True
        
        self.o.set(self.contents)

    def identifiers(self):
        return [self.name, self.i.name, self.o.name]

    def adjacencies(self):
        return [
                self.i.adjacencies(),
                [self.i.name, self.name + "(i)"],
                [self.name + "(o)", self.o.name],
        ]

    def connected(self):
        if self.i.output is None:
            return [self.i.name]
        else:
            return []

    def reset(self):
        self.i.reset()
        self.o.reset()
        self._called = False

class BRAM:
    def __init__(self, size, name):
        self._called = False

        self.name = name
        self.contents = [NULL for _ in range(size)]

        self.i = Input(self, f"i")
        self.iaddr = Input(self, f"iaddr")

        self.o = Output(self, f"o")
        self.oaddr = Input(self, f"oaddr")
        
        self.verbose = False

    def write(self):
        i = self.i()
        iaddr = self.iaddr()
        if i is not NULL and iaddr is not NULL:
            if i is RESET:
                i = NULL
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

        if self.verbose:
            print(f"{self.name}:")
            print("\tINPUTS")
            print(f"\t\toaddr: {self.oaddr.val}")
            print("\tOUTPUTS")
            print(f"\t\to: {self.o.val}")


    def identifiers(self):
        return [self.name, self.i.name, self.iaddr.name, self.o.name, self.oaddr.name]

    def adjacencies(self):
        return [
                self.iaddr.adjacencies(),
                [self.iaddr.name, self.name+"(i)"],
                self.i.adjacencies(),
                [self.i.name, self.name+"(i)"],
                self.oaddr.adjacencies(),
                [self.oaddr.name, self.name+"(o)"],
                [self.name+"(o)", self.o.name]
        ]

    def connected(self):
        ret = []
        for i in [self.i, self.iaddr, self.oaddr]:
            if i.output is None:
                ret.append(i.name)
        return ret

    def reset(self):
        self.i.reset()
        self.o.reset()
        self.iaddr.reset()
        self.oaddr.reset()
        
        self._called = False


class Logic(ABC):
    def __init__(self, name):
        self._init = True
        self._called = False
        self._inputs = []
        self._outputs = []
        self._pipeline = deque([]) 
        self.name = name
        self.verbose = False
        self.debug = False
        self.empty = Output(self,"empty")
        self._n = 0 # number of inputs in pipeline

    def __setattr__(self, k, v):
        if k != "_init" and not hasattr(self,"_init"):
            raise Exception("Must call super().__init__() on hls.Logic before setting any attributes")

        super(Logic, self).__setattr__(k, v)

    @abstractmethod
    def logic(self):
        pass

    def pipeline(self, n):
        self._pipeline = deque([[NULL for _ in self._outputs] for _ in range(n)])
 
    def __call__(self):
        if self._called == True:
            return
        self._called = True
        debug(di() + f"Evaluating {self.name}")
        if self.debug:
            breakpoint()
        self.logic()
        self.empty.set(NULL)

        for o in self._outputs:
            passed = True
            if o.val is None:
                print(f"{o.name} is None after calling parent logic")
                passed = False
            if not passed:
                print(f"ERROR: Must set all Outputs to non-None in {self.name}.logic()")
                exit(1)

        self._pipeline.append(
                [o.val for o in self._outputs]
        )

        if any([o.val is not NULL for o in self._outputs]):
            self._n += 1

        for o, val in zip(self._outputs, self._pipeline.popleft()):
            o.val = val
 
        if any([o.val is not NULL for o in self._outputs]):
            self._n -= 1
 
        self.empty.val = self._n == 0

        if self.verbose:
            print(f"{self.name}:")
            print("\tINPUTS")
            for i in self._inputs:
                print(f"\t\t{i.name.split('/')[1]}: {i.val}")
            print("\tOUTPUTS")
            for o in self._outputs:
                print(f"\t\t{o.name.split('/')[1]}: {o.val}")


    def reset(self):
        debug("reset", self.name)
        for i in self._inputs:
            i.reset()

        for o in self._outputs:
            o.reset()

        self._called = False

    def identifiers(self):
        return [self.name] + [i.name for i in self._inputs] + [o.name for o in self._outputs]

    def adjacencies(self):
        return [i.adjacencies() for i in self._inputs] + [[i.name, self.name] for i in self._inputs] + [[self.name, o.name] for o in self._outputs]

    def connected(self):
        ret = []
        for i in self._inputs:
            if i.output is None:
                ret.append(i.name)
        return ret

class MockFPGA:
    def __init__(self):
        self.units = []
        self._init = False
        self.verbose = False

    def add(self, obj):
        t = type(obj)
        if not isinstance(obj, Logic) and t is not Register and t is not BRAM:
            raise TypeError(f"MockFPGA units must be either Logic, Register, or BRAM (got {t})")
        self.units.append(
                obj
        )
        return self.units[-1]
    
                 
    def clock(self):
        if not self._init:
            if not self.validate():
                print("Validation of FPGA failed")
                exit(1) 

        for unit in self.units:
            unit()

        for unit in self.units:
            if type(unit) is BRAM or type(unit) is Register:
                unit.write()

        for unit in self.units:
            unit.reset()
    
    def validate(self):
        if not self.validate_identifiers():
            print("validate_identifiers failed")
            exit(1)
        if not self.validate_connections():
            print("validate_connections failed")
            exit(1)
        if False and not self.validate_dag():
            print("validate_dag failed")
            exit(1)
        return True

    def validate_identifiers(self):
        identifiers = []
        [identifiers.extend(u.identifiers()) for u in self.units]
        counts = {ident: 0 for ident in identifiers}
        for ident in identifiers:
            counts[ident] += 1

        passed = True
        for ident, c in counts.items():
            if c != 1:
                passed = False
                print(f"{ident} is used {c} times (all identifiers should be unique)")
        return passed

    def validate_dag(self):
        E = []
        [E.extend(u.adjacencies()) for u in self.units]
        return dfs(E)
        
    def validate_connections(self):
        passed = True
        for u in self.units:
            msg = ", ".join(u.connected())

            if len(msg):
                print(f"Inputs not connected in {u.name}: {msg}")
                passed = False
        return passed

def connect(o, i):
    if type(o) is not Output:
        raise TypeError("Connected output is not type Output()")
    if type(i) is not Input:
        raise TypeError("Connected input is not type Input()")
    if i.output is not None:
        raise ValueError(f"{i.name} already receiving output from {o.name}")
    o.inputs.append(i)
    i.output = o

def dfs(E):
    V = set()
    for u, v in E:
        V.add(u)
        V.add(v)

    adj = {u: [] for u in V}
    for u, v in E:
        adj[u].append(v)

    color = {u: "w" for u in V}
    pi = {u: None for u in V}
    passed = True
    cycled = {u: False for u in V}

    def print_cycle(u):
        if cycled[u]:
            return
        passed = False
        msg = u
        cur = pi[u]
        while cur != u:
            cycled[cur] = True
            msg = f"{cur} -> {msg}"
            cur = pi[cur]
        print(msg)

    def visit(u):
        if color[u] == "g":
            print_cycle(u)
            return
        color[u] = "g"
        for v in adj[u]:
                pi[v] = u
                visit(v)
        color[u] = "b"

    for u in V:
        if color[u] == "w":
            visit(u)

    return passed

