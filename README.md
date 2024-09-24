Mock HLS library for python. This library allows you to prototype different memory and interconnect configurations for hardware projects.        
                                                                                                                                                                                                                       Usage:                                                                                                                  This library defines 3 classes that represent 3 types of hardware: Register, BRAM, and Logic.                

It also defines two module IO classes: Input and Output. These are used to connect the different components and are referred to generally as "ports"

These are all collected and simulated by a MockFPGA object.

# Logic

class Logic, as the name implies, represents the logic circuits of the FPGA. The subclasses represent different kinds of logic circuits you'd find on the FPGA. For our project, these would be things like particle pair filters, force pipelines, and multiplexors. Logic is an abstract class, and it's subclasses must implement two methods: __init__() and logic()

Logic.__init__(): is used for two things: declaring IO and pipeline length. Each IO must be an attribute of class Input() or Output(). Pipeline length can be declared by calling the pipeline() method.
Logic.logic(): actually implements the logic of your class. It takes no parameters and returns no value. Instead, IO is performed by calling the inputs and outputs declared in `__init__`. To access a Logic's io, use the get() and set() methods for input and output, respectively.

All `Input.get()` calls within a method must be made before any `Output.set()` calls. Otherwise, circular data dependencies that deadlock your system can easily be formed.

When you make a logic unit pipelined with self.pipeline(n), it will cause the output to lag the input by n cycles.


# Register
Both registers and memory give your mock FPGA state. Registers only have a single value and do not require addressing, making them easier to use.

A register is a single value with an Input (i) and Output (o) associated with it. At the beginning of each cycle, it will write it's stored value to it's output, and and the end of each cycle it will write the input to it's stored value.

By passing write_enable=True to Register's initializer, it will gain an addition write-enable Input called write_enable.

You initialize, read, and modify the register's value by accessing the content attribute.

# BRAM
BRAM is the general memory of the mock FPGA. It's an array of length `size` and port count `ports` (input ports and output ports may be specified separately).

Each port is composed of one data port and one address port. The address port indexes into the memory array while the data port is the actual data I/O.

The ports are organized into four arrays. iaddr[x] is the address port for the input i[x]. oaddr[x] is the address port for the output o[x].

Like registers, the memory contents may be initialized, read, and modified by accessing the "contents" attribute.

# connections
To actually wire the FPGA together, call the `connect` function on individual ports. e.g.:

```
reg = Register()
mem = BRAM(size=100,ports=2)
connect(reg.o, mem.oaddr[1])
```

# MockFPGA
To run your circuit, instantiate the MockFPGA object and add your components via the add() method. Then, each call to MockFPGA.clock() will simulate a single cycle of your circuit's operation.
