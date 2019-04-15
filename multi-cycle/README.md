# Multi-cycle MIPS in Verilog

Notation:

- **Hardware components** are in bold.

- `Data` and `Signal` are in code block.

  

## Difference from Single-cycle

The multi-cycle processor breaks a single instruction into multiple shorter steps. In each short step, the processor can read or write the memory or register file or use the ALU.

### Added Internal Registers

Registers are added to hold data that is generated in an early cycle but used in a later cycle. The following registers are added.

- Instruction register **IR**
- Memory data register **MDR**
- ALU input registers **A** and **B**
- **ALUOut** register

### PC Updated Twice

PC may be updated twice. The first update is just a simple increment 4. The second is a branch or jump update.

## Datapath

- Clock signal rises, updating **Register File** if `RegWrite` is asserted, updating **PC Register** with PC computed out by the previous cycle, which is controlled by `PCSrc`.

  Fetch state: fetching instruction from **Memory**, making a prediction for next PC, e.g. PC + 4, and enabling the `PCWrite`.

- Clock signal rises, updating **IR** with the instruction fetched from **Memory**, and updating **PC Register** with the predicted PC.

  Decode state: obtaining next state w.r.t instruction and reading **Register File**.

- Clock signal rises, read contents from **Register File**, `ReadData1` and `ReadData2`, are stored into ALU input registers **A** and **B** respectively, providing potential operands for ALU.

  Execute state (if necessary): performing ALU operations.

- Clock signal rises, updating **ALUOut** with the result from ALU.

  Memory state: Reading from or writing to **Memory** 

- Clock signal rises, updating **Memory** if `MemWrite` is asserted.

  Writeback state: sending data to `WriteData` port and destination register to `WriteAddr`  port of **Register File** 

### Datapath for R





## Signals

PCWrite

MemWrite

IRWrite

RegWrite

ALUControl

ALUSrcA

ALUSrcB

## Pipelined Multi-cycle Design

