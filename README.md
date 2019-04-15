# MIPS in Verilog

MIPS variances including single cycle, multi cycle, and pipeline.

## Single-cycle
```
single-cycle/
|
|-- mips-single-cycle.v: Design source
|-- testbench.v: Simulation source
|-- constrs.xdc: Constraints
|-- test.py: Checker in python3
|-- Instr_tb_behav.wdb: Waveform result of simulation
|-- lab1-single-cycle/: The Vivado project
```

## Instructions

* R-Type

Func | rs | rt | rd |  shift (shamt) | Opcode 
-|-|-|-|-|-
6 bits | 5 bits | 5 bits | 5 bits | 5 bits | 6 bits 

Instr | Func | Opcode | Format | Description
-| :-: | :-: |--- |--- 
SLL | 000000 | 000000 | `SLL rd, rt, sa` | `GPR[rd] ← GPR[rt] << sa` 
SRL | 000000 | 000010 | `SRL rd, rt, sa` | `GPR[rd] ← GPR[rt] >> sa (logical)` 
SRA | 000000 | 000011 | `SRA rd, rt, sa` | `GPR[rd] ← GPR[rt] >> sa (arithmetic)` 
SLLV | 000000 | 000100 | `SLLV rd, rt, rs` | `GPR[rd] ← GPR[rt] << GPR[rs]` 
SRLV | 000000 | 000110 | `SRLV rd, rt, rs` | `GPR[rd] ← GPR[rt] >> GPR[rs] (logical)` 
SRAV | 000000 | 000111 | `SRAV rd, rt, rs` | `GPR[rd] ← GPR[rt] >> GPR[rs] (arithmetic)` 
JR | 000000 | 001000 | `JR rs` | `PC ← GPR[rs]` 
JALR | 000000 | 001001 | `JALR rd, rs` | `GPR[rd] ← PC + 4, PC ← GPR[rs]`
ADD | 000000 | 100000 | `ADD rd, rs, rt` | `GPR[rd] ← GPR[rs] + GPR[rt]` 
ADDU | 000000 | 100001 | `ADDU rd, rs, rt` | `GPR[rd] ← GPR[rs] + GPR[rt]` 
SUB | 000000 | 100010 | `SUB rd, rs, rt` | `GPR[rd] ← GPR[rs] - GPR[rt]` 
SUBU | 000000 | 100011 | `SUBU rd, rs, rt` | `GPR[rd] ← GPR[rs] - GPR[rt]` 
AND | 000000 | 100100 | `AND rd, rs, rt` | `GPR[rd] ← GPR[rs] AND GPR[rt]` 
OR | 000000 | 100101 | `OR rd, rs, rt` | `GPR[rd] ← GPR[rs] OR GPR[rt]` 
XOR | 000000 | 100110 | `XOR rd, rs, rt` | `GPR[rd] ← GPR[rs] XOR GPR[rt]` 
NOR | 000000 | 100111 | `NOR rd, rs, rt` | `GPR[rd] ← GPR[rs] NOR GPR[rt]` 
SLT | 000000 | 101010 | `SLT rd, rs, rt` | `GPR[rd] ← (GPR[rs] < GPR[rt])` 
SLTU | 000000 | 101011 | `SLTU rd, rs, rt` | `GPR[rd] ← (GPR[rs] < GPR[rt])` 
MUL | 011100 | 000010 | `MUL rd, rs, rt` | `GPR[rd] ← GPR[rs] × GPR[rt]` 

* I-Type

Opcode | rs (base) | rt | IMM
-| - | - |- 
6 bits | 5 bits | 5 bits | 16 bits 

Instr | Opcode | Format | Description
-| - | - | -
BEQ | 000100 | `BEQ rs, rt, offset` | `if GPR[rs] = GPR[rt] then branch` 
BNE | 000101 | `BEQ rs, rt, offset` |`if GPR[rs] ≠ GPR[rt] then branch`
BLEZ | 000110 | `BLEZ rs, offset` | `if GPR[rs] ≤ 0 then branch` 
BGTZ | 000111 | `BGTZ rs, offset` |`if GPR[rs] > GPR[rt] then branch`
ADDI | 001000 | `ADDI rt, rs, immediate` | `GPR[rt] ←  GPR[rs] + immediate` 
ADDIU | 001001 | `ADDIU rt, rs, immediate` |`GPR[rt] ← GPR[rs] + immediate`
SLTI | 001010 |  `SLTI rt, rs, immediate` |`GPR[rt] ← (GPR[rs] < immediate)`
SLTIU | 001011 |  `SLTIU rt, rs, immediate` |`GPR[rt] ← (GPR[rs] < immediate)`
ANDI | 001100 | `ANDI rt, rs, immediate` |`GPR[rt] ← GPR[rs] AND immediate`
ORI | 001101 | `ORI rt, rs, immediate` |`GPR[rt] ← GPR[rs] OR immediate`
XORI | 001110 | `XORI rt, rs, immediate` |`GPR[rt] ← GPR[rs] XOR immediate`
LUI | 001111 | `LUI rt, immediate` |`GPR[rt] ← immediate << 16`
LB | 100000 |`LB rt, offset(base)` | `GPR[rt] ← memory[GPR[base] + offset]` 
LH | 100001 |`LH rt, offset(base)` | `GPR[rt] ← memory[GPR[base] + offset]` 
LW | 100011 |`LW rt, offset(base)` | `GPR[rt] ← memory[GPR[base] + offset]` 
LBU | 100100 |`LBU rt, offset(base)` | `GPR[rt] ← memory[GPR[base] + offset]` 
LHU | 100101 |`LHU rt, offset(base)` | `GPR[rt] ← memory[GPR[base] + offset]` 
SB | 101000 |`SB rt, offset(base)` | `memory[GPR[base] + offset] ← GPR[rt]` 
SH | 101001 |`SH rt, offset(base)` | `memory[GPR[base] + offset] ← GPR[rt]` 
SW | 101011 | `SW rt, offset(base)` | `memory[GPR[base] + offset] ← GPR[rt]`

* J-Type

Opcode | Address 
-| - 
6 bits | 26 bits 

Instr | Opcode | Format 
-| - | - 
J | 000010 | `J target` 
JAL | 000011 | `JAL target` 
NOP | 000000 | `NOP` 

## Registers

Floating point registers are not implemented !

Register Number | Conventional Name | Usage
-| - | -
$0 | $zero | Hard-wired to 0
$1 | $at | Reserved for pseudo-instructions
\$2 - $3 | \$v0, \$v1 | Return values from functions
\$4 - $7 | \$a0 - $a3 | Arguments to functions - not preserved by subprograms
\$8 - $15 | \$t0 - $t7 | Temporary data, not preserved by subprograms
\$16 - $23 | \$s0 - $s7 | Saved registers, preserved by subprograms
\$24 - $25 | \$t8 - $t9 | More temporary registers, not preserved by subprograms
\$26 - $27 | \$k0 - $k1 | Reserved for kernel. Do not use.
$28 | $gp | Global Area Pointer (base of global data segment)
$29 | $sp | Stack Pointer
$30 | $fp | Frame Pointer
$31 | $ra | Return Address
\$f0 - $f3 | - | Floating point return values
\$f4 - $f10 | - | Temporary registers, not preserved by subprograms
\$f12 - $f14 | - | First two arguments to subprograms, not preserved by subprograms
\$f16 - $f18 | - | More temporary registers, not preserved by subprograms
\$f20 - $f31 | - | Saved registers, preserved by subprograms

## Control unit

The control unit computes the control signals based on `Opcode` ,  `Func`  and `ZF` `OF` `SF` generated by ALU. Slightly different from that in the textbook, signal `Branch` and `Jump` are generalized and absorbed by a new signal `PCSrc` and the `BranchSrc` signal is computed to control the conditional branches.

- Signals for branch Instruction, based on `ZF` `OF` `SF`

  **BranchSrc** : Decide whether the branch target should be taken, when executing `BEQ` , `BNE` , `BLEZ` and `BGTZ` .

- Signals based on `Opcode` and `Func`

Signal |Description | Values or Select lines 
-| - | -
PCSrc | Source of next PC | PC + 4, Branch Target, Immediate ( `J` and `JAL` ) , Instruction[25: 21] ( `JR` and `JALR` ) 
RegWrite |Write enable of Register File|0, 1
RegDst |Destination register to write | Instruction[20:16] (I-Type), Instruction[15:11] (R-Type), 31 ( `JAL` and `JARL` ) 
RegWriteData |Data to write to the destination register | ALU result, Read Data in Data Memory, PC + 4 ( `JAL` and `JALR` ) 
ALUSrcA |Operand A of ALU|Read Data 1, Instruction[10: 5] (Shamt for shift operations)
ALUSrcB |Operand B of ALU|Read Data 2, Immediate
ALUControl |Operation of ALU|`SLL` , `SRL` , `SRA` , `LUI` , `MUL` , `MULU` , `ADD` , `ADDU` , `SUB` , `SUBU` , `AND` , `OR` , `XOR` , `NOR` , `SLT` , `SLTU`
MemWrite |Write enable of Data Memory|0, 1
MemSigned | Sign-extended options for `LOAD` instructions. ( `LBU` and `LHU` should load zero-extended byte or half-word into the register. ) |0, 1
MemWidth | The width of data fetched or written in Data Memory | byte, half-word, word 


## Memory-mapped I/O

To make it behaves as a real CPU, I try to simulate the I/O Mapping in MIPS.

- The inputs, e.g. `SW[1:0]`, are mapping directly into the Data Memory with a fixed address, decided by the designer.
- The outputs, e.g. `seed`, is mapped at another predefined address. 
- Then, for CPU, it only need to read Data Memory to fetch the inputs, and modify the value at specific address to govern the outputs, using `LOAD` and `STORE` instructions.

