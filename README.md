# 16-bit ALU in Verilog (gate-level)

A 16-bit **Arithmetic Logic Unit** built in **structural Verilog from `xor` / `and` / `or` / `not` primitives** ; no behavioural `+` or `-` operators. The ALU is wired to an 8 × 16-bit register file and a 1024 × 16-bit RAM, and exposes six instructions (increment, decrement, subtract, add, store, load) selected by a 4-bit opcode.

[![Language](https://img.shields.io/badge/HDL-Verilog-FF6F00)](alu.v)
[![Style](https://img.shields.io/badge/Style-Gate--level%20structural-525252)](#design)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Block diagram

```
                ┌──────────────────────────┐
   reg1[2:0] ──►│        regfile           ├─► register1 [15:0] ─┐
   reg2[2:0] ──►│  8 × 16-bit (Reg0..Reg7) ├─► register2 [15:0] ─┤
                └──────────────────────────┘                     │
                                                                 ▼
                ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐
                │ INC    │  │ DEC    │  │ SUB    │  │ ADD    │  │ RAM    │
                │ a + 1  │  │ a - 1  │  │ a - b  │  │ a + b  │  │ 1024×16│
                └───┬────┘  └───┬────┘  └───┬────┘  └───┬────┘  └───┬────┘
                    │ a0,s0    │ a1,s1    │ a2,s2    │ a3,s3    │ a5,s5
                    └──────────┴──────────┴──────────┴──────────┘
                                       │
                                  14-to-1 MUX
                                       │
                                       ▼
                              result [15:0]  status [2:0]
```

## Instruction set

| Opcode  | Operation                                                         | Effect on `status` |
|---------|-------------------------------------------------------------------|--------------------|
| `0000`  | `reg1 = reg1 + 1` &nbsp;(increment)                                | overflow, zero, carry |
| `0001`  | `reg1 = reg1 − 1` &nbsp;(decrement)                                | overflow, zero, carry |
| `0010`  | `result = reg1 − reg2` &nbsp;(two's-complement subtraction)        | overflow, zero, carry |
| `0011`  | `result = reg1 + reg2` &nbsp;(addition)                            | overflow, zero, carry |
| `0100`  | **Store** `reg1 → MEM[reg2]`                                       | &mdash;            |
| `0101`  | **Load** `reg1 ← MEM[reg2]`                                        | zero               |

`status[0]` = overflow &nbsp;·&nbsp; `status[1]` = zero &nbsp;·&nbsp; `status[2]` = carry-out

## Design

Everything is built up from a single 1-bit full adder. Major modules in [`alu.v`](alu.v):

| Module              | Style                                  | What it does |
|---------------------|----------------------------------------|--------------|
| `fulladder`         | gate-level (`xor`, `and`, `or`)        | 1-bit full adder &mdash; the building block for everything arithmetic |
| `bitadder_16`       | 16 instances of `fulladder`            | 16-bit ripple-carry adder; emits overflow / zero / carry |
| `subtractor`        | XOR-invert + `bitadder_16` with C-in=1 | two's-complement subtractor |
| `inncrement`        | `bitadder_16` with `c = 1`             | `a + 1` |
| `decrement`         | `subtractor` with `c = 1`              | `a − 1` |
| `or16bititself`     | gate-level OR reduction                | true if any bit of the bus is high &rarr; zero-flag generator |
| `regfile`           | behavioural `always @(*)`              | 8 × 16-bit register bank, dual read or single write |
| `ram`               | behavioural, sync on `negedge clk`     | 1024 × 16-bit memory, RD/WR-selected, zero-initialised |
| `store_flg_mod` / `load_flg_mod` | gate-level decoders       | one-hot the store (`0100`) and load (`0101`) opcodes |
| `multiplexer_14_1`  | behavioural `case`                     | selects which functional unit's output reaches `result` / `status` |
| `myClock`           | always #1                              | generates a 2-tick clock for the RAM |
| **`ALU`**           | structural top-level                   | instantiates and wires all of the above |

> Status / overflow detection follows the textbook rule: `overflow = carry_in(MSB) XOR carry_out(MSB)`. Zero-flag is `~OR(result)`.

## Testbench

A self-contained `test` module at the bottom of [`alu.v`](alu.v) drives six cases through the ALU and `$display`s the result + status bus for each:

| Case | Operation | Operands         | Description |
|------|-----------|------------------|-------------|
| 0    | INC       | `Reg2` (value 2) | increment Reg2 |
| 1    | DEC       | `Reg1` (value 1) | decrement Reg1 |
| 2    | SUB       | `Reg3 − Reg1`    | 3 − 1 |
| 3    | ADD       | `Reg5 + Reg6`    | 5 + 6 |
| 4    | STORE     | `Reg3 → MEM[Reg5]` | store value of Reg3 to memory at address 5 |
| 5    | LOAD      | `Reg4 ← MEM[Reg5]` | load MEM[5] into Reg4 |

The register file is pre-initialised so `Reg<i> = i`.

## How to run

### Icarus Verilog (open-source, easiest)

```bash
# install once
sudo apt install iverilog gtkwave         # Ubuntu/WSL
brew install icarus-verilog gtkwave       # macOS

# simulate
iverilog -o alu_tb alu.v
vvp alu_tb
```

You should see six `$display` lines, one per test case, with the result bus and the three status bits.

### ModelSim / QuestaSim

```tcl
vlib work
vlog alu.v
vsim -c work.test -do "run -all; quit"
```

## Files

```
.
├── alu.v          # all Verilog modules + self-contained testbench
├── README.md
└── LICENSE
```

## Course context

Built for the Digital Logic / Computer Architecture course at PSUT. Demonstrates structural Verilog at the gate level (no behavioural arithmetic), datapath construction, integration with register file + RAM, and the simulation flow with a `$display`-driven testbench.

## Author

**Leen Almousa** &mdash; [github.com/leenalmousa](https://github.com/leenalmousa)

## License

Released under the [MIT License](LICENSE).
