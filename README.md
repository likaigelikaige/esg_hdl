# ESG communication protocol
The ESG communication protocol stands for: Execute, Set and Get. The protocol permits reliable control of FPGA by means of executing commands, setting and reading data. ASCII symbols are used for keywords and parameters as well as decimal numbers.
## Features
- Reliably execute commands. e.g.: turn power on or enable led
- Write/read of a set of predefined parameters
- Automatic binary to ASCII decimal and vice-versa convertion

# Usage
To use the protocol, information about custom parameters must be provided. The parameters are application-speific and thus are not provided here. User must specify register addresses in `p10_reg_defines.sv` and initialize parameter info ROM in `p10_reg_rom.sv`. These two files should be located at `../src/verilog`
## Register definition sample
p10_reg_defines.sv
```
parameter ADDR_FOO = 0;
parameter ADDR_BAR = 1;
parameter ADDR_NYA = 2;
```
## ROM initialization sample
```
  prm_rom[ADDR_FOO].prm    = "foo";
  prm_rom[ADDR_FOO].min    = 0;
  prm_rom[ADDR_FOO].max    = 100;
  prm_rom[ADDR_FOO].units  = "foos";
  prm_rom[ADDR_FOO].rights = rw;
  prm_rom[ADDR_FOO].is_exec = 0;
```
