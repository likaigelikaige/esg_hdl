# p10
Convenient FPGA communication
## Introduction
The main purpose of the protocol is to configure, monitor and control FPGA based devices over digital interfaces such as UART (VCOM) or TCP/IP. The p10 stands for protocol decimal as it operates with decimal notation numbers. All text and numbers are ASCII symbols. 
The p10 is a request-reply protocol in which an FPGA acts as a server replying to a client. There are several fields in a request: `command` a `parameter` and a `value`.
### Command description
- `set`. Set parameter to a specific value. The value is expressed in integer decimal notation with a possible -k suffix representing x1000 multiplier. If -k suffix is used, a dot may be inserted in the number. As an example request suppose one sets parameter `freq` to a value of 12500: 
``\`set-freq:12.5k;``. User can define a parameter read-only and set limits within which the value is considered valid. It is described later.

- `get`. Readout parameter's value. The returned number will always be an integer and will be appended with `units`
Each request starts with a \` symbol and ends with a ;. Each reply starts with > and ends with ;.

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
