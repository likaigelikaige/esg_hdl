// An ASCII-based decimal protocol for interfacing FPGAs from various command lines over COM port, UDP, TCP, etc.
// p10 supports multimple "commands":
// - "help" lists a help string
// - "list" lists commands
// - "set" sets a "parameter" to a val
// - "read" reads out a "parameter"
// - "mon" monitors a "parameter"

// Common package for p10. Normally shouldn't be modified
`define VERSION "p10: 2.0; 04.25.2020"
package p10_pkg_common;
  // define maximum lengths of fields
  localparam int CMD_LEN = 8;
  localparam int PRM_LEN = 8;
  localparam int DAT_LEN = 8;
  localparam int EXEC_LEN = 8;
  localparam int DAT_LEN_BITS = $clog2(10**DAT_LEN);
  localparam int UNITS_LEN = 5;
  localparam int PRM_COUNT = 9;

  // ASCII definitions
  localparam byte START_RX = "`"; // start symbol
  localparam byte START_TX = ">"; // start symbol
  localparam byte CMD_DLM  = "-"; // command delimiter
  localparam byte PRM_DLM  = ":"; // parameter delimiter
  localparam byte DAT_DLM  = ";"; // data delimiter
  localparam byte K_MOD  = "k"; // x1000 modfier
  localparam byte DOT    = "."; // dot symbol

  typedef enum byte {
    type_decimal,
    type_fixed,
    type_string,
    type_nodata
  } cmd_data_type_t;

  typedef logic [CMD_LEN-1:0][7:0] cmd_t;
  typedef logic [PRM_LEN-1:0][7:0] prm_t;
  typedef logic [EXEC_LEN-1:0][7:0] exec_t;

  typedef struct packed {
    cmd_t cmd;
    byte  cmd_len;
    prm_t prm;
    byte  prm_len;
    logic [DAT_LEN-1:0][3:0] dat_bcd;
    logic [DAT_LEN-1:0][7:0] dat_str;
  } op_t;

  // Errors
  
  typedef enum logic [13:0] {
    none            = 14'b00000000000001,
    rx_cmd_bad      = 14'b00000000000010,
    rx_data_bad     = 14'b00000000000100,
    rx_prm_bad      = 14'b00000000001000,
    rx_data_not_int = 14'b00000000010000,
    prm_read_only   = 14'b00000000100000,
    prm_exec_only   = 14'b00000001000000,
    prm_not_exec    = 14'b00000010000000,
    prm_exec_err    = 14'b00000100000000, 
    prm_exec_to     = 14'b00001000000000, 
    prm_not_found   = 14'b00010000000000,
    prm_low         = 14'b00100000000000,
    prm_high        = 14'b01000000000000,
    rx_timeout      = 14'b10000000000000
  } err_t;

  typedef enum logic [4:0] {
    rx_ok        = 5'b00001,
    cmd_bad      = 5'b00010,
    data_bad     = 5'b00100, 
    prm_bad      = 5'b01000,
    data_not_int = 5'b10000
  } err_rx_t;

  typedef enum logic [8:0] {
     prm_ok    = 9'b000000001,
     not_found = 9'b000000010,
     read_only = 9'b000000100, 
     exec_only = 9'b000001000, 
     not_exec  = 9'b000010000, 
     exec_err  = 9'b000100000, 
     exec_to   = 9'b001000000, 
     low       = 9'b010000000,
     high      = 9'b100000000
  } err_prm_t;

  // FSMs

  typedef enum logic [8:0] {
    rx_idle_s,
    rx_wait_start_s,
    rx_cmd_s,
    rx_prm_s,
    rx_dat_s,
    rx_err_s,
    rx_wait_rst_s
  } fsm_rx_t;

  typedef enum logic [3:0] {
    bcd_idle_s,
    bcd_shift_s,
    bcd_conv_s,
    bcd_wait_rst_s
  } fsm_bcd_t;

  typedef enum logic [8:0] {
    tx_idle_s,
    tx_cmd_s,
    tx_prm_s,
    tx_prm_dlm_s,
    tx_dat_s,
    tx_stop_s,
    tx_string_s,
    tx_units_s,
    tx_cr_s,
    tx_lf_s,
    tx_wait_rst_s
  } fsm_tx_t;

  typedef enum logic [5:0] {
    prm_idle_s,
    prm_scan_s,
    prm_check_s,
    prm_write_s,
    prm_conv_s,
    prm_read_s,
    prm_wait_rst_s
  } prm_fsm_t;

  typedef enum logic {
    r,
    rw
  } rights_t;

  typedef struct packed {
    prm_t                      prm;
    logic [DAT_LEN_BITS-1:0]   min;
    logic [DAT_LEN_BITS-1:0]   max;
    logic [UNITS_LEN-1:0][7:0] units;
    rights_t                   rights;
    bit                        is_exec;
  } prm_entry_t;

  function [3:0] ascii2bcd;
    input [7:0] in;
    case (in)
      "0" : ascii2bcd = 0;
      "1" : ascii2bcd = 1;
      "2" : ascii2bcd = 2;
      "3" : ascii2bcd = 3;
      "4" : ascii2bcd = 4;
      "5" : ascii2bcd = 5;
      "6" : ascii2bcd = 6;
      "7" : ascii2bcd = 7;
      "8" : ascii2bcd = 8;
      "9" : ascii2bcd = 9;
      ".", "k", DAT_DLM : ascii2bcd = 11; // exclude to avoid error
      default : ascii2bcd = 4'hf;
    endcase  
  endfunction : ascii2bcd

  function [7:0] bcd2ascii;
    input [3:0] in;
    case (in)
      0 : bcd2ascii = 8'h30;
      1 : bcd2ascii = 8'h31;
      2 : bcd2ascii = 8'h32;
      3 : bcd2ascii = 8'h33;
      4 : bcd2ascii = 8'h34;
      5 : bcd2ascii = 8'h35;
      6 : bcd2ascii = 8'h36;
      7 : bcd2ascii = 8'h37;
      8 : bcd2ascii = 8'h38;
      9 : bcd2ascii = 8'h39;
    endcase  
  endfunction : bcd2ascii

endpackage

import p10_pkg_common::*;

module p10 (
  input  logic clk,
  input  logic rst,

  input  logic [7:0] rxd,
  input  logic       rxv,

  output logic [7:0] txd,
  output logic       txv,
  output logic       txen,
  input  logic       cts,

  input  logic                           clk_ram,
  input  logic [$clog2(PRM_COUNT+1)-1:0] prm_addr,
  input  logic [DAT_LEN_BITS-1:0]        prm_ram_d,
  output logic [DAT_LEN_BITS-1:0]        prm_ram_q,
  input  logic                           prm_ram_w,

  output prm_t exec_prm,
  output logic exec_val,
  input  logic exec_bad,
  input  logic exec_ok
);

`include "p10_regs.sv"
`include "p10_reg_defines.sv"

parameter integer BIN_W = $clog2(10**DAT_LEN);
parameter integer TIMEOUT_TICKS = 50000000;
parameter integer EXECUTION_TIMEOUT_TICKS = 10000000;
parameter integer INPUT_FIFO_DEPTH = 5;
parameter integer OUTPUT_FIFO_DEPTH = 5;
parameter integer READ_BY_ONE = 1; // force read by one byte. (needed for UART as it deasserts cts 1 tick later than needed, so every second bit is lost)

// Command definitions
cmd_t set   = "set";   // Set val for a parameter. ex: "set.freq:10.25k;"
cmd_t get   = "get";   // Get current val of a parameter. ex: "get.freq;"
cmd_t mon   = "mon";   // Monitor a parameter. ex: "mon.current;"
cmd_t save  = "save";
cmd_t ver   = "ver";
cmd_t exec  = "exec";

prm_entry_t prm_rom [0:PRM_COUNT-1];

fsm_rx_t fsm_rx;
fsm_tx_t fsm_tx;
fsm_bcd_t fsm_bcd;

logic fsm_rst;
err_t err;
op_t cur_rx, cur_tx;

logic dot_pres; 
logic k_mod_pres;
logic bcd2bin_rdy;
logic check_prm;

logic [$clog2(CMD_LEN+1)-1:0] cmd_ctr_rx, cmd_ctr_tx;
logic [$clog2(PRM_LEN+1)-1:0] prm_ctr_rx, prm_ctr_tx;
logic [$clog2(DAT_LEN+1)-1:0] dat_ctr_rx, dat_ctr_tx, dot_pos;

logic [2:0] shift;
logic [BIN_W-1:0] cur_rx_bin, cur_tx_bin;
logic [DAT_LEN-1:0][3:0] tx_bcd, cur_tx_bcd, rx_bcd, rx_bcd_shifted = 0;
logic conv_bcd2bin, conv_bin2bcd;

// input buffer
fifo_sc_if #(INPUT_FIFO_DEPTH, 8) rx_buf(.*);
fifo_sc #(INPUT_FIFO_DEPTH, 8) rx_buf_inst (.fifo (rx_buf));

assign rx_buf.rst = rst;
assign rx_buf.clk = clk;

assign rx_buf.write = rxv;
assign rx_buf.data_in = rxd;

// output buffer
fifo_sc_if #(OUTPUT_FIFO_DEPTH, 8) tx_buf(.*);
fifo_sc #(OUTPUT_FIFO_DEPTH, 8) tx_buf_inst (.fifo (tx_buf));

assign tx_buf.rst = rst;
assign tx_buf.clk = clk;

// parameter ram

logic [$clog2(PRM_COUNT+2)-1:0] prm_rom_addr_int, prm_ram_scan_addr;

prm_entry_t prm_rom_q, cur_check;
logic prm_ram_w_int;
logic [DAT_LEN_BITS-1:0] prm_ram_q_int, prm_ram_d_int;
logic [$clog2(PRM_COUNT+1)-1:0] prm_addr_int, prm_addr_prev;

// timeout

logic rsp_err, rsp_ok, timeout;

logic conv_bcd, rx_buf_q_v;
logic scan_prm, stop_read;

err_prm_t err_prm;
prm_fsm_t prm_fsm;

logic rx_fsm_rdy, rx_fifo_req, tx_done;

err_rx_t err_rx;

logic [$clog2(TIMEOUT_TICKS+1)-1:0] to_ctr;
logic [$clog2(EXECUTION_TIMEOUT_TICKS+1)-1:0] exec_to_ctr;

always @ (posedge clk) begin
  if (fsm_rst) begin
    to_ctr <= 0;
    timeout <= 0;
  end
  else begin
    to_ctr <= (fsm_rx == rx_idle_s) ? 0 : to_ctr + 1;
    if (to_ctr == TIMEOUT_TICKS) timeout <= 1;    
  end
end

// ram/rom logic

always @ (posedge clk) begin
  prm_rom_q <= prm_rom[prm_addr_int]; // readout ROM entry containing info about current parameter
end

ram_if_dp #($clog2(PRM_COUNT+1), DAT_LEN_BITS) prm_ram(.*); // IPv4 bits = 32, MAC bits = 48;
ram_dp #($clog2(PRM_COUNT+1), DAT_LEN_BITS) prm_ram_inst (.mem_if (prm_ram)); // IPv4 bits = 32, MAC bits = 48;

assign prm_ram.rst   = 1'b0;

assign prm_ram.clk_a = clk;
assign prm_ram.a_a   = prm_addr_int;
assign prm_ram.d_a   = cur_rx_bin;
assign prm_ram.w_a   = prm_ram_w_int;
assign cur_tx_bin    = prm_ram.q_a;

assign prm_ram.clk_b = clk_ram;
assign prm_ram.a_b   = prm_addr;
assign prm_ram.d_b   = prm_ram_d;
assign prm_ram.w_b   = prm_ram_w;
assign prm_ram_q     = prm_ram.q_b;

always @ (posedge clk) begin
  if (rst) begin
    fsm_rst <= 1;
  end
  else begin
    fsm_rst <= tx_done;
    rx_buf.read <= rx_fsm_rdy && !rx_buf.empty;
    rx_buf_q_v <= rx_buf.read;
  end
end

assign rx_fsm_rdy = (fsm_rx != rx_wait_rst_s);
logic rsp_ver;
always @ (posedge clk) begin
  if (fsm_rst) begin
    fsm_rx     <= rx_idle_s;
    cmd_ctr_rx <= 0;
    prm_ctr_rx <= 0;
    dat_ctr_rx <= 0;
    cur_rx     <= 0;
    dot_pos    <= 0;
    dot_pres   <= 0;
    k_mod_pres <= 0;
    conv_bcd   <= 0;
    scan_prm   <= 0;
    stop_read  <= 0;
    err_rx     <= rx_ok;
    rsp_ver    <= 0;
  end
  else if (rx_buf.valid_out) begin
    case (fsm_rx)
      rx_idle_s : begin
        if (rx_buf.data_out == START_RX) fsm_rx <= rx_cmd_s;
      end
      rx_cmd_s : begin
        if (cmd_ctr_rx == CMD_LEN) begin
          err_rx <= cmd_bad;
          $display("cmd too long");
        end
        if (rx_buf.data_out == CMD_DLM) begin
          $display("-> cmd: %s", cur_rx.cmd);
          cur_rx.cmd_len = cmd_ctr_rx; // latch cmd len for correct response
          case (cur_rx.cmd)
            set, get, mon : begin
              fsm_rx <= rx_prm_s;
            end
            exec : begin
              fsm_rx <= rx_prm_s;
            end
            //stop : begin

            //end
            // load, save  : begin
            //   fsm_rx <= done_s;
            // end
            endcase
          end
        else if (rx_buf.data_out == DAT_DLM) begin
          if (cur_rx.cmd == ver) begin
            rsp_ver <= 1;
            fsm_rx <= rx_wait_rst_s;	  
          end
        end
	      else begin
          cmd_ctr_rx <= cmd_ctr_rx + 1;
          cur_rx.cmd[CMD_LEN-1:1] <= cur_rx.cmd[CMD_LEN-2:0];
          cur_rx.cmd[0] <= rx_buf.data_out;
        end
      end
      rx_prm_s : begin
        if (prm_ctr_rx == PRM_LEN) begin
          err_rx <= prm_bad;
          $display("prm too long");
        end
        if (rx_buf.data_out == PRM_DLM && cur_rx.cmd == set) begin
          cur_rx.prm_len = prm_ctr_rx;
          $display("-> prm: %s", cur_rx.prm);
          fsm_rx <= rx_dat_s;
        end
        else if (rx_buf.data_out == DAT_DLM && (cur_rx.cmd == get || cur_rx.cmd == exec)) begin
          cur_rx.prm_len = prm_ctr_rx;
          $display("-> prm: %s", cur_rx.prm);
          fsm_rx <= rx_wait_rst_s;
          scan_prm <= 1;
        end
        else begin
          prm_ctr_rx <= prm_ctr_rx + 1;
          cur_rx.prm[PRM_LEN-1:1] <= cur_rx.prm[PRM_LEN-2:0];
          cur_rx.prm[0] <= rx_buf.data_out;
        end
      end
      rx_dat_s : begin
        // user can input bad data in multiply ways. check it
        if (dat_ctr_rx == DAT_LEN || // data is too long
        dot_pos == 5 || // bad dot postion
        ascii2bcd(rx_buf.data_out) == 4'hf || 
        (rx_buf.data_out != DAT_DLM && k_mod_pres) || 
        (rx_buf.data_out == DAT_DLM && !k_mod_pres && dot_pres)) begin
          err_rx <= data_bad;
          $display("data too long");
        end
        if (rx_buf.data_out == K_MOD) k_mod_pres <= 1;
        if (rx_buf.data_out == DOT) dot_pres <= 1;
        if (rx_buf.data_out == DAT_DLM) begin
          $display("-> dat: %h", cur_rx.dat_bcd);
          conv_bcd <= 1;
          fsm_rx <= rx_wait_rst_s;
        end
        else begin
          dat_ctr_rx <= dat_ctr_rx + 1;
          if (rx_buf.data_out != "." && rx_buf.data_out != "k") begin
            cur_rx.dat_bcd[DAT_LEN-1:1] <= cur_rx.dat_bcd[DAT_LEN-2:0];
            cur_rx.dat_bcd[0] <= ascii2bcd(rx_buf.data_out);
            if (dot_pres) dot_pos <= dot_pos + 1;
          end
        end
      end
      rx_wait_rst_s : begin

      end
    endcase
  end
end

/*
 * Shift data 
 */

always @ (posedge clk) begin
  if (fsm_rst) begin
    fsm_bcd <= bcd_idle_s;
    shift <= 0;
    rx_bcd <= 0;
    check_prm <= 0;
    conv_bcd2bin <= 0;
  end
  else begin
    case (fsm_bcd)
      bcd_idle_s : begin
        if (conv_bcd) begin
          shift   <= (dot_pres) ? 3-dot_pos : 3;
          fsm_bcd <= bcd_shift_s;
	  			rx_bcd  <= cur_rx.dat_bcd;
        end
      end
      bcd_shift_s : begin // shift bcd to account for k modifier
        shift <= shift - 1;
        if (shift == 0 || !k_mod_pres) begin
          fsm_bcd <= bcd_conv_s;
          conv_bcd2bin <= 1;
        end
        else rx_bcd[DAT_LEN-1:0] <= {rx_bcd[DAT_LEN-2:0], 4'h0};
      end
      bcd_conv_s : begin
        conv_bcd2bin <= 0;
        if (bcd2bin_rdy && !conv_bcd2bin) begin
          check_prm <= 1;
          $display("shifted real val. result: %h", rx_bcd);
        end
      end
    endcase
  end
end

/*
 * BCD to binary conversion when receiving
 * Vice versa when replying
 */

bcd2bin #(
	.DEC_W (DAT_LEN)
) bcd2bin_inst (
	.clk  (clk),
	.rst  (rst),
	.in   (rx_bcd),
	.out  (cur_rx_bin),
	.conv (conv_bcd2bin),
	.rdy  (bcd2bin_rdy)
);
bin2bcd #(
	.DEC_W (DAT_LEN)
) bin2bcd_inst (
	.clk  (clk),
	.rst  (rst),
	.in   (cur_tx_bin),
	.out  (tx_bcd),
	.conv (conv_bin2bcd),
	.rdy  (bin2bcd_rdy)
);

/////////////////////
// Parameter check //
/////////////////////

always @ (posedge clk) begin
  if (fsm_rst) begin
    prm_addr_int <= 0;
    prm_addr_prev <= 0;
    prm_fsm <= prm_idle_s;
    rsp_ok <= 0;
    conv_bin2bcd <= 0;
    prm_ram_w_int <= 0;
    err_prm <= prm_ok;
    exec_val <= 0;
    exec_prm <= 0;
    exec_to_ctr <= 0;
    cur_check <= 0;
  end
  else begin
    case (prm_fsm)
      prm_idle_s : begin
        if (check_prm || scan_prm) prm_fsm <= prm_scan_s;
      end
      prm_scan_s : begin
        if (prm_rom_q.prm == cur_rx.prm) begin
          cur_check <= prm_rom_q;
          prm_addr_int <= prm_addr_prev;
          $display ("Found parameter %s, val %d", prm_rom_q.prm, prm_ram_q);
          case (cur_rx.cmd)
            set : begin
              if (!prm_rom_q.is_exec) prm_fsm <= prm_check_s; else
              begin
                err_prm <= exec_only;
                prm_fsm <= prm_wait_rst_s;
              end
            end
            exec : begin // executable command
              if (prm_rom_q.is_exec) begin // check for "is executable" flag
                exec_prm <= cur_rx.prm; // 
                exec_val <= 1;
              end
              else err_prm <= not_exec;
              prm_fsm <= prm_wait_rst_s;
            end
            get : prm_fsm <= prm_conv_s; // go directly to convert internal binary to BCD
          endcase
        end
        else begin
          prm_addr_int <= prm_addr_int + 1; // scan ROM for parameter string
        end
        prm_addr_prev <= prm_addr_int;
        if (prm_addr_int == PRM_COUNT + 1) begin // ROM scanned, no parameter with "cur_rx.prm" found
          $display ("Parameter not found");
          err_prm <= not_found;
        end
      end
      prm_check_s : begin
        if (cur_check.rights == r) begin
          err_prm <= read_only;
          prm_fsm <= prm_wait_rst_s;
          $display ("Cannot write to a read only parameter");
        end
        else if (cur_rx_bin < cur_check.min) begin
          $display ("Received val %d less than minimum %d", cur_rx_bin, cur_check.min);
          err_prm <= low;
          prm_fsm <= prm_wait_rst_s;
        end
        else if (cur_rx_bin > cur_check.max) begin
          $display ("Received val %d more than maximum %d", cur_rx_bin, cur_check.max);
          err_prm <= high;
          prm_fsm <= prm_wait_rst_s;
        end
        else begin
          rsp_ok <= 1;
          $display ("Received val %d within limits: [%d..%d]", cur_rx_bin, cur_check.min, cur_check.max);
          prm_fsm <= prm_write_s;
        end
      end
      prm_write_s : begin
        rsp_ok        <= 1;
        prm_ram_w_int <= 1;
        prm_ram_d_int <= cur_rx_bin;
        prm_fsm       <= prm_wait_rst_s;
      end
      prm_conv_s : begin
        conv_bin2bcd <= 1;
        prm_fsm <= prm_read_s;
      end
      prm_read_s : begin
        conv_bin2bcd <= 0;
        if (bin2bcd_rdy && !conv_bin2bcd) begin
          prm_fsm <= prm_wait_rst_s;
          rsp_ok <= 1;
        end
      end
      prm_wait_rst_s : begin
        prm_ram_w_int <= 0;
        exec_to_ctr <= to_ctr + 1;
        if (exec_to_ctr == EXECUTION_TIMEOUT_TICKS) err_prm <= exec_to; 
        else if (exec_bad) err_prm <= exec_err;
        else if (exec_ok) rsp_ok <= 1;
      end
    endcase
  end
end

//////////////////////
// Error gatherning //
//////////////////////

always @ (posedge clk) begin
  if (fsm_rst) begin
    err <= none;
    rsp_err <= 0;
  end
  else begin
    rsp_err <= (err != none);
    if (timeout) err <= rx_timeout;
    else if (err_rx != rx_ok) begin
      case (err_rx)
        rx_ok        : err <= none;
        cmd_bad      : err <= rx_cmd_bad;
        data_bad     : err <= rx_data_bad;
        prm_bad      : err <= rx_prm_bad;
        data_not_int : err <= rx_data_not_int;
        default      : err <= none;
      endcase
    end
    else if (err_prm != prm_ok) begin
      case (err_prm)
        prm_ok       : err <= none;
        not_found    : err <= prm_not_found;
        exec_only    : err <= prm_exec_only;
        not_exec     : err <= prm_not_exec;
        exec_err     : err <= prm_exec_err;
        exec_to      : err <= prm_exec_to;
        read_only    : err <= prm_read_only;
        low          : err <= prm_low;
        high         : err <= prm_high;
        default      : err <= none;
      endcase
    end
  end
end

//////////////
// Response //
//////////////

logic cmd_tx_len;
logic prm_tx_len;
logic value_tx_len;
logic tx_bcd_val;
logic [7:0] cur_dig;

parameter integer STRING_LEN = 32;
parameter integer NUM_ERRORS = 13;

typedef struct packed {
  logic [0:STRING_LEN-1][7:0] val;
  err_t err;
} rsp_err_t;

rsp_err_t rsp_rom [NUM_ERRORS-1:0];
rsp_err_t rsp_rom_q;

logic [$clog2(NUM_ERRORS+1)-1:0] rsp_rom_addr;

always @ (posedge clk) begin
  rsp_rom_q <= rsp_rom[rsp_rom_addr];
end

logic [0:STRING_LEN-1][7:0] cur_rsp_string, param_set_str, exec_success_str, ver_string, unknown_err_str;
logic [7:0] cur_rsp_ctr;

initial begin
  rsp_rom[0].val = "error: bad command";
  rsp_rom[0].err = rx_cmd_bad;

  rsp_rom[1].val = "error: bad data";
  rsp_rom[1].err = rx_data_bad;

  rsp_rom[2].val = "error: bad parameter";
  rsp_rom[2].err = rx_prm_bad;

  rsp_rom[3].val = "error: data not int";
  rsp_rom[3].err = rx_data_not_int;

  rsp_rom[4].val = "error: read-only";
  rsp_rom[4].err = prm_read_only;   

  rsp_rom[5].val = "error: parameter not found";
  rsp_rom[5].err = prm_not_found;   

  rsp_rom[6].val = "error: executable only";
  rsp_rom[6].err = prm_exec_only;   

  rsp_rom[7].val = "error: not executable";
  rsp_rom[7].err = prm_not_exec;   
    
  rsp_rom[8].val = "error: failed to execute";
  rsp_rom[8].err = prm_exec_err;   

  rsp_rom[9].val = "error: execution timeout";
  rsp_rom[9].err = prm_exec_to;   

  rsp_rom[10].val = "error: value too low";
  rsp_rom[10].err = prm_low;   

  rsp_rom[11].val = "error: value too high";
  rsp_rom[11].err = prm_high;

  rsp_rom[12].val = "error: timeout";
  rsp_rom[12].err = rx_timeout;

  param_set_str = ": parameter set";
  exec_success_str = ": executed successfully";
  unknown_err_str = "error: unknown error";
  ver_string = `VERSION;
end

logic [$clog2(UNITS_LEN+1)-1:0] units_rsp_ctr;
logic [UNITS_LEN-1:0][7:0] cur_tx_units;
	
always @ (posedge clk) begin
  if (fsm_rst) begin
    tx_buf.data_in     <= 0;
    tx_buf.write     <= 0;
    cmd_ctr_tx     <= 0;
    prm_ctr_tx     <= 0;
    dat_ctr_tx     <= 0;
    cur_tx_bcd     <= 0;
    fsm_tx         <= tx_idle_s;
    tx_bcd_val     <= 0;
    cur_dig        <= 0;
    tx_done        <= 0;
    rsp_rom_addr   <= 0;
    cur_rsp_string <= 0;
    cur_rsp_ctr    <= 0;
    units_rsp_ctr  <= 0;
    cur_tx_units   <= 0;
  end
  else begin
    case (fsm_tx)
      tx_idle_s : begin
        tx_buf.data_in <= ">";
        cur_tx <= cur_rx;
        if (rsp_ver) begin
          cur_rsp_string <= ver_string;
          fsm_tx <= tx_string_s;
        end
        else if (rsp_ok) begin
          tx_buf.write <= 1;
          cmd_ctr_tx <= 0;
          prm_ctr_tx <= 0;
          dat_ctr_tx <= 0;
          cur_tx_bcd <= tx_bcd;
          cur_tx_units <= prm_rom_q.units;
          case (cur_rx.cmd)
            set : cur_rsp_string <= param_set_str;
            exec : cur_rsp_string <= exec_success_str;
				    default : cur_rsp_string <= unknown_err_str;
			    endcase	
          fsm_tx <= tx_prm_s;
        end
        else if (rsp_err) begin
          rsp_rom_addr <= rsp_rom_addr + 1;
          if (rsp_rom_q.err == err) begin
            fsm_tx <= tx_string_s;
            cur_rsp_string <= rsp_rom_q.val;
          end
        end
      end
      tx_cmd_s : begin
        cur_tx.cmd[CMD_LEN-1:1] <= cur_tx.cmd[CMD_LEN-2:0];
        tx_buf.write <= (cur_tx.cmd[CMD_LEN-1] != "");
        tx_buf.data_in <= cur_tx.cmd[CMD_LEN-1];
        cmd_ctr_tx <= cmd_ctr_tx + 1;
        if (cmd_ctr_tx == CMD_LEN - 1) fsm_tx <= tx_prm_s;
      end
      tx_prm_s : begin
        cur_tx.prm[PRM_LEN-1:1] <= cur_tx.prm[PRM_LEN-2:0];
        tx_buf.write <= (cur_tx.prm[PRM_LEN-1] != "");
        tx_buf.data_in <= cur_tx.prm[PRM_LEN-1];
        prm_ctr_tx <= prm_ctr_tx + 1;
        if (prm_ctr_tx == PRM_LEN - 1) begin
          case (cur_rx.cmd)     
            set, exec : fsm_tx <= tx_string_s;
            get : fsm_tx <= tx_prm_dlm_s;
            default : fsm_tx <= tx_stop_s;
          endcase
        end
      end
      tx_prm_dlm_s : begin
        tx_buf.data_in <= PRM_DLM;
        tx_buf.write <= 1;
        fsm_tx <= tx_dat_s;
      end
      tx_dat_s : begin
        dat_ctr_tx <= dat_ctr_tx + 1;
        if (cur_tx_bcd[DAT_LEN-1] != 0 || (dat_ctr_tx == DAT_LEN-1)) tx_bcd_val <= 1; // append last symbol (zero) anyway
        cur_tx_bcd[DAT_LEN-1:1] <= cur_tx_bcd[DAT_LEN-2:0];
        cur_dig <= bcd2ascii(cur_tx_bcd[DAT_LEN-1]);
        tx_buf.data_in <= cur_dig;
        tx_buf.write <= tx_bcd_val;
        if (dat_ctr_tx == DAT_LEN) fsm_tx <= tx_units_s;
      end
      tx_units_s : begin
        units_rsp_ctr <= units_rsp_ctr + 1;
        cur_tx_units[UNITS_LEN-1:1] <= cur_tx_units[UNITS_LEN-2:0];
        if (units_rsp_ctr == UNITS_LEN-1) fsm_tx <= tx_stop_s;
        tx_buf.write <= (cur_tx_units[UNITS_LEN-1] != "");
        tx_buf.data_in <= cur_tx_units[UNITS_LEN-1];
      end      
      tx_stop_s : begin
        tx_buf.data_in <= DAT_DLM;
        tx_buf.write <= 1;
        fsm_tx <= tx_cr_s;
      end
      tx_cr_s : begin
        tx_buf.data_in <= "\r";
        tx_buf.write <= 1;
        fsm_tx <= tx_lf_s;
      end
      tx_lf_s : begin
        tx_buf.data_in <= "\n";
        tx_buf.write <= 1;
        fsm_tx <= tx_wait_rst_s;
      end
      tx_string_s : begin // transmit response string
        cur_rsp_ctr <= cur_rsp_ctr + 1;
        if (cur_rsp_ctr == STRING_LEN-1) fsm_tx <= tx_stop_s;
        if (cur_rsp_string[0] != 0) tx_buf.write <= 1;
        else if (cur_rsp_ctr == 0) tx_buf.write <= 0;
        cur_rsp_string[0:STRING_LEN-2] <= cur_rsp_string[1:STRING_LEN-1];
        tx_buf.data_in <= cur_rsp_string[0];
      end
      tx_wait_rst_s : begin
        tx_buf.write <= 0;
        tx_buf.data_in <= 0;
        tx_done <= 1;
      end
      default fsm_tx <= tx_idle_s;
    endcase
  end
end

always @ (posedge clk) begin
  if (rst) begin
    tx_buf.read <= 0;
  end
  else begin
    tx_buf.read <= (cts && !tx_buf.empty && !(READ_BY_ONE && tx_buf.read)); // readout bytes by one if clear to send
  end
end

assign txv = tx_buf.valid_out;
assign txd = tx_buf.data_out;

assign txen = !tx_buf.empty;

endmodule : p10
