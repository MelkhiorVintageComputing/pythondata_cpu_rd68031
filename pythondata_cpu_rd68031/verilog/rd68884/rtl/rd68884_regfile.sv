// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68884

// RD68884 - SystemVerilog MC68881 floating-point coprocessor
//
// The register file: FP0-FP7, the exceptional operand (entry 8) and the
// microcode's temporaries, 128 entries of {sign, exponent[17:0],
// mantissa[71:0]} (doc/microcode.md). Port A is the sequencer's: a write, or
// a read with a registered output, at one address. Port B is RD68885's
// conversion unit's, the same (doc/rd68885.md): only with PORT_B = 1, the
// MC68882 build. A true dual-port block RAM on every
// target. The two ports never touch one entry in a clock: the CU writes only
// registers the APU's instruction does not (FPU table 5-5).
//
// Neither the array nor the read register is reset. The array is not a
// register (tools/reset_audit.py does not count memories), and the reset
// microcode writes every entry the programmer can see before anything reads
// it (FPU 9.9). The read registers are named exemptions in
// tools/reset_audit.py: a block RAM keeps them inside the primitive, and
// nothing reads one before its port has issued a read.

module rd68884_regfile #(
    parameter int PORT_B = 0
) (
    input  logic        clk,
    input  logic        we,              // port A: write, or read, at a
    input  logic        re,
    input  logic [6:0]  a,
    input  logic [90:0] wd,
    output logic [90:0] q,
    input  logic        web,             // port B: the same, at ab
    input  logic        reb,
    input  logic [6:0]  ab,
    input  logic [90:0] wdb,
    output logic [90:0] qb
);

  // One address per port, the memory declared where it is used: the shape
  // every tool's block-RAM inference takes (Vivado refuses a third address).
  generate
    if (PORT_B != 0) begin : g_tdp
      (* ram_style = "block" *)
      logic [90:0] mem [0:127];
      always_ff @(posedge clk) begin
        if (we) begin
          mem[a] <= wd;
        end
        if (re) begin
          q <= mem[a];
        end
      end
      always_ff @(posedge clk) begin
        if (web) begin
          mem[ab] <= wdb;
        end
        if (reb) begin
          qb <= mem[ab];
        end
      end
    end else begin : g_sdp
      (* ram_style = "block" *)
      logic [90:0] mem [0:127];
      always_ff @(posedge clk) begin
        if (we) begin
          mem[a] <= wd;
        end
        if (re) begin
          q <= mem[a];
        end
      end
      assign qb = 91'd0;
      logic unused_b;
      assign unused_b = &{1'b0, web, reb, ab, wdb};
    end
  endgenerate

endmodule
