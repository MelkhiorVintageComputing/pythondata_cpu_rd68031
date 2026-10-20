// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68884

// RD68884 - SystemVerilog MC68881 floating-point coprocessor
//
// Two-rank input synchroniser.
//
// FPU 10.4: the asynchronous bus cycles are "independent of the FPCP clock", so
// every strobe the main processor drives (CS, AS, DS, R/W) and RESET can change
// at any instant relative to clk_core. Two ranks bring each into the core domain.
// The design has one clock edge (doc/coding-standard.md), so both ranks are
// positive-edge clocked.

module rd68884_sync #(
    parameter int WIDTH = 1,
    // Explicit, because there is no power-on state: an active-low input resets to
    // its negated value so that nothing looks asserted while rst_n is held. Sized
    // by WIDTH rather than fixed at 32 bits, so instantiations do not widen it.
    parameter logic [WIDTH-1:0] RESET_VAL = '1
) (
    input  logic             clk,
    input  logic             rst_n,
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
);

  logic [WIDTH-1:0] rank0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rank0 <= RESET_VAL;
      q     <= RESET_VAL;
    end else begin
      rank0 <= d;
      q     <= rank0;
    end
  end

endmodule
