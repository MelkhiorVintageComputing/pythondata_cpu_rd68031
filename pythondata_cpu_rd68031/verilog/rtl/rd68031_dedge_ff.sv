// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// An output that changes on both clock edges, built from a positive-edge flop and a
// negative-edge flop combined with XOR.
//
// DBEN is the pin that needs it: on a read it is asserted on the rising edge
// entering S2 and negated on the falling edge entering S5; on a write it is
// asserted on the falling edge entering S1 and negated on the rising edge that
// ends S5 (UM 7.1.6, specifications 40 to 43). Decoded from the two bus-state
// registers instead, it glitches whenever two state bits change on one edge --
// doc/bugs-found.md.
//
// Only one side can change at any instant, so the XOR is glitch-free, and every one
// of the six front-ends infers it correctly (doc/coding-standard.md, measured).
//
// toggle_p flips the positive-edge half, toggle_n the negative-edge half. The
// output is the XOR, so the caller toggles a half exactly when the value after
// that edge is to differ from the value before it.

module rd68031_dedge_ff #(
    parameter bit RESET_VAL = 1'b0
) (
    input  logic clk,
    input  logic rst_n,
    input  logic toggle_p,   // flip on the next rising edge
    input  logic toggle_n,   // flip on the next falling edge
    output logic q
);

  logic half_p;
  logic half_n;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      half_p <= RESET_VAL;
    end else if (toggle_p) begin
      half_p <= ~half_p;
    end
  end

  always_ff @(negedge clk or negedge rst_n) begin
    if (!rst_n) begin
      half_n <= 1'b0;
    end else if (toggle_n) begin
      half_n <= ~half_n;
    end
  end

  assign q = half_p ^ half_n;

endmodule
