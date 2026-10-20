// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// Two-rank input synchroniser.
//
// UM 7.1: "for all asynchronous inputs, the processor latches the level of the
// input during a sample window around the falling edge of the clock signal"
// (figure 7-2), and figure 7-1 shows the synchronisation delay that follows. Both
// ranks are therefore negative-edge clocked.
//
// This is for the inputs that may change at any time relative to a bus cycle: IPL,
// RESET, HALT, CDIS, MMUDIS, BR and BGACK. DSACK, BERR, AVEC and the HALT that
// terminates a cycle are sampled directly by the bus unit's falling-edge
// next-state logic, which is the sample UM 7.3.1 describes and which must not have
// a rank of latency added to it. The synchronous inputs -- STERM, CBACK, CIIN --
// need no synchroniser at all: "input signals are externally synchronized to the
// processor clock" (UM 7.1).

module rd68031_sync #(
    parameter int WIDTH = 1,
    // Explicit, because there is no power-on state: an active-low input resets to
    // its negated value so that nothing looks asserted while rst_n is held. Sized
    // by WIDTH rather than fixed at 32 bits, or every instantiation is a
    // width-expansion warning.
    parameter logic [WIDTH-1:0] RESET_VAL = '1
) (
    input  logic             clk,
    input  logic             rst_n,
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
);

  logic [WIDTH-1:0] rank0;

  always_ff @(negedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rank0 <= RESET_VAL;
      q     <= RESET_VAL;
    end else begin
      rank0 <= d;
      q     <= rank0;
    end
  end

endmodule
