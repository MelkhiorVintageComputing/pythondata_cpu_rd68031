// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// The on-chip instruction cache -- UM section 6, 6.1.1 and figure 6-2.
//
// "Both on-chip caches are 256-byte direct-mapped caches, each organized as 16
// lines. Each line consists of four entries, and each entry contains four bytes.
// The tag field for each line contains a valid bit for each entry in the line;
// each entry is independently replaceable" (UM 6.1). For the instruction cache
// the tag is "the 24 most significant logical address bits, the FC2 function
// code bit ... and the four valid bits"; "address bits A7-A4 select one of 16
// lines", "a cache hit occurs when there is a tag match and the corresponding
// valid bit (selected by A3-A2) is set" (UM 6.1.1). A1 picks the word, which is
// the cache holding register's business: a lookup here answers with the long
// word.
//
// THE THREE STORES, AND WHICH ARE RESET. The data is a 64 x 32 RAM, written on
// a rising edge and read without one, and is NOT reset -- no flop in it, and
// nothing reads it for an entry whose valid bit is clear. The sixteen tags and
// the sixty-four valid bits are flip-flops with an asynchronous reset, and the
// valid bits are the whole of UM 6.2: "when a hardware reset of the processor
// occurs, all valid bits of both caches are cleared". A tag is only ever looked
// at beside a valid bit, so its reset value is never observed either; it is
// reset because it is a flop, and every flop is.
//
// WHAT A FILL DOES TO THE LINE (UM 6.1.1): "if the function code and address
// bits match and the corresponding long word is not valid (but one or more of
// the other three valid bits for that line are set) a single entry fill
// operation replaces the required long word only". If the tag does NOT match,
// the line now belongs to the new address: its tag is replaced and the other
// three entries, which belonged to the old one, are invalidated. A burst then
// validates the rest of the line one long word at a time, each through the same
// port, as its beats arrive.
//
// `cold` is UM 6.1.3.2's condition for a burst: "a read cycle ... misses due to
// the indexed tag not matching", or "a read cycle tag matches, but all long
// words in the line are invalid".

module rd68031_icache (
    input  logic        clk,
    input  logic        rst_n,

    // Lookup: the long word at {la, 2'b00} in supervisor (fc2) or user program
    // space. Combinational.
    input  logic [29:0] la,
    input  logic        lfc2,
    output logic        hit,
    output logic        cold,         // the line would be burst filled
    output logic [31:0] rdata,

    // Fill: one long word, with its tag -- UM 6.1.1 and 6.1.3.
    input  logic        fill,
    input  logic [29:0] fa,
    input  logic        ffc2,
    input  logic [31:0] fdata,

    // Invalidation. `inv_all` is CACR's CI (UM 6.3.1.8) and the RESET pin's
    // reset; `inv_caar` is CEI, the entry CAAR's index field names (UM
    // 6.3.1.9, 6.3.2); `inv_ent` and `inv_line` are a bus error on a fill --
    // "the cache entry corresponding to that cycle is marked invalid", and on
    // the first cycle of a burst "the entire associated cache line is marked
    // invalid" (UM 6.1.3.2).
    input  logic        inv_all,
    input  logic        inv_caar,
    input  logic  [5:0] caar_idx,     // CAAR bits 7-2
    input  logic        inv_ent,
    input  logic        inv_line,
    input  logic  [5:0] err_idx       // the long word that faulted, A7-A2
);

  localparam int TW = 25;              // A31-A8 and FC2

  // --------------------------------------------------------------------------
  // The data RAM: written on the rising edge, read without one.
  // --------------------------------------------------------------------------
  logic [31:0] mem [0:63];
  logic  [5:0] fidx;
  assign fidx = fa[5:0];

  always_ff @(posedge clk) begin
    if (fill) mem[fidx] <= fdata;
  end

  assign rdata = mem[la[5:0]];

  // --------------------------------------------------------------------------
  // The tags, one register per line. A generate block of sixteen registers
  // rather than an array written by index: an indexed write inside an
  // asynchronously reset block makes yosys stage the index and a lookahead copy
  // in registers of their own, dead flops that `make audit` counts.
  // --------------------------------------------------------------------------
  // ... read through an unpacked array: an indexed part-select of a packed
  // vector (`tags[line*TW +: TW]`) has Quartus multiply the index by TW in a
  // DSP block ahead of a shifter (M15).
  wire  [TW-1:0]    tags [0:15];
  logic [TW-1:0]    ltag, ftag;
  logic [3:0]       lline, fline;
  logic [63:0]      valid_q;

  assign lline = la[5:2];
  assign fline = fa[5:2];
  assign ltag  = {la[29:6], lfc2};
  assign ftag  = {fa[29:6], ffc2};

  // The fill's line belongs to another address: the tag is replaced, and the
  // three other entries go with the old one.
  logic ftag_match;
  assign ftag_match = (tags[fline] == ftag);

  genvar g;
  generate
    for (g = 0; g < 16; g = g + 1) begin : g_line
      logic [TW-1:0] tag_q;
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                     tag_q <= '0;
        else if (fill && (fline == g) && !ftag_match)   tag_q <= ftag;
      end
      assign tags[g] = tag_q;
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Lookup
  // --------------------------------------------------------------------------
  logic       ltag_match;
  logic [3:0] lvalid;
  assign ltag_match = (tags[lline] == ltag);
  assign lvalid     = valid_q[{lline, 2'b00} +: 4];
  assign hit        = ltag_match && valid_q[la[5:0]];
  assign cold       = !ltag_match || (lvalid == 4'b0000);

  // --------------------------------------------------------------------------
  // The valid bits, as whole-vector updates (see the tags above). An
  // invalidation in the same clock as a fill wins: CI means "every instruction
  // the cache holds from before this is suspect", and a long word read from
  // memory before the MOVEC finished is one of them.
  // --------------------------------------------------------------------------
  localparam logic [63:0] ONE = 64'd1;

  logic [63:0] set_m, line_m, clr_m;
  assign set_m  = fill ? (ONE << fa[5:0]) : '0;
  assign line_m = (fill && !ftag_match) ? (64'hF << {fline, 2'b00}) : '0;
  assign clr_m  = (inv_caar ? (ONE << caar_idx) : '0)
                | (inv_ent  ? (ONE << err_idx)  : '0)
                | (inv_line ? (64'hF << {err_idx[5:2], 2'b00}) : '0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)       valid_q <= '0;
    else if (inv_all) valid_q <= '0;
    else              valid_q <= ((valid_q & ~line_m) | set_m) & ~clr_m;
  end

endmodule
