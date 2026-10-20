// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68884

// RD68885 - the MC68882 build of RD68884 (doc/rd68885.md)
//
// What the conversion unit does with a command word (FPU 5.1.1.2):
//   takes  it takes it while the APU computes: register to register (FMOVECR
//          too), or opclass 010 with a B, W, L, S, D or X source -- not an
//          illegal command word (FPU 6.1.11), not packed decimal;
//   prim   its first primitive: the null CA = 0 release ($0900, figure 7-17);
//          evaluate <ea> and transfer data with CA = 0 for S, D, X ($1504,
//          $1608, $160C, figure 7-19), with CA = 1 for B, W, L ($9501,
//          $9502, $9504, figure 7-18); the PC bit if an exception is enabled;
//   n      the operand long words that follow;
//   bwl    a B, W or L source, which waits for the APU with $8900;
//   len    the operand's length for the port sizing (FPU 10.1): 1 byte,
//          2 word, 0 a long word or more.
// Combinational; tools/iss/biu.py (cu_takes, cu_first) is the definition.

module rd68884_cu_decode (
    input  logic [15:0] word,
    input  logic        pcen,
    output logic        takes,
    output logic [15:0] prim,
    output logic [1:0]  n,
    output logic        bwl,
    output logic [1:0]  len
);

  logic [2:0] opclass, rx;
  assign opclass = word[15:13];
  assign rx      = word[12:10];
  logic unused_word;             // the destination and the opmode: not the CU's concern
  assign unused_word = &{1'b0, word[9:7], word[5:0]};

  always_comb begin
    takes = 1'b0;
    prim  = {1'b0, pcen, 14'h0900};
    n     = 2'd0;
    bwl   = 1'b0;
    len   = 2'd0;
    if (opclass == 3'd0) begin
      takes = ~word[6];
    end else if (opclass == 3'd2) begin
      takes = (rx == 3'd7) | (~word[6] & (rx != 3'd3));
      case (rx)
        3'd1: begin prim = {1'b0, pcen, 14'h1504}; n = 2'd1; end                 // S
        3'd5: begin prim = {1'b0, pcen, 14'h1608}; n = 2'd2; end                 // D
        3'd2: begin prim = {1'b0, pcen, 14'h160C}; n = 2'd3; end                 // X
        3'd0: begin prim = {1'b1, pcen, 14'h1504}; n = 2'd1; bwl = 1'b1; end     // L
        3'd4: begin prim = {1'b1, pcen, 14'h1502}; n = 2'd1; bwl = 1'b1; len = 2'd2; end // W
        3'd6: begin prim = {1'b1, pcen, 14'h1501}; n = 2'd1; bwl = 1'b1; len = 2'd1; end // B
        default: ;                                                              // FMOVECR, P
      endcase
    end
  end

endmodule
