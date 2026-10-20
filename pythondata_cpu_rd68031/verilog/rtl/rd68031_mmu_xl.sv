// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// One address translation: the transparent translation registers and the
// address translation cache looked up for one logical address, and the verdict
// -- UM 9.2.1 and figure 9-8. Purely combinational; rd68031_mmu instantiates
// it once, for the cycle the bus unit is about to start and, on the two
// clocks the walker reads it, for the walker's own address (until M15 phase B
// a second instance served the walker), and owns every register it reads.
//
// The order is figure 9-8's: CPU space is never translated; a TTx match is
// used as it is; with translation disabled (TC.E clear, or MMUDIS asserted --
// UM 9.2.3) the logical address is the physical one; and otherwise the ATC.
//
//   kind 00  the access may go ahead at `pb` | (la & mask), with CIOUT = `ci`
//        01  it may not: a bus error on the access -- an entry with B set, or
//            a write (or any cycle of a read-modify-write, UM 8.2.3) to an
//            entry with WP set
//        10  a table search is needed first: a miss, or a write to an entry
//            whose M bit is clear (UM 9.4, "the MC68030 aborts the access and
//            initiates a table search, setting the M bit")

module rd68031_mmu_xl #(
    parameter int N = 22
) (
    input  logic [31:0]     la,
    input  logic  [2:0]     fc,
    input  logic            rw,       // 1 = read
    input  logic            rmc,
    input  logic            xlate,    // TC.E set and MMUDIS negated
    input  logic  [3:0]     ps,       // TC.PS
    input  logic [31:0]     tt0,
    input  logic [31:0]     tt1,
    input  logic [N-1:0]    v,
    input  logic [N*27-1:0] tag,      // per entry {FC2-FC0, LA31-LA8}
    input  logic [N*28-1:0] dat,      // per entry {B, CI, WP, M, PA31-PA8}
    output logic [N-1:0]    hitv,     // the entries whose tag matches
    output logic            hit,
    output logic            e_b,
    output logic            e_ci,
    output logic            e_wp,
    output logic            e_m,
    output logic            tt,       // TT0 or TT1 matches
    output logic [31:0]     mask,     // the page offset bits
    output logic  [1:0]     kind,
    output logic [31:0]     pb,       // the physical page base, offset bits zero
    output logic            ci,
    output logic            used      // the ATC entry decided it
);

  // UM 9.7.2: PS is 1000 (256 bytes) to 1111 (32K bytes); the other codes are
  // refused by the configuration check, but a TC loaded with translation
  // disabled can hold anything, and PTEST and PLOAD still run then. Below
  // eight is taken as eight: nothing is translated with such a TC, and every
  // page size gives the same identity mapping.
  logic [4:0] psv;
  assign psv  = (ps < 4'd8) ? 5'd8 : {1'b0, ps};
  assign mask = ~(32'hFFFF_FFFF << psv);

  // UM 9.4: "All 24 bits of this field are used in the comparison of this
  // entry to an incoming logical address when the page size is 256 bytes. For
  // larger page sizes, the appropriate number of least significant bits of
  // this field are ignored."
  logic [23:0] cmp_m;
  assign cmp_m = ~mask[31:8];

  int unsigned i_h;
  always_comb begin
    for (i_h = 0; i_h < N; i_h = i_h + 1)
      hitv[i_h] = v[i_h] && (tag[i_h*27 + 24 +: 3] == fc)
               && (((tag[i_h*27 +: 24] ^ la[31:8]) & cmp_m) == 24'd0);
  end

  // The lowest matching entry. Two can only match after a PMOVEFD changed the
  // page size under live entries, which UM 9.7.5.1 leaves to the programmer.
  logic [27:0] e_dat;
  int unsigned i_s;
  always_comb begin
    hit   = 1'b0;
    e_dat = 28'd0;
    for (i_s = N; i_s > 0; i_s = i_s - 1)
      if (hitv[i_s - 1]) begin
        hit   = 1'b1;
        e_dat = dat[(i_s - 1)*28 +: 28];
      end
  end
  assign e_b  = e_dat[27];
  assign e_ci = e_dat[26];
  assign e_wp = e_dat[25];
  assign e_m  = e_dat[24];

  // UM 9.3 and 9.7.3, figure 9-37: E bit 15, CI 10, R/W 9, RWM 8, FC BASE 6:4,
  // FC MASK 2:0, LOGICAL ADDRESS BASE 31:24, LOGICAL ADDRESS MASK 23:16. "The
  // read/write mask bit (RWM) must be set for transparent translation of
  // addresses used by instructions that execute read-modify-write operations.
  // Otherwise, neither the read nor write portions of read-modify-write
  // operations are mapped transparently." CPU space is never translated at
  // all (figure 9-8's first branch), so no TTx matches it.
  logic m0, m1;
  always_comb begin
    m0 = tt0[15] && (fc != 3'b111)
      && (((tt0[6:4] ^ fc) & ~tt0[2:0]) == 3'd0)
      && (((tt0[31:24] ^ la[31:24]) & ~tt0[23:16]) == 8'd0)
      && (tt0[8] || (!rmc && (tt0[9] == rw)));
    m1 = tt1[15] && (fc != 3'b111)
      && (((tt1[6:4] ^ fc) & ~tt1[2:0]) == 3'd0)
      && (((tt1[31:24] ^ la[31:24]) & ~tt1[23:16]) == 8'd0)
      && (tt1[8] || (!rmc && (tt1[9] == rw)));
  end
  assign tt = m0 || m1;

  // UM 9.4 and 9.5.1.1: a write -- and "an access is considered to be a write
  // for updating purposes if either the R/W or RMC signal is low", which UM
  // 8.2.3 repeats for protection.
  logic wr;
  assign wr = !rw || rmc;

  always_comb begin
    kind = 2'b00;
    pb   = la & ~mask;
    ci   = 1'b0;
    if (fc == 3'b111) begin
      // Figure 9-8: "PA <- LA, CIOUT <- 0".
      kind = 2'b00;
    end else if (tt) begin
      // UM 9.3: "If both registers match, the CI bits are ORed together".
      ci   = (m0 && tt0[10]) || (m1 && tt1[10]);
    end else if (!xlate) begin
      kind = 2'b00;
    end else if (!hit) begin
      kind = 2'b10;
    end else if (e_b || (e_wp && wr)) begin
      kind = 2'b01;
    end else if (!e_m && wr) begin
      kind = 2'b10;
    end else begin
      pb   = {e_dat[23:0], 8'd0} & ~mask;
      ci   = e_ci;
    end
  end

  assign used = hit && xlate && !tt && (fc != 3'b111);

  logic unused_xl;
  assign unused_xl = &{1'b1, tt0[14:11], tt0[7], tt0[3],
                       tt1[14:11], tt1[7], tt1[3]};

endmodule
