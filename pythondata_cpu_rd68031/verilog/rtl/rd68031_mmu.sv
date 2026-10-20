// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// The memory management unit -- UM section 9. Its registers (TC, CRP, SRP,
// TT0, TT1, MMUSR -- UM 9.7), the 22-entry address translation cache (UM
// 9.4), the lookups the bus unit makes before it starts a cycle (UM 9.2.1),
// and the table walker (UM 9.5.2, 9.5.4, figures 9-25 to 9-29).
//
// THE WALKER IS HARDWARE, NOT MICROCODE. UM 9.1 says the part's search is
// "performed by microcode and dedicated logic"; what is architectural is the
// search's bus traffic and its results, and both are reproduced here. A
// search starts when an access -- a data operand, an instruction prefetch,
// RTE's rerun of a faulted operand, a posted write -- needs a translation the
// ATC does not have, and most of those have no microword to run it: the fetch
// unit prefetches by itself and a posted write belongs to no microword
// (doc/checkpoint.md rule 9). A walker in the bus unit's shadow starts from the
// access, needs nothing from the sequencer, and makes rule 11 (a search is
// never checkpointed or interrupted) a property of the hardware. doc/mmu.md has
// the argument in full.
//
// It is a requester of the bus unit like the fetch unit: four-byte operands at
// physical addresses, in supervisor data space with RMC asserted -- UM 7.3.3,
// "table search accesses required for the MMU are always read-modify-write
// cycles to the supervisor data space" -- dynamically sized, never cached and
// never with CIOUT (UM 12.1: "the MC68030 asserts RMC but not CIOUT").
//
// PTEST and PLOAD use the same walker, started by the sequencer, which stalls
// until it is done.

module rd68031_mmu #(
    parameter int N = 22                   // UM 9.4: 22 entries
) (
    input  logic        clk,
    input  logic        rst_n,
    // The RESET pin -- UM 8.1.1 step 7 and 9.2.2: "the E bits of the TC and
    // TTx registers are cleared ... A reset of the processor does not
    // invalidate any entries in the ATC". The rest of TC, the root pointers,
    // TT0, TT1, MMUSR and the ATC keep what they held; the walker is reset.
    input  logic        crst,
    input  logic        mmudis_sync_n,     // UM 5.11.2, 9.2.3

    // The sequencer -- the MMU instructions -------------------------------
    input  logic  [2:0] cmd,               // rd68031_ucode_pkg::U_MMU_*
    input  logic        cmd_go,            // apply it, or start the search
    input  logic [15:0] cmd_xw,            // the command word
    input  logic  [2:0] cmd_fc,            // the function code it names
    input  logic [31:0] cmd_ea,            // the logical address, for <ea>
    input  logic [31:0] wd_hi,             // PMOVE's operand: T0, the upper half
    input  logic [31:0] wd_lo,             // ... and T1
    output logic [31:0] rd_hi,             // the register the command word names
    output logic [31:0] rd_lo,
    output logic        cfg_bad,           // wd_hi/wd_lo would be refused -- 9.7.5.3
    output logic [31:0] ptest_da,          // PTEST's last descriptor address
    output logic        busy,              // a table search is under way
    output logic        done,              // PTEST or PLOAD has finished

    // The bus unit's lookup: the first byte of the operand it is about to
    // start, or of the first cycle of one that enters its next page ---------
    input  logic [31:0] l1_addr,
    input  logic  [2:0] l_fc,
    input  logic        l_rw,
    input  logic        l_rmc,
    output logic  [1:0] l1_kind,
    output logic [31:0] l1_pb,
    output logic        l1_ci,
    output logic [31:0] l_mask,
    // Whether any translation is enabled: TC.E, and TT0.E or TT1.E -- for the
    // bus unit, which needs no lookup for an operand's later page when none
    // is (figure 9-8).
    output logic        l_tc_on,
    output logic        l_tt_on,
    input  logic        l_take,            // the cycle looked up was started

    // ... and its request for a table search -------------------------------
    input  logic        wq_valid,
    input  logic [31:0] wq_addr,
    input  logic  [2:0] wq_fc,
    input  logic        wq_wr,
    output logic        w_sts,             // a search for an access begins

    // The walker's own bus operands ----------------------------------------
    output logic        tw_valid,
    output logic        tw_want,          // tw_valid but for tw_last
    output logic [31:0] tw_addr,
    output logic        tw_wr,
    output logic [31:0] tw_wdata,
    input  logic        tw_ack,
    input  logic        tw_last,
    input  logic [31:0] tw_rdata,
    input  logic        tw_berr
);

  // ==========================================================================
  // The registers -- UM 9.7
  // ==========================================================================
  logic [31:0] tc_q;
  logic [63:0] crp_q, srp_q;
  logic [31:0] tt0_q, tt1_q;
  logic [15:0] mmusr_q;

  // UM 9.7.2: "All unimplemented fields of this register are read as zeros and
  // must always be written as zeros" -- E (31), SRE (25), FCL (24), PS, IS and
  // the four TIx fields. UM 9.5.1.1 makes the reserved bits of the root
  // pointers "not alterable", and UM 9.7.1 says bits 3-0 "are not used and are
  // ignored when written": a root pointer keeps L/U, LIMIT, DT and the table
  // address. TT0 and TT1 keep the fields figure 9-37 names, MMUSR the bits
  // figure 9-38 does.
  localparam logic [31:0] TC_BITS    = 32'h83FF_FFFF;
  localparam logic [31:0] RPHI_BITS  = 32'hFFFF_0003;
  localparam logic [31:0] RPLO_BITS  = 32'hFFFF_FFF0;
  localparam logic [31:0] TT_BITS    = 32'hFFFF_8777;
  localparam logic [15:0] MMUSR_BITS = 16'hEE47;

  logic       tc_e, tc_sre, tc_fcl;
  logic [3:0] tc_ps, tc_is, tc_tia, tc_tib, tc_tic, tc_tid;
  assign tc_e   = tc_q[31];
  assign tc_sre = tc_q[25];
  assign tc_fcl = tc_q[24];
  assign tc_ps  = tc_q[23:20];
  assign tc_is  = tc_q[19:16];
  assign tc_tia = tc_q[15:12];
  assign tc_tib = tc_q[11:8];
  assign tc_tic = tc_q[7:4];
  assign tc_tid = tc_q[3:0];

  // Which register a PMOVE command word names -- PRM 6: bits 15:13 are 000
  // for TT0 (P-register 010) and TT1 (011), 010 for TC (000), SRP (010) and CRP
  // (011), and 011 for MMUSR.
  logic [2:0] grp, preg;
  assign grp  = cmd_xw[15:13];
  assign preg = cmd_xw[12:10];

  always_comb begin
    rd_hi = 32'd0;
    rd_lo = 32'd0;
    if (grp == 3'b000) begin
      rd_lo = (preg == 3'b010) ? tt0_q : tt1_q;
    end else if (grp == 3'b010) begin
      unique case (preg)
        3'b000:  rd_lo = tc_q;
        3'b010:  begin rd_hi = srp_q[63:32]; rd_lo = srp_q[31:0]; end
        default: begin rd_hi = crp_q[63:32]; rd_lo = crp_q[31:0]; end
      endcase
    end else begin
      rd_lo = {16'd0, mmusr_q};
    end
  end

  // UM 9.7.5.3: "When the TC register is loaded with the E bit set, the MMU
  // performs a consistency check of the values in all the four bit fields.
  // The values in the TIx fields are added until the first zero is
  // encountered. The values in the PS and IS fields are added to the sum of
  // the TIx fields. If the sum is not equal to 32, the PMOVE instruction
  // causes an MMU configuration exception. The instruction also causes a
  // configuration exception when a reserved value ($0-$7) is placed in the PS
  // field." And "a PMOVE instruction that loads either the CRP or the SRP
  // causes an MMU configuration exception if the new value of the DT field is
  // zero".
  logic [5:0] cfg_sum;
  logic       tc_bad;
  always_comb begin
    cfg_sum = {2'd0, wd_lo[23:20]} + {2'd0, wd_lo[19:16]} + {2'd0, wd_lo[15:12]};
    if (wd_lo[15:12] != 4'd0) begin
      cfg_sum = cfg_sum + {2'd0, wd_lo[11:8]};
      if (wd_lo[11:8] != 4'd0) begin
        cfg_sum = cfg_sum + {2'd0, wd_lo[7:4]};
        if (wd_lo[7:4] != 4'd0) cfg_sum = cfg_sum + {2'd0, wd_lo[3:0]};
      end
    end
    tc_bad = wd_lo[31] && ((wd_lo[23:20] < 4'd8) || (cfg_sum != 6'd32));
  end
  assign cfg_bad = (grp == 3'b010)
                && (((preg == 3'b000) && tc_bad)
                    || (((preg == 3'b010) || (preg == 3'b011)) && (wd_hi[1:0] == 2'b00)));

  // ==========================================================================
  // The address translation cache -- UM 9.4
  //
  // Each entry is the 28-bit tag of the manual (V, FC, LA31-LA8) and its
  // 28-bit data (B, CI, WP, M, PA31-PA8), and one history bit for the
  // replacement: "If possible, when the ATC stores a new address translation,
  // it replaces an entry that is no longer valid. When all entries in the ATC
  // are valid, the ATC selects a valid entry to be replaced, using a pseudo
  // least recently used algorithm. The ATC uses a validity bit and an internal
  // history bit to implement this replacement algorithm." The history bit is
  // set when the entry is loaded and when an access the bus unit starts uses
  // it; once every valid entry's is set, all but the ones just set are
  // cleared. The victim is the lowest invalid entry, or else the lowest whose
  // history bit is clear.
  // ==========================================================================
  logic [N-1:0] v_q, h_q;
  logic [26:0]  tag_q [0:N-1];
  logic [27:0]  dat_q [0:N-1];

  logic [N*27-1:0] tag_flat;
  logic [N*28-1:0] dat_flat;
  genvar ge;
  generate
    for (ge = 0; ge < N; ge = ge + 1) begin : g_flat
      assign tag_flat[ge*27 +: 27] = tag_q[ge];
      assign dat_flat[ge*28 +: 28] = dat_q[ge];
    end
  endgenerate

  // UM 9.2.3: MMUDIS "prevents the MMU from performing searches of the ATC
  // and the execution unit from performing table searches"; UM 9.7.2, TC.E.
  logic xlate;
  assign xlate = tc_e && mmudis_sync_n;

  logic [N-1:0] h1v, h3v;
  logic         hit3, e3_b, e3_wp, e3_m, tt3;

  // The walker's own address, for PTEST level 0 and for the entry a search
  // replaces.
  logic [31:0] w_la;
  logic  [2:0] w_fc;
  logic        w_rw;


  // The replacement victim.
  logic [4:0] vict;
  int unsigned i_vc;
  always_comb begin
    vict = 5'd0;
    for (i_vc = N; i_vc > 0; i_vc = i_vc - 1)
      if (h_q[i_vc - 1] == 1'b0) vict = 5'(i_vc - 1);
    for (i_vc = N; i_vc > 0; i_vc = i_vc - 1)
      if (v_q[i_vc - 1] == 1'b0) vict = 5'(i_vc - 1);
  end

  // ... and the entry a search's result goes into: the one already holding
  // that address, if any -- UM 9.4, PLOAD "writes a new entry ... for the
  // specified logical address", and a search to set the M bit is "invalidating
  // the old ATC entry, and creating a new entry with the M bit set" -- or the
  // victim.
  logic [4:0] wslot;
  int unsigned i_ws;
  always_comb begin
    wslot = vict;
    for (i_ws = N; i_ws > 0; i_ws = i_ws - 1)
      if (h3v[i_ws - 1]) wslot = 5'(i_ws - 1);
  end

  // PFLUSH -- PRM 6, MC68030: "Each bit in the mask that is set to one
  // indicates that the corresponding bit of the FC operand applies to the
  // operation", and with an <ea> "the instruction invalidates the page
  // descriptor for that effective address entry in each selected function
  // code".
  logic [N-1:0] fl_hit;
  logic [23:0]  fl_cmp;
  assign fl_cmp = ~l_mask[31:8];
  int unsigned i_fl;
  always_comb begin
    for (i_fl = 0; i_fl < N; i_fl = i_fl + 1)
      fl_hit[i_fl] = (((tag_q[i_fl][26:24] ^ cmd_fc) & cmd_xw[7:5]) == 3'd0)
                  && ((cmd != rd68031_ucode_pkg::U_MMU_FLUSHEA)
                      || (((tag_q[i_fl][23:0] ^ cmd_ea[31:8]) & fl_cmp) == 24'd0));
  end

  // ==========================================================================
  // The table walker -- UM 9.5.2, figures 9-25 to 9-29
  // ==========================================================================
  localparam logic [3:0] W_IDLE = 4'd0;
  localparam logic [3:0] W_INIT = 4'd1;   // choose the root pointer
  localparam logic [3:0] W_LVL  = 4'd2;   // limit check, then the next table
  localparam logic [3:0] W_F0   = 4'd3;   // fetch a descriptor's first long word
  localparam logic [3:0] W_F1   = 4'd4;   // ... and a long one's second
  localparam logic [3:0] W_DESC = 4'd5;   // what it says
  localparam logic [3:0] W_UPD  = 4'd6;   // the U or M bit written back
  localparam logic [3:0] W_ET   = 4'd7;   // early termination: limit, address
  localparam logic [3:0] W_FIN  = 4'd8;   // the ATC entry, or MMUSR
  localparam logic [3:0] W_PT0  = 4'd9;   // PTEST level 0: the ATC alone

  localparam logic [1:0] K_ACC   = 2'd0;  // an access needs the translation
  localparam logic [1:0] K_PLOAD = 2'd1;
  localparam logic [1:0] K_PTEST = 2'd2;

  logic [3:0]  w_st, w_ret;
  logic [1:0]  w_kind;
  logic        w_wr;                      // a write for history: M is set
  logic [2:0]  w_lvl;                     // PTEST's level operand
  logic [2:0]  w_x;                       // 0 FC level, 1-4 levels A-D
  logic [5:0]  w_pos;                     // bits of LA consumed: IS + TIx so far
  logic [31:0] w_ta;                      // the table base
  logic [1:0]  w_dt;                      // the size of its descriptors
  logic        w_limv, w_lu;              // a limit applies to the next index
  logic [14:0] w_lim;
  logic        w_wp, w_s, w_ci, w_m;      // accrued status -- figure 9-26
  logic [2:0]  w_n;                       // tables accessed -- MMUSR N
  logic [31:0] w_d0, w_d1;
  logic [31:0] w_daddr;                   // the descriptor being fetched
  logic [31:0] w_lastda;                  // the last one fetched whole
  logic        w_inval, w_limx, w_berr;   // how it ended, if badly
  logic        w_indir;                   // the fetch under way is indirect
  logic [31:0] w_ebase;                   // an early termination's page address
  logic [23:0] w_pfa;                     // the page frame address
  logic        w_pend;
  logic [31:0] w_raddr, w_rdat;
  logic        w_rwr;
  logic        done_q, sts_q;

  // ONE LOOKUP, SHARED (M15 phase B, doc/size-and-speed.md). The walker
  // asks the ATC two things, each on one clock of its own: at W_FIN, which
  // entry already holds the address it is about to write (`wslot`), and at
  // W_PT0, what PTEST level 0 reports. On both clocks a search, or PTEST, is
  // under way, and the bus unit takes nothing that it would look up: every
  // requester but the walker waits for `busy`, an operand that waited for the
  // search is not looked up until it is over, and the walker's own operands
  // are physical. So the bus unit's lookup serves the walker on those two
  // clocks, chosen by the walker's state -- a register -- and what the bus
  // unit is told on them is never acted on: it has no new candidate (`busy`)
  // and no operand of its own entering a page. Before M15 phase B this was a
  // second comparator of 22 ways, `u_xl3`.
  logic w_look;
  assign w_look = (w_st == W_FIN) || (w_st == W_PT0);

  logic [31:0] x_la;
  logic  [2:0] x_fc;
  logic        x_rw, x_rmc;
  assign x_la  = w_look ? w_la : l1_addr;
  assign x_fc  = w_look ? w_fc : l_fc;
  assign x_rw  = w_look ? w_rw : l_rw;
  assign x_rmc = w_look ? 1'b0 : l_rmc;

  logic u1;
  logic h1, e1_b, e1_ci, e1_wp, e1_m, t1m;

  rd68031_mmu_xl #(.N (N)) u_xl1 (
      .la (x_la), .fc (x_fc), .rw (x_rw), .rmc (x_rmc), .xlate (xlate),
      .ps (tc_ps), .tt0 (tt0_q), .tt1 (tt1_q), .v (v_q), .tag (tag_flat),
      .dat (dat_flat), .hitv (h1v), .hit (h1), .e_b (e1_b), .e_ci (e1_ci),
      .e_wp (e1_wp), .e_m (e1_m), .tt (t1m), .mask (l_mask), .kind (l1_kind),
      .pb (l1_pb), .ci (l1_ci), .used (u1));

  // What the walker reads of it.
  assign h3v   = h1v;
  assign hit3  = h1;
  assign e3_b  = e1_b;
  assign e3_wp = e1_wp;
  assign e3_m  = e1_m;
  assign tt3   = t1m;

  // The table index fields -- UM 9.1.1, Table 9-1. The index at a level is the
  // TIx bits after the IS bits and the indexes above it.
  logic [3:0] ti_x, ti_n;
  always_comb begin
    unique case (w_x)
      3'd1:    ti_x = tc_tia;
      3'd2:    ti_x = tc_tib;
      3'd3:    ti_x = tc_tic;
      3'd4:    ti_x = tc_tid;
      default: ti_x = 4'd0;
    endcase
    unique case (w_x)
      3'd0:    ti_n = tc_tia;
      3'd1:    ti_n = tc_tib;
      3'd2:    ti_n = tc_tic;
      3'd3:    ti_n = tc_tid;
      default: ti_n = 4'd0;
    endcase
  end

  logic [31:0] la_sh;
  logic [14:0] idx;
  logic [31:0] pg_mask, off;
  always_comb begin
    la_sh = (w_pos >= 6'd32) ? 32'd0 : (w_la << w_pos[4:0]);
    if (ti_x == 4'd0) idx = 15'd0;
    else              idx = 15'(la_sh >> (6'd32 - {2'd0, ti_x}));
    // UM 9.5.3.1: "the physical address in the ATC entry is the sum of the
    // page address field in the descriptor plus an offset. The offset is the
    // logical address with the bits used in the search set to zero" -- and
    // without the page offset, which the logical address supplies itself.
    pg_mask = l_mask;
    off     = ((w_pos >= 6'd32) ? 32'd0 : (32'hFFFF_FFFF >> w_pos[4:0]))
            & w_la & ~pg_mask;
  end

  // UM 9.5.2 and figure 9-28: "When the L/U bit is set, the limit is a lower
  // limit, and an index less than the limit is out of bounds. When the L/U
  // bit is zero, the limit is an upper limit, and an index greater than the
  // limit is out of bounds."
  logic lim_bad;
  assign lim_bad = w_limv && (w_lu ? (idx < w_lim) : (idx > w_lim));

  // What a descriptor says -- UM 9.5.1, figures 9-10 to 9-18. In every format
  // DT is bits 1:0 of the first long word, WP bit 2, U bit 3; a page
  // descriptor's M is bit 4 and CI bit 6; a long one's S is bit 8, L/U bit 31
  // and LIMIT bits 30:16, and its address is in the second long word.
  logic        d_long;
  logic [1:0]  d_dt;
  logic        d_s, d_wp_n, d_s_n, d_sv;
  logic        d_u_upd, d_m_upd, d_write;
  logic [31:0] d_new;
  logic [23:0] d_pga;
  logic [31:0] d_ta, d_ia;
  logic        d_last;                    // no index field follows this level
  logic        d_ind;                     // ... and this is an indirect descriptor
  always_comb begin
    d_long  = (w_dt == 2'b11);
    d_dt    = w_d0[1:0];
    d_last  = (w_x == 3'd4) || ((w_x != 3'd0) && (ti_n == 4'd0));
    // An indirect descriptor has no status: figure 9-17 gives a short one
    // "DESCRIPTOR ADDRESS" in bits 31-2 -- bits 2 and 3 are address bits, not
    // WP and U -- and figure 9-18 a long one an unused first long word. So it
    // accrues no WP or S and has no U bit to write (doc/manual-contradictions
    // 21: figure 9-29 would write one).
    d_ind   = d_dt[1] && d_last && !w_indir;
    d_s     = d_long && w_d0[8] && !d_ind;
    d_wp_n  = w_wp || (w_d0[2] && !d_ind);
    d_s_n   = w_s || d_s;
    // UM 9.5.5.3 and Table 9-3: an S bit encountered with FC2 clear.
    d_sv    = d_s_n && !w_fc[2];
    // UM 9.5.1.1, U: "automatically set by the processor when a descriptor is
    // accessed in which the U bit is clear except after a supervisor
    // violation is detected"; M: set "before a write operation to a page for
    // which the M bit is zero, except after a descriptor with the WP bit set
    // is encountered, or after a supervisor violation is encountered". PTEST
    // "alters neither the used or modified bits" (PRM 6), and an invalid
    // descriptor is never modified (UM 9.5.3.4).
    d_u_upd = (w_kind != K_PTEST) && (d_dt != 2'b00) && !w_d0[3] && !d_sv && !d_ind;
    d_m_upd = (w_kind != K_PTEST) && (d_dt == 2'b01) && w_wr && !w_d0[4]
           && !d_wp_n && !d_sv;
    // ... nor is the descriptor an indirect one points at when it is not a
    // page descriptor, which is "treated as invalid" (UM 9.5.3.2).
    d_write = (d_u_upd || d_m_upd) && !(w_indir && d_dt[1]);
    d_new   = w_d0 | {27'd0, d_m_upd, d_u_upd, 3'd0};
    d_pga   = d_long ? w_d1[31:8] : w_d0[31:8];
    d_ta    = d_long ? {w_d1[31:4], 4'd0} : {w_d0[31:4], 4'd0};
    d_ia    = d_long ? {w_d1[31:2], 2'd0} : {w_d0[31:2], 2'd0};
  end

  // The root pointer the search starts from -- UM Table 9-2: "The translation
  // tree with its root defined by the SRP register is selected only when SRE
  // and FC2 are both set."
  logic [63:0] rp;
  assign rp = (w_fc[2] && tc_sre) ? srp_q : crp_q;

  // ATC write.
  logic        a_wr;
  logic        a_b;
  assign a_wr = (w_st == W_FIN) && (w_kind != K_PTEST);
  assign a_b  = w_inval || w_limx || w_berr || (w_s && !w_fc[2]);

  // The history bits after this edge -- see the ATC above.
  logic [N-1:0] h_set, h_or, v_after, h_after;
  logic [N-1:0] fl_now;
  logic         flush_all;
  always_comb begin
    flush_all = cmd_go
             && ((cmd == rd68031_ucode_pkg::U_MMU_FLUSHA)
                 || ((cmd == rd68031_ucode_pkg::U_MMU_WR) && (grp != 3'b011)
                     && !cmd_xw[8]));
    fl_now = '0;
    if (flush_all) fl_now = '1;
    else if (cmd_go && ((cmd == rd68031_ucode_pkg::U_MMU_FLUSH)
                        || (cmd == rd68031_ucode_pkg::U_MMU_FLUSHEA)))
      fl_now = fl_hit;
    h_set = '0;
    if (l_take && u1) h_set = h_set | h1v;
    if (a_wr) h_set = h_set | ({{(N-1){1'b0}}, 1'b1} << wslot);
    v_after = v_q & ~fl_now;
    if (a_wr) v_after = (v_after & ~h3v) | ({{(N-1){1'b0}}, 1'b1} << wslot);
    h_or    = (h_q | h_set) & v_after;
    h_after = (&(h_or | ~v_after)) ? (h_set & v_after) : h_or;
  end

  // Starting a search. The sequencer's PTEST or PLOAD before the bus unit's
  // access: the bus unit asks again on the next edge.
  logic seq_go, acc_go;
  assign seq_go = cmd_go && (w_st == W_IDLE)
               && ((cmd == rd68031_ucode_pkg::U_MMU_PLOAD)
                   || (cmd == rd68031_ucode_pkg::U_MMU_PTEST));
  assign acc_go = wq_valid && (w_st == W_IDLE) && !seq_go;

  // ==========================================================================
  // The architectural registers and the ATC. rst_n resets them; the RESET pin
  // clears the three E bits and nothing else (UM 9.2.2), and leaves the ATC as
  // it was.
  // ==========================================================================
  // The reset loop's index is the loop's own (as rd68031_seq's register file
  // does): a module-scope variable assigned only in the reset branch holds its
  // value on every other path, and Quartus infers a latch for it (Warning
  // 10240, measured at M15). yosys makes no flip-flops of a loop index in the
  // asynchronous reset branch (doc/coding-standard.md).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tc_q    <= '0;
      crp_q   <= '0;
      srp_q   <= '0;
      tt0_q   <= '0;
      tt1_q   <= '0;
      mmusr_q <= '0;
      v_q     <= '0;
      h_q     <= '0;
      for (int i = 0; i < N; i++) begin
        tag_q[i] <= '0;
        dat_q[i] <= '0;
      end
    end else if (crst) begin
      tc_q[31]  <= 1'b0;
      tt0_q[15] <= 1'b0;
      tt1_q[15] <= 1'b0;
    end else begin
      v_q <= v_after;
      h_q <= h_after;
      if (a_wr) begin
        tag_q[wslot] <= {w_fc, w_la[31:8]};
        // UM 9.4: B "is set for an entry if a bus error, an invalid
        // descriptor, a supervisor violation, or a limit violation is
        // encountered during the table search"; WP "when a WP bit is set in
        // any of the descriptors encountered"; CI and M from the page
        // descriptor; the physical address from figure 9-27.
        dat_q[wslot] <= a_b ? {1'b1, 27'd0} : {1'b0, w_ci, w_wp, w_m, w_pfa};
      end
      if (cmd_go && (cmd == rd68031_ucode_pkg::U_MMU_WR)) begin
        if (grp == 3'b000) begin
          if (preg == 3'b010) tt0_q <= wd_lo & TT_BITS;
          else                tt1_q <= wd_lo & TT_BITS;
        end else if (grp == 3'b010) begin
          unique case (preg)
            // UM 9.7.2: "If an MMU configuration exception occurs, the TC
            // register is updated with the data, and the E bit is cleared."
            3'b000:  tc_q  <= wd_lo & TC_BITS & ~{tc_bad, 31'd0};
            3'b010:  srp_q <= {wd_hi & RPHI_BITS, wd_lo & RPLO_BITS};
            default: crp_q <= {wd_hi & RPHI_BITS, wd_lo & RPLO_BITS};
          endcase
        end else begin
          mmusr_q <= wd_lo[15:0] & MMUSR_BITS;
        end
      end
      // PTEST's verdict -- UM Table 9-3.
      if (w_st == W_PT0) begin
        if (tt3)
          mmusr_q <= 16'h0040;                          // T, the rest undefined
        else
          mmusr_q <= {hit3 && e3_b, 1'b0, 1'b0, 1'b0, hit3 && e3_wp,
                      !hit3 || e3_b, hit3 && e3_m, 9'd0};
      end else if ((w_st == W_FIN) && (w_kind == K_PTEST)) begin
        mmusr_q <= {w_berr, w_limx, w_s && !w_fc[2], 1'b0, w_wp,
                    w_inval || w_berr || w_limx, w_m, 6'd0, w_n};
      end
    end
  end

  // ==========================================================================
  // The walker. Cycle-level, and reset by either reset.
  // ==========================================================================
  logic core_rst_n;
  assign core_rst_n = rst_n && !crst;

  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      w_st     <= W_IDLE;
      w_ret    <= W_IDLE;
      w_kind   <= K_ACC;
      w_la     <= '0;
      w_fc     <= '0;
      w_rw     <= 1'b1;
      w_wr     <= 1'b0;
      w_lvl    <= '0;
      w_x      <= '0;
      w_pos    <= '0;
      w_ta     <= '0;
      w_dt     <= '0;
      w_limv   <= 1'b0;
      w_lu     <= 1'b0;
      w_lim    <= '0;
      w_wp     <= 1'b0;
      w_s      <= 1'b0;
      w_ci     <= 1'b0;
      w_m      <= 1'b0;
      w_n      <= '0;
      w_d0     <= '0;
      w_d1     <= '0;
      w_daddr  <= '0;
      w_lastda <= '0;
      w_inval  <= 1'b0;
      w_limx   <= 1'b0;
      w_berr   <= 1'b0;
      w_indir  <= 1'b0;
      w_ebase  <= '0;
      w_pfa    <= '0;
      w_pend   <= 1'b0;
      w_raddr  <= '0;
      w_rdat   <= '0;
      w_rwr    <= 1'b0;
      done_q   <= 1'b0;
      sts_q    <= 1'b0;
      ptest_da <= '0;
    end else begin
      done_q <= 1'b0;
      sts_q  <= 1'b0;
      if (tw_ack) w_pend <= 1'b0;

      unique case (w_st)
        W_IDLE: begin
          if (seq_go) begin
            w_kind <= (cmd == rd68031_ucode_pkg::U_MMU_PLOAD) ? K_PLOAD : K_PTEST;
            w_la   <= cmd_ea;
            w_fc   <= cmd_fc;
            w_rw   <= cmd_xw[9];
            // PRM 6, PLOAD: "PLOADW causes U and M bits in the translation
            // tables to be updated as if a write access had occurred". R/W
            // is bit 9, 0 for a write.
            w_wr   <= !cmd_xw[9];
            w_lvl  <= cmd_xw[12:10];
            w_st   <= ((cmd == rd68031_ucode_pkg::U_MMU_PTEST)
                       && (cmd_xw[12:10] == 3'd0)) ? W_PT0 : W_INIT;
          end else if (acc_go) begin
            w_kind <= K_ACC;
            w_la   <= wq_addr;
            w_fc   <= wq_fc;
            w_rw   <= !wq_wr;
            w_wr   <= wq_wr;
            w_lvl  <= 3'd7;
            w_st   <= W_INIT;
            // UM 8.1.1, Table 8-2: STATUS for three clocks, "MMU address
            // translation cache miss -- processor to begin table search".
            sts_q  <= 1'b1;
          end
        end

        W_INIT: begin
          // Figure 9-26: the accrued status starts clear.
          w_wp     <= 1'b0;
          w_s      <= 1'b0;
          w_ci     <= 1'b0;
          w_m      <= 1'b0;
          w_n      <= 3'd0;
          w_lastda <= 32'd0;
          w_inval  <= 1'b0;
          w_limx   <= 1'b0;
          w_berr   <= 1'b0;
          w_indir  <= 1'b0;
          w_pos    <= {2'd0, tc_is};
          w_ta     <= {rp[31:4], 4'd0};
          w_dt     <= rp[33:32];
          w_lu     <= rp[63];
          w_lim    <= rp[62:48];
          unique case (rp[33:32])
            // Not reachable through PMOVE, which takes the configuration
            // exception (UM 9.5.1.2) -- but the register then holds it.
            2'b00: begin
              w_inval <= 1'b1;
              w_st    <= W_FIN;
            end
            // UM 9.7.1, DT = 1: "the MC68030 internally calculates an ATC
            // entry ... by adding (unsigned) the value in the table address
            // field to the incoming logical address ... For this case, the
            // processor performs a limit check, regardless of the state of
            // the FCL bit". PTEST returns "$0" for the descriptor address.
            2'b01: begin
              w_x     <= 3'd1;
              w_limv  <= 1'b1;
              w_ebase <= {rp[31:4], 4'd0};
              w_st    <= W_ET;
            end
            default: begin
              if (tc_fcl) begin
                // UM 9.5.2: "The table is indexed by either the function code
                // or the set of logical address bits defined by the TIA field
                // ... the descriptor-type field of the root pointer selects
                // the scale factor". UM 9.7.1: "the function code level
                // cannot be limited".
                w_x     <= 3'd0;
                w_limv  <= 1'b0;
                w_daddr <= {rp[31:4], 4'd0}
                         + ((rp[33:32] == 2'b11) ? {26'd0, w_fc, 3'd0}
                                                 : {27'd0, w_fc, 2'd0});
                w_n     <= 3'd1;
                w_pend  <= 1'b1;
                w_raddr <= {rp[31:4], 4'd0}
                         + ((rp[33:32] == 2'b11) ? {26'd0, w_fc, 3'd0}
                                                 : {27'd0, w_fc, 2'd0});
                w_rwr   <= 1'b0;
                w_st    <= W_F0;
              end else begin
                // UM 9.5.2: "The root pointer includes a limit field that
                // applies when the function code lookup is not used".
                w_x    <= 3'd1;
                w_limv <= 1'b1;
                w_st   <= W_LVL;
              end
            end
          endcase
        end

        W_LVL: begin
          if (lim_bad) begin
            // UM 9.5.2: "if a limit violation is detected, the ATC is loaded
            // with an entry having the bus error (B) bit set ... during a
            // table search for a PTEST instruction, the invalid (I) and limit
            // (L) bits are set in the MMUSR".
            w_limx <= 1'b1;
            w_st   <= W_FIN;
          end else begin
            w_daddr <= w_ta + ((w_dt == 2'b11) ? {14'd0, idx, 3'd0}
                                                : {15'd0, idx, 2'd0});
            w_raddr <= w_ta + ((w_dt == 2'b11) ? {14'd0, idx, 3'd0}
                                                : {15'd0, idx, 2'd0});
            w_rwr   <= 1'b0;
            w_pend  <= 1'b1;
            w_n     <= w_n + 3'd1;
            w_st    <= W_F0;
          end
        end

        W_F0: begin
          if (tw_ack) begin
            if (tw_berr) begin
              // Figure 9-29: an abnormal termination of the fetch ends the
              // search with the entry invalid.
              w_berr <= 1'b1;
              w_st   <= W_FIN;
            end else begin
              w_d0 <= tw_rdata;
              if (w_dt == 2'b11) begin
                w_pend  <= 1'b1;
                w_raddr <= w_daddr + 32'd4;
                w_rwr   <= 1'b0;
                w_st    <= W_F1;
              end else begin
                w_st    <= W_DESC;
              end
            end
          end
        end

        W_F1: begin
          if (tw_ack) begin
            if (tw_berr) begin
              w_berr <= 1'b1;
              w_st   <= W_FIN;
            end else begin
              w_d1 <= tw_rdata;
              w_st <= W_DESC;
            end
          end
        end

        W_DESC: begin
          // PRM 6, PTEST: "the physical address of the last descriptor
          // successfully fetched" -- "for a long descriptor, the address of
          // the first long word".
          w_lastda <= w_daddr;
          if (d_dt == 2'b00) begin
            // UM 9.5.1.1: "A table search ends when an invalid descriptor is
            // encountered."
            w_inval <= 1'b1;
            w_st    <= W_FIN;
          end else begin
            w_wp <= d_wp_n;
            w_s  <= d_s_n;
            if (d_dt == 2'b01) begin
              // A page descriptor. UM 9.5.3.1: "The table search ends when the
              // search encounters a page descriptor, whether or not the page
              // descriptor is in a page descriptor table at the lowest level".
              w_ci <= w_d0[6];
              w_m  <= w_d0[4] || d_m_upd;
              if (w_indir || d_last) begin
                w_pfa <= d_pga;
                w_ret <= W_FIN;
              end else begin
                // Early termination. "If an early termination page descriptor
                // is a long format, the limit field is applied to the next
                // index field of the logical address" (UM 9.5.3.1), and the
                // page frame address is the descriptor's plus the unused
                // logical page address -- figure 9-27.
                w_x     <= w_x + 3'd1;
                if (w_x != 3'd0) w_pos <= w_pos + {2'd0, ti_x};
                w_limv  <= d_long;
                w_lu    <= w_d0[31];
                w_lim   <= w_d0[30:16];
                w_ebase <= {d_pga, 8'd0};
                w_ret   <= W_ET;
              end
            end else if (w_indir) begin
              // UM 9.5.3.2: the descriptor an indirect one points at "must not
              // have a DT field with a long or short encoding ... Otherwise,
              // the descriptor is treated as invalid".
              w_inval <= 1'b1;
              w_ret   <= W_FIN;
            end else if (d_last) begin
              // UM 9.5.3.2: at the bottom of the tree a "valid long" or "valid
              // short" descriptor is an indirect descriptor, and the page
              // descriptor is fetched, in the format its DT names, from "the
              // highest order 30 bits".
              w_indir <= 1'b1;
              w_dt    <= d_dt;
              w_daddr <= d_ia;
              w_ret   <= W_F0;
            end else begin
              // A table descriptor: the next level -- figure 9-25's NEXT x.
              w_ta   <= d_ta;
              w_dt   <= d_dt;
              w_limv <= d_long;
              w_lu   <= w_d0[31];
              w_lim  <= w_d0[30:16];
              w_x    <= w_x + 3'd1;
              if (w_x != 3'd0) w_pos <= w_pos + {2'd0, ti_x};
              w_ret  <= W_LVL;
            end
            // Where it goes once the history bits are written. PTEST stops at
            // the level it was given -- PRM 6, "the search ends at the
            // specified level" -- unless this level ended it anyway.
            if ((w_kind == K_PTEST) && (w_n >= w_lvl) && (d_dt != 2'b01)
                && !w_indir) begin
              w_ret <= W_FIN;
            end
            if (d_write) begin
              // Figure 9-29: "EXECUTE WRITE CYCLE U <- 1" (and M), the first
              // long word written back -- still under RMC, UM 9.5.2: "the read
              // and write operations to update the history bits are
              // guaranteed to be uninterrupted".
              w_pend  <= 1'b1;
              w_raddr <= w_daddr;
              w_rwr   <= 1'b1;
              w_rdat  <= d_new;
              w_st    <= W_UPD;
            end else begin
              w_st    <= W_UPD;
            end
          end
        end

        W_UPD: begin
          // The write-back, if there was one; and then onward.
          if (!w_pend || tw_ack) begin
            if (w_pend && tw_berr) begin
              w_berr <= 1'b1;
              w_st   <= W_FIN;
            end else if (w_ret == W_F0) begin
              w_pend  <= 1'b1;
              w_raddr <= w_daddr;
              w_rwr   <= 1'b0;
              w_n     <= w_n + 3'd1;
              w_st    <= W_F0;
            end else begin
              w_st <= w_ret;
            end
          end
        end

        W_ET: begin
          if (lim_bad) begin
            w_limx <= 1'b1;
          end else begin
            w_pfa  <= 24'((w_ebase + off) >> 8);
          end
          w_st <= W_FIN;
        end

        W_FIN: begin
          if (w_kind == K_PTEST) ptest_da <= w_lastda;
          if (w_kind != K_ACC)   done_q   <= 1'b1;
          w_st <= W_IDLE;
        end

        W_PT0: begin
          ptest_da <= 32'd0;
          done_q   <= 1'b1;
          w_st     <= W_IDLE;
        end

        default: w_st <= W_IDLE;
      endcase
    end
  end

  assign l_tc_on  = tc_e;
  assign l_tt_on  = tt0_q[15] || tt1_q[15];
  assign busy     = (w_st != W_IDLE);
  assign done     = done_q;
  assign w_sts    = sts_q;
  assign tw_valid = w_pend && !tw_last && !tw_ack;
  assign tw_want  = w_pend && !tw_ack;
  assign tw_addr  = w_raddr;
  assign tw_wr    = w_rwr;
  assign tw_wdata = w_rdat;

  logic unused_mmu;
  assign unused_mmu = &{1'b1, e1_ci, w_d1[7:0], w_d0[7],
                        w_d0[5], cmd_xw[4:0], cmd_ea[7:0], rp[47:34], rp[3:0]};

endmodule
