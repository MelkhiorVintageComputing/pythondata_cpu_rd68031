// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// The on-chip data cache -- UM 6.1.2, figure 6-3.
//
// "Both on-chip caches are 256-byte direct-mapped caches, each organized as 16
// lines. Each line consists of four entries, and each entry contains four bytes"
// (UM 6.1). "The tag of each line in the data cache contains function code bits
// FC0, FC1, and FC2 in addition to address bits A31-A8. The cache control
// circuitry selects the tag using bits A7-A4 ... Address bits A3-A2 select the
// valid bit for the appropriate long word" (UM 6.1.2). It is logically
// addressed: the tag is the address the program used, in the space it used.
//
// What this module decides and what the bus unit does. The bus unit owns the
// cycles; this module owns the arrays and every rule about what goes in them.
// It answers three questions about the request the sequencer is presenting --
// does it hit (and which part of it, since "misaligned data transfers may span
// two data cache entries ... the hit and miss are treated independently"), may
// a miss fill, and should the miss ask for a burst -- and it takes three kinds
// of update: the long words the bus reads for it, the writes the program makes,
// and CACR's clears.
//
// THE STORES. The data is four banks, one per long word of a line (A3-A2), of
// sixteen entries each, and each bank is four byte-wide RAMs. A misaligned
// operand touches two consecutive long words, which are always in two
// different banks, so one read port and one write port per RAM serve both
// halves of it in the same clock. The RAMs are not reset and need no reset:
// nothing reads an entry whose valid bit is clear. The sixteen 27-bit tags and
// the sixty-four valid bits are flip-flops with an asynchronous reset -- UM
// 6.2, "when a hardware reset of the processor occurs, all valid bits of both
// caches are cleared".
//
// WHEN EACH UPDATE LANDS -- doc/caches.md has the timing argument in full.
//   - A long word the bus reads is written on the rising edge that ends its bus
//     cycle (the `bn_*` port is the bus unit's view of that edge). The bus unit
//     takes no cachable read on that same edge, so a lookup never misses on an
//     entry that is being written.
//   - A write is captured on the edge the bus unit takes it and applied one
//     clock later: the earliest any request could read it back is a clock after
//     that, and a read cycle that ended on the take edge has by then been
//     written, so the two never land together and the write lands second, as
//     it came second on the bus. "Since the write to the cache completes before
//     the write to external memory, the cache contains the new value even if
//     the external write terminates in a bus error" (UM 6.1.2).
//   - ... except a write's second long word when it is in another page, which
//     waits for that page's translation (`wb_go`, `wb_drop`). The bus unit
//     answers no read from the cache meanwhile, and takes nothing else until
//     the write's last cycle, so the captured write is still this one.
//   - A CACR clear lands on the edge its MOVEC commits, after anything else
//     that edge does.

module rd68031_dcache (
    input  logic        clk,
    input  logic        rst_n,

    // Cache control ---------------------------------------------------------
    input  logic [31:0] cacr,
    input  logic [31:0] caar,
    input  logic  [3:0] cach_op,      // {CED, CD, CEI, CI}, pulsed by MOVEC
    input  logic        cdis_sync_n,
    // The MMU's cache inhibit -- the CI bit of the ATC entry or TTx register
    // that maps a page, which also drives CIOUT (UM 6.1, 5.7.2), from
    // rd68031_mmu through the bus unit. It is a page's, and a misaligned
    // operand's two long words may be in two pages with two different CI bits,
    // so each long word goes by its own page's (doc/bugs-found.md, M11 phase
    // B). A read that hits ignores it -- "whenever a read access occurs and
    // the required ... data operand is resident in the appropriate on-chip
    // cache ... the state of the corresponding CI bits in the MMU are also
    // ignored" (UM 6.1); a read cycle with CIOUT does not fill (`bn_ci`); and
    // a write ignores the cache altogether for a long word in a CI page --
    // "when CIOUT is asserted, the data cache is completely ignored, even on
    // write cycles operating in the write-allocation mode" (UM 6.1.2.1).
    //
    // `dc_inhibit` is the CI of the page the presented request starts in,
    // sampled with a write as it is taken. A write that crosses into a second
    // page (`wr_xpage`) leaves its long word there alone until the bus unit
    // has that page's translation: `wb_go` when it is cachable, and the long
    // word is treated then as it would have been with the first; `wb_drop`
    // when the operand will not write it after all, and it is invalidated.
    input  logic        dc_inhibit,
    input  logic        wr_xpage,
    input  logic        wb_go,
    input  logic        wb_drop,

    // The sequencer's request, as presented: a combinational lookup ----------
    input  logic [31:0] lk_addr,
    input  logic  [2:0] lk_fc,
    input  logic  [2:0] lk_bytes,     // 1..5
    input  logic        lk_read,
    input  logic        lk_rmc,
    input  logic [39:0] lk_wdata,     // right justified
    output logic        lk_on,        // a read this cache is looked up for
    output logic        lk_span,      // ... that spans two entries
    output logic        lk_hit_a,     // the entry with its first byte hits
    output logic        lk_hit_b,     // ... the entry with its last, if it spans
    output logic [39:0] lk_data,      // the bytes that hit, in place, right justified
    output logic  [2:0] lk_nb,        // how many of its bytes are in the second entry
    output logic        lk_fill,      // what the bus reads for it may be cached
    output logic        lk_burst,     // ... and the line asked for -- CBREQ

    // The bus unit is taking the presented WRITE on this rising edge ---------
    input  logic        wr_take,

    // ... or the MMU has refused one: UM 6.1.2, "if the MMU determines that the
    // access is invalid, the write is aborted, the corresponding entry is
    // invalidated, and a bus error exception is taken". The entries the
    // refused bytes are in lose their valid bits on this edge -------------
    input  logic        inv_v,
    input  logic [31:0] inv_addr,
    input  logic  [2:0] inv_fc,
    input  logic  [2:0] inv_bytes,

    // The bus cycle, or burst beat, ending on this rising edge ----------------
    input  logic        bn_valid,     // it ended normally with data
    input  logic        bn_err,       // ... or with a bus error
    input  logic [31:0] bn_data,      // the data bus as latched
    input  logic [31:0] bn_addr,      // the long word it carries
    input  logic  [1:0] bn_num,       // 0 for the cycle the operand asked for
    input  logic  [1:0] bn_port,      // DSACK encoding; STERM is a 32-bit port
    input  logic        bn_ciin,
    input  logic  [2:0] bn_fc,
    input  logic        bn_dfill,     // the operand it belongs to may fill this cache
    input  logic        bn_brq,       // the cycle asserted CBREQ
    input  logic        bn_first,     // the first cycle of its operand
    input  logic        bn_ci         // the cycle asserted CIOUT
);

  localparam int TW = 27;             // FC2-FC0, A31-A8

  // ==========================================================================
  // State
  // ==========================================================================
  // One register per line, below, read through an unpacked array: an indexed
  // part-select of a packed vector (`tags[line*TW +: TW]`) has Quartus
  // multiply the index by TW in a DSP block ahead of a shifter (M15).
  wire  [TW-1:0]    tags [0:15];
  logic [63:0]      valid_q;

  // A long word being gathered from a port narrower than 32 bits -- UM 6.1.3.1:
  // "since a single valid bit applies to an entire long word, a single entry
  // mode operation must provide a full 32 bits of data. Ports less than 32
  // bits wide require several read cycles for each entry."
  logic        asm_v_q;
  logic [29:0] asm_ent_q;
  logic  [2:0] asm_fc_q;
  logic  [3:0] asm_mask_q;     // bit k: the byte at offset k has arrived
  logic [31:0] asm_data_q;

  // A CACR clear landed while a fill was still bringing long words in.
  logic        dkill_q;

  // The write captured on the edge the bus unit took it, applied one clock on.
  logic        dw_v_q;
  logic [31:0] dw_addr_q;
  logic  [2:0] dw_fc_q;
  logic  [2:0] dw_bytes_q;
  logic [39:0] dw_data_q;
  logic        dw_en_q;        // the cache takes part in this write at all
  logic        dw_wa_q;        // ... and allocates on a miss
  logic        dw_xp_q;        // ... its second long word is in another page

  // ==========================================================================
  // Control bits
  // ==========================================================================
  logic ed, fd, wa, dbe;
  assign ed  = cacr[rd68031_pkg::CACR_ED] && cdis_sync_n;   // UM 6.3.1.6, 6.1
  assign fd  = cacr[rd68031_pkg::CACR_FD];                  // UM 6.3.1.5
  assign wa  = cacr[rd68031_pkg::CACR_WA];                  // UM 6.3.1.1
  assign dbe = cacr[rd68031_pkg::CACR_DBE];                 // UM 6.3.1.2

  // CD clears all; CED the entry CAAR's index names -- UM 6.3.1.3, 6.3.1.4,
  // "regardless of the states of the ED and FD bits".
  logic inv_all, inv_caar;
  assign inv_all  = cach_op[2];
  assign inv_caar = cach_op[3] && !cach_op[2];

  // "The data cache stores data references to any address space except CPU
  // space (FC = $7)" (UM 6.1.2).
  logic lk_space_ok;
  assign lk_space_ok = (lk_fc != rd68031_pkg::FC_CPU);

  // ==========================================================================
  // Lookup
  // ==========================================================================
  logic [29:0] ea, eb;         // the long words holding the first and last bytes
  logic  [3:0] line_a, line_b;
  logic  [1:0] bank_a, bank_b;
  logic [TW-1:0] tg_a, tg_b;
  logic        tok_a, tok_b;   // the indexed tag matches

  assign ea     = lk_addr[31:2];
  assign eb     = ea + 30'd1;
  assign line_a = ea[5:2];
  assign line_b = eb[5:2];
  assign bank_a = ea[1:0];
  assign bank_b = eb[1:0];
  assign tg_a   = {lk_fc, ea[29:6]};
  assign tg_b   = {lk_fc, eb[29:6]};
  assign tok_a  = (tags[line_a] == tg_a);
  assign tok_b  = (tags[line_b] == tg_b);

  logic [3:0] end_off, end_b;
  assign end_off = {2'b00, lk_addr[1:0]} + {1'b0, lk_bytes};
  assign end_b   = end_off - 4'd4;
  assign lk_span = (end_off > 4'd4);
  assign lk_nb   = lk_span ? end_b[2:0] : 3'd0;

  // UM 6.1.2.2: "the read portion of a read-modify-write cycle is always forced
  // to miss in the data cache". A disabled cache is not looked up (UM 6.3.1.6);
  // a frozen one is -- freezing stops replacement, not hits (UM 6.3.1.5).
  assign lk_on    = lk_read && !lk_rmc && lk_space_ok && ed;
  assign lk_hit_a = lk_on && tok_a && valid_q[ea[5:0]];
  assign lk_hit_b = lk_on && lk_span && tok_b && valid_q[eb[5:0]];

  // An external read is "cachable" when the cache is enabled, CDIS, CIIN and
  // CIOUT are negated and the MMU validates it (UM 6.1); and a frozen cache is
  // not filled (UM 6.3.1.5). CIIN and CIOUT are the bus's, cycle by cycle --
  // CIOUT because it is a page's, and an operand's cycles may be in two
  // (`bn_ci`, below). A read-modify-write's read is forced to miss but its
  // data still "update[s] a matching entry ... or creates a new entry" (UM
  // 6.1.2.2).
  assign lk_fill  = lk_read && lk_space_ok && ed && !fd;

  // UM 6.1.3.2: the burst is asked for when "a read cycle ... misses due to the
  // indexed tag not matching" or "tag matches, but all long words in the line
  // are invalid", with DBE set -- for the line the burst would fill, which is
  // the one holding the operand's last byte: an operand spanning two lines
  // gets no burst on its first access. "No burst filling of the data cache
  // occurs during a read-modify-write operation" (UM 7.3.3).
  logic [3:0] bl_line;
  logic       bl_tok;
  assign bl_line  = lk_span ? line_b : line_a;
  assign bl_tok   = lk_span ? tok_b : tok_a;
  assign lk_burst = lk_fill && !lk_rmc && dbe
                 && (!bl_tok || (valid_q[{bl_line, 2'b00} +: 4] == 4'b0000));

  // ==========================================================================
  // The data RAMs: bank k holds long word k of every line, byte lane j its
  // byte at offset j (lane 0 is A1A0 = 00, the most significant).
  // ==========================================================================
  logic [3:0]  rd_line [0:3];
  wire  [127:0] bank_q;        // bank k in bits 32k+31..32k
  logic [3:0]  wr_line [0:3];
  logic [31:0] wr_data [0:3];
  logic [3:0]  wr_be   [0:3];

  // Each bank is read at the line of whichever of the two long words is in it.
  int unsigned k_r;
  always_comb begin
    for (k_r = 0; k_r < 4; k_r = k_r + 1)
      rd_line[k_r] = (bank_a == k_r[1:0]) ? line_a : line_b;
  end

  genvar gk, gj;
  generate
    for (gk = 0; gk < 4; gk = gk + 1) begin : g_bank
      for (gj = 0; gj < 4; gj = gj + 1) begin : g_lane
        logic [7:0] m [0:15];
        logic [3:0] wl;
        logic [7:0] wd;
        assign wl = wr_line[gk];
        assign wd = wr_data[gk][(3-gj)*8 +: 8];
        always_ff @(posedge clk) begin
          if (wr_be[gk][gj]) m[wl] <= wd;
        end
        assign bank_q[gk*32 + (3-gj)*8 +: 8] = m[rd_line[gk]];
      end
    end
  endgenerate

  logic [31:0] d_a, d_b;
  assign d_a = bank_q[bank_a*32 +: 32];
  assign d_b = bank_q[bank_b*32 +: 32];

  // The operand, assembled from the entries that hit: byte j of it (j = 0 at
  // the lowest address) is at offset A1A0 + j, in the first entry when that is
  // below four, and lands right justified, the last byte least significant.
  int unsigned p_l;
  logic [3:0]  off_l;
  logic [7:0]  da_b [0:3];
  logic [7:0]  db_b [0:3];
  always_comb begin
    da_b[0] = d_a[31:24]; da_b[1] = d_a[23:16]; da_b[2] = d_a[15:8]; da_b[3] = d_a[7:0];
    db_b[0] = d_b[31:24]; db_b[1] = d_b[23:16]; db_b[2] = d_b[15:8]; db_b[3] = d_b[7:0];
    lk_data = '0;
    off_l   = '0;
    for (p_l = 0; p_l < 5; p_l = p_l + 1) begin
      // Byte p, counting up from the least significant, is byte
      // (bytes - 1 - p) from the lowest address.
      if (p_l < lk_bytes) begin
        off_l = {2'b00, lk_addr[1:0]} + {1'b0, lk_bytes} - 4'd1 - p_l[3:0];
        if (!off_l[2]) begin
          if (lk_hit_a) lk_data[p_l*8 +: 8] = da_b[off_l[1:0]];
        end else begin
          if (lk_hit_b) lk_data[p_l*8 +: 8] = db_b[off_l[1:0]];
        end
      end
    end
  end

  // ==========================================================================
  // A write -- UM 6.1.2 and 6.1.2.1, figure 6-4
  //
  // Write-through: the bus unit writes memory whatever happens here. In the
  // cache, each of the (at most two) entries the write touches is treated on
  // its own:
  //   - a hit is updated, "regardless of the operand size and even if the cache
  //     is frozen" (UM 6.1.2), whatever WA says (figure 6-4, examples 1 and 2);
  //   - a miss with WA clear, or with the cache frozen -- "if the data cache is
  //     disabled or frozen, the WA bit is ignored" (UM 6.3.1.1) -- changes
  //     nothing: "the cache is updated only during a write hit";
  //   - a miss with WA set and long-word data that is long-word aligned is
  //     written and validated, the tag replaced if it did not match and the
  //     line's other three entries then invalidated (examples 3 and 4);
  //   - any other miss with WA set leaves the data and the tag alone and clears
  //     the entry's valid bit (example 5: "the data is not written in the
  //     cache, the tag is unaltered, and the valid bit(s) are cleared"). With
  //     the tag matching, the entry was already invalid.
  // ==========================================================================
  logic [29:0] wea, web;
  logic  [3:0] wline_a, wline_b;
  logic  [1:0] wbank_a, wbank_b;
  logic [TW-1:0] wtg_a, wtg_b;
  logic        wtok_a, wtok_b, whit_a, whit_b;
  logic  [3:0] wm_a, wm_b;     // the bytes written into each entry, by offset
  logic [31:0] wd_a, wd_b;
  logic        wfull;          // long-word data, long-word aligned

  assign wea     = dw_addr_q[31:2];
  assign web     = wea + 30'd1;
  assign wline_a = wea[5:2];
  assign wline_b = web[5:2];
  assign wbank_a = wea[1:0];
  assign wbank_b = web[1:0];
  assign wtg_a   = {dw_fc_q, wea[29:6]};
  assign wtg_b   = {dw_fc_q, web[29:6]};
  assign wtok_a  = (tags[wline_a] == wtg_a);
  assign wtok_b  = (tags[wline_b] == wtg_b);
  assign whit_a  = wtok_a && valid_q[wea[5:0]];
  assign whit_b  = wtok_b && valid_q[web[5:0]];
  assign wfull   = (dw_bytes_q == 3'd4) && (dw_addr_q[1:0] == 2'b00);

  int unsigned j_w;
  logic [3:0]  off_w;
  logic [7:0]  wv_b [0:7];
  always_comb begin
    for (j_w = 0; j_w < 8; j_w = j_w + 1)
      wv_b[j_w] = (j_w < 5) ? dw_data_q[j_w*8 +: 8] : 8'h00;
    wm_a  = '0;
    wm_b  = '0;
    wd_a  = '0;
    wd_b  = '0;
    off_w = '0;
    for (j_w = 0; j_w < 5; j_w = j_w + 1) begin
      // Byte j_w from the lowest address is byte (bytes - 1 - j_w) of the
      // right-justified data.
      if (j_w < dw_bytes_q) begin
        off_w = {2'b00, dw_addr_q[1:0]} + j_w[3:0];
        if (!off_w[2]) begin
          wm_a[off_w[1:0]] = 1'b1;
          wd_a[{~off_w[1:0], 3'b000} +: 8] = wv_b[dw_bytes_q - 3'd1 - j_w[2:0]];
        end else begin
          wm_b[off_w[1:0]] = 1'b1;
          wd_b[{~off_w[1:0], 3'b000} +: 8] = wv_b[dw_bytes_q - 3'd1 - j_w[2:0]];
        end
      end
    end
  end

  logic w_go, wspan;
  assign w_go  = dw_v_q && dw_en_q;
  assign wspan = (wm_b != 4'b0000);

  // The second long word: with the first, a clock after the take, when both
  // are in one page; when the bus unit has its page's translation, and only if
  // that is cachable, when it is in another. The write is the last the bus
  // unit took, so the captured write is still this one.
  logic wb_now;
  assign wb_now = dw_xp_q ? (wb_go && ed && (dw_fc_q != rd68031_pkg::FC_CPU))
                          : w_go;

  logic w_upd_a, w_upd_b;      // bytes go into the entry
  logic w_val_a;               // ... and it becomes valid (WA, aligned long)
  logic w_rep_a;               // ... with its line's tag replaced
  logic w_clr_a, w_clr_b;      // the entry's valid bit is cleared
  assign w_val_a = w_go && !whit_a && dw_wa_q && wfull;
  assign w_rep_a = w_val_a && !wtok_a;
  assign w_upd_a = (w_go && whit_a) || w_val_a;
  assign w_upd_b = wb_now && wspan && whit_b;
  assign w_clr_a = w_go && !whit_a && dw_wa_q && !wfull && !wtok_a;
  assign w_clr_b = (wb_now && wspan && !whit_b && dw_wa_q && !wtok_b)
                || (wb_drop && dw_xp_q && wtok_b);

  // ==========================================================================
  // A long word from the bus -- UM 6.1.3
  //
  // "When a cachable read cycle provides data with both CIIN and BERR negated,
  // the MC68030 attempts to fill the cache entry" (UM 6.1.3.1). A 32-bit port
  // gives the whole long word in one cycle -- "for a 32-bit port, the slave
  // device ... supplies the long word beginning at the long-word boundary"
  // (UM 7.2.7) -- and so does every beat of a burst. A narrower port gives a
  // word or a byte per cycle, and the bus unit runs the cycles that complete
  // the entry; they are gathered here and the entry written when its four
  // bytes are in. Until then the entry is invalid: a read-modify-write, which
  // gets no completing cycles, may be refreshing an entry that was valid, and
  // it must not be left holding the old bytes around the new ones.
  //
  //   - CIIN on any cycle of the entry: "if any byte is not cachable, CIIN must
  //     be asserted for all corresponding bus cycles" -- nothing is written, and
  //     the entry is invalid.
  //   - BERR: "the corresponding entry is marked invalid" (UM 6.1.3.1); on the
  //     first cycle of a burst "the entire associated cache line is marked
  //     invalid" (UM 6.1.3.2).
  // ==========================================================================
  logic [29:0] fe;
  logic  [3:0] fline;
  logic  [1:0] fbank;
  logic [TW-1:0] ftg;
  logic        ftok;
  assign fe    = bn_addr[31:2];
  assign fline = fe[5:2];
  assign fbank = fe[1:0];
  assign ftg   = {bn_fc, fe[29:6]};
  assign ftok  = (tags[fline] == ftg);

  // A beat that is this cache's: it belongs to an operand that may fill, and
  // no clear has landed since that operand's first cycle.
  logic f_use;
  assign f_use = bn_dfill && !bn_ci && (!dkill_q || bn_first);

  logic f_narrow;
  assign f_narrow = (bn_port == rd68031_pkg::DSACK_8)
                 || (bn_port == rd68031_pkg::DSACK_16);

  // The bytes a narrow cycle brought, by offset within the long word, and the
  // long word they make with what has been gathered. "A 16-bit port must
  // supply 16 bits, even for 8-bit transfers" on D31-D16, the word at the
  // lower word boundary; a byte port "supplies the addressed byte on D24-D31"
  // (UM 7.2.7).
  logic  [3:0] n_mask;
  logic [31:0] n_data;
  always_comb begin
    if (bn_port == rd68031_pkg::DSACK_16) begin
      n_mask = bn_addr[1] ? 4'b1100 : 4'b0011;
      n_data = bn_addr[1] ? {16'h0000, bn_data[31:16]} : {bn_data[31:16], 16'h0000};
    end else begin
      n_mask = 4'b0001 << bn_addr[1:0];
      n_data = {bn_data[31:24], 24'h000000} >> {bn_addr[1:0], 3'b000};
    end
  end

  logic        a_cont;         // this cycle continues the long word being gathered
  logic  [3:0] a_mask;
  logic [31:0] a_data;
  logic [31:0] a_keep;         // the gathered bytes this cycle does not bring
  assign a_cont = asm_v_q && !bn_first && (asm_ent_q == fe) && (asm_fc_q == bn_fc);
  always_comb begin
    a_keep = '0;
    if (a_cont) begin
      a_keep[31:24] = (asm_mask_q[0] && !n_mask[0]) ? asm_data_q[31:24] : 8'h00;
      a_keep[23:16] = (asm_mask_q[1] && !n_mask[1]) ? asm_data_q[23:16] : 8'h00;
      a_keep[15:8]  = (asm_mask_q[2] && !n_mask[2]) ? asm_data_q[15:8]  : 8'h00;
      a_keep[7:0]   = (asm_mask_q[3] && !n_mask[3]) ? asm_data_q[7:0]   : 8'h00;
    end
  end
  assign a_mask = (a_cont ? asm_mask_q : 4'b0000) | n_mask;
  assign a_data = a_keep | n_data;

  logic f_good, f_bad;
  assign f_good = f_use && bn_valid && !bn_ciin;
  assign f_bad  = f_use && ((bn_valid && bn_ciin) || bn_err);

  logic        f_wr;           // a whole long word goes in
  logic [31:0] f_wdata;
  assign f_wr    = f_good && (!f_narrow || (a_mask == 4'b1111));
  assign f_wdata = f_narrow ? a_data : bn_data;

  logic f_clr_ent, f_clr_line;
  assign f_clr_line = f_bad && bn_err && bn_brq && (bn_num == 2'd0);
  assign f_clr_ent  = (f_bad && !f_clr_line)
                   || (f_good && f_narrow && !f_wr && !a_cont);

  // ==========================================================================
  // The write ports. A fill and a write never land in the same clock
  // (header); the write is given the port if they ever did.
  // ==========================================================================
  int unsigned k_p;
  always_comb begin
    for (k_p = 0; k_p < 4; k_p = k_p + 1) begin
      wr_line[k_p] = fline;
      wr_data[k_p] = f_wdata;
      wr_be[k_p]   = (f_wr && (fbank == k_p[1:0])) ? 4'b1111 : 4'b0000;
      if (w_upd_a && (wbank_a == k_p[1:0])) begin
        wr_line[k_p] = wline_a;
        wr_data[k_p] = wd_a;
        // An entry newly validated by an aligned long word is written whole;
        // a hit only where the write's bytes are.
        wr_be[k_p]   = w_val_a ? 4'b1111 : wm_a;
      end
      if (w_upd_b && (wbank_b == k_p[1:0])) begin
        wr_line[k_p] = wline_b;
        wr_data[k_p] = wd_b;
        wr_be[k_p]   = wm_b;
      end
    end
  end

  // ==========================================================================
  // Tags and valid bits
  // ==========================================================================
  logic          tag_we;
  logic  [3:0]   tag_wline;
  logic [TW-1:0] tag_wval;
  assign tag_we    = (f_wr && !ftok) || w_rep_a;
  assign tag_wline = w_rep_a ? wline_a : fline;
  assign tag_wval  = w_rep_a ? wtg_a : ftg;

  genvar gl;
  generate
    for (gl = 0; gl < 16; gl = gl + 1) begin : g_tag
      logic [TW-1:0] tag_q;
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                             tag_q <= '0;
        else if (tag_we && (tag_wline == gl))   tag_q <= tag_wval;
      end
      assign tags[gl] = tag_q;
    end
  endgenerate

  // The entries a refused write touches: one long word, or two.
  logic [29:0] iea, ieb;
  logic [3:0]  iend;
  logic        itok_a, itok_b, i_clr_a, i_clr_b;
  assign iea     = inv_addr[31:2];
  assign ieb     = iea + 30'd1;
  assign iend    = {2'b00, inv_addr[1:0]} + {1'b0, inv_bytes};
  assign itok_a  = (tags[iea[5:2]] == {inv_fc, iea[29:6]});
  assign itok_b  = (tags[ieb[5:2]] == {inv_fc, ieb[29:6]});
  assign i_clr_a = inv_v && itok_a;
  assign i_clr_b = inv_v && itok_b && (iend > 4'd4);

  localparam logic [63:0] ONE = 64'd1;
  logic [63:0] v_set, v_line, v_clr;
  assign v_set  = ((f_wr)    ? (ONE << fe[5:0])  : '0)
                | ((w_val_a) ? (ONE << wea[5:0]) : '0);
  assign v_line = ((f_wr && !ftok) ? (64'hF << {fline, 2'b00})   : '0)
                | ((w_rep_a)       ? (64'hF << {wline_a, 2'b00}) : '0);
  assign v_clr  = ((f_clr_ent)  ? (ONE << fe[5:0])              : '0)
                | ((f_clr_line) ? (64'hF << {fline, 2'b00})     : '0)
                | ((w_clr_a)    ? (ONE << wea[5:0])             : '0)
                | ((w_clr_b)    ? (ONE << web[5:0])             : '0)
                | ((inv_caar)   ? (ONE << caar[7:2])            : '0)
                | ((i_clr_a)    ? (ONE << iea[5:0])             : '0)
                | ((i_clr_b)    ? (ONE << ieb[5:0])             : '0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)       valid_q <= '0;
    else if (inv_all) valid_q <= '0;
    else              valid_q <= ((valid_q & ~v_line) | v_set) & ~v_clr;
  end

  // ==========================================================================
  // The long word being gathered, the clear-while-filling flag, and the
  // captured write.
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      asm_v_q    <= 1'b0;
      asm_ent_q  <= '0;
      asm_fc_q   <= '0;
      asm_mask_q <= '0;
      asm_data_q <= '0;
      dkill_q    <= 1'b0;
      dw_v_q     <= 1'b0;
      dw_addr_q  <= '0;
      dw_fc_q    <= '0;
      dw_bytes_q <= '0;
      dw_data_q  <= '0;
      dw_en_q    <= 1'b0;
      dw_wa_q    <= 1'b0;
      dw_xp_q    <= 1'b0;
    end else begin
      if (f_good && f_narrow && !f_wr) begin
        asm_v_q    <= 1'b1;
        asm_ent_q  <= fe;
        asm_fc_q   <= bn_fc;
        asm_mask_q <= a_mask;
        asm_data_q <= a_data;
      end else if (f_wr || f_bad || (bn_dfill && bn_first)) begin
        asm_v_q    <= 1'b0;
      end

      // CD or CED with a fill in flight: the rest of it was read from memory
      // around the clear, which is the program saying the cache's copy is no
      // longer to be trusted, so none of it goes in. The next operand's first
      // cycle starts afresh.
      if (inv_all || inv_caar) begin
        dkill_q <= 1'b1;
        asm_v_q <= 1'b0;
      end else if (bn_dfill && bn_first && (bn_valid || bn_err)) begin
        dkill_q <= 1'b0;
      end

      dw_v_q <= wr_take;
      if (wr_take) begin
        dw_addr_q  <= lk_addr;
        dw_fc_q    <= lk_fc;
        dw_bytes_q <= lk_bytes;
        dw_data_q  <= lk_wdata;
        dw_en_q    <= lk_space_ok && ed && !dc_inhibit;
        dw_wa_q    <= wa && !fd;
        dw_xp_q    <= wr_xpage;
      end
    end
  end

  logic unused_dcache;
  assign unused_dcache = &{1'b1, caar[31:8], caar[1:0], cach_op[1:0],
                           off_l[3], off_w[3], end_b[3]};

endmodule
