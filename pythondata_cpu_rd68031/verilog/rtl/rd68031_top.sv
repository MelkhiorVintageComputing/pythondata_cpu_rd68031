// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// Top level. Pin list per doc/pinout.md, which is UM section 5's: every three-state
// or bidirectional pin of the original is split into _i / _o / _oe, with _oe active
// high meaning the core drives.
//
// The processor is five units: rd68031_seq runs the microcode and holds the
// datapath, rd68031_ifu holds the instruction pipe and the instruction cache,
// rd68031_dcache the data cache, rd68031_mmu the memory management unit, and
// rd68031_biu drives the pins. This wires them together and does nothing else.
//
// Note the fully-scoped rd68031_pkg:: references and the plain-vector ports: yosys
// 0.52 supports neither `import pkg::*` nor user types on ports. See
// doc/coding-standard.md.

module rd68031_top #(
    // The two caches, each built (any non-zero value: the MC68030's 64 long
    // words, 16 lines of four -- UM 6.1) or built out (zero). A core without
    // them is still fully software compatible -- a cache that never hits is a
    // disabled one, which the part allows -- and `make cache` holds the two
    // builds to the same writes and to no more reads with the caches than
    // without. CACR, CAAR and CDIS exist and behave either way. doc/caches.md.
    parameter int ICACHE_ENTRIES = 64,
    parameter int DCACHE_ENTRIES = 64,
    // Whether the coprocessor interface is built. Zero takes an F-line exception on
    // every coprocessor opcode, which is also the right behaviour for a machine with
    // no coprocessor attached.
    parameter bit COPROCESSOR = 1'b0,
    // Whether the address group is released between bus cycles. The part does
    // (specification 7); a board that needs it held can clear this.
    parameter bit ADDR_HIZ_BETWEEN_CYCLES = 1'b1
) (
    // Clock and hardware reset -----------------------------------------------
    input  logic        clk,      // free-running, both edges used (UM 5.12)
    input  logic        rst_n,    // not an MC68030 pin: async init, see doc/pinout.md

    // Function codes (UM 5.2) ------------------------------------------------
    output logic  [2:0] fc_o,
    output logic        fc_oe,

    // Address bus (UM 5.3) ---------------------------------------------------
    output logic [31:0] a_o,      // A1 and A0 are real pins
    output logic        a_oe,

    // Data bus (UM 5.4) ------------------------------------------------------
    input  logic [31:0] d_i,
    output logic [31:0] d_o,      // all 32 bits driven on every write (UM 7.2.4)
    output logic        d_oe,

    // Transfer size (UM 5.5) -------------------------------------------------
    output logic  [1:0] siz_o,    // bytes REMAINING, not operand size (UM 7.2.1)
    output logic        siz_oe,

    // Bus control (UM 5.6) ---------------------------------------------------
    output logic        ecs_n_o,  // one half clock, every bus cycle; never three-stated
    output logic        ocs_n_o,  // ... but only the first cycle of an operand
    output logic        rw_o,     // high = read, low = write
    output logic        rw_oe,
    output logic        rmc_n_o,
    output logic        rmc_oe,
    output logic        as_n_o,
    output logic        as_oe,
    output logic        ds_n_o,   // not asserted in a two-clock synchronous write
    output logic        ds_oe,
    output logic        dben_n_o, // active low (Table 5-2)
    output logic        dben_oe,
    input  logic  [1:0] dsack_n_i,  // [1] is DSACK1; sample both on the same edge
    input  logic        sterm_n_i,  // synchronous: sampled on the rising edge

    // Cache control (UM 5.7) -------------------------------------------------
    input  logic        ciin_n_i,   // synchronous; ignored on writes
    output logic        ciout_n_o,
    output logic        ciout_oe,
    output logic        cbreq_n_o,
    output logic        cbreq_oe,
    input  logic        cback_n_i,  // synchronous

    // Interrupt control (UM 5.8) ---------------------------------------------
    input  logic  [2:0] ipl_n_i,
    output logic        ipend_n_o,  // never three-stated
    input  logic        avec_n_i,

    // Bus arbitration (UM 5.9) -----------------------------------------------
    input  logic        br_n_i,
    output logic        bg_n_o,     // never three-stated
    input  logic        bgack_n_i,

    // Bus exception control (UM 5.10) ----------------------------------------
    input  logic        berr_n_i,
    input  logic        reset_n_i,
    output logic        reset_n_o,  // open drain: constant 0
    output logic        reset_n_oe, // the RESET instruction holds it 512 clocks
    input  logic        halt_n_i,   // input only on the MC68030 (Table 5-2)

    // Emulator support (UM 5.11) ---------------------------------------------
    input  logic        cdis_n_i,
    input  logic        mmudis_n_i,
    output logic        refill_n_o, // never three-stated
    output logic        status_n_o  // never three-stated
);

  // ==========================================================================
  // Sequencer to bus unit: one request per OPERAND, not per bus cycle.
  //
  // The bus unit owns the split into one to four cycles, the SIZ encoding, the
  // byte lanes of UM Tables 7-4 and 7-5 and the assembly of the result, so the microcode
  // issues exactly one request whatever the alignment and whatever port answers.
  // That is also what lets OCS be driven from this handshake (UM 7.1.1).
  // ==========================================================================
  logic        req_valid;
  logic        req_want;
  logic  [2:0] req_kind;
  logic  [2:0] req_fc;
  logic [31:0] req_addr;
  logic  [2:0] req_bytes;      // 1..5; five is a bit-field span, which SIZ cannot encode
  logic [39:0] req_wdata;      // right-justified
  logic        req_rmc;
  logic  [3:0] req_cpuspace;
  logic  [7:0] req_cpuaddr;
  logic        req_cpfault;
  logic        req_cpuraw;

  logic        req_ack;
  logic        req_start;
  logic        req_last;
  logic        req_early;
  logic [39:0] req_rdata;
  logic  [2:0] req_end;
  logic        req_fault;
  logic        req_fault_wr;
  logic  [1:0] req_dsack;

  // The residual of the in-flight operand, sampled at a fault: what the frame
  // records, and what RTE reloads through the restore port below.
  logic [31:0] flt_addr;
  logic  [2:0] flt_bytes;
  logic  [2:0] flt_fc;
  logic        flt_rw;
  logic        flt_rmc;
  logic [31:0] flt_dob;
  logic [31:0] flt_dib;

  logic        rst_op_valid;
  logic        rst_cancel;
  logic [31:0] rst_addr;
  logic  [2:0] rst_bytes;
  logic  [2:0] rst_fc;
  logic        rst_rw;
  logic        rst_rmc;
  logic [31:0] rst_dob;

  // ==========================================================================
  // Sequencer to instruction fetch unit
  // ==========================================================================
  logic  [1:0] pf_op;          // NONE / ADV / FILL / FLUSH
  logic [31:0] pf_addr;        // the new fetch address on FLUSH
  logic        pf_super;       // which program space to fetch in
  logic        pf_ready;
  logic        pf_dvalid;
  logic [15:0] stg_d;
  logic [15:0] stg_c;
  logic [15:0] stg_b;
  logic        stg_c_fault;
  logic        stg_b_fault;
  logic        stg_c_rerun;
  logic        stg_b_rerun;
  logic        pf_stuck;
  logic        pf_odd;
  logic [31:0] pc_d;
  logic [31:0] stg_b_addr;

  // The IFU's checkpoint port, marshalled into and out of a format $A or $B
  // frame by the sequencer. The cache holding register is deliberately absent:
  // doc/checkpoint.md records why it is not saved.
  logic        ckpt_save;
  logic        ckpt_wr;
  logic  [2:0] ckpt_sel;
  logic [31:0] ckpt_data;
  logic        ckpt_load;
  logic        ckpt_hold;
  logic [31:0] ckpt_pc_fetch;

  // ==========================================================================
  // Instruction fetch unit to bus unit
  // ==========================================================================
  logic        fetch_valid;
  logic [31:0] fetch_addr;     // always long-word aligned (UM Table 7-6 note)
  logic  [2:0] fetch_fc;
  logic        fetch_ack;
  logic        fetch_last;
  logic [31:0] fetch_rdata;
  logic        fetch_fault;
  logic        bus_abort;      // combinational, valid during S0 (UM20 5.2.5)

  // ==========================================================================
  // Cache control, which exists whether or not the cache does
  // ==========================================================================
  logic [31:0] cacr;
  logic [31:0] caar;
  logic  [3:0] cach_op;        // {CED, CD, CEI, CI}, pulsed by the MOVEC that sets them
  logic        cdis_sync_n;

  // ==========================================================================
  // Status
  // ==========================================================================
  logic  [2:0] ipl_sync_n;
  logic        reset_sync_n;
  logic        halt_sync_n;
  logic        bus_idle;
  logic        bus_granted;
  logic        reset_req;
  logic        reset_busy;
  logic        dbf;            // double bus fault
  // The RESET pin, from outside -- UM 7.8, 8.1.1. `rst_n` is the power-on
  // initialisation, which is not an MC68030 pin, and resets every register;
  // the RESET pin resets the processor as the manual describes it, which
  // leaves the data and address registers, USP, MSP, SFC, DFC and CAAR alone
  // (rd68031_seq) and the arbiter running (rd68031_biu). The fetch unit and its
  // cache keep nothing a reset spares, so the pin resets all of them --
  // "invalidates all entries in the instruction and data caches" (UM 8.1.1).
  logic        crst;
  logic        core_rst_n;
  assign core_rst_n = rst_n && !crst;
  logic        sts_bound, sts_pend, sts_exc, sts_refill;   // UM 12.7.1
  logic        mmudis_sync_n;

  // ==========================================================================
  // The caches and the bus -- doc/caches.md
  //
  // A line fill (CBREQ) is each cache's to ask for. CIOUT is the MMU's to say,
  // and so is the cache inhibit that goes with it: "whenever the appropriate CI
  // bit is set for either a read or a write access and an external bus cycle is
  // required, CIOUT is asserted and the instruction and data caches are
  // ignored for the access" (UM 6.1). The bus unit translates every cycle it
  // starts (doc/mmu.md) and drives CIOUT from the translation, gives the data
  // cache each read cycle's CIOUT with its data (`bnow_ci`), a write's first
  // page's CI with the write (`dc_inhibit`) and its second page's, if it has
  // one, when that page is translated (`dc_wb_go`, `dc_wb_drop`); and the
  // instruction cache a prefetch's with its answer (`ic_inhibit`) -- a
  // prefetch is an aligned long word, in one page.
  // ==========================================================================
  logic        req_burst, fetch_burst;
  logic        ic_inhibit, dc_inhibit;
  logic        dc_inv;
  logic [31:0] dc_inv_addr;
  logic  [2:0] dc_inv_fc, dc_inv_bytes;

  // ==========================================================================
  // The MMU -- UM section 9, doc/mmu.md
  // ==========================================================================
  logic  [2:0] mmu_cmd, mmu_fc;
  logic        mmu_go, mmu_cfg_bad, mmu_busy, mmu_done, mmu_sts;
  logic [15:0] mmu_xw;
  logic [31:0] mmu_ea, mmu_wd_hi, mmu_wd_lo, mmu_rd_hi, mmu_rd_lo, mmu_da;
  logic [31:0] l1_addr, l1_pb, l_mask;
  logic  [2:0] l_fc;
  logic        l_rw, l_rmc, l1_ci, l_take;
  logic        l_tc_on, l_tt_on;
  logic  [1:0] l1_kind;
  logic        wq_valid, wq_wr;
  logic [31:0] wq_addr;
  logic  [2:0] wq_fc;
  logic        tw_valid, tw_wr, tw_ack, tw_last, tw_berr;
  logic        tw_want;
  logic [31:0] tw_addr, tw_wdata, tw_rdata;
  logic        req_ciin, fetch_ciin;
  logic        beat_valid, beat_err, beat_ciin, beat_fetch;
  logic [31:0] beat_data, beat_addr;
  logic  [1:0] beat_num, beat_port;
  logic        burst_tail;
  // The beat ending now, for the data cache, and the data cache's answers.
  logic        bnow_valid, bnow_err, bnow_ciin, bnow_dfill, bnow_brq, bnow_first;
  logic        bnow_ci, dc_xpage, dc_wb_go, dc_wb_drop;
  logic [31:0] bnow_data, bnow_addr;
  logic  [1:0] bnow_num, bnow_port;
  logic  [2:0] bnow_fc;
  logic        req_dfill, dc_on, dc_span, dc_hit_a, dc_hit_b, dc_wr_take;
  logic  [2:0] dc_nb;
  logic [39:0] dc_data;
  // Posted writes -- doc/checkpoint.md rule 9.
  logic        req_post, req_taken, post_busy, req_fault_post, rst_post;

  rd68031_seq #(.COPROCESSOR (COPROCESSOR)) u_seq (
      .clk            (clk),
      .rst_n          (rst_n),
      .crst           (crst),

      .req_valid      (req_valid),
      .req_want       (req_want),
      .req_kind       (req_kind),
      .req_fc         (req_fc),
      .req_addr       (req_addr),
      .req_bytes      (req_bytes),
      .req_wdata      (req_wdata),
      .req_rmc        (req_rmc),
      .req_cpuspace   (req_cpuspace),
      .req_cpuaddr    (req_cpuaddr),
      .req_cpfault    (req_cpfault),
      .req_cpuraw     (req_cpuraw),
      .req_ack        (req_ack),
      .req_start      (req_start),
      .req_last       (req_last),
      .req_early      (req_early),
      .req_rdata      (req_rdata),
      .req_end        (req_end),
      .req_fault      (req_fault),
      .req_fault_wr   (req_fault_wr),
      .req_post       (req_post),
      .req_taken      (req_taken),
      .post_busy      (post_busy),
      .req_fault_post (req_fault_post),
      .req_dsack      (req_dsack),

      .flt_addr       (flt_addr),
      .flt_bytes      (flt_bytes),
      .flt_fc         (flt_fc),
      .flt_rw         (flt_rw),
      .flt_rmc        (flt_rmc),
      .flt_dob        (flt_dob),
      .flt_dib        (flt_dib),
      .rst_op_valid   (rst_op_valid),
      .rst_cancel     (rst_cancel),
      .rst_post       (rst_post),
      .rst_addr       (rst_addr),
      .rst_bytes      (rst_bytes),
      .rst_fc         (rst_fc),
      .rst_rw         (rst_rw),
      .rst_rmc        (rst_rmc),
      .rst_dob        (rst_dob),

      .pf_op          (pf_op),
      .pf_addr        (pf_addr),
      .pf_super       (pf_super),
      .pf_ready       (pf_ready),
      .pf_dvalid      (pf_dvalid),
      .stg_d          (stg_d),
      .stg_c          (stg_c),
      .stg_b          (stg_b),
      .stg_c_fault    (stg_c_fault),
      .stg_b_fault    (stg_b_fault),
      .stg_c_rerun    (stg_c_rerun),
      .stg_b_rerun    (stg_b_rerun),
      .pf_stuck       (pf_stuck),
      .pf_odd         (pf_odd),
      .pc_d           (pc_d),
      .stg_b_addr     (stg_b_addr),
      .ckpt_save      (ckpt_save),
      .ckpt_wr        (ckpt_wr),
      .ckpt_sel       (ckpt_sel),
      .ckpt_data      (ckpt_data),
      .ckpt_load      (ckpt_load),
      .ckpt_hold      (ckpt_hold),
      .ckpt_pc_fetch  (ckpt_pc_fetch),

      .cacr           (cacr),
      .caar           (caar),
      .cach_op        (cach_op),

      .mmu_cmd        (mmu_cmd),
      .mmu_go         (mmu_go),
      .mmu_xw         (mmu_xw),
      .mmu_fc         (mmu_fc),
      .mmu_ea         (mmu_ea),
      .mmu_wd_hi      (mmu_wd_hi),
      .mmu_wd_lo      (mmu_wd_lo),
      .mmu_rd_hi      (mmu_rd_hi),
      .mmu_rd_lo      (mmu_rd_lo),
      .mmu_cfg_bad    (mmu_cfg_bad),
      .mmu_da         (mmu_da),
      .mmu_busy       (mmu_busy),
      .mmu_done       (mmu_done),

      .ipl_sync_n     (ipl_sync_n),
      .reset_sync_n   (reset_sync_n),
      .halt_sync_n    (halt_sync_n),
      .bus_idle       (bus_idle),
      .bus_granted    (bus_granted),
      .reset_req      (reset_req),
      .reset_busy     (reset_busy),
      .dbf            (dbf),
      .ipend_n_o      (ipend_n_o),
      .sts_bound      (sts_bound),
      .sts_pend       (sts_pend),
      .sts_exc        (sts_exc),
      .sts_refill     (sts_refill)
  );

  rd68031_ifu #(.ICACHE_ENTRIES (ICACHE_ENTRIES)) u_ifu (
      .clk            (clk),
      .rst_n          (core_rst_n),

      .pf_op          (pf_op),
      .pf_addr        (pf_addr),
      .pf_super       (pf_super),
      .pf_ready       (pf_ready),
      .pf_dvalid      (pf_dvalid),
      .stg_d          (stg_d),
      .stg_c          (stg_c),
      .stg_b          (stg_b),
      .stg_c_fault    (stg_c_fault),
      .stg_b_fault    (stg_b_fault),
      .stg_c_rerun    (stg_c_rerun),
      .stg_b_rerun    (stg_b_rerun),
      .pf_stuck       (pf_stuck),
      .pf_odd         (pf_odd),
      .pc_d           (pc_d),
      .stg_b_addr     (stg_b_addr),
      .ckpt_save      (ckpt_save),
      .ckpt_wr        (ckpt_wr),
      .ckpt_sel       (ckpt_sel),
      .ckpt_data      (ckpt_data),
      .ckpt_load      (ckpt_load),
      .ckpt_hold      (ckpt_hold),
      .ckpt_pc_fetch  (ckpt_pc_fetch),

      .cacr           (cacr),
      .caar           (caar),
      .cach_op        (cach_op),
      .cdis_sync_n    (cdis_sync_n),
      .ic_inhibit     (ic_inhibit),

      .fetch_valid    (fetch_valid),
      .fetch_addr     (fetch_addr),
      .fetch_fc       (fetch_fc),
      .fetch_burst    (fetch_burst),
      .fetch_ack      (fetch_ack),
      .fetch_last     (fetch_last),
      .fetch_rdata    (fetch_rdata),
      .fetch_fault    (fetch_fault),
      .fetch_ciin     (fetch_ciin),
      .bus_abort      (bus_abort),

      .beat_valid     (beat_valid),
      .beat_err       (beat_err),
      .beat_data      (beat_data),
      .beat_addr      (beat_addr),
      .beat_num       (beat_num),
      .beat_ciin      (beat_ciin),
      .beat_fetch     (beat_fetch),
      .burst_tail     (burst_tail)
  );

  // ==========================================================================
  // The data cache -- UM 6.1.2. It looks up the request the sequencer is
  // presenting, and the bus unit acts on what it says.
  // ==========================================================================
  generate
    if (DCACHE_ENTRIES > 0) begin : g_dcache
      logic dc_read;
      assign dc_read = (req_kind != rd68031_pkg::CT_WRITE);
      rd68031_dcache u_dcache (
          .clk         (clk),
          .rst_n       (core_rst_n),
          .cacr        (cacr),
          .caar        (caar),
          .cach_op     (cach_op),
          .cdis_sync_n (cdis_sync_n),
          .dc_inhibit  (dc_inhibit),
          .wr_xpage    (dc_xpage),
          .wb_go       (dc_wb_go),
          .wb_drop     (dc_wb_drop),
          .lk_addr     (req_addr),
          .lk_fc       (req_fc),
          .lk_bytes    (req_bytes),
          .lk_read     (dc_read),
          .lk_rmc      (req_rmc),
          .lk_wdata    (req_wdata),
          .lk_on       (dc_on),
          .lk_span     (dc_span),
          .lk_hit_a    (dc_hit_a),
          .lk_hit_b    (dc_hit_b),
          .lk_data     (dc_data),
          .lk_nb       (dc_nb),
          .lk_fill     (req_dfill),
          .lk_burst    (req_burst),
          .wr_take     (dc_wr_take),
          .inv_v       (dc_inv),
          .inv_addr    (dc_inv_addr),
          .inv_fc      (dc_inv_fc),
          .inv_bytes   (dc_inv_bytes),
          .bn_valid    (bnow_valid),
          .bn_err      (bnow_err),
          .bn_data     (bnow_data),
          .bn_addr     (bnow_addr),
          .bn_num      (bnow_num),
          .bn_port     (bnow_port),
          .bn_ciin     (bnow_ciin),
          .bn_fc       (bnow_fc),
          .bn_dfill    (bnow_dfill),
          .bn_brq      (bnow_brq),
          .bn_first    (bnow_first),
          .bn_ci       (bnow_ci));
    end else begin : g_nodcache
      // No data cache: nothing ever hits, nothing is filled, no burst is asked
      // for, and the bus runs every operand exactly as written.
      assign dc_on     = 1'b0;
      assign dc_span   = 1'b0;
      assign dc_hit_a  = 1'b0;
      assign dc_hit_b  = 1'b0;
      assign dc_data   = 40'd0;
      assign dc_nb     = 3'd0;
      assign req_dfill = 1'b0;
      assign req_burst = 1'b0;
      logic unused_nodcache;
      assign unused_nodcache = &{1'b1, dc_wr_take, bnow_valid, bnow_err, bnow_data,
                                 bnow_addr, bnow_num, bnow_port, bnow_ciin,
                                 bnow_fc, bnow_dfill, bnow_brq, bnow_first,
                                 bnow_ci, dc_xpage, dc_wb_go, dc_wb_drop,
                                 dc_inhibit, dc_inv, dc_inv_addr, dc_inv_fc,
                                 dc_inv_bytes};
    end
  endgenerate

  rd68031_biu #(.ADDR_HIZ_BETWEEN_CYCLES (ADDR_HIZ_BETWEEN_CYCLES)) u_biu (
      .clk            (clk),
      .rst_n          (rst_n),

      .req_valid      (req_valid),
      .req_want       (req_want),
      .req_kind       (req_kind),
      .req_fc         (req_fc),
      .req_addr       (req_addr),
      .req_bytes      (req_bytes),
      .req_wdata      (req_wdata),
      .req_rmc        (req_rmc),
      .req_cpuspace   (req_cpuspace),
      .req_cpuaddr    (req_cpuaddr),
      .req_cpfault    (req_cpfault),
      .req_cpuraw     (req_cpuraw),
      .req_burst      (req_burst),
      .req_dfill      (req_dfill),
      .dc_on          (dc_on),
      .dc_span        (dc_span),
      .dc_hit_a       (dc_hit_a),
      .dc_hit_b       (dc_hit_b),
      .dc_nb          (dc_nb),
      .dc_data        (dc_data),
      .dc_wr_take     (dc_wr_take),
      .dc_inhibit     (dc_inhibit),
      .dc_xpage       (dc_xpage),
      .dc_wb_go       (dc_wb_go),
      .dc_wb_drop     (dc_wb_drop),
      .dc_inv         (dc_inv),
      .dc_inv_addr    (dc_inv_addr),
      .dc_inv_fc      (dc_inv_fc),
      .dc_inv_bytes   (dc_inv_bytes),
      .req_ack        (req_ack),
      .req_start      (req_start),
      .req_last       (req_last),
      .req_early      (req_early),
      .req_rdata      (req_rdata),
      .req_end        (req_end),
      .req_fault      (req_fault),
      .req_fault_wr   (req_fault_wr),
      .req_post       (req_post),
      .req_taken      (req_taken),
      .post_busy      (post_busy),
      .req_fault_post (req_fault_post),
      .req_dsack      (req_dsack),
      .req_ciin       (req_ciin),

      .flt_addr       (flt_addr),
      .flt_bytes      (flt_bytes),
      .flt_fc         (flt_fc),
      .flt_rw         (flt_rw),
      .flt_rmc        (flt_rmc),
      .flt_dob        (flt_dob),
      .flt_dib        (flt_dib),
      .rst_op_valid   (rst_op_valid),
      .rst_cancel     (rst_cancel),
      .rst_post       (rst_post),
      .rst_addr       (rst_addr),
      .rst_bytes      (rst_bytes),
      .rst_fc         (rst_fc),
      .rst_rw         (rst_rw),
      .rst_rmc        (rst_rmc),
      .rst_dob        (rst_dob),

      .fetch_valid    (fetch_valid),
      .fetch_addr     (fetch_addr),
      .fetch_fc       (fetch_fc),
      .fetch_burst    (fetch_burst),
      .fetch_ack      (fetch_ack),
      .fetch_last     (fetch_last),
      .fetch_rdata    (fetch_rdata),
      .fetch_fault    (fetch_fault),
      .fetch_ciin     (fetch_ciin),
      .fetch_ci       (ic_inhibit),
      .bus_abort      (bus_abort),

      .l1_addr        (l1_addr),
      .l_fc           (l_fc),
      .l_rw           (l_rw),
      .l_rmc          (l_rmc),
      .l1_kind        (l1_kind),
      .l1_pb          (l1_pb),
      .l1_ci          (l1_ci),
      .l_mask         (l_mask),
      .l_tc_on        (l_tc_on),
      .l_tt_on        (l_tt_on),
      .l_take         (l_take),
      .wq_valid       (wq_valid),
      .wq_addr        (wq_addr),
      .wq_fc          (wq_fc),
      .wq_wr          (wq_wr),
      .mmu_busy       (mmu_busy),
      .mmu_sts        (mmu_sts),
      .tw_valid       (tw_valid),
      .tw_want        (tw_want),
      .tw_addr        (tw_addr),
      .tw_wr          (tw_wr),
      .tw_wdata       (tw_wdata),
      .tw_ack         (tw_ack),
      .tw_last        (tw_last),
      .tw_rdata       (tw_rdata),
      .tw_berr        (tw_berr),

      .beat_valid     (beat_valid),
      .beat_err       (beat_err),
      .beat_data      (beat_data),
      .beat_addr      (beat_addr),
      .beat_num       (beat_num),
      .beat_port      (beat_port),
      .beat_ciin      (beat_ciin),
      .beat_fetch     (beat_fetch),
      .bnow_valid     (bnow_valid),
      .bnow_err       (bnow_err),
      .bnow_data      (bnow_data),
      .bnow_addr      (bnow_addr),
      .bnow_num       (bnow_num),
      .bnow_port      (bnow_port),
      .bnow_ciin      (bnow_ciin),
      .bnow_fc        (bnow_fc),
      .bnow_dfill     (bnow_dfill),
      .bnow_brq       (bnow_brq),
      .bnow_first     (bnow_first),
      .bnow_ci        (bnow_ci),
      .burst_tail     (burst_tail),

      .reset_req      (reset_req),
      .reset_busy     (reset_busy),
      .crst           (crst),
      .dbf            (dbf),
      .sts_bound      (sts_bound),
      .sts_pend       (sts_pend),
      .sts_exc        (sts_exc),
      .sts_refill     (sts_refill),
      .ipl_sync_n     (ipl_sync_n),
      .reset_sync_n   (reset_sync_n),
      .halt_sync_n    (halt_sync_n),
      .cdis_sync_n    (cdis_sync_n),
      .mmudis_sync_n  (mmudis_sync_n),
      .bus_idle       (bus_idle),
      .bus_granted    (bus_granted),

      .fc_o           (fc_o),        .fc_oe      (fc_oe),
      .a_o            (a_o),         .a_oe       (a_oe),
      .d_i            (d_i),         .d_o        (d_o),       .d_oe    (d_oe),
      .siz_o          (siz_o),       .siz_oe     (siz_oe),
      .ecs_n_o        (ecs_n_o),     .ocs_n_o    (ocs_n_o),
      .rw_o           (rw_o),        .rw_oe      (rw_oe),
      .rmc_n_o        (rmc_n_o),     .rmc_oe     (rmc_oe),
      .as_n_o         (as_n_o),      .as_oe      (as_oe),
      .ds_n_o         (ds_n_o),      .ds_oe      (ds_oe),
      .dben_n_o       (dben_n_o),    .dben_oe    (dben_oe),
      .dsack_n_i      (dsack_n_i),   .sterm_n_i  (sterm_n_i),
      .ciin_n_i       (ciin_n_i),
      .ciout_n_o      (ciout_n_o),   .ciout_oe   (ciout_oe),
      .cbreq_n_o      (cbreq_n_o),   .cbreq_oe   (cbreq_oe),
      .cback_n_i      (cback_n_i),
      .ipl_n_i        (ipl_n_i),     .avec_n_i   (avec_n_i),
      .br_n_i         (br_n_i),      .bg_n_o     (bg_n_o),    .bgack_n_i (bgack_n_i),
      .berr_n_i       (berr_n_i),
      .reset_n_i      (reset_n_i),   .reset_n_o  (reset_n_o), .reset_n_oe (reset_n_oe),
      .halt_n_i       (halt_n_i),
      .cdis_n_i       (cdis_n_i),    .mmudis_n_i (mmudis_n_i),
      .refill_n_o     (refill_n_o),  .status_n_o (status_n_o)
  );

  rd68031_mmu u_mmu (
      .clk            (clk),
      .rst_n          (rst_n),
      .crst           (crst),
      .mmudis_sync_n  (mmudis_sync_n),
      .cmd            (mmu_cmd),
      .cmd_go         (mmu_go),
      .cmd_xw         (mmu_xw),
      .cmd_fc         (mmu_fc),
      .cmd_ea         (mmu_ea),
      .wd_hi          (mmu_wd_hi),
      .wd_lo          (mmu_wd_lo),
      .rd_hi          (mmu_rd_hi),
      .rd_lo          (mmu_rd_lo),
      .cfg_bad        (mmu_cfg_bad),
      .ptest_da       (mmu_da),
      .busy           (mmu_busy),
      .done           (mmu_done),
      .l1_addr        (l1_addr),
      .l_fc           (l_fc),
      .l_rw           (l_rw),
      .l_rmc          (l_rmc),
      .l1_kind        (l1_kind),
      .l1_pb          (l1_pb),
      .l1_ci          (l1_ci),
      .l_mask         (l_mask),
      .l_tc_on        (l_tc_on),
      .l_tt_on        (l_tt_on),
      .l_take         (l_take),
      .wq_valid       (wq_valid),
      .wq_addr        (wq_addr),
      .wq_fc          (wq_fc),
      .wq_wr          (wq_wr),
      .w_sts          (mmu_sts),
      .tw_valid       (tw_valid),
      .tw_want        (tw_want),
      .tw_addr        (tw_addr),
      .tw_wr          (tw_wr),
      .tw_wdata       (tw_wdata),
      .tw_ack         (tw_ack),
      .tw_last        (tw_last),
      .tw_rdata       (tw_rdata),
      .tw_berr        (tw_berr));

  // ==========================================================================
  // What the units offer that nothing consumes. The operand's CIIN is the bus
  // unit's report to the sequencer, which has no use for it -- the data cache
  // sees CIIN cycle by cycle, on the bnow port -- and the beat's port size is
  // the registered copy of what bnow carries.
  // ==========================================================================
  logic unused_top;
  assign unused_top = &{1'b1, req_ciin, beat_port};

endmodule
