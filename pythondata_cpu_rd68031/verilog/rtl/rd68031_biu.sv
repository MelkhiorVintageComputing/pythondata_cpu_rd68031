// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// Bus interface unit. Owns every pin and every clock.
//
// ONE BUS STATE PER CLK HALF PERIOD. Even-numbered states begin on a rising edge
// and odd-numbered ones on a falling edge, so an asynchronous bus cycle with no
// wait states is three clocks (UM 7.3.1) and a synchronous one two (UM 7.3.4).
// The state machine is two registers, one per edge, each computing its next state
// from the other.
//
// THREE KINDS OF CYCLE, AND THE CYCLE DOES NOT KNOW WHICH IT IS UNTIL IT ENDS.
// Every cycle starts the same way (UM 7.3.4: a synchronous cycle "asserts and
// responds to the same signals, in the same sequence"); the device decides by how
// it terminates it:
//
//   - DSACK1/DSACK0, sampled on the falling edges (UM 7.3.1 state 3): the
//     asynchronous cycle inherited from the MC68020, three clocks at least, with
//     dynamic bus sizing;
//   - STERM, sampled on the rising edges (UM 7.3.4 state 2): a 32-bit port, two
//     clocks at least;
//   - STERM with CBACK on a cycle that asserted CBREQ: a burst, up to four long
//     words with AS held throughout (UM 7.3.7).
//
// THE UNIT OF THE CONTRACT IS AN OPERAND, NOT A BUS CYCLE. A requester presents
// one request and stalls until req_ack; this unit splits it into the one to four
// bus cycles Table 7-6 requires for that size, alignment and port width, drives
// SIZ1/SIZ0 with the number of bytes REMAINING (UM 7.2.1), routes the byte lanes
// per Tables 7-4 and 7-5, and assembles the result. Nothing above this line knows
// how wide the port was, or whether it was synchronous.
//
// That is also what makes OCS correct for free: UM 7.1.1 asserts ECS at the start
// of every bus cycle and OCS only at the start of the first cycle *of an operand*,
// and "operand" there is exactly this handshake.
//
// A burst fills a cache line, which is not an operand: its beats are reported one
// by one -- on the bnow_* port as the edge that ends each is reached, for the
// data cache, and on the registered beat_* port a clock later, for the
// instruction cache -- and the operand that asked for it completes as soon as
// its own bytes have arrived (UM 6.1.3.2, "when data from the first cycle is
// returned, it is immediately available for the execution unit"), while the
// burst runs on.
//
// THE DATA CACHE SITS IN FRONT OF THE OPERAND ENGINE (M10, doc/caches.md). A
// read the data cache holds whole is answered from it, with no bus cycle and
// whatever the bus is doing; one it holds half of starts its bus cycles at the
// half it does not -- "misaligned data transfers may span two data cache
// entries ... the hit and miss are treated independently" (UM 6.1.2); and a
// cachable read from a port narrower than 32 bits runs the extra cycles that
// complete each long word it touches (UM 6.1.3.1, figures 6-5 to 6-10).
//
// EVERY CYCLE IS TRANSLATED BEFORE IT STARTS (M11, doc/mmu.md). A new operand
// is looked up in rd68031_mmu at its first byte, and an operand that crosses
// into another page is looked up again at the cycle that enters it -- UM
// 7.1.1, a miss in the ATC aborts "the cycle". One that needs a table search
// waits while the MMU's walker runs its own operands through this engine --
// set aside (`pk_*`) if part of it has already gone -- one the MMU refuses
// "starts" as ST_ABT and ends with a bus error, and one that may go keeps its
// page's translation for the rest of its cycles in that page. The engine and
// the caches work in logical addresses; only A31-A0 are physical.

module rd68031_biu #(
    parameter bit ADDR_HIZ_BETWEEN_CYCLES = 1'b1
) (
    input  logic        clk,
    input  logic        rst_n,

    // Operand request from the sequencer ------------------------------------
    input  logic        req_valid,
    // The request without the sequencer's req_last term: what it would
    // present were its own operand not finishing on this edge. For the ATC
    // lookup only (below).
    input  logic        req_want,
    input  logic  [2:0] req_kind,
    input  logic  [2:0] req_fc,
    input  logic [31:0] req_addr,
    input  logic  [2:0] req_bytes,
    input  logic [39:0] req_wdata,
    input  logic        req_rmc,
    input  logic  [3:0] req_cpuspace,
    input  logic  [7:0] req_cpuaddr,
    input  logic        req_cpfault,
    input  logic        req_cpuraw,   // MOVES: the address is the request's own
    // A line fill may be requested for this operand: CBREQ is asserted on the
    // cycle that qualifies (UM 7.3.7, 6.1.3.2). The data cache drives it.
    input  logic        req_burst,
    // What the data cache says about the request presented -- rd68031_dcache.
    // `req_dfill`: what the bus reads for it may be cached, so it reports its
    // beats as the data cache's and, unless it is a read-modify-write, runs the
    // cycles that complete a long word from a narrow port -- each cycle as its
    // own page's CI allows, which is not the request's to know (`bnow_ci`). `dc_on`: it is a
    // read the cache is looked up for; `dc_hit_a`/`dc_hit_b` which of its
    // (at most two) long words hit, `dc_span` whether there are two, `dc_nb`
    // how many of its bytes are in the second, and `dc_data` the bytes that hit,
    // in place in the right-justified operand.
    input  logic        req_dfill,
    input  logic        dc_on,
    input  logic        dc_span,
    input  logic        dc_hit_a,
    input  logic        dc_hit_b,
    input  logic  [2:0] dc_nb,
    input  logic [39:0] dc_data,
    // The presented request is a write, and is taken on this rising edge: the
    // data cache captures it here and updates itself a clock later.
    output logic        dc_wr_take,
    // The MMU's cache inhibit for the request presented: the CI bit of the ATC
    // entry or TTx register that maps the operand's first page (UM 6.1). The
    // data cache samples it with a write it takes, for the long word in that
    // page -- rd68031_dcache. A read's cycles are cached or not as each one's
    // own CIOUT says (`bnow_ci`).
    output logic        dc_inhibit,
    // The write being taken crosses into a second page, whose translation is
    // not known until its cycles reach it (doc/mmu.md): the data cache leaves
    // the long word in that page alone until then. `dc_wb_go`: that page's
    // first cycle is starting, and the page is cachable -- the long word is
    // updated now, as a write to it would be. `dc_wb_drop`: the operand will
    // not write that page -- refused there, or ended by a bus error before it
    // got there -- and the long word is invalidated. A CI second page gets
    // neither: "when CIOUT is asserted, the data cache is completely ignored"
    // (UM 6.1.2.1).
    output logic        dc_xpage,
    output logic        dc_wb_go,
    output logic        dc_wb_drop,
    // A write the MMU refused: the data cache entries it touches are
    // invalidated -- UM 6.1.2, "the write is aborted, the corresponding entry
    // is invalidated, and a bus error exception is taken". The residual's
    // address, space and length; on the rising edge the refusal ends.
    output logic        dc_inv,
    output logic [31:0] dc_inv_addr,
    output logic  [2:0] dc_inv_fc,
    output logic  [2:0] dc_inv_bytes,
    // A posted write -- UM 11.2.5.2, doc/checkpoint.md rule 9. It gets no
    // acknowledge: `req_taken` says it has been taken, the clock after, and
    // the requester goes on. `post_busy` says one is still outstanding, and a
    // fault on it is `req_fault` with `req_fault_post`.
    input  logic        req_post,
    output logic        req_taken,
    output logic        post_busy,
    output logic        req_fault_post,
    output logic        req_ack,
    // The request presented is taken on this rising edge, which starts its
    // first bus cycle's S0. Combinational: for IPEND, which UM 8.1.9 negates
    // "during state 0 (S0) of an interrupt acknowledge cycle".
    output logic        req_start,
    output logic        req_last,
    // The data operand finishes cleanly at the rising edge that ends S5 -- no
    // bus error, no retry. A falling-edge register, valid for that half clock.
    output logic        req_early,
    output logic [39:0] req_rdata,
    output logic  [2:0] req_end,
    output logic        req_fault,
    output logic        req_fault_wr,
    output logic  [1:0] req_dsack,
    // CIIN was asserted on a read cycle that carried bytes of this operand --
    // UM 5.7.1, "interpreted on a bus-cycle-by-bus-cycle basis", "ignored
    // during all write cycles". Valid with req_ack.
    output logic        req_ciin,

    // The residual of the in-flight operand, for the fault frame -------------
    output logic [31:0] flt_addr,
    output logic  [2:0] flt_bytes,
    output logic  [2:0] flt_fc,
    output logic        flt_rw,
    output logic        flt_rmc,
    output logic [31:0] flt_dob,
    output logic [31:0] flt_dib,

    // ... and back in, when RTE says to finish it ----------------------------
    input  logic        rst_op_valid,
    // The microword RTE resumed has retired without asking for an operand: the
    // fault was an instruction prefetch at a boundary, and there is nothing to
    // hand back. Whatever is pending is dropped.
    input  logic        rst_cancel,
    // The operand handed back is a posted write: run it now, for nobody.
    input  logic        rst_post,
    input  logic [31:0] rst_addr,
    input  logic  [2:0] rst_bytes,
    input  logic  [2:0] rst_fc,
    input  logic        rst_rw,
    input  logic        rst_rmc,
    input  logic [31:0] rst_dob,

    // Instruction fetch ------------------------------------------------------
    input  logic        fetch_valid,
    input  logic [31:0] fetch_addr,
    input  logic  [2:0] fetch_fc,
    input  logic        fetch_burst,
    output logic        fetch_ack,
    output logic        fetch_last,
    output logic [31:0] fetch_rdata,
    output logic        fetch_fault,
    output logic        fetch_ciin,
    // ... and whether the MMU inhibited caching it (UM 6.1): the instruction
    // cache does not take a long word whose page or TTx block has CI set.
    // Valid with fetch_ack.
    output logic        fetch_ci,
    input  logic        bus_abort,

    // The MMU -- rd68031_mmu, doc/mmu.md ---------------------------------------
    // Every cycle is translated before it starts (UM 9.2.1). An operand is
    // looked up at its first byte when it is taken, and again at the first
    // cycle that enters its next page, if it crosses into one; each page's
    // translation is kept for the operand's other cycles in that page. MMUDIS
    // as the MMU sees it is held for as long as an operand is under way -- UM
    // 9.2.3, "the disabling of address translation does not become effective
    // until the entire transfer is complete".
    output logic [31:0] l1_addr,
    output logic  [2:0] l_fc,
    output logic        l_rw,
    output logic        l_rmc,
    input  logic  [1:0] l1_kind,      // 00 go, 01 refused, 10 search first
    input  logic [31:0] l1_pb,        // the physical page base
    input  logic        l1_ci,
    input  logic [31:0] l_mask,       // the page offset bits
    input  logic        l_tc_on,      // TC.E
    input  logic        l_tt_on,      // TT0.E or TT1.E
    output logic        l_take,       // the operand looked up is taken
    // A table search, for the operand that has to wait for one.
    output logic        wq_valid,
    output logic [31:0] wq_addr,
    output logic  [2:0] wq_fc,
    output logic        wq_wr,
    input  logic        mmu_busy,     // a search is under way: nothing else starts
    input  logic        mmu_sts,      // ... and has just begun, for STATUS
    // The walker's own operands: four bytes, physical, supervisor data space,
    // RMC asserted, never cached -- UM 7.3.3, 9.5.2.
    input  logic        tw_valid,
    input  logic        tw_want,      // tw_valid but for tw_last, for the lookup
    input  logic [31:0] tw_addr,
    input  logic        tw_wr,
    input  logic [31:0] tw_wdata,
    output logic        tw_ack,
    output logic        tw_last,
    output logic [31:0] tw_rdata,
    output logic        tw_berr,

    // Every read bus cycle, and every beat of a burst, as it ends ------------
    // One clock per beat, registered on the rising edge that ends it. This is
    // what fills a cache entry: the whole of the 32 bits latched, the address
    // of the long word they are (for a burst beat, A3/A2 as the system
    // incremented them -- UM 7.3.7), the port that answered and its CIIN.
    output logic        beat_valid,   // the beat ended normally
    output logic        beat_err,     // ... or with a bus error: its entry is invalid
    output logic [31:0] beat_data,
    output logic [31:0] beat_addr,
    output logic  [1:0] beat_num,     // 0 is the cycle the requester asked for
    output logic  [1:0] beat_port,    // the DSACK encoding; a synchronous beat is 32
    output logic        beat_ciin,
    output logic        beat_fetch,   // the operand was an instruction prefetch

    // ... and the same beat, combinationally, for the rising edge that ends it:
    // what the data cache writes on that edge. Valid while beat_end is.
    output logic        bnow_valid,
    output logic        bnow_err,
    output logic [31:0] bnow_data,
    output logic [31:0] bnow_addr,
    output logic  [1:0] bnow_num,
    output logic  [1:0] bnow_port,
    output logic        bnow_ciin,
    output logic  [2:0] bnow_fc,
    output logic        bnow_dfill,   // its operand may fill the data cache
    output logic        bnow_brq,     // the cycle asserted CBREQ
    output logic        bnow_first,   // the first cycle of its operand
    output logic        bnow_ci,      // the cycle asserted CIOUT: its page is CI
    // A burst is under way past its first long word: from the rising edge after
    // that long word to the one that ends the last. Registered.
    output logic        burst_tail,

    // Status -----------------------------------------------------------------
    input  logic        reset_req,
    output logic        reset_busy,
    // The RESET pin, asserted from outside: the processor is being reset --
    // UM 7.8, 8.1.1. Registered on a rising edge, and held for as long as the
    // pin is; the sequencer and the fetch unit are held in their reset state
    // by it, and so is everything in this unit but the arbiter.
    output logic        crst,
    input  logic        dbf,
    // What the microsequencer tells the STATUS and REFILL pins -- UM 12.7.1.
    // Each a one-clock pulse, registered on a rising edge.
    input  logic        sts_bound,    // an instruction boundary
    input  logic        sts_pend,     // ... at which a trace or interrupt is taken
    input  logic        sts_exc,      // exception processing begins
    input  logic        sts_refill,   // the pipeline is refilled
    output logic  [2:0] ipl_sync_n,
    output logic        reset_sync_n,
    output logic        halt_sync_n,
    output logic        cdis_sync_n,
    output logic        mmudis_sync_n,
    output logic        bus_idle,
    output logic        bus_granted,

    // Pins -------------------------------------------------------------------
    output logic  [2:0] fc_o,
    output logic        fc_oe,
    output logic [31:0] a_o,
    output logic        a_oe,
    input  logic [31:0] d_i,
    output logic [31:0] d_o,
    output logic        d_oe,
    output logic  [1:0] siz_o,
    output logic        siz_oe,
    output logic        ecs_n_o,
    output logic        ocs_n_o,
    output logic        rw_o,
    output logic        rw_oe,
    output logic        rmc_n_o,
    output logic        rmc_oe,
    output logic        as_n_o,
    output logic        as_oe,
    output logic        ds_n_o,
    output logic        ds_oe,
    output logic        dben_n_o,
    output logic        dben_oe,
    input  logic  [1:0] dsack_n_i,
    input  logic        sterm_n_i,
    input  logic        ciin_n_i,
    output logic        ciout_n_o,
    output logic        ciout_oe,
    output logic        cbreq_n_o,
    output logic        cbreq_oe,
    input  logic        cback_n_i,
    input  logic  [2:0] ipl_n_i,
    input  logic        avec_n_i,
    input  logic        br_n_i,
    output logic        bg_n_o,
    input  logic        bgack_n_i,
    input  logic        berr_n_i,
    input  logic        reset_n_i,
    output logic        reset_n_o,
    output logic        reset_n_oe,
    input  logic        halt_n_i,
    input  logic        cdis_n_i,
    input  logic        mmudis_n_i,
    output logic        refill_n_o,
    output logic        status_n_o
);

  // iverilog rejects a declaration inside an unnamed begin/end, so every loop
  // variable lives at module scope -- and each always_comb needs its own, or a
  // shared one is reported as MULTIDRIVEN by the lint pass.
  int unsigned i_v;
  int unsigned i_rv;
  int unsigned i_rd;
  int unsigned i_wr;

  // ==========================================================================
  // Input synchronisers -- UM 7.1, figures 7-1 and 7-2
  //
  // "For all asynchronous inputs, the processor latches the level of the input
  // during a sample window around the falling edge of the clock signal", so both
  // ranks are negedge. DSACK, BERR, HALT and AVEC are not here: they go straight
  // to the falling-edge next-state logic, which is the sample UM 7.3.1 describes
  // and which must not have a rank of latency added to it. STERM, CBACK and CIIN
  // are not here either: they are SYNCHRONOUS inputs (UM 7.1), "externally
  // synchronized to the processor clock", and are sampled on the rising edges.
  // ==========================================================================
  rd68031_sync #(.WIDTH (3), .RESET_VAL (3'b111)) u_sync_ipl (
      .clk (clk), .rst_n (rst_n), .d (ipl_n_i), .q (ipl_sync_n));

  rd68031_sync #(.WIDTH (1), .RESET_VAL (1'b1)) u_sync_reset (
      .clk (clk), .rst_n (rst_n), .d (reset_n_i), .q (reset_sync_n));

  rd68031_sync #(.WIDTH (1), .RESET_VAL (1'b1)) u_sync_halt (
      .clk (clk), .rst_n (rst_n), .d (halt_n_i), .q (halt_sync_n));

  // CDIS goes on to both caches' lookups, and the data cache's lookup to the
  // ATC's address and the decision to start a cycle: so it is taken once more
  // on a rising edge (M15 phase B), giving that chain a whole clock from it
  // rather than half of one. The caches see it half a clock later; the
  // manual gives CDIS no latency to keep (UM 5.11.1, 6.1).
  logic cdis_raw_n;
  rd68031_sync #(.WIDTH (1), .RESET_VAL (1'b1)) u_sync_cdis (
      .clk (clk), .rst_n (rst_n), .d (cdis_n_i), .q (cdis_raw_n));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) cdis_sync_n <= 1'b1;
    else        cdis_sync_n <= cdis_raw_n;
  end

  // UM 5.11.2: MMUDIS "dynamically disables the translation of addresses by the
  // MMU". Synchronised here, and held for the MMU while an operand is under
  // way -- below, with the translation.
  logic mmudis_raw_n;
  rd68031_sync #(.WIDTH (1), .RESET_VAL (1'b1)) u_sync_mmudis (
      .clk (clk), .rst_n (rst_n), .d (mmudis_n_i), .q (mmudis_raw_n));

  logic br_sync_n;
  logic bgack_sync_n;
  logic hiz_q;

  rd68031_sync #(.WIDTH (1), .RESET_VAL (1'b1)) u_sync_br (
      .clk (clk), .rst_n (rst_n), .d (br_n_i), .q (br_sync_n));

  rd68031_sync #(.WIDTH (1), .RESET_VAL (1'b1)) u_sync_bgack (
      .clk (clk), .rst_n (rst_n), .d (bgack_n_i), .q (bgack_sync_n));

  // ==========================================================================
  // The operand in flight
  //
  // op_rem is the number of bytes still to move. It is what drives SIZ1/SIZ0, it
  // is what the fault frame records as the residual, and it reaching zero is what
  // acknowledges the request.
  // ==========================================================================
  logic        op_active;
  logic [31:0] op_addr;
  logic  [2:0] op_rem;
  logic  [2:0] op_fc;
  logic        op_rw;        // 1 = read
  logic        op_rmc;
  logic        op_first;     // no bus cycle of this operand has started yet -- OCS
  logic        op_isfetch;   // the request came from the instruction fetch unit
  logic        op_istw;      // ... or from the MMU's table walker
  // The translation of the page the operand's cycles are in now: the page
  // offset mask, that page's logical base, its physical base and its cache
  // inhibit. Taken when the operand is, and again when a cycle of it enters
  // the next page (UM 9.2.1, doc/mmu.md).
  logic [31:0] op_pmask;
  logic [31:0] op_lp1;
  logic [31:0] op_pb1;
  logic        op_ci1;
  logic        wb_pend;      // a write's second page's long word, not yet settled in the data cache
  logic        op_posted;    // a posted write: no acknowledge -- rule 9
  logic        rst_post_q;   // the operand RTE handed back is a posted write
  // A bus error on this CPU-space operand is a bus error and not an answer: a
  // coprocessor interface register other than the one that starts an
  // instruction -- UM 10.5.2.8.
  logic        op_cpflt;
  // A line fill may be asked for (req_burst / fetch_burst), and whether one has
  // been: UM 6.1.3.2 asserts CBREQ once, and a second read cycle for the rest of
  // a misaligned operand goes "with CBREQ negated".
  logic        op_burst;
  logic        op_brq_used;
  logic        op_ciin;      // CIIN seen on a cycle that carried operand bytes
  // The data cache's part in the operand -- doc/caches.md.
  logic        op_dfill;     // its reads may fill the data cache
  logic        op_fillc;     // ... and a long word from a narrow port is completed
  logic  [2:0] op_tail;      // its last bytes hit in the data cache: stop short
  // The cycles that complete a cache entry from a narrow port -- UM 6.1.3.1.
  // They carry none of the operand: op_addr and op_rem stand still while they
  // run, and only the data cache keeps what they read.
  logic        fl_on;        // the next cycle is one of them
  logic [31:0] fl_addr;      // ... at this address
  logic  [2:0] fl_rem;       // ... with this many bytes of the entry still to come
  logic [29:0] ent_q;        // the long word the last read cycle carried
  logic  [3:0] ent_got;      // ... the bytes of it read so far, by offset
  logic        ent_bad;      // ... and CIIN or BERR on one of them

  // An operand set aside part-way: its first page's bytes have gone, and the
  // page its next cycle enters needs a table search (UM 9.2.1). The walker's
  // operands go through this same engine, so the operand's state waits here
  // while they run, and comes back as it was -- the residual, what has been
  // read or is still to be written, and how far its line fill and CBREQ got.
  // Nothing else is taken while one waits: the walker's operands, then this
  // one, then whatever is next. Cycle-level, as the operand registers are: no
  // fault is taken while it waits (doc/checkpoint.md rule 11), and one on its
  // residual is taken once it is back in op_*.
  logic        pk_valid;
  logic [31:0] pk_addr;
  logic  [2:0] pk_rem;
  logic  [2:0] pk_fc;
  logic        pk_rw;
  logic        pk_rmc;
  logic [39:0] pk_data;
  logic        pk_posted;
  logic        pk_cpflt;
  logic        pk_burst;
  logic        pk_brq_used;
  logic        pk_ciin;
  logic        pk_dfill;
  logic        pk_fillc;
  logic  [2:0] pk_tail;

  // Declared here, above the logic that reads them, rather than beside the
  // logic that drives them: Quartus, Vivado and Questa all refuse a use above
  // the declaration, and tools/src_lint.py holds `make lint` to that.
  logic        rst_pend_q;
  logic        rsto_q;
  // The RESET pin's reset -- below, with the RESET instruction's counter --
  // and the reset of everything in this unit but the arbiter: rst_n, or the
  // pin. UM 7.8: "the external RESET signal resets the processor and the
  // entire system".
  logic        crst_q;
  logic        eng_rst_n;
  assign eng_rst_n = rst_n && !crst_q;
  // ... and the same reset for the falling-edge registers, from a copy of
  // crst_q taken on the falling edge (M15 phase B, doc/size-and-speed.md).
  // From crst_q itself, one LUT and a fan-out over the whole unit had half a
  // clock to reach them. The falling-edge registers are reset half a clock
  // after the rest and released half a clock after them: on that edge the
  // bus is idle and every one of them would have taken its reset value
  // anyway -- the rising-edge state is IDLE, and the sequencer and the MMU,
  // released on the same rising edge as this unit, present nothing until the
  // next. The pins they drive are not driven during the reset (eng_rst_n,
  // below), STATUS and REFILL aside, which go inactive half a clock later.
  logic        crst_nq;
  logic        eng_rst_nn;
  assign eng_rst_nn = rst_n && !crst_nq;
  logic [39:0] op_data;      // write data, or the read accumulator, right justified

  // The result of the last completed operand of each kind, held until the next
  // one of that kind completes. op_data cannot serve: it is the accumulator of
  // the operand in flight, and the instruction fetch unit refills the pipe by
  // itself, so a prefetch starting in the clock after a data read would destroy
  // the data before the sequencer had read it.
  logic [39:0] rdata_q;
  logic [31:0] frdata_q;

  // The cycle in flight, latched at the rising edge entering S0 so that every pin
  // is stable for the whole cycle whatever the operand state does. A burst holds
  // all of them for its whole length: "the address bus of the MC68030 remains
  // driven to a constant value for the duration of a burst transfer operation"
  // (UM 7.3.7).
  logic [31:0] cyc_addr;     // logical: the caches and the operand engine's
  logic [31:0] cyc_paddr;    // physical: A31-A0
  logic        cyc_abt;      // the MMU refused it: no pin moves -- ST_ABT
  logic  [2:0] cyc_fc;
  logic  [1:0] cyc_siz;
  logic  [2:0] cyc_n;        // bytes this cycle asks for: min(op_rem, 4)
  logic        cyc_rw;
  logic        cyc_brq;      // CBREQ is asserted for this cycle
  logic        cyc_fill;     // a cycle that completes a cache entry -- UM 6.1.3.1
  logic        cyc_ciout;

  // ==========================================================================
  // Bus state. Two registers, one per edge.
  //
  //   st_p holds S0, S2, S4, WH, BH, IDLE, HALT or RETRY  (rising edge)
  //   st_n holds S1, S3, WL, S5, BL, BW or IDLE           (falling edge)
  //
  // WH/WL are the wait pair inserted between S3 and S4: UM 7.3.1 state 3, "if wait
  // states are added, the processor continues to sample the DSACKx signals on the
  // falling edges of the clock until one is recognized", so one wait is one whole
  // clock and the sample repeats at ST_WL. The same pair serves a synchronous
  // cycle's waits, whose STERM sample is at ST_WH: "if STERM is negated at the
  // beginning of S2, wait states are inserted after S2, and STERM is sampled on
  // every rising edge thereafter until it is recognized" (UM 7.3.4 state 2).
  //
  // ST_S5 is the half clock in which a cycle ENDS, whichever way it ends: the
  // asynchronous cycle's S5, and the synchronous cycle's S3, which UM 7.3.4
  // describes in the same words -- "the processor negates AS, DS, and DBEN"; the
  // address and the rest stay valid; the next cycle may start at the rising edge
  // that ends it.
  //
  // BL, BH and BW are the burst: BL is the half clock after a long word has been
  // latched with the burst continuing (the manual's S3, S5 and S7, AS held), BH
  // the rising edge that tests STERM for the next long word (S4, S6, S8), and BW
  // a wait state between them (UM 7.3.7 state 4, "wait states are inserted
  // instead of S4 and S5").
  // ==========================================================================
  rd68031_pkg::bus_state_e st_p, st_p_nxt;
  rd68031_pkg::bus_state_e st_n, st_n_nxt;

  logic [1:0] dsack_q;       // the port size sampled by DSACKx
  logic [2:0] end_now;       // how the cycle now ending ended, live
  logic [2:0] req_end_q;     // ... and the copy the sequencer is given
  logic       term_q;        // ... and whether it terminated the cycle at all
  logic       term_err;      // ... as a bus error (Table 7-8 cases 3 and 4)
  logic       term_rty;      // ... as a retry (cases 5 and 6)
  logic       term_hlt;      // ... normally, but with HALT asserted (case 2)
  logic       term_avc;      // ... by AVEC, on an interrupt acknowledge cycle
  logic       term_syn;      // ... by STERM
  logic       term_bfe;      // ... a burst fill beat ended by BERR (UM 7.5.1)

  // The synchronous inputs as the last rising-edge sample saw them -- UM 7.2.10:
  // "the assertion or negation of CBACK and CIIN is internally latched on the
  // rising edge of the clock for which STERM is asserted". For a cycle ended
  // by DSACKx, CIIN is "latched on the rising edge of bus cycle state 4"
  // (UM 7.2.8, 7.2.9), and that is the same register.
  logic       sterm_q;
  logic       cback_q;
  logic       ciin_q;

  // How many long words this cycle has latched, counted on the falling edges
  // that latch them. A burst stops at four (UM 7.3.7), and CBREQ is negated
  // after the third (state 7).
  logic [2:0] beat_q;

  // RMC is a qualifier held across a run of ordinary cycles, not a cycle kind
  // (UM 7.3.3). It is raised by the first request of the run and stays up until
  // a request without it starts a cycle -- which is exactly UM 7.1.1's "RMC is
  // guaranteed to be negated before the end of state 0 for a bus cycle following
  // a read-modify-write operation" -- or until the bus goes idle without one.
  logic rmc_hold;

  // UM 7.5.3: HALT alone does not terminate a cycle; it stops the next one. While
  // halted the data bus goes high impedance and the bus control signals are driven
  // inactive, but the address, FC, SIZ and R/W "remain in the same state" --
  // driven, not released, which is what distinguishes this from an ordinary idle
  // bus.
  logic halt_hold;

  // ---------------------------------------------------------------------------
  // Bus arbitration -- UM 7.7.4
  // ---------------------------------------------------------------------------
  rd68031_pkg::arb_state_e arb, arb_nxt;

  logic arb_r;   // BR, synchronised and in positive logic
  logic arb_a;   // BGACK, likewise
  assign arb_r = ~br_sync_n;
  assign arb_a = ~bgack_sync_n;

  // "The BG output will not be asserted while RMC is asserted" -- the note under
  // figure 7-61, and 7.7.4: the arbiter ignores "bus requests (assertions of BR)
  // that occur after the first read cycle of the read-modify-write sequence".
  //
  // The one exception is that first read cycle: "the MC68030 allows normal bus
  // arbitration during this read cycle; a normal relinquish and retry operation
  // (asserting BERR, HALT, and BR at the same time) is used" (UM 7.7.4), and
  // "the MC68030 does not relinquish the bus during a read-modify-write
  // operation, except during the first read cycle" (UM 7.5.2). So while the
  // first bus cycle of a locked run is waiting to be retried, the lock does not
  // hold the bus.
  logic rmc_first_q;   // the cycle under way is the first of a locked run
  logic rmc_lock;
  assign rmc_lock = rmc_hold
                 && !(rmc_first_q && (st_p == rd68031_pkg::ST_RETRY));

  logic arb_req;
  assign arb_req = arb_r && !rmc_lock;

  function automatic logic arb_g_of(input rd68031_pkg::arb_state_e st);
    arb_g_of = (st == rd68031_pkg::ARB_GRANT) || (st == rd68031_pkg::ARB_WAIT)
            || (st == rd68031_pkg::ARB_REGRANT) || (st == rd68031_pkg::ARB_REWAIT);
  endfunction

  function automatic logic arb_t_of(input rd68031_pkg::arb_state_e st);
    arb_t_of = (st != rd68031_pkg::ARB_IDLE);
  endfunction

  // The operand finishes with the cycle (or the burst beat) that is ending now.
  // This is the signal the whole handshake turns on: req_last exports it, and the
  // sequencer is expected to have the next request already presented by the
  // rising edge that consumes it -- which is what the microword's successor
  // previews are for.
  logic op_finishing;
  logic op_continuing;

  // A request is available to start a bus cycle. Data beats instruction fetch:
  // the sequencer is stalled on a data operand and the pipe is not.
  //
  // The arbiter's NEXT state decides whether a cycle may start, not its current
  // one. Reading the current state there is a real bug with a real failure mode:
  // on the single edge where the arbiter reaches its granting state, a cycle
  // begins anyway and then runs with its address bus already in high impedance,
  // which on the MC68010 project cost a long-word read one of its two words every
  // few thousand DMA transfers.
  logic want_cycle;
  // RTE has a posted write to rerun, and the edge a posted write faults on.
  logic rst_self, post_flt_end;
  // Nothing new starts on the edge a posted write ends in a bus error: the
  // sequencer is about to take it, and with nothing taken the state machine
  // would go to S0 by its retry arm and run the faulted write again.
  //
  // A read the data cache answers whole (`hit_take`) wants no cycle, and nor
  // does one held back for a clock while the data cache is being written
  // (`rd_block`); neither lets a prefetch past it, because the second is still
  // the sequencer's next bus operand. `req_bus` is the request as the bus sees
  // it. All three are assigned with the data cache's logic below.
  logic hit_take, rd_block, req_bus, fetch_go;
  // A table search holds everything but its own operands -- UM 9.5.2, "the
  // entire table search completes without interruption" -- and an operand set
  // aside for one comes back before anything else.
  assign want_cycle = op_continuing
                   || (!post_flt_end && (tw_valid
                                         || (!mmu_busy
                                             && (pk_valid || req_bus || fetch_go
                                                 || rst_self))));

  logic bus_is_idle;
  assign bus_is_idle = (st_p == rd68031_pkg::ST_IDLE)
                       && (st_n == rd68031_pkg::ST_IDLE);

  // HALT stops the NEXT cycle from starting, whether the bus was busy or idle
  // when it came: "the MC68030 halts external bus activity at the next bus
  // cycle boundary" (UM 7.5.3). Synchronised, as every asynchronous input
  // other than the termination samples is.
  // ... and an operand whose translation needs a table search does not start:
  // the search does, and the operand is looked up again when it is over (UM
  // 9.2.1, "an ATC entry is created after the table search, and the access is
  // retried"). One the MMU refuses starts as ST_ABT rather than S0. Both are
  // assigned with the translation, below.
  logic xl_wait, cyc_abt_now;
  logic start_ok;
  logic s5_cross;
  assign start_ok = want_cycle && halt_sync_n && !arb_t_of(arb_nxt) && !xl_wait
                 && !s5_cross;

  // Retry clears when BOTH BERR and HALT have been negated -- UM 7.5.2, "does not
  // begin another bus cycle until the HALT signal is negated by external logic",
  // and BERR "should be negated before S2 of the read cycle".
  //
  // Sampled on the FALLING edge, like every other use of these two pins. They
  // are asynchronous inputs, which UM 7.1 latches "during a sample window around
  // the falling edge of the clock" and specifications 47A and 47B bound only
  // there; a device is free to move them at a rising edge -- UM 7.5 even
  // recommends it. This was the raw pins, read by the rising-edge state
  // register's next-state logic, which is a metastable sample of a multi-bit
  // register exactly where a device changes them (doc/bugs-found.md, M3). One
  // falling-edge flop, as the termination samples are: no synchroniser rank is
  // added, so the restart is at most half a clock later than it was.
  logic retry_clr_q;
  logic retry_clear;
  assign retry_clear = retry_clr_q;

  // The asynchronous termination inputs, in positive logic, for the falling-edge
  // samples; and the synchronous ones, for the rising-edge samples.
  logic berr_s, halt_s, avec_s;
  logic sterm_s, cback_s, ciin_s;
  assign berr_s  = ~berr_n_i;
  // AVEC is active low and is sampled on the same edge as DSACK. Only an
  // interrupt acknowledge cycle means anything by it -- UM 7.1.7.
  assign avec_s  = avec_n_i;
  assign halt_s  = ~halt_n_i;
  assign sterm_s = ~sterm_n_i;
  assign cback_s = ~cback_n_i;
  assign ciin_s  = ~ciin_n_i;

  // The burst goes on past the long word being latched at this falling edge.
  // UM 7.3.7: "burst mode is only initiated if both of these signals [CBREQ and
  // CBACK] are asserted for a synchronous cycle"; it is aborted by "CIIN
  // asserted, BERR asserted, or CBACK negated prematurely"; and "the MC68030
  // allows a burst of as many as four long words".
  logic burst_go;
  assign burst_go = cyc_brq && sterm_q && cback_q && !ciin_q && !berr_s
                 && (beat_q < 3'd3);

  // The if/else inside each case item is not a style choice: iverilog rejects a
  // ternary of two enum values assigned to an enum variable with "This assignment
  // requires an explicit cast" (doc/coding-standard.md, measured).
  always_comb begin
    st_p_nxt = st_p;
    if (st_p == rd68031_pkg::ST_RETRY) begin
      // UM 7.5.2: "after a synchronization delay, the processor retries the
      // previous cycle using the same access information". Nothing in cyc_* or
      // op_* was updated by the faulted cycle, so re-entering S0 reissues it --
      // with CBREQ again, if it had it: "on the initial access of a burst
      // operation, a retry ... causes the processor to retry the bus cycle and
      // assert CBREQ again".
      if (retry_clear && !arb_t_of(arb_nxt)) st_p_nxt = rd68031_pkg::ST_S0;
      else                                   st_p_nxt = rd68031_pkg::ST_RETRY;
    end else if (st_p == rd68031_pkg::ST_HALT) begin
      if (!halt_sync_n)                st_p_nxt = rd68031_pkg::ST_HALT;
      else if (start_ok && cyc_abt_now) st_p_nxt = rd68031_pkg::ST_ABT;
      else if (start_ok)               st_p_nxt = rd68031_pkg::ST_S0;
      else                             st_p_nxt = rd68031_pkg::ST_IDLE;
    end else begin
      unique case (st_n)
      rd68031_pkg::ST_IDLE: begin
        if (start_ok && cyc_abt_now) st_p_nxt = rd68031_pkg::ST_ABT;
        else if (start_ok)           st_p_nxt = rd68031_pkg::ST_S0;
        else                         st_p_nxt = rd68031_pkg::ST_IDLE;
      end
      rd68031_pkg::ST_S1: st_p_nxt = rd68031_pkg::ST_S2;
      rd68031_pkg::ST_S3: begin
        if (term_q) st_p_nxt = rd68031_pkg::ST_S4;
        else        st_p_nxt = rd68031_pkg::ST_WH;
      end
      rd68031_pkg::ST_WL: begin
        if (term_q) st_p_nxt = rd68031_pkg::ST_S4;
        else        st_p_nxt = rd68031_pkg::ST_WH;
      end
      // A burst: the next long word is tested for on this rising edge, whether
      // the last one was latched (state 4) or not yet (a wait state).
      rd68031_pkg::ST_BL,
      rd68031_pkg::ST_BW: st_p_nxt = rd68031_pkg::ST_BH;
      // The rising edge that ends S5. Table 7-8 decides where it goes. A burst
      // passes through here once, at its end: "for the purposes of halting the
      // processor or arbitrating the bus away from the processor with BR, a
      // burst operation is a single cycle" (UM 7.3.7).
      rd68031_pkg::ST_S5: begin
        if (term_rty)                st_p_nxt = rd68031_pkg::ST_RETRY;
        else if (term_hlt || !halt_sync_n)
                                     st_p_nxt = rd68031_pkg::ST_HALT;
        else if (start_ok && cyc_abt_now)
                                     st_p_nxt = rd68031_pkg::ST_ABT;
        else if (start_ok)           st_p_nxt = rd68031_pkg::ST_S0;
        else                         st_p_nxt = rd68031_pkg::ST_IDLE;
      end
      default: st_p_nxt = rd68031_pkg::ST_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // The arbiter's next state -- UM figure 7-61, all seven states
  //
  // Read off a rendering of the figure (its overbars are lost in the text
  // layer, not on the page). R is BR and A is BGACK, both synchronised; G is
  // BG and T the three-state control. Each arc is the figure's label:
  //
  //   0 --RA'--> 1    0 --XA--> 4    0 --R'A'--> 0
  //   1 --XX---> 2
  //   2 --RA'--> 2    2 --XA or R'A'--> 3
  //   3 --XX---> 4
  //   4 --R'A--> 4    4 --R'A'--> 0  4 --RX--> 5
  //   5 --XX---> 6
  //   6 --RA---> 6    6 --RA'--> 2   6 --R'X--> 3
  //
  // 0-1-2-3-4-0 is the ordinary relinquish 7.7.4 walks through in prose. 0-4 is
  // single-wire arbitration: "BGACK alone can be used to place the processor's
  // external bus buffers in the high-impedance state". 4-5-6 is the re-grant of
  // 7.7.3, "if a BR is still pending after the assertion of BGACK, another BG is
  // asserted within a few clocks of the negation of BG", with "the processor
  // does not perform any external bus cycles before it reasserts BG" -- T stays
  // asserted throughout -- and state 6 holds that grant for as long as the old
  // master still asserts BGACK, handing over to state 2 to wait for the new
  // one's.
  //
  // R is the request as the RMC lock leaves it wherever it can assert G:
  // "the BG output will not be asserted while RMC is asserted".
  // ---------------------------------------------------------------------------
  always_comb begin
    arb_nxt = arb;
    unique case (arb)
      // "Request R and acknowledge A keep the arbiter in state 0 as long as they
      // are both negated. When a request R is received, both grant G and signal T
      // are asserted" -- "except when the MC68030 has made an internal decision
      // to execute a bus cycle. Then, the assertion of BG is deferred until the
      // bus cycle has begun" (UM 7.7).
      //
      // That deferral is exactly one edge: the edge on which an idle bus with a
      // request pending enters S0. Testing st_p_nxt for it would be the direct
      // transcription and is a combinational loop -- st_p_nxt depends on
      // start_ok, which depends on this state -- so the test is on the registered
      // state instead, which says the same thing one expression earlier. A bus
      // held by HALT has made no such decision, and does not defer.
      rd68031_pkg::ARB_IDLE: begin
        if (arb_a)
          arb_nxt = rd68031_pkg::ARB_HELD;
        // Nor has a processor held in reset by the RESET pin, whatever its
        // sequencer is presenting -- UM 7.7, arbitration "during RESET
        // assertion".
        else if (arb_req && !(bus_is_idle && want_cycle && halt_sync_n
                              && !crst_q))
          arb_nxt = rd68031_pkg::ARB_GRANT;
        else
          arb_nxt = rd68031_pkg::ARB_IDLE;
      end
      // "The next clock causes a change to state 2."
      rd68031_pkg::ARB_GRANT: arb_nxt = rd68031_pkg::ARB_WAIT;
      // "The bus arbiter remains in that state until acknowledge A is asserted or
      // request R is negated."
      rd68031_pkg::ARB_WAIT: begin
        if (arb_a || !arb_r) arb_nxt = rd68031_pkg::ARB_DROP;
        else                 arb_nxt = rd68031_pkg::ARB_WAIT;
      end
      // "The next clock takes the arbiter to state 4."
      rd68031_pkg::ARB_DROP: arb_nxt = rd68031_pkg::ARB_HELD;
      // "With acknowledge A asserted, the arbiter remains in state 4 until A is
      // negated or request R is again asserted. When A is negated, the arbiter
      // returns to the original state."
      rd68031_pkg::ARB_HELD: begin
        if (arb_req)     arb_nxt = rd68031_pkg::ARB_REGRANT;
        else if (!arb_a) arb_nxt = rd68031_pkg::ARB_IDLE;
        else             arb_nxt = rd68031_pkg::ARB_HELD;
      end
      rd68031_pkg::ARB_REGRANT: arb_nxt = rd68031_pkg::ARB_REWAIT;
      rd68031_pkg::ARB_REWAIT: begin
        if (!arb_r)      arb_nxt = rd68031_pkg::ARB_DROP;
        else if (!arb_a) arb_nxt = rd68031_pkg::ARB_WAIT;
        else             arb_nxt = rd68031_pkg::ARB_REWAIT;
      end
      default: arb_nxt = rd68031_pkg::ARB_IDLE;
    endcase
  end

  // The falling-edge state. Two of the rising-edge states, S2 and WH, are where a
  // synchronous cycle may have been terminated, and BH is where a burst beat is:
  // "once STERM is recognized, data is latched on the next falling edge of the
  // clock (corresponding to the beginning of S3)" (UM 7.3.4 state 2).
  always_comb begin
    st_n_nxt = st_n;
    unique case (st_p)
      rd68031_pkg::ST_S0:   st_n_nxt = rd68031_pkg::ST_S1;
      rd68031_pkg::ST_S2: begin
        if (!sterm_q)      st_n_nxt = rd68031_pkg::ST_S3;
        else if (burst_go) st_n_nxt = rd68031_pkg::ST_BL;
        else               st_n_nxt = rd68031_pkg::ST_S5;
      end
      rd68031_pkg::ST_S4:   st_n_nxt = rd68031_pkg::ST_S5;
      // A cycle the MMU refused ends here, with no strobe ever asserted.
      rd68031_pkg::ST_ABT:  st_n_nxt = rd68031_pkg::ST_S5;
      rd68031_pkg::ST_WH: begin
        if (!sterm_q)      st_n_nxt = rd68031_pkg::ST_WL;
        else if (burst_go) st_n_nxt = rd68031_pkg::ST_BL;
        else               st_n_nxt = rd68031_pkg::ST_S5;
      end
      // A burst beat. BERR recognised here, with or without STERM, ends the
      // burst (UM 7.5.1: "when a bus error occurs after the burst mode has been
      // entered ... the processor terminates the burst operation").
      rd68031_pkg::ST_BH: begin
        if (!sterm_q && !berr_s) st_n_nxt = rd68031_pkg::ST_BW;
        else if (burst_go)       st_n_nxt = rd68031_pkg::ST_BL;
        else                     st_n_nxt = rd68031_pkg::ST_S5;
      end
      default:              st_n_nxt = rd68031_pkg::ST_IDLE;
    endcase
  end

  // The falling edge that latches a long word and ends a synchronous transfer --
  // the first cycle's, or a burst beat's.
  logic syn_edge;
  assign syn_edge = ((st_p == rd68031_pkg::ST_S2) || (st_p == rd68031_pkg::ST_WH)
                     || (st_p == rd68031_pkg::ST_BH))
                 && ((st_n_nxt == rd68031_pkg::ST_S5)
                     || (st_n_nxt == rd68031_pkg::ST_BL));

  // The rising edge that consumes a transfer: the end of S5, or the end of a
  // burst beat's BL.
  logic beat_end;
  assign beat_end = (st_n == rd68031_pkg::ST_S5) || (st_n == rd68031_pkg::ST_BL);

  // ==========================================================================
  // Dynamic bus sizing -- UM 7.2.1, Tables 7-1, 7-4, 7-5, 7-6 and 7-7
  //
  // "Dynamic bus sizing requires that the portion of the data bus used for a
  // transfer to or from a particular port size be fixed. A 32-bit port must reside
  // on data bus bits 0-31, a 16-bit port must reside on data bus bits 16-31, and
  // an 8-bit port must reside on data bus bits 24-31."
  //
  // So the number of bytes a cycle actually moves is however many of the ones it
  // asked for fit in the port from the offset the address lands on. That single
  // expression reproduces every row of Table 7-6. STERM is a 32-bit port (UM
  // 5.6.9, "the addressed port size is 32 bits").
  // ==========================================================================
  logic [1:0] port_code;     // the DSACK encoding of the port that answered
  logic [2:0] port_bytes;    // 1, 2 or 4
  logic [1:0] port_off;      // the address's offset within the port
  logic [2:0] xfer_n;        // bytes this transfer will actually move

  // The port size of a DSACK cycle is taken one clock after the edge that
  // recognised DSACK, at the falling edge that latches the data -- see the
  // falling-edge block. In the half clock before that edge (S4, after the
  // rising edge that entered it) it is the pins themselves, for early_q, the
  // only register clocked on that edge that reads it; no rising-edge register
  // ever sees this arm, because st_n is S5 again before the next rising edge.
  logic [1:0] dsack_port;
  assign dsack_port = ((st_p == rd68031_pkg::ST_S4) && (st_n != rd68031_pkg::ST_S5))
                    ? dsack_n_i : dsack_q;

  assign port_code = sterm_q ? rd68031_pkg::DSACK_32 : dsack_port;

  // The long word a transfer carries. The first is the cycle's own address; a
  // burst beat is the next long word in the line, wrapping -- "the count
  // sequence wraps back to zero and continues as necessary" (UM 7.3.7) -- which
  // the system, not this unit, puts on A3/A2.
  logic [1:0]  bt_k;
  logic [31:0] bt_addr;
  logic [2:0]  bt_n;         // bytes of the operand the transfer may carry

  always_comb begin
    if (beat_q == 3'd0) bt_k = 2'd0;
    else                bt_k = beat_q[1:0] - 2'd1;
  end

  always_comb begin
    if (bt_k == 2'd0) bt_addr = cyc_addr;
    else              bt_addr = {cyc_addr[31:4], cyc_addr[3:2] + bt_k, 2'b00};
  end

  // A burst beat carries operand bytes only if the operand's next byte is in
  // it: the rest of an operand misaligned within the line (UM 6.1.3.2, "if the
  // data for the cycle contains part of the requested operand, the execution
  // unit uses that data").
  //
  // The comparison with the beat's long word is split (M15 phase B): the line
  // and the byte offset are compared from registers, a full clock ahead, and
  // only the long word in the line waits for the beat count.
  logic bt_eqh;
  assign bt_eqh = (op_addr[31:4] == cyc_addr[31:4]) && (op_addr[1:0] == 2'b00);
  always_comb begin
    if (bt_k == 2'd0)
      bt_n = cyc_n;
    else if (op_active && (op_rem != 3'd0) && bt_eqh
             && (op_addr[3:2] == (cyc_addr[3:2] + bt_k)))
      bt_n = (op_rem > 3'd4) ? 3'd4 : op_rem;
    else
      bt_n = 3'd0;
  end

  always_comb begin
    unique case (port_code)
      rd68031_pkg::DSACK_8:  port_bytes = 3'd1;
      rd68031_pkg::DSACK_16: port_bytes = 3'd2;
      rd68031_pkg::DSACK_32: port_bytes = 3'd4;
      default:               port_bytes = 3'd4;
    endcase
  end

  always_comb begin
    unique case (port_code)
      rd68031_pkg::DSACK_8:  port_off = 2'd0;
      rd68031_pkg::DSACK_16: port_off = {1'b0, bt_addr[0]};
      rd68031_pkg::DSACK_32: port_off = bt_addr[1:0];
      default:               port_off = bt_addr[1:0];
    endcase
  end

  always_comb begin
    xfer_n = port_bytes - {1'b0, port_off};
    if (xfer_n > bt_n) xfer_n = bt_n;
  end

  // What the transfer actually moved. A bus error moves nothing -- the frame has
  // to record the residual as it was BEFORE the faulted access, because RTE
  // reruns that access -- and a retried cycle moves nothing either, because UM
  // 7.5.2 reruns it "using the same access information". A burst beat ended by
  // BERR moves nothing too: if it carried part of the operand, that part is read
  // again by a cycle of its own (UM 7.5.1, figure 7-53).
  logic [2:0] xfer_done;
  always_comb begin
    if (term_err || term_rty || term_bfe) xfer_done = 3'd0;
    else                                  xfer_done = xfer_n;
  end

  // ==========================================================================
  // The data cache -- UM 6.1.2, 6.1.3; doc/caches.md
  // ==========================================================================
  // A read the data cache holds whole is answered from it on the next edge,
  // with no bus cycle, while the bus does whatever else it is doing -- the hit
  // path below. Not a read-modify-write's (forced to miss, UM 6.1.2.2), nor
  // CPU space, nor while CACR or CDIS disable the cache: `dc_on` says all of
  // that. Not the operand RTE is handing back either, which is the frame's
  // business (doc/checkpoint.md rule 3), nor on the edge a posted write's bus
  // error is being reported.
  logic dc_full, dc_pa, dc_pb;
  assign dc_full = dc_hit_a && (dc_hit_b || !dc_span);
  assign dc_pa   = dc_hit_a && dc_span && !dc_hit_b;
  assign dc_pb   = !dc_hit_a && dc_hit_b;

  // The data cache is written on the rising edge a cachable read cycle ends,
  // and a lookup made before that edge has not seen it: a read taken to the bus
  // on that edge could miss on the very long word being written, and read it
  // again. So a read the cache is looked up for waits that one edge and is
  // looked up again.
  logic bnow_dc;
  assign bnow_dc  = beat_end && cyc_rw && op_dfill && !cyc_ciout && !cyc_abt;

  // ... and only a request the bus has not already taken: the sequencer holds
  // its request up until it is acknowledged, and a read whose first long word
  // the bus has just brought in may hit whole in the middle of its own second
  // cycle. A posted write in flight is not the sequencer's current request.
  // One set aside for a table search still is.
  logic seq_op_busy;
  assign seq_op_busy = (op_active && !op_isfetch && !op_posted && !op_istw)
                    || (pk_valid && !pk_posted);
  // Nor while a write across two pages has its second page's long word still
  // to settle in the cache (`wb_pend`, below): a read hitting it would read
  // what the write is about to change.
  assign hit_take = req_valid && dc_full && !rst_pend_q && !post_flt_end
                 && !seq_op_busy && !wb_pend;
  assign rd_block = req_valid && dc_on && !rst_pend_q && bnow_dc;
  assign req_bus  = req_valid && !hit_take && !rd_block;
  // Data beats instruction fetch, a held-back read included.
  assign fetch_go = fetch_valid && !(req_valid && !hit_take);

  // The cycles that complete a cache entry -- UM 6.1.3.1: "if the port size of
  // the responding device is smaller than 32 bits, the MC68030 executes all
  // bus cycles necessary to fill the long word". Figure 6-5 (a byte at $03 from
  // an 8-bit port: $03, then the 3-byte remainder from $00, $01, $02), 6-6 (a
  // 16-bit port: the word at $02, then $00), 6-8 and 6-9 (a long word at $06:
  // $06, $07, then $04 and $05 to fill that entry, then $08 and $09 of the
  // operand, then $0A and $0B) all say the same thing: once the operand's
  // bytes in a long word have been read, the rest of that long word is read
  // from the byte after the last one received, wrapping within it, SIZ giving
  // the bytes still missing; and only then the operand's next long word.
  //
  // Not on a cycle that ended in a bus error, or that had CIIN, or that had
  // CIOUT -- the entry is not going to be cached, so there is nothing to
  // complete (an operand's cycles in a CI page, the page they are in, which
  // may not be its first's: doc/mmu.md); not for a
  // read-modify-write, whose locked sequence is the operand's cycles and no
  // others; and never on a 32-bit port, which supplies the whole long word in
  // one cycle (UM 7.2.7).
  int unsigned i_fl;
  logic  [3:0] rcv_m;        // the bytes of the long word this cycle brought
  logic        same_ent;
  logic  [3:0] got_now;
  logic        bad_now;
  logic [31:0] nx_addr;
  logic  [2:0] nx_rem;
  logic        ent_done;
  logic        need_fill;
  logic  [1:0] fl_start;
  logic  [2:0] fl_count;
  logic        fill_end;
  logic        fl_on_n;
  logic [31:0] fl_addr_n;
  logic  [2:0] fl_rem_n;

  always_comb begin
    unique case (port_code)
      rd68031_pkg::DSACK_8:  rcv_m = 4'b0001 << bt_addr[1:0];
      rd68031_pkg::DSACK_16: rcv_m = bt_addr[1] ? 4'b1100 : 4'b0011;
      default:               rcv_m = 4'b1111;
    endcase
  end

  assign same_ent  = !op_first && (cyc_addr[31:2] == ent_q);
  assign got_now   = (same_ent ? ent_got : 4'b0000)
                   | ((term_err || term_rty || term_bfe) ? 4'b0000 : rcv_m);
  assign bad_now   = (same_ent && ent_bad) || ciin_q || term_err || term_bfe;
  // The residual address after this transfer, op_addr + xfer_done, as a
  // carry-select (M15 phase B, doc/size-and-speed.md): xfer_done is at most
  // four and arrives late, from the falling-edge beat count through the port
  // size; op_addr is a rising-edge register, with a whole clock to spare. So
  // the three low bits are added late, and their carry picks op_addr's upper
  // bits or those plus one, both ready long before. Whether it leaves the long
  // word the cycle read, and whether it enters another page, are taken the
  // same way.
  logic  [3:0] nx_lo;
  logic [28:0] oa_hi1;
  logic        nx_eq0, nx_eq1;
  assign nx_lo     = {1'b0, op_addr[2:0]} + {1'b0, xfer_done};
  assign oa_hi1    = op_addr[31:3] + 29'd1;
  assign nx_addr   = {(nx_lo[3] ? oa_hi1 : op_addr[31:3]), nx_lo[2:0]};
  assign nx_eq0    = (op_addr[31:3] == cyc_addr[31:3]);
  assign nx_eq1    = (oa_hi1 == cyc_addr[31:3]);
  assign nx_rem    = op_rem - xfer_done;
  assign ent_done  = (nx_rem == op_tail)
                  || !((nx_lo[3] ? nx_eq1 : nx_eq0) && (nx_lo[2] == cyc_addr[2]));
  assign need_fill = op_fillc && !cyc_ciout && !cyc_fill && ent_done && !bad_now
                  && !term_rty && (got_now != 4'b1111);

  // Where the completion starts, and how much of it there is: the first byte
  // missing after the run received, and the bytes missing.
  always_comb begin
    fl_start = 2'd0;
    for (i_fl = 0; i_fl < 4; i_fl = i_fl + 1)
      if (!got_now[i_fl] && got_now[(i_fl + 3) % 4]) fl_start = i_fl[1:0];
    fl_count = 3'd4 - ({2'b00, got_now[0]} + {2'b00, got_now[1]}
                       + {2'b00, got_now[2]} + {2'b00, got_now[3]});
  end

  // A completing cycle ends the completion when it brings the last byte, and
  // abandons it on a bus error or CIIN -- UM 6.1.3.1, "the occurrence of a bus
  // error while attempting to load a cache entry aborts the entry fill
  // operation but does not necessarily cause a bus error exception": none, for
  // a cycle that carries none of the operand.
  assign fill_end = term_err || ciin_q || (fl_rem == xfer_done);

  always_comb begin
    fl_on_n   = fl_on;
    fl_addr_n = fl_addr;
    fl_rem_n  = fl_rem;
    if (need_fill) begin
      fl_on_n   = 1'b1;
      fl_addr_n = {cyc_addr[31:2], fl_start};
      fl_rem_n  = fl_count;
    end else if (cyc_fill && !term_rty) begin
      fl_on_n   = !fill_end;
      fl_addr_n = {fl_addr[31:2], fl_addr[1:0] + xfer_done[1:0]};
      fl_rem_n  = fl_rem - xfer_done;
    end
  end


  // ==========================================================================
  // Read: which lane each byte arrives on -- UM Table 7-4
  //
  // The most significant byte still owed lands on the lane the address selects
  // (D31-D24 is lane 0), and the rest follow on successively lower lanes. A 16-bit
  // port offsets within its own two lanes; an 8-bit port always uses lane 0.
  // ==========================================================================
  // UM 7.3.1 state 4: "at the end of S4, the processor latches the incoming
  // data" -- the falling edge entering S5; and UM 7.3.4, at the falling edge
  // after STERM. The lanes below are that latched copy, so the merge that
  // follows on the next rising edge is not looking at the pins.
  logic [31:0] d_latched;

  logic [7:0] lane [0:3];
  assign lane[0] = d_latched[31:24];
  assign lane[1] = d_latched[23:16];
  assign lane[2] = d_latched[15:8];
  assign lane[3] = d_latched[7:0];

  // The operand as a byte array, least significant first, so that every index
  // below is a plain 3-bit number rather than a part-select whose width the tools
  // disagree about. Eight entries for a five-byte operand: the top three are never
  // read, and having them removes every out-of-range index warning.
  logic [7:0] opv [0:7];

  always_comb begin
    for (i_v = 0; i_v < 8; i_v = i_v + 1) begin
      opv[i_v] = (i_v < 5) ? op_data[i_v * 8 +: 8] : 8'h00;
    end
  end

  logic [7:0] rdv [0:7];
  logic [2:0] rd_sel;
  logic [1:0] rd_lane;
  logic [39:0] rd_merged;

  always_comb begin
    for (i_rv = 0; i_rv < 8; i_rv = i_rv + 1) begin
      rdv[i_rv] = opv[i_rv];
    end
    rd_sel  = '0;
    rd_lane = '0;
    for (i_rd = 0; i_rd < 4; i_rd = i_rd + 1) begin
      if (i_rd < xfer_done) begin
        // Byte i_rd of this transfer is the (op_rem-1-i_rd)-th byte of the
        // operand, counting up from the least significant.
        rd_sel  = op_rem - 3'd1 - i_rd[2:0];
        // ... and it arrives on the lane the address offset selects. The sum
        // never exceeds three: xfer_n is capped at port_bytes - port_off.
        rd_lane = port_off + i_rd[1:0];
        rdv[rd_sel] = lane[rd_lane];
      end
    end
    rd_merged = {rdv[4], rdv[3], rdv[2], rdv[1], rdv[0]};
  end

  // The combinational view of the beat ending on this edge, for the data cache.
  // A burst's later beats are not "the first cycle of the operand", whatever
  // op_first has gone back to once the operand finished.
  assign bnow_valid = beat_end && cyc_rw && !cyc_abt && !term_rty && !term_err
                   && !term_bfe;
  assign bnow_err   = beat_end && cyc_rw && !cyc_abt && (term_err || term_bfe);
  assign bnow_data  = d_latched;
  assign bnow_addr  = bt_addr;
  assign bnow_num   = bt_k;
  assign bnow_port  = port_code;
  assign bnow_ciin  = ciin_q;
  assign bnow_fc    = cyc_fc;
  assign bnow_dfill = op_dfill;
  assign bnow_brq   = cyc_brq;
  assign bnow_first = op_first && (bt_k == 2'd0);
  assign bnow_ci    = cyc_ciout;

  // ==========================================================================
  // Write: the internal-to-external multiplexer -- UM Table 7-5
  //
  // Transcribed rather than reduced to a formula, because two of its entries are
  // footnoted "due to the current implementation, this byte is output but never
  // used" and a formula would quietly disagree with the manual about them.
  //
  // opb[j] is the j-th byte of the group this cycle is presenting, j = 0 being the
  // most significant. The manual names them OP0..OP3 by position in the whole
  // operand, so for a size of n bytes its OPk is our opb[k - (4 - n)].
  //
  // All four lanes are always driven: "the MC68030 always drives all sections of
  // the data bus because, at the start of a write cycle, the bus controller does
  // not know the port size" (UM 7.2.4).
  // ==========================================================================
  logic [7:0] opb [0:3];
  logic [2:0] wr_sel;

  always_comb begin
    wr_sel = '0;
    for (i_wr = 0; i_wr < 4; i_wr = i_wr + 1) begin
      if (op_rem > i_wr[2:0]) begin
        wr_sel     = op_rem - 3'd1 - i_wr[2:0];
        opb[i_wr]  = opv[wr_sel];
      end else begin
        opb[i_wr]  = 8'h00;
      end
    end
  end

  // The byte just above the ones still to send -- the operand byte the previous
  // cycle of this operand sent last. op_data keeps the whole operand and only
  // op_rem counts down, so it is still there. Table 7-5 puts it on D7-D0 of a
  // three-byte transfer at A1A0 = 00, which only arises as the rest of a long
  // word begun at A1A0 = 11: the manual's OP0.
  logic [7:0] op_above;
  assign op_above = opv[op_rem];

  logic [31:0] wr_lanes;

  always_comb begin
    unique case ({cyc_siz, cyc_addr[1:0]})
      // Byte: OP3 on every lane, whatever A1 and A0 are.
      {rd68031_pkg::SIZ_BYTE,  2'b00},
      {rd68031_pkg::SIZ_BYTE,  2'b01},
      {rd68031_pkg::SIZ_BYTE,  2'b10},
      {rd68031_pkg::SIZ_BYTE,  2'b11}: wr_lanes = {opb[0], opb[0], opb[0], opb[0]};

      // Word: A1 is a don't care.
      {rd68031_pkg::SIZ_WORD,  2'b00},
      {rd68031_pkg::SIZ_WORD,  2'b10}: wr_lanes = {opb[0], opb[1], opb[0], opb[1]};
      {rd68031_pkg::SIZ_WORD,  2'b01},
      {rd68031_pkg::SIZ_WORD,  2'b11}: wr_lanes = {opb[0], opb[0], opb[1], opb[0]};

      // 3 bytes. The D7-D0 entry of the first row is OP0, the byte sent
      // before these three -- footnoted "output but never used", and Table 7-7
      // leaves the lane disabled, but it is what the part drives and so what
      // this drives.
      {rd68031_pkg::SIZ_3BYTE, 2'b00}: wr_lanes = {opb[0], opb[1], opb[2], op_above};
      {rd68031_pkg::SIZ_3BYTE, 2'b01}: wr_lanes = {opb[0], opb[0], opb[1], opb[2]};
      {rd68031_pkg::SIZ_3BYTE, 2'b10}: wr_lanes = {opb[0], opb[1], opb[0], opb[1]};
      {rd68031_pkg::SIZ_3BYTE, 2'b11}: wr_lanes = {opb[0], opb[0], opb[1], opb[0]};

      // Long word.
      {rd68031_pkg::SIZ_LONG,  2'b00}: wr_lanes = {opb[0], opb[1], opb[2], opb[3]};
      {rd68031_pkg::SIZ_LONG,  2'b01}: wr_lanes = {opb[0], opb[0], opb[1], opb[2]};
      {rd68031_pkg::SIZ_LONG,  2'b10}: wr_lanes = {opb[0], opb[1], opb[0], opb[1]};
      {rd68031_pkg::SIZ_LONG,  2'b11}: wr_lanes = {opb[0], opb[0], opb[1], opb[0]};

      default:                         wr_lanes = {opb[0], opb[1], opb[2], opb[3]};
    endcase
  end

  // ==========================================================================
  // The strobes, declared here because the rising-edge block below uses as_win
  // and Questa and Vivado both reject a variable read above its own declaration
  // -- (vlog-2730) and [Synth 8-6901], which scripts/synth.tcl promotes from an
  // info to an error for exactly this reason. The two lint front-ends invent an
  // implicit net instead and say nothing.
  //
  // EVERY STROBE IS A FLIP-FLOP, NOT A DECODE OF THE STATE. A strobe decoded
  // from a multi-bit state register glitches whenever two of its bits change on
  // the same edge and the decode is true of a code in between: the burst states
  // made that visible -- AS went high for zero time between BL and BW in
  // simulation (doc/bugs-found.md) -- and the asynchronous cycle had the same
  // hazard on ECS, OCS, DBEN and the data enable at S2 -> S4. So each pin is
  // the registered value of the same decode applied to the NEXT state, which
  // is the same waveform from a flop. A pin that moves on one edge is a flop on
  // that edge; DBEN, which moves on both, is rd68031_dedge_ff.
  // ==========================================================================

  // The states AS is asserted in: from the falling edge entering S1 to the
  // falling edge entering S5 -- the asynchronous S5 and the synchronous S3
  // alike. A burst holds it: "it maintains AS, DS, R/W, A0-A31, FC0-FC2,
  // SIZ0-SIZ1 in their current state throughout the burst operation" (UM 7.3.7).
  function automatic logic as_set(input rd68031_pkg::bus_state_e st);
    as_set = (st == rd68031_pkg::ST_S1) || (st == rd68031_pkg::ST_S3)
          || (st == rd68031_pkg::ST_WL) || (st == rd68031_pkg::ST_BL)
          || (st == rd68031_pkg::ST_BW);
  endfunction

  // DBEN, active high here, as a function of the two states -- UM 7.1.6:
  // "during a read operation, DBEN is asserted one clock cycle after the
  // beginning of the bus cycle and is negated as DS is negated. In a write
  // operation, DBEN is asserted at the time AS is asserted and is held active
  // for the duration of the cycle" -- through S5 (UM 7.3.2 state 5), and
  // through a synchronous write's S3 (UM 7.3.5 state 3, "R/W, SIZ0-SIZ1,
  // FC0-FC2, and DBEN also remain valid throughout S3"). A burst keeps it
  // asserted with AS and DS (UM 7.3.7 state 3).
  function automatic logic dben_of(input rd68031_pkg::bus_state_e p,
                                   input rd68031_pkg::bus_state_e n,
                                   input logic rw);
    if (rw)
      dben_of = ((p == rd68031_pkg::ST_S2) || (p == rd68031_pkg::ST_S4)
                 || (p == rd68031_pkg::ST_WH) || (p == rd68031_pkg::ST_BH))
                && (n != rd68031_pkg::ST_S5);
    else
      dben_of = as_set(n)
                || ((n == rd68031_pkg::ST_S5)
                    && ((p == rd68031_pkg::ST_S4) || (p == rd68031_pkg::ST_S2)
                        || (p == rd68031_pkg::ST_WH)));
  endfunction

  logic as_win;     // AS asserted: a falling-edge register
  logic ds_q;       // DS asserted: likewise
  logic cbreq_q;    // CBREQ asserted: likewise
  logic ecs_q;      // the rising edge entering S0 has passed
  logic ocs_q;      // ... and it began an operand
  logic doe_q;      // write data driven: a rising-edge register
  logic [2:0] beat_nxt;

  // The long words latched so far, as the coming falling edge leaves them: none
  // at the start of a cycle, one more on each edge that latches one. A
  // continuous assignment: written as an always_comb, this crashes iverilog 12
  // (ivl_nexus_ptrs: Assertion `net' failed) -- doc/coding-standard.md.
  assign beat_nxt = (st_n_nxt == rd68031_pkg::ST_S1) ? 3'd0
                  : ((st_n_nxt == rd68031_pkg::ST_S5) || (st_n_nxt == rd68031_pkg::ST_BL))
                    ? (beat_q + 3'd1) : beat_q;

  // ==========================================================================
  // Starting a cycle, and starting an operand
  // ==========================================================================
  logic        take_rst;
  logic        take_req;
  logic        take_fetch;
  logic [31:0] next_addr;
  logic  [2:0] next_rem;
  logic  [2:0] next_fc;
  logic        next_rw;
  logic        next_rmc;
  logic        next_burst;
  logic        next_ciout;
  logic        next_spent;
  logic        next_fill;    // the cycle about to start completes a cache entry

  // A new operand may be taken when none is active, or at the very edge the
  // active one completes.
  // UM 8.2.1: RTE "reruns the faulted data access". It is the SAME operand, not
  // a new one -- doc/checkpoint.md rule 3, the unit of restart is the operand --
  // so it comes in with the residual the frame carried and the bytes already
  // gathered, and outranks anything the sequencer or the pipe is asking for.
  // A restored operand outlives the microword that handed it over. The
  // sequencer gives it to the bus unit and jumps to the microword that faulted
  // in the same clock, and the bus is not necessarily idle on that clock -- so
  // this is a latched request and not a level. Without the latch the handover
  // was simply missed, the resumed microword issued its own full-length request
  // instead, and RTE rewrote the bytes that had already gone.
  //
  // ... and it is handed to the resumed microword's OWN request, not to
  // whichever request comes first. RTE resumes at the microword that faulted;
  // when that one has a bus request, its request is the first presented, and it
  // takes the operand. When it has none -- an instruction prefetch fault taken
  // while the pipe was empty, at a boundary -- the operand is for nobody, and
  // the sequencer cancels it when that microword retires. Taking it without a
  // request gave its acknowledge, and its data, to the next instruction's first
  // bus microword: an RTS at the resumed address jumped to the frame's data
  // input buffer without reading its stack (RD68021 doc/bugs-found.md).
  //
  // A prefetch is not held up behind it either: the resumed microword may be
  // waiting for exactly that word.
  //
  // A posted write handed back is run at once, for nobody: the microword RTE
  // resumed is not the one that wrote it -- doc/checkpoint.md rule 9.
  //
  // And nothing new is taken on the edge a posted write ends in a bus error.
  // The sequencer is about to take the fault at whatever microword it is on,
  // and a request that microword had presented must not run.
  assign rst_self     = rst_pend_q && rst_post_q && (rst_bytes != 3'd0);
  assign post_flt_end = op_finishing && op_posted && term_err;
  // The table walker's operands outrank everything, and while a search is
  // under way nothing else is taken.
  // An operand set aside for its next page's search comes back as soon as
  // the search is over, before anything new is taken.
  //
  // Which requester is next is decided from registers and from the
  // requesters' own signals (`sel_*`); whether it is taken on this edge also
  // needs the operand under way to be finishing on it (`take_ok`), which is
  // known only late in the half clock before the edge. The ATC is looked up
  // for the requester `sel_*` names, so that its address never waits for the
  // operand engine (M15 phase B, below).
  logic take_tw, take_pk, take_ok;
  logic sel_tw, sel_pk, sel_rst, sel_req, sel_fetch;
  assign sel_tw     = tw_valid;
  assign sel_pk     = pk_valid && !mmu_busy;
  assign sel_rst    = rst_pend_q && !mmu_busy && !pk_valid
                   && (req_valid || rst_post_q) && (rst_bytes != 3'd0);
  assign sel_req    = !rst_pend_q && !mmu_busy && !pk_valid && req_bus;
  assign sel_fetch  = fetch_go && !mmu_busy && !pk_valid && !rst_self;
  assign take_ok    = !op_continuing && !post_flt_end;
  // The same choice for the lookup, made without the sequencer's req_last:
  // req_valid drops on the edge the sequencer's own operand finishes, and
  // reading it made the lookup's address wait for the operand engine after
  // all. While that operand is under way (`seq_own`) the request presented
  // is that operand, never a new one, so it is not a candidate; and with
  // none under way req_last is low and req_want is req_valid. So `lk_*`
  // names whatever `sel_*` names whenever something is taken.
  logic seq_own, req_lk, hit_lk, req_bus_lk;
  logic lk_tw, lk_pk, lk_rst, lk_req;
  // The walker's in the same way: tw_valid drops with tw_last.
  assign lk_tw       = tw_want && !(op_active && op_istw);
  assign seq_own     = op_active && !op_isfetch && !op_posted && !op_istw;
  assign req_lk      = req_want && !seq_own;
  // (Nothing is taken on the edge a posted write faults, so post_flt_end,
  // which is late too, has no say in the lookup.)
  assign hit_lk      = req_lk && dc_full && !rst_pend_q && !seq_op_busy && !wb_pend;
  // A read held back while the data cache is written (rd_block) holds back
  // a prefetch too, so on that edge nothing the lookup would choose after
  // the walker, the operand set aside or RTE's is taken: rd_block, which
  // reads the falling-edge state, need not choose.
  assign req_bus_lk  = req_lk && !hit_lk;
  assign lk_pk       = sel_pk;
  assign lk_rst      = rst_pend_q && !mmu_busy && !pk_valid
                    && (req_lk || rst_post_q) && (rst_bytes != 3'd0);
  assign lk_req      = !rst_pend_q && !mmu_busy && !pk_valid && req_bus_lk;
  assign take_tw    = take_ok && sel_tw;
  assign take_pk    = take_ok && sel_pk;
  assign take_rst   = take_ok && sel_rst;
  assign req_start  = take_req && (st_p_nxt == rd68031_pkg::ST_S0);
  assign dc_wr_take = take_req && (st_p_nxt == rd68031_pkg::ST_S0)
                   && (req_kind == rd68031_pkg::CT_WRITE);
  assign take_req   = take_ok && sel_req;
  assign take_fetch = take_ok && sel_fetch;

  // UM 8.2.1: with DF cleared "it assumes that the data input buffer value on
  // the stack is valid for a read or that the data has been correctly written
  // to memory for a write". The sequencer says so by handing back an operand
  // with nothing left of it, and the answer is the buffer out of the frame.
  //
  // It still has to be handed back rather than skipped: the microword that
  // faulted is re-executed -- doc/checkpoint.md rule 2 -- and everything else
  // it does has to happen exactly once. Satisfying its request from the frame
  // is what lets it run again without running the access again.
  logic rst_done;
  assign rst_done = rst_pend_q && !rst_post_q && !op_continuing && req_valid
                 && (rst_bytes == 3'd0);

  // A posted write is outstanding -- in flight, or set aside for its next
  // page's search -- or RTE has one to rerun.
  assign post_busy = (op_active && op_posted) || (pk_valid && pk_posted) || rst_self;

  // OCS for a cycle starting at this edge: the first of a new operand, or the
  // next of one none of whose cycles has finished yet (op_first, as this edge
  // leaves it). Not the cycle an operand set aside resumes with: its first
  // page's cycles have run, and OCS is "the first external bus cycle of an
  // operand transfer" only (UM 7.1.1).
  logic first_now;
  assign first_now = take_tw || take_rst || take_req || take_fetch
                  || (!take_pk && op_first && !(beat_end && op_active));

  // CPU space synthesises its address from the type field -- UM figure 7-42.
  logic [31:0] cpu_space_addr;
  always_comb begin
    unique case (req_cpuspace)
      rd68031_pkg::CPUS_IACK:
        // UM 7.4.1.1: A1-A3 the level, A20-A31, A4-A15 and A0 set to one.
        cpu_space_addr = {28'hFFF_FFFF, req_cpuaddr[2:0], 1'b1};
      rd68031_pkg::CPUS_BKPT:
        // The breakpoint number on A4-A2 -- UM 7.4.2.
        cpu_space_addr = {12'h000, rd68031_pkg::CPUS_BKPT, 11'h000,
                          req_cpuaddr[2:0], 2'b00};
      rd68031_pkg::CPUS_COPROC:
        // UM 7.4.3: the CpID on A13-A15 and the interface register on A0-A4,
        // and figure 7-42 shows every other address line zero.
        cpu_space_addr = {12'h000, rd68031_pkg::CPUS_COPROC, req_cpuaddr[7:5],
                          8'h00, req_cpuaddr[4:0]};
      default:
        cpu_space_addr = {12'h000, req_cpuspace, req_cpuaddr, 8'h00};
    endcase
  end

  // What the next cycle carries, from three sources:
  //   - a new operand (`cn_*`): the requester `sel_*` names, taken if the
  //     operand under way is finishing or there is none;
  //   - the operand under way, continuing on the edge that ends S5 (`s5_*`):
  //     its residual after the transfer now ending, or the cycle completing
  //     a cache entry if one is due;
  //   - the operand under way from its registers (`rg_*`): after an idle
  //     clock, a retry or a halt.
  logic [31:0] cn_addr;
  logic  [2:0] cn_rem;
  logic  [2:0] cn_fc;
  logic        cn_rw, cn_rmc, cn_burst;
  always_comb begin
    if (lk_tw) begin
      // A descriptor, read or written back: UM 7.3.3, "table search accesses
      // ... are always read-modify-write cycles to the supervisor data space".
      cn_addr  = tw_addr;
      cn_rem   = 3'd4;
      cn_fc    = rd68031_pkg::FC_SUPER_DATA;
      cn_rw    = !tw_wr;
      cn_rmc   = 1'b1;
      cn_burst = 1'b0;
    end else if (lk_pk) begin
      // The operand set aside, at the first byte of its next page.
      cn_addr  = pk_addr;
      cn_rem   = pk_rem;
      cn_fc    = pk_fc;
      cn_rw    = pk_rw;
      cn_rmc   = pk_rmc;
      cn_burst = pk_burst;
    end else if (lk_rst) begin
      cn_addr  = rst_addr;
      cn_rem   = rst_bytes;
      cn_fc    = rst_fc;
      cn_rw    = rst_rw;
      cn_rmc   = rst_rmc;
      cn_burst = 1'b0;
    end else if (lk_req) begin
      // An operand whose first long word hit in the data cache starts its bus
      // cycles at its second -- UM 6.1.2, "the hit and miss are treated
      // independently" -- and that cycle is the first of the operand, with OCS
      // (UM 7.2.7: "in the case of a misaligned data transfer where the first
      // portion of the operand results in a cache hit ... and the second
      // portion in a cache miss, OCS is asserted for the second portion").
      if ((req_fc == rd68031_pkg::FC_CPU) && !req_cpuraw)
                                         cn_addr = cpu_space_addr;
      else if (dc_pa)                    cn_addr = {req_addr[31:2] + 30'd1, 2'b00};
      else                               cn_addr = req_addr;
      cn_rem   = dc_pa ? dc_nb : req_bytes;
      cn_fc    = req_fc;
      cn_rw    = (req_kind != rd68031_pkg::CT_WRITE);
      cn_rmc   = req_rmc;
      cn_burst = req_burst;
    end else begin
      // A prefetch, or nothing: the fetch unit's address is as good as any.
      cn_addr  = {fetch_addr[31:2], 2'b00};  // always long-word aligned
      cn_rem   = 3'd4;
      cn_fc    = fetch_fc;
      cn_rw    = 1'b1;
      cn_rmc   = 1'b0;
      cn_burst = fetch_burst;
    end
  end

  // Re-entering S0 other than at the end of S5 -- after an idle clock (the one
  // a cycle entering a new page waits, below), a retry (UM 7.5.2, "retries the
  // previous cycle using the same access information") or a halt. The residual
  // is already whatever the last cycle left it as, and it must NOT be advanced
  // again: xfer_done is only meaningful while the cycle that produced it is
  // still current, and by now term_rty has been cleared, so adding it here
  // would retry at the wrong address with a residual of zero.
  //
  // A cycle completing a cache entry is retried as itself, and after a halt
  // the completion that was due is still due.
  logic [31:0] rg_addr;
  logic  [2:0] rg_rem;
  assign rg_addr = fl_on ? fl_addr : op_addr;
  assign rg_rem  = fl_on ? fl_rem  : op_rem;

  logic take_any;
  assign take_any = take_tw || take_pk || take_rst || take_req || take_fetch;

  always_comb begin
    next_fill = 1'b0;
    if (take_any) begin
      next_addr  = cn_addr;
      next_rem   = cn_rem;
      next_fc    = cn_fc;
      next_rw    = cn_rw;
      next_rmc   = cn_rmc;
      next_burst = cn_burst;
      next_spent = take_pk && pk_brq_used;
    end else if (st_n == rd68031_pkg::ST_S5) begin
      // Continuing a multi-cycle operand on the edge that ends S5. op_addr and
      // op_rem are updated by this same edge, so what the next cycle must carry
      // is the residual *after* this transfer, not before it -- and so is
      // whether this operand has had its CBREQ. A cycle completing a cache
      // entry comes first if one is due, and the operand resumes after it.
      if (fl_on_n) begin
        next_addr = fl_addr_n;
        next_rem  = fl_rem_n;
        next_fill = 1'b1;
      end else if (cyc_fill) begin
        next_addr = op_addr;
        next_rem  = op_rem;
      end else begin
        next_addr = nx_addr;
        next_rem  = nx_rem;
      end
      next_fc    = op_fc;
      next_rw    = op_rw;
      next_rmc   = op_rmc;
      next_burst = op_burst;
      next_spent = op_brq_used || (cyc_brq && !term_rty);
    end else begin
      next_addr  = rg_addr;
      next_rem   = rg_rem;
      next_fill  = fl_on;
      next_fc    = op_fc;
      next_rw    = op_rw;
      next_rmc   = op_rmc;
      next_burst = op_burst;
      next_spent = op_brq_used;
    end
  end

  // ==========================================================================
  // Translation -- UM 9.2.1, figure 9-8; doc/mmu.md
  //
  // A cycle is looked up when it is the first of a new operand -- the
  // sequencer's, the fetch unit's, the one RTE hands back, or one coming back
  // from being set aside -- and when it is the first of a continuing operand
  // to enter a page other than the one its cycles have been in so far. An
  // operand is at most five bytes and a page at least 256, so that is at most
  // once more. Bytes the data cache holds never reach a cycle, and are never
  // looked up. Then:
  //   - the page needs a table search (a miss, or a write to a page whose M
  //     bit is clear): the cycle does not start, and the search runs. For a
  //     continuing operand that is between its first page's cycles and its
  //     second's -- what has gone has gone -- and the operand is set aside
  //     while the walker has the engine (`park`, below);
  //   - the page is refused: the cycle starts as ST_ABT and ends there with a
  //     bus error. For a continuing operand its first page's bytes have gone,
  //     and the frame carries the residual -- its size and its logical
  //     address -- exactly as for a BERR on that cycle (doc/checkpoint.md);
  //   - otherwise it starts, and the page's translation is kept for the
  //     operand's other cycles in it.
  //
  // This is the order of UM 7.1.1 -- "if a miss occurs in the address
  // translation cache (ATC) of the memory management unit (MMU), or if the MMU
  // finds a fault with the access, the processor aborts the cycle" -- applied
  // to each cycle of an operand as it comes. The manual does not say more; two
  // independent implementations do the same for a read (doc/m11-mismatches.md,
  // Q-A; for a write they differ, c12).
  //
  // The walker's own operands are physical.
  // ==========================================================================
  //
  // THE ATC IS NEVER ASKED ABOUT A CYCLE ON THE EDGE ITS ADDRESS IS COMPUTED
  // (M15 phase B, doc/size-and-speed.md). The address a continuing operand's
  // next cycle has at the end of S5 depends on the transfer now ending -- the
  // falling-edge beat count, the port size, the residual -- and looking it up
  // in the 22-way ATC and deciding on the answer in the same half clock was
  // the design's limiting path. So the lookup is made only for addresses
  // that are registers or a requester's: a new operand (`cn_*`), or the
  // operand under way from its own registers (`rg_*`). An operand's later
  // cycle in the page it is already in needs no lookup -- it runs on the
  // translation kept for that page -- and one that enters another page waits
  // an idle clock at the end of S5 (`s5_cross`), and is looked up from its
  // registers then. That is a clock on a misaligned operand across a page
  // boundary, with translation enabled -- doc/timing-divergences.md. With
  // none possible (TC.E clear or MMUDIS, and neither TTx enabled: figure
  // 9-8's "PA <- LA") every page is its own logical address and needs no
  // lookup at all, so nothing waits (`s5_ident`).
  logic        cand_new, cont_new, xl_new, park_now;
  logic        lk_cont;
  logic [31:0] next_paddr;
  assign cand_new = take_rst || take_req || take_fetch || take_pk;
  // The lookup is the operand under way's, from its registers, between its
  // cycles -- the clocks from which a cycle of it can start other than the
  // end of S5: idle, halted, waiting for a retry. Otherwise it is the next
  // requester's: the edge that ends S5 may end the operand, and in the
  // cycle's other states nothing starts. Chosen by the rising-edge state
  // alone, so that the lookup has the whole clock (on the edge that ends S5
  // the rising-edge state is the one that led into S5).
  assign lk_cont  = op_active && ((st_p == rd68031_pkg::ST_IDLE)
                                  || (st_p == rd68031_pkg::ST_HALT)
                                  || (st_p == rd68031_pkg::ST_RETRY));
  assign l1_addr  = lk_cont ? rg_addr : cn_addr;
  assign l_fc     = lk_cont ? op_fc   : cn_fc;
  assign l_rw     = lk_cont ? op_rw   : cn_rw;
  assign l_rmc    = lk_cont ? op_rmc  : cn_rmc;
  // The next cycle of the operand under way is in a page it has not been
  // translated for -- from its registers ...
  assign cont_new = lk_cont && (((rg_addr & ~op_pmask) != op_lp1));
  assign xl_new   = cand_new || cont_new;
  assign xl_wait  = xl_new && (l1_kind == 2'b10);
  assign cyc_abt_now = xl_new && (l1_kind == 2'b01);
  // A continuing operand whose next page needs a search, with the bus idle
  // between its cycles: set it aside, so that the walker can have the engine.
  assign park_now = cont_new && (l1_kind == 2'b10) && bus_is_idle;

  // ... and at the end of S5. Each candidate's page is compared with the
  // operand's from registers, and only the choice between them waits for the
  // transfer: the completion of a cache entry stays in the long word just
  // read; the operand's own next byte is op_addr, or op_addr plus what has
  // just moved -- whose page is op_addr's unless the carry out of its three
  // low bits takes it into the next one (a page is at least 256 bytes).
  logic mmudis_q, op_mid, mmudis_hold;
  logic in_p2_cyc, in_p2_h0, in_p2_h1, s5_in_p2, s5_ident, s5_xid;
  assign in_p2_cyc = ((cyc_addr & ~op_pmask) != op_lp1);
  assign in_p2_h0  = (({op_addr[31:3], 3'b000} & ~op_pmask) != op_lp1);
  assign in_p2_h1  = (({oa_hi1, 3'b000} & ~op_pmask) != op_lp1);
  assign s5_in_p2  = (st_n == rd68031_pkg::ST_S5) && op_continuing
                  && (fl_on_n ? in_p2_cyc
                              : ((cyc_fill || !nx_lo[3]) ? in_p2_h0 : in_p2_h1));
  // No translation is possible for the operand: the MMUDIS it is held to,
  // and TC.E and TTx, which only an instruction changes and none runs while a
  // data operand is under way.
  assign s5_ident  = (!l_tt_on && (!l_tc_on || !mmudis_q))
                  || (op_fc == rd68031_pkg::FC_CPU);
  assign s5_cross  = s5_in_p2 && !s5_ident;
  assign s5_xid    = s5_in_p2 &&  s5_ident;

  always_comb begin
    if (take_tw) begin
      next_paddr = tw_addr;
      next_ciout = 1'b0;
    end else if (xl_new) begin
      next_paddr = (l1_pb & ~l_mask) | (next_addr & l_mask);
      next_ciout = l1_ci;
    end else if (s5_xid) begin
      next_paddr = next_addr;
      next_ciout = 1'b0;
    end else begin
      next_paddr = (op_pb1 & ~op_pmask) | (next_addr & op_pmask);
      next_ciout = op_ci1;
    end
  end

  // The search, for the page that needs it, in the operand's space: a write for
  // the history bits if it is one, or any cycle of a read-modify-write -- UM
  // 9.5.1.1, "an access is considered to be a write for updating purposes if
  // either the R/W or RMC signal is low".
  assign wq_valid   = xl_wait;
  assign wq_addr    = l1_addr;
  assign wq_fc      = l_fc;
  assign wq_wr      = !l_rw || l_rmc;
  // The MMU's replacement history: the entries this operand used.
  assign l_take     = xl_new && ((st_p_nxt == rd68031_pkg::ST_S0)
                                 || (st_p_nxt == rd68031_pkg::ST_ABT));

  // MMUDIS as the MMU sees it: held from the edge an operand is taken to the
  // edge it finishes, including while it is set aside -- UM 9.2.3, "the
  // disabling of address translation does not become effective until the
  // entire transfer is complete". Only an operand can cross into a second
  // page, so a prefetch and the walker's operands do not hold it.
  assign op_mid        = pk_valid
                      || (op_active && !op_finishing && !op_istw && !op_isfetch);
  assign mmudis_hold   = op_mid ? mmudis_q : mmudis_raw_n;
  // ... and as the lookup sees it, chosen from registers (`lk_cont`, above):
  // the operand's held value when the lookup is the operand's own, or the
  // operand set aside's; otherwise the lookup is for a new operand, which sees
  // the pin. Where this differs from op_mid's choice -- the clocks of a cycle
  // under way -- no lookup is acted on: nothing is taken, and the operand's
  // own next page is looked up only between its cycles. (op_mid reads
  // op_finishing, late in the half clock, and the ATC's verdict waited for
  // it -- M15 phase B.)
  assign mmudis_sync_n = (pk_valid || (lk_cont && !op_istw && !op_isfetch))
                       ? mmudis_q : mmudis_raw_n;
  // UM 6.1: "whenever the appropriate CI bit is set for either a read or a
  // write access and an external bus cycle is required, CIOUT is asserted and
  // the instruction and data caches are ignored for the access". It is the
  // page's, and an operand across two pages has two: each cycle is cached, or
  // not, as its own CIOUT says -- a read's beats by `bnow_ci`, the cycles that
  // complete a long word by `need_fill` -- and a write's long word in each
  // page as that page says. For the first page that is known as the write is
  // taken; for the second, when its first cycle starts (doc/mmu.md,
  // doc/bugs-found.md M11 phase B).
  assign dc_inhibit = l1_ci;

  logic [2:0] this_n;
  always_comb begin
    this_n = (next_rem > 3'd4) ? 3'd4 : next_rem;
  end

  logic [1:0] this_siz;
  always_comb begin
    unique case (this_n)
      3'd1:    this_siz = rd68031_pkg::SIZ_BYTE;
      3'd2:    this_siz = rd68031_pkg::SIZ_WORD;
      3'd3:    this_siz = rd68031_pkg::SIZ_3BYTE;
      default: this_siz = rd68031_pkg::SIZ_LONG;
    endcase
  end

  // Whether the cycle about to start asserts CBREQ -- UM 6.1.3.2's list of
  // the cases in which "the processor does not assert CBREQ", the ones that are
  // the bus unit's to know:
  //   - "the current operation is the read portion of a read-modify-write
  //     operation" (and UM 7.3.3: "no burst filling of the data cache occurs
  //     during a read-modify-write operation");
  //   - "the MMU has inhibited caching for the current page" -- CIOUT;
  //   - "the cycle is for the first access of an operand that spans two cache
  //     lines (crosses a modulo 16 boundary)": the rest of the access, from
  //     this cycle's address on, must lie in the same 16-byte line;
  // and it is a read, of course: a burst fills a cache. The others -- the cache
  // enabled, its burst enable, its freeze bit, a tag miss -- are the
  // requester's, and are what req_burst and fetch_burst say.
  // The line test is on the four low bits: the last byte, next_addr +
  // next_rem - 1, is in next_addr's line exactly when the offset in the line
  // plus the length is at most sixteen (M15 phase B: a 32-bit add and compare
  // on the continuation's path before).
  logic [4:0]  next_lend;
  logic        next_brq;
  assign next_lend = {1'b0, next_addr[3:0]} + {2'b00, next_rem};
  assign next_brq  = next_burst && !next_spent && next_rw && !next_rmc
                  && !next_ciout && (next_fc != rd68031_pkg::FC_CPU)
                  && !next_fill
                  && ((next_rem == 3'd0) ? (next_addr[3:0] != 4'd0)
                                         : (next_lend <= 5'd16));

  // The write being taken crosses into a second page (doc/mmu.md), and the
  // data cache's handling of its long word there waits for that page.
  // The new operand's (dc_xpage goes with dc_wr_take, which is take_req's).
  logic [31:0] cn_last;
  assign cn_last    = cn_addr + {29'd0, cn_rem} - 32'd1;
  assign dc_xpage   = (((cn_addr ^ cn_last) & ~l_mask) != 32'd0);

  // A write taken across two pages, its second page's long word not yet
  // settled in the data cache. It settles when the operand's first cycle in
  // that page starts or is refused -- a continuing operand's (`cont_new`) or
  // one coming back from being set aside for the page's search (`take_pk`) --
  // or, if the operand ends before it gets there, as it ends.
  logic wb_at, wb_end;
  assign wb_at      = wb_pend && (cont_new || take_pk || s5_xid)
                   && ((st_p_nxt == rd68031_pkg::ST_S0) || (st_p_nxt == rd68031_pkg::ST_ABT));
  // The table walker's operands run while the write is set aside, and their
  // ends are not its end.
  assign wb_end     = wb_pend && !wb_at && beat_end && op_active && op_finishing
                   && !op_istw;
  assign dc_wb_go   = wb_at && (st_p_nxt == rd68031_pkg::ST_S0) && (s5_xid || !l1_ci);
  assign dc_wb_drop = (wb_at && (st_p_nxt == rd68031_pkg::ST_ABT)) || wb_end;

  // A faulted operand finishes too: the requester has to be unstalled either way,
  // and req_fault is what tells it which happened.
  //
  // An operand may finish part-way through a burst, at the rising edge that ends
  // a BL, as soon as its last byte has arrived. The burst carries on without it.
  //
  // With the data cache: an operand whose last bytes hit stops `op_tail` short
  // of the end (UM 6.1.2), and one whose long word is being completed from a
  // narrow port finishes with the last completing cycle rather than with its
  // own last byte (UM 6.1.3.1). A bus error on a completing cycle finishes
  // nothing early and faults nothing: it only abandons the completion.
  assign op_finishing  = op_active
                         && (((st_n == rd68031_pkg::ST_S5)
                              && !term_rty
                              && (cyc_fill
                                  ? (fill_end && (op_rem == op_tail))
                                  : (term_err || ((nx_rem == op_tail) && !need_fill))))
                             || ((st_n == rd68031_pkg::ST_BL)
                                 && (xfer_done != 3'd0) && (nx_rem == op_tail)));
  assign op_continuing = op_active && !op_finishing;

  // Combinational, and true throughout S5: the operand completes at the rising
  // edge that ends S5, which gives the sequencer half a clock to present the next
  // request. That half clock is the design's tightest path, by construction.
  assign req_last   = op_finishing && !op_isfetch && !op_posted && !op_istw;
  assign fetch_last = op_finishing &&  op_isfetch;
  assign tw_last    = op_finishing &&  op_istw;

  // CIIN, for the operand: seen on a read transfer that carried some of its bytes.
  logic ciin_now;
  assign ciin_now = ciin_q && op_rw && (xfer_done != 3'd0) && !cyc_fill;

  // ==========================================================================
  // Rising-edge domain
  // ==========================================================================
  // The arbiter, on rst_n alone. UM 7.7: "bus arbitration requests are
  // recognized during normal processing, RESET assertion, HALT assertion, and
  // even when the processor has halted due to a double bus fault" -- so the
  // RESET pin, which resets everything else in this unit, does not reset it.
  // rst_n, which is not an MC68030 pin, does: doc/divergences.md records the
  // user's decision that no arbitration happens under it.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      arb   <= rd68031_pkg::ARB_IDLE;
      hiz_q <= 1'b0;
    end else begin
      arb <= arb_nxt;
      // "If T is true, the address, data, and control buses are placed in the
      // high-impedance state after the next rising edge following the negation
      // of AS and RMC" -- UM 7.7.4. Registered on the way up, so the release
      // waits for that edge; combinational on the way down, because "the bus
      // control signals (controlled by T) are driven by the processor,
      // immediately following a state change, when bus mastership is
      // returned".
      //
      // RMC holds the bus only while it locks it. Single-wire arbitration is
      // the other exception: it "applies to all bus cycles of a
      // read-modify-write sequence" -- "an alternate master forces the MC68030
      // to release the bus by asserting BGACK and waits for AS to negate
      // before taking the bus" (UM 7.7.4) -- so with A asserted the release
      // does not wait for RMC.
      hiz_q <= arb_t_of(arb_nxt) && !as_win && (!rmc_lock || arb_a);
    end
  end

  // The bus engine, which the RESET pin resets: "resetting the processor
  // causes any bus cycle in progress to terminate as if DSACKx, BERR, or STERM
  // had been asserted" (UM 7.8).
  always_ff @(posedge clk or negedge eng_rst_n) begin
    if (!eng_rst_n) begin
      st_p       <= rd68031_pkg::ST_IDLE;
      op_active  <= 1'b0;
      op_addr    <= '0;
      op_rem     <= '0;
      op_fc      <= '0;
      op_rw      <= 1'b1;
      op_rmc     <= 1'b0;
      op_first   <= 1'b1;
      op_isfetch <= 1'b0;
      op_istw    <= 1'b0;
      op_pmask   <= '1;
      op_lp1     <= '0;
      op_pb1     <= '0;
      op_ci1     <= 1'b0;
      wb_pend    <= 1'b0;
      pk_valid   <= 1'b0;
      pk_addr    <= '0;
      pk_rem     <= 3'd0;
      pk_fc      <= 3'd0;
      pk_rw      <= 1'b1;
      pk_rmc     <= 1'b0;
      pk_data    <= '0;
      pk_posted  <= 1'b0;
      pk_cpflt   <= 1'b0;
      pk_burst   <= 1'b0;
      pk_brq_used <= 1'b0;
      pk_ciin    <= 1'b0;
      pk_dfill   <= 1'b0;
      pk_fillc   <= 1'b0;
      pk_tail    <= 3'd0;
      mmudis_q   <= 1'b1;
      op_cpflt   <= 1'b0;
      op_burst   <= 1'b0;
      op_brq_used <= 1'b0;
      op_ciin    <= 1'b0;
      op_dfill   <= 1'b0;
      op_fillc   <= 1'b0;
      op_tail    <= 3'd0;
      fl_on      <= 1'b0;
      fl_addr    <= '0;
      fl_rem     <= 3'd0;
      ent_q      <= '0;
      ent_got    <= 4'b0000;
      ent_bad    <= 1'b0;
      cyc_fill   <= 1'b0;
      burst_tail <= 1'b0;
      op_data    <= '0;
      cyc_addr   <= '0;
      cyc_paddr  <= '0;
      cyc_abt    <= 1'b0;
      cyc_fc     <= '0;
      cyc_siz    <= rd68031_pkg::SIZ_LONG;
      cyc_n      <= '0;
      cyc_rw     <= 1'b1;
      cyc_brq    <= 1'b0;
      cyc_ciout  <= 1'b0;
      sterm_q    <= 1'b0;
      cback_q    <= 1'b0;
      ciin_q     <= 1'b0;
      ecs_q      <= 1'b0;
      ocs_q      <= 1'b0;
      doe_q      <= 1'b0;
      req_ack    <= 1'b0;
      req_end_q  <= rd68031_pkg::CE_NONE;
      req_ciin   <= 1'b0;
      rst_pend_q <= 1'b0;
      rst_post_q <= 1'b0;
      op_posted  <= 1'b0;
      req_taken  <= 1'b0;
      req_fault_post <= 1'b0;
      fetch_ack  <= 1'b0;
      fetch_ciin <= 1'b0;
      fetch_ci   <= 1'b0;
      tw_ack     <= 1'b0;
      tw_berr    <= 1'b0;
      tw_rdata   <= '0;
      rdata_q    <= '0;
      frdata_q   <= '0;
      req_fault    <= 1'b0;
      req_fault_wr <= 1'b0;
      fetch_fault  <= 1'b0;
      beat_valid <= 1'b0;
      beat_err   <= 1'b0;
      beat_data  <= '0;
      beat_addr  <= '0;
      beat_num   <= '0;
      beat_port  <= rd68031_pkg::DSACK_32;
      beat_ciin  <= 1'b0;
      beat_fetch <= 1'b0;
      rmc_hold   <= 1'b0;
      rmc_first_q <= 1'b0;
      halt_hold  <= 1'b0;
    end else begin
      st_p      <= st_p_nxt;
      req_ack   <= 1'b0;
      req_taken <= 1'b0;
      mmudis_q  <= mmudis_hold;

      // A burst is under way past its first long word while the falling-edge
      // state says another is coming: BL, a long word latched with the burst
      // going on, or BW, a wait before the next. The edge that ends the last
      // long word's S5 clears it.
      burst_tail <= (st_n == rd68031_pkg::ST_BL) || (st_n == rd68031_pkg::ST_BW);

      if (rst_op_valid) begin
        rst_pend_q <= 1'b1;
        rst_post_q <= rst_post;
      // Taken, it is the operand engine's -- and it is taken as much when the
      // MMU refuses its page (ST_ABT) as when its cycle starts. UM 8.2.3: a
      // rerun that faults again is an ordinary bus error, with a new frame.
      // Left pending, the refused rerun was taken a second time by whatever
      // came next -- a posted one at once, any other by the next request
      // presented, the fault entry's own first frame write -- faulted again
      // inside the exception processing, and halted the processor on a double
      // bus fault. SunOS's first user-stack growth found it (doc/bugs-found.md).
      end else if (take_rst && ((st_p_nxt == rd68031_pkg::ST_S0)
                                || (st_p_nxt == rd68031_pkg::ST_ABT)))
        rst_pend_q <= 1'b0;
      else if (rst_done) begin
        rst_pend_q <= 1'b0;
        // A posted microword is satisfied by being taken, not acknowledged; an
        // acknowledge would be read by the microword after it.
        req_ack    <= !req_post;
        req_taken  <= req_post;
        req_end_q  <= rd68031_pkg::CE_DSACK;
        req_ciin   <= 1'b0;
        rdata_q    <= {8'd0, rst_dob};
      end else if (rst_pend_q && rst_post_q && (rst_bytes == 3'd0))
        // The handler wrote it -- UM 8.2.2 -- and there is nothing to run.
        rst_pend_q <= 1'b0;
      else if (rst_cancel && !rst_post_q)
        rst_pend_q <= 1'b0;

      fetch_ack <= 1'b0;
      tw_ack    <= 1'b0;
      req_fault    <= 1'b0;
      req_fault_wr <= 1'b0;
      req_fault_post <= 1'b0;
      fetch_fault  <= 1'b0;
      beat_valid   <= 1'b0;
      beat_err     <= 1'b0;

      halt_hold <= (st_p_nxt == rd68031_pkg::ST_HALT);

      // ECS and OCS from the rising edge entering S0 (specification 6A); write
      // data from the rising edge entering S2 to the one that ends S5 (UM 7.3.2
      // state 2, specification 23). cyc_rw is already this cycle's at S2.
      ecs_q <= (st_p_nxt == rd68031_pkg::ST_S0);
      ocs_q <= (st_p_nxt == rd68031_pkg::ST_S0) && first_now;
      doe_q <= !cyc_rw && ((st_p_nxt == rd68031_pkg::ST_S2)
                           || (st_p_nxt == rd68031_pkg::ST_S4)
                           || (st_p_nxt == rd68031_pkg::ST_WH));

      // The synchronous inputs. STERM is tested at the start of S2 and on every
      // rising edge after it while the cycle waits (UM 7.3.4 state 2), and at
      // the start of each burst beat (UM 7.3.7 state 4); CBACK and CIIN are
      // latched with it. A cycle DSACKx is ending has its CIIN latched "on the
      // rising edge of bus cycle state 4" (UM 7.2.9). CIIN "is ignored during
      // write cycles" (UM 5.7.1).
      if (st_p_nxt == rd68031_pkg::ST_S0) begin
        sterm_q <= 1'b0;
        cback_q <= 1'b0;
        ciin_q  <= 1'b0;
      end else if ((st_p_nxt == rd68031_pkg::ST_S2)
                   || (st_p_nxt == rd68031_pkg::ST_WH)
                   || (st_p_nxt == rd68031_pkg::ST_BH)) begin
        sterm_q <= sterm_s;
        cback_q <= sterm_s && cback_s;
        ciin_q  <= sterm_s && ciin_s && cyc_rw;
      end else if (st_p_nxt == rd68031_pkg::ST_S4) begin
        ciin_q  <= ciin_s && cyc_rw;
      end

      // The rising edge that consumes a transfer -- the end of S5, or the end
      // of a burst beat. Take the bytes it moved and advance the residual.
      // Every register the operand owns is written here and in the S0 arm
      // below, both in this one block -- a register written from both edge
      // domains is two conflicting drivers, which is a defect `make lint` now
      // fails on.
      if (beat_end) begin
        // The beat, for whoever fills a cache from it. A retried cycle is not
        // one: it moved nothing and will be run again.
        beat_valid <= cyc_rw && !cyc_abt && !term_rty && !term_err && !term_bfe;
        beat_err   <= cyc_rw && !cyc_abt && (term_err || term_bfe);
        beat_data  <= d_latched;
        beat_addr  <= bt_addr;
        beat_num   <= bt_k;
        beat_port  <= port_code;
        beat_ciin  <= ciin_q;
        beat_fetch <= op_isfetch;
      end

      if (beat_end && op_active) begin
        // A retried cycle is rerun "using the same access information" (UM
        // 7.5.2), and that includes OCS if the cycle had it: a retry of the
        // first cycle of an operand is still the first cycle of the operand.
        if (!term_rty) op_first <= 1'b0;
        // A cycle completing a cache entry carries none of the operand.
        if (!cyc_fill) begin
          if (op_rw) op_data <= rd_merged;
          op_addr <= nx_addr;
          op_rem  <= nx_rem;
        end
        if (ciin_now) op_ciin <= 1'b1;
        // The long word this cycle read, for the completion -- UM 6.1.3.1.
        if ((st_n == rd68031_pkg::ST_S5) && op_rw && !term_rty && !cyc_abt) begin
          ent_q   <= cyc_addr[31:2];
          ent_got <= got_now;
          ent_bad <= bad_now;
          fl_on   <= fl_on_n;
          fl_addr <= fl_addr_n;
          fl_rem  <= fl_rem_n;
        end
        if ((st_n == rd68031_pkg::ST_S5) && cyc_brq && !term_rty && !cyc_abt)
          op_brq_used <= 1'b1;
        if (op_finishing) begin
          op_active <= 1'b0;
          op_first  <= 1'b1;
          fl_on     <= 1'b0;
          if (op_isfetch) begin
            fetch_ack   <= 1'b1;
            fetch_fault <= term_err;
            fetch_ciin  <= op_ciin || ciin_now;
            fetch_ci    <= op_ci1;
            if (op_rw) frdata_q <= rd_merged[31:0];
          end else if (op_istw) begin
            // A descriptor, to the walker. Its bus error is the search's, not
            // an exception -- UM 9.5.2.
            tw_ack   <= 1'b1;
            tw_berr  <= term_err;
            tw_rdata <= rd_merged[31:0];
          end else begin
            // A posted write is acknowledged to nobody.
            req_ack      <= !op_posted;
            // An operand that finishes on a cycle completing a cache entry
            // finished cleanly, whatever that cycle met.
            req_end_q    <= cyc_fill ? rd68031_pkg::CE_DSACK : end_now;
            req_ciin     <= op_ciin || ciin_now;
            // A bus error on a CPU-SPACE cycle is not a bus error. UM 8.1.9
            // makes one on an interrupt acknowledge the spurious interrupt, and
            // UM 7.4.2 makes one on a breakpoint acknowledge an illegal
            // instruction; the microcode reads it off the end code and decides.
            // Raising a fault here as well would take the bus error exception
            // instead, and the spurious interrupt would be unreachable.
            req_fault    <= term_err && !cyc_fill
                                     && ((op_fc != rd68031_pkg::FC_CPU) || op_cpflt);
            req_fault_wr <= term_err && !cyc_fill
                                     && ((op_fc != rd68031_pkg::FC_CPU) || op_cpflt)
                                     && !op_rw;
            req_fault_post <= term_err && op_posted;
            if (op_rw) rdata_q <= cyc_fill ? op_data : rd_merged;
          end
        end
      end

      // Entering S0: latch everything the pins will carry for this whole cycle,
      // and start a new operand if this is the first cycle of one. This arm runs
      // after the one above in the same edge when a cycle follows immediately,
      // which is why next_addr and next_rem are written to see the post-transfer
      // residual.
      if ((st_p_nxt == rd68031_pkg::ST_S0) || (st_p_nxt == rd68031_pkg::ST_ABT)) begin
        // A refused cycle moves no pin: the address, function code, size,
        // R/W, CIOUT, CBREQ and RMC stay as the last cycle left them.
        if (st_p_nxt == rd68031_pkg::ST_S0) begin
          rmc_hold  <= next_rmc;
          // The first bus cycle of a locked run, and it stays the first
          // through however many times it is retried.
          if (st_p != rd68031_pkg::ST_RETRY)
            rmc_first_q <= next_rmc && !rmc_hold;
          cyc_paddr <= next_paddr;
          cyc_fc    <= next_fc;
          cyc_siz   <= this_siz;
          cyc_rw    <= next_rw;
          cyc_brq   <= next_brq;
          cyc_ciout <= next_ciout;
        end
        cyc_abt   <= (st_p_nxt == rd68031_pkg::ST_ABT);
        cyc_addr  <= next_addr;
        cyc_n     <= this_n;
        cyc_fill  <= next_fill;

        if (take_tw || take_rst || take_req || take_fetch || take_pk) begin
          // What the data cache said about it -- nothing, for a prefetch or an
          // operand RTE hands back (a rerun recomputes cacheability only by
          // reading the bus again: doc/checkpoint.md rule 12); what it said
          // when it was taken, for one coming back from being set aside.
          if (take_pk) begin
            op_dfill <= pk_dfill && (st_p_nxt == rd68031_pkg::ST_S0);
            op_fillc <= pk_fillc && (st_p_nxt == rd68031_pkg::ST_S0);
            op_tail  <= pk_tail;
          end else begin
            op_dfill <= take_req && req_dfill && (st_p_nxt == rd68031_pkg::ST_S0);
            op_fillc <= take_req && req_dfill && !req_rmc
                     && (st_p_nxt == rd68031_pkg::ST_S0);
            op_tail  <= (take_req && dc_pb) ? dc_nb : 3'd0;
          end
          fl_on      <= 1'b0;
          op_active  <= 1'b1;
          op_addr    <= next_addr;
          op_rem     <= next_rem;
          op_fc      <= next_fc;
          op_rw      <= next_rw;
          op_rmc     <= next_rmc;
          op_first   <= !take_pk;
          op_isfetch <= take_fetch;
          op_istw    <= take_tw;
          if (take_tw) begin
            op_pmask <= '1;
            op_lp1   <= '0;
            op_pb1   <= '0;
            op_ci1   <= 1'b0;
          end else begin
            op_pmask <= l_mask;
            op_lp1   <= next_addr & ~l_mask;
            op_pb1   <= l1_pb;
            op_ci1   <= l1_ci;
          end
          op_burst   <= next_burst;
          op_brq_used <= take_pk && pk_brq_used;
          op_ciin    <= take_pk && pk_ciin;
          // Posted: the operand RTE reruns by itself, or a posted request --
          // which may be taking a hand-back of its own -- or one set aside.
          if (take_pk)
            op_posted <= pk_posted;
          else
            op_posted <= take_rst ? (rst_post_q || req_post)
                                  : (take_req && req_post);
          req_taken  <= req_post && (take_req || (take_rst && !rst_post_q));
          // An operand RTE hands back in CPU space can only be a coprocessor
          // access that faulted, so it faults again if it has to.
          // So can a MOVES to CPU space: its bus error is a bus error.
          if (take_pk)
            op_cpflt <= pk_cpflt;
          else
            op_cpflt <= take_rst ? 1'b1 : (take_req && (req_cpfault || req_cpuraw));
          // A restarted operand keeps what it had already transferred: the
          // residual says how much is left, and the buffer holds the rest.
          // A read starts from nothing. Its bytes are gathered right justified
          // into this register, and a read of fewer than four bytes leaves the
          // rest as they started -- which, taken from the request's write data,
          // was whatever the requesting microword's ALU was producing. That was
          // always zero until a read began to compute its own result.
          //
          // A read half of which hit in the data cache starts with that half
          // already in place, and the bus fills in the rest.
          if (take_tw)         op_data <= {8'd0, tw_wdata};
          else if (take_pk)    op_data <= pk_data;
          else if (take_rst)   op_data <= {8'd0, rst_dob};
          else if (take_fetch) op_data <= '0;
          else if (next_rw)    op_data <= dc_data;
          else                 op_data <= req_wdata;
          if (take_pk) pk_valid <= 1'b0;
        end else if (cont_new) begin
          // A continuing operand's first cycle in its next page: that page's
          // translation, for this cycle and the rest of the operand's.
          op_pmask <= l_mask;
          op_lp1   <= next_addr & ~l_mask;
          op_pb1   <= l1_pb;
          op_ci1   <= l1_ci;
        end else if (s5_xid) begin
          // ... or with no translation possible, its own address.
          op_lp1   <= next_addr & ~op_pmask;
          op_pb1   <= next_addr & ~op_pmask;
          op_ci1   <= 1'b0;
        end
      end else if (park_now) begin
        // The bus is idle between the operand's cycles, and the page its next
        // one enters needs a table search: set the operand aside, as it
        // stands, and let the walker have the engine. It comes back through
        // take_pk once the search is over, translated again from the top.
        pk_valid    <= 1'b1;
        pk_addr     <= op_addr;
        pk_rem      <= op_rem;
        pk_fc       <= op_fc;
        pk_rw       <= op_rw;
        pk_rmc      <= op_rmc;
        pk_data     <= op_data;
        pk_posted   <= op_posted;
        pk_cpflt    <= op_cpflt;
        pk_burst    <= op_burst;
        pk_brq_used <= op_brq_used;
        pk_ciin     <= op_ciin;
        pk_dfill    <= op_dfill;
        pk_fillc    <= op_fillc;
        pk_tail     <= op_tail;
        op_active   <= 1'b0;
        fl_on       <= 1'b0;
      end else if ((st_p_nxt == rd68031_pkg::ST_IDLE) && !req_rmc && !mmu_busy
                   && !(op_continuing && op_rmc)) begin
        // Nothing locked is pending and the bus is going idle, so let go of RMC
        // rather than blocking arbitration until the next cycle happens along.
        // Not between the cycles of a table search, which holds it from its
        // first cycle to its last (UM 9.5.2), nor between an operand's own
        // cycles -- the idle clock before one that enters a new page.
        rmc_hold <= 1'b0;
      end

      // A write across two pages: its second page's long word waits in the
      // data cache for that page's translation. A new one may be taken on the
      // very edge the last one ends.
      if (dc_wr_take && dc_xpage) wb_pend <= 1'b1;
      else if (wb_at || wb_end)   wb_pend <= 1'b0;

      // A read the data cache holds whole: answered on this edge, no bus cycle
      // -- UM 6.1, "when a hit occurs in the instruction or data cache ... the
      // information is transferred from the cache (on a read)". Last, so that a
      // posted write finishing on this same edge, which acknowledges nobody,
      // does not clear it.
      if (hit_take) begin
        req_ack   <= 1'b1;
        req_end_q <= rd68031_pkg::CE_DSACK;
        req_ciin  <= 1'b0;
        rdata_q   <= dc_data;
      end
    end
  end

  // ==========================================================================
  // Falling-edge domain
  //
  // Three things happen here and nowhere else: the asynchronous termination
  // inputs are sampled at the end of S2 (UM 7.3.1 state 3, "as long as at least
  // one of the DSACKx signals is recognized by the end of S2"), the read data is
  // latched (at the end of S4, or at the falling edge after STERM), and the
  // verdict of a synchronous cycle is taken on that same edge.
  //
  // DSACK1 and DSACK0 TERMINATE the cycle on the edge where either is first
  // recognised, and give the PORT SIZE one clock later, on the edge that
  // latches the data. MC68030EC specification 31A, footnote 3: "the maximum
  // allowable skew between DSACK0 to DSACK1 asserted or DSACK1 to DSACK0
  // asserted; specification #47A must be met by DSACK0 or DSACK1" -- by one of
  // them, so the other may arrive after the recognising edge. UM 7.3.1 state 3
  // terminates "as long as at least one of the DSACKx signals is recognized",
  // and UM 7.3 makes "the bus master ... responsible for de-skewing the
  // acknowledge and data signals from the slave devices". A port size taken on
  // the recognising edge read a 32-bit port whose DSACK1 trailed DSACK0 by a
  // legal 3 ns as an 8-bit port, and the operand engine assembled the wrong
  // bytes with no error anywhere (doc/bugs-found.md, M3). One clock later both
  // are valid whenever 31A + 47A fits in a clock, which it does at every grade,
  // and the device still holds them: it "keeps its data and DSACKx signals
  // asserted until it detects the negation of AS" (UM 7.3.1 state 5), which is
  // this same edge. Both bits are still captured by one flop pair on one edge.
  // ==========================================================================
  // The six acceptable asynchronous terminations -- UM Table 7-8, with the case
  // numbers the manual gives them. The table indexes on two samples a clock
  // apart, "asserted on rising edge of state N" and "N+2"; here those are the
  // falling edge entering S3 (the end of S2) and the falling edge entering S5 (the
  // end of S4).
  //
  //   1  DSACK, no BERR, no HALT          normal
  //   2  HALT at or before DSACK          normal, then halt
  //   3  BERR in lieu of / at / before    bus error
  //   4  BERR one state pair after DSACK  bus error, deferred -- the late window
  //   5  BERR and HALT in lieu of / at / before   retry
  //   6  BERR and HALT after DSACK        retry, deferred
  //
  // Table 7-9 is the same six for STERM, with one difference that matters: there
  // is no state N+2 -- "state N+2 not part of bus cycle". The late window is the
  // falling edge after the rising edge that recognised STERM (UM 7.5.1, "BERR is
  // asserted and recognized on the next falling clock edge following the rising
  // clock edge on which STERM is asserted and recognized"), which is the edge
  // that latches the data and ends the cycle.
  //
  // Cases 4 and 6 are the ones worth being careful about. On the MC68010 project
  // the equivalent late assertion was detected and then never delivered, because
  // the late path set an end code without raising the fault; the exception was
  // simply not taken. Here the late sample writes term_err and term_rty, which is
  // the same place the early sample writes them.
  // The cycle under way is an interrupt acknowledge: CPU space, type $F on
  // A19-A16 -- UM figure 7-42. Both were latched at S0 and hold for the cycle.
  logic cyc_iack;
  assign cyc_iack = (cyc_fc == rd68031_pkg::FC_CPU)
                 && (cyc_addr[19:16] == rd68031_pkg::CPUS_IACK);
  logic early_q;

  // "The BG signal transitions on the falling edge of the clock after a state
  // is reached during which G changes" -- UM 7.7.4. With the arbiter, on rst_n
  // alone: arbitration goes on under the RESET pin (UM 7.7).
  always_ff @(negedge clk or negedge rst_n) begin
    if (!rst_n) bg_n_o <= 1'b1;
    else        bg_n_o <= ~arb_g_of(arb);
  end

  always_ff @(negedge clk or negedge eng_rst_nn) begin
    if (!eng_rst_nn) begin
      st_n      <= rd68031_pkg::ST_IDLE;
      dsack_q   <= rd68031_pkg::DSACK_WAIT;
      term_q    <= 1'b0;
      term_err  <= 1'b0;
      term_rty  <= 1'b0;
      term_avc  <= 1'b0;
      term_hlt  <= 1'b0;
      term_syn  <= 1'b0;
      term_bfe  <= 1'b0;
      beat_q    <= 3'd0;
      as_win    <= 1'b0;
      ds_q      <= 1'b0;
      cbreq_q   <= 1'b0;
      d_latched <= '0;
      early_q   <= 1'b0;
      retry_clr_q <= 1'b1;
    end else begin
      st_n <= st_n_nxt;

      // BERR and HALT both negated, for the end of a retry -- see retry_clear.
      retry_clr_q <= berr_n_i && halt_n_i;

      // The rising edge that ended S5, or a burst beat's BL, has consumed the
      // verdict; clear it for the next transfer. A retried cycle keeps nothing
      // either: UM 7.5.2 reruns it from the same access information, and the
      // rerun takes its own samples. This comes first, so that a burst beat
      // being latched on this same edge writes its own verdict over it.
      if ((st_n == rd68031_pkg::ST_S5) || (st_n == rd68031_pkg::ST_BL)) begin
        term_q   <= 1'b0;
        term_err <= 1'b0;
        term_rty <= 1'b0;
        term_avc <= 1'b0;
        term_hlt <= 1'b0;
        term_syn <= 1'b0;
        term_bfe <= 1'b0;
      end

      // A cycle the MMU refused: its verdict is a bus error, given on the edge
      // that takes it to S5 -- UM 8.1.2, "a bus error exception also occurs
      // when the MMU detects that a successful address translation is not
      // possible". Nothing else on this edge samples anything for it.
      if (st_p == rd68031_pkg::ST_ABT) begin
        term_q   <= 1'b1;
        term_err <= 1'b1;
      end

      // Decided on the edge entering S5, which is where the last sample is
      // taken: after it nothing can turn this cycle into a bus error or a retry,
      // and the residual after it is known. Everything the sequencer does with
      // it is a half clock -- the early retire, doc/timing-divergences.md. Only
      // for the first transfer of a cycle: a burst beat's operand bytes, if any,
      // are taken on the ordinary acknowledge.
      //
      // Not when the operand ends short of its last byte because that hit in
      // the data cache, nor on a cycle completing a cache entry, nor when one
      // is about to follow -- a cachable read whose port turned out narrower
      // than 32 bits (UM 6.1.3.1); the ordinary acknowledge takes those.
      early_q <= (st_n_nxt == rd68031_pkg::ST_S5) && op_active && !op_isfetch
              && !op_posted && !op_istw && (st_p != rd68031_pkg::ST_ABT)
              && (beat_q == 3'd0)
              && !term_err && !term_rty && !berr_s && (op_rem == xfer_n)
              && !cyc_fill && (op_tail == 3'd0)
              && !(op_fillc && !cyc_ciout && (port_code != rd68031_pkg::DSACK_32));

      beat_q <= (st_p == rd68031_pkg::ST_ABT) ? 3'd1 : beat_nxt;

      // AS, DS and CBREQ move on falling edges only.
      //
      // DS: on a read it follows AS (UM 7.1.5, "it is asserted at the same time
      // as AS during a read cycle"). On a write it waits until S3, "one full
      // clock cycle after the assertion of AS", indicating that the data is
      // stable (UM 7.3.2 state 3). A synchronous write with no wait state never
      // reaches S3, and so never asserts it: "DS is not asserted for two-clock
      // synchronous write cycles" (UM 5.6.6, 7.3.5); one with wait states
      // asserts it "at the end of S2" (UM 7.3.5 state 2), which is the same
      // edge.
      //
      // CBREQ: "if the burst mode is enabled for the appropriate on-chip cache
      // and all four long words of the cache entry are invalid ... CBREQ is
      // asserted" in S1 (UM 7.3.4 state 1), with AS (specification 9), and
      // negated with it -- or, in a full burst, "during S7", once the third
      // long word is latched, "indicating that the MC68030 only requests one
      // more long word" (UM 7.3.7).
      as_win  <= as_set(st_n_nxt);
      ds_q    <= cyc_rw ? as_set(st_n_nxt)
                        : ((st_n_nxt == rd68031_pkg::ST_S3)
                           || (st_n_nxt == rd68031_pkg::ST_WL));
      cbreq_q <= cyc_brq && as_set(st_n_nxt) && (beat_nxt < 3'd3);

      // The asynchronous sample: entering S3, and again at every ST_WL while
      // waiting.
      if (st_n_nxt == rd68031_pkg::ST_S3 || st_n_nxt == rd68031_pkg::ST_WL) begin
        if (berr_s && halt_s) begin
          term_q   <= 1'b1;  term_rty <= 1'b1;               // case 5
        end else if (berr_s) begin
          term_q   <= 1'b1;  term_err <= 1'b1;               // case 3
        end else if (!avec_s && cyc_iack) begin
          // UM 7.4.1.2 and 7.1.7: AVEC terminates an interrupt acknowledge
          // cycle in place of DSACK or STERM and says "use the autovector for
          // this level". It is sampled on the same edge as DSACK. On any other
          // cycle it is not a termination at all -- UM 7.1.7, "AVEC is ignored
          // during all other bus cycles" -- and the cycle waits for DSACK: a
          // board that ties AVEC low to autovector everything must still get
          // its wait states. Ending the cycle on it lost the writes of a
          // Sun-3/60 replica's PROM (RD68021 doc/bugs-found.md).
          term_q   <= 1'b1;  term_avc <= 1'b1;
        end else if (dsack_n_i != rd68031_pkg::DSACK_WAIT) begin
          term_q   <= 1'b1;  term_hlt <= halt_s;             // cases 1 and 2
        end else begin
          term_q   <= 1'b0;
        end
      end

      // A long word is latched: at the end of S4 (UM 7.3.1 state 4), or at the
      // falling edge after STERM (UM 7.3.4 state 2, 7.3.7 state 4). "During a
      // read cycle, the data is latched by the processor on the last falling
      // edge of the clock for that bus cycle" (UM 7.1.4).
      if ((st_n_nxt == rd68031_pkg::ST_S5) || (st_n_nxt == rd68031_pkg::ST_BL))
        d_latched <= d_i;

      // Table 7-8's second sample, for a cycle DSACKx ended. It applies only to
      // a cycle that terminated normally -- a cycle already in error does not get
      // a second verdict. The port size is taken here too, one clock after the
      // recognising edge -- specification 31A, above.
      if ((st_p == rd68031_pkg::ST_S4) && (st_n_nxt == rd68031_pkg::ST_S5)) begin
        dsack_q <= dsack_n_i;
        if (!term_err && !term_rty) begin
          if (berr_s && halt_s)   term_rty <= 1'b1;          // case 6
          else if (berr_s)        term_err <= 1'b1;          // case 4
        end
      end

      // The verdict of a synchronous transfer.
      if (syn_edge) begin
        term_q <= 1'b1;
        if (st_p == rd68031_pkg::ST_BH) begin
          // A burst fill beat. UM 7.5.1: "the cache entry corresponding to that
          // cycle is marked invalid, but the processor does not take an
          // exception"; and UM 7.5.2: BERR with HALT "during the second, third,
          // or fourth cycle of a burst operation does not cause a retry
          // operation" -- it "causes independent BERR and HALT operations".
          // BERR without STERM ends a beat too.
          term_syn <= sterm_q;
          term_bfe <= berr_s;
          term_hlt <= halt_s;
        end else begin
          // The first cycle: Table 7-9. HALT alone is case 2.
          term_syn <= 1'b1;
          if (berr_s && halt_s)   term_rty <= 1'b1;          // cases 5 and 6
          else if (berr_s)        term_err <= 1'b1;          // cases 3 and 4
          else                    term_hlt <= halt_s;        // cases 1 and 2
        end
      end
    end
  end

  // ==========================================================================
  // Pins
  //
  // Each is written as the manual states it, in terms of the two state
  // registers, and each comes from a flip-flop -- see "The strobes" above.
  // ==========================================================================

  // ECS: one half clock at the start of every bus cycle. Asserted on the rising
  // edge entering S0 (specification 6A) and negated on the falling edge entering
  // S1 (specification 12A), which is the whole of specification 10's width.
  // ecs_q is set only for that half clock, and the one edge in it at which st_n
  // moves is the one that takes it into S1, so the AND moves once.
  assign ecs_n_o = ~(ecs_q && (st_n != rd68031_pkg::ST_S1));

  // OCS: identical, but only for the first bus cycle of an operand (UM 7.1.1).
  assign ocs_n_o = ~(ocs_q && (st_n != rd68031_pkg::ST_S1));

  assign as_n_o = ~as_win;

  assign ds_n_o = ~ds_q;

  // DBEN, active low (Table 5-2), from rd68031_dedge_ff: its value after each
  // edge is dben_of() of the states after that edge. At the rising edge into S0
  // it is negated whatever the direction, which is what lets the new cycle's
  // R/W, latched on that same edge, not matter there.
  logic dben_q, dben_rise, dben_fall, dben_tp, dben_tn;
  always_comb begin
    if (st_p_nxt == rd68031_pkg::ST_S0) dben_rise = 1'b0;
    else                                dben_rise = dben_of(st_p_nxt, st_n, cyc_rw);
    dben_fall = dben_of(st_p, st_n_nxt, cyc_rw);
  end
  assign dben_tp   = dben_rise ^ dben_q;
  assign dben_tn   = dben_fall ^ dben_q;

  rd68031_dedge_ff #(.RESET_VAL (1'b0)) u_dben (
      .clk (clk), .rst_n (eng_rst_nn), .toggle_p (dben_tp), .toggle_n (dben_tn),
      .q (dben_q));

  assign dben_n_o = ~dben_q;

  assign cbreq_n_o = ~cbreq_q;

  // CIOUT "becomes valid" in S0 with the address (UM 7.3.1 state 0,
  // specification 6), and is held for the cycle.
  assign ciout_n_o = ~cyc_ciout;

  assign fc_o    = cyc_fc;
  assign a_o     = cyc_paddr;
  assign siz_o   = cyc_siz;
  assign rw_o    = cyc_rw;
  // UM 7.1.1 makes RMC a qualifier over a RUN of cycles, not a property of one:
  // it "is asserted at the beginning of the first bus cycle of a read-modify-write
  // operation and remains asserted until completion of the final bus cycle of
  // the operation", INCLUDING the clocks between them, because that is the whole
  // point -- nothing else may get at the location in the gap. `rmc_hold` is
  // exactly that run, and is already what inhibits arbitration.
  assign rmc_n_o = ~rmc_hold;
  assign d_o     = wr_lanes;

  // Bus relinquish. Combinational on the way down so that the processor drives
  // again "immediately following a state change, when bus mastership is returned".
  assign bus_granted = hiz_q && arb_t_of(arb);

  // Output enables.
  //
  // UM 7.8: "during the reset period, the entire bus three-states (except for
  // non-three-statable signals, which are driven to their inactive state)".
  // Both resets count: rst_n, the power-on initialisation that is not an
  // MC68030 pin (doc/pinout.md), and the RESET pin from outside -- every
  // enable below drops with eng_rst_n, which is either.
  //
  // The address group is driven from S0 and released at the rising edge that ends
  // S5 (specification 7) -- except while halted, where UM 7.5.3 says the address,
  // function code, size and read/write signals "remain in the same state",
  // driven rather than released. That is the one thing that distinguishes a
  // halted bus from an idle one on these pins. CIOUT is in this group:
  // specifications 6, 7 and 8 measure it with the address.
  logic cyc_drive;
  assign cyc_drive = (st_p != rd68031_pkg::ST_IDLE) && (st_p != rd68031_pkg::ST_ABT);

  logic addr_drive;
  assign addr_drive = ADDR_HIZ_BETWEEN_CYCLES ? (cyc_drive || halt_hold) : 1'b1;

  assign a_oe     = addr_drive && !bus_granted && eng_rst_n;
  assign fc_oe    = a_oe;
  assign siz_oe   = a_oe;
  assign ciout_oe = a_oe;
  // ... and it is DRIVEN for the whole of that run as well. The address may go
  // to high impedance between cycles -- ADDR_HIZ_BETWEEN_CYCLES -- and if RMC
  // followed it there, an external wrapper would three-state the one signal
  // whose job is to stay asserted in the gap.
  assign rmc_oe = (addr_drive || rmc_hold) && !bus_granted && eng_rst_n;

  // Write data is driven from the rising edge entering S2 and held through S5.
  // "When the processor completes a bus cycle with the HALT signal asserted, the
  // data bus is placed in the high-impedance state" -- so no halt term here.
  assign d_oe = doe_q && !bus_granted && eng_rst_n;

  // The control group is driven except on relinquish and reset. UM 7.5.3 is
  // explicit that halting drives these inactive rather than releasing them, and
  // 7.7.4's T is what releases them. CBREQ is in this group: specification 16
  // is "Clock High to AS, DS, R/W, DBEN, CBREQ High Impedance".
  assign as_oe    = !bus_granted && eng_rst_n;
  assign ds_oe    = !bus_granted && eng_rst_n;
  assign rw_oe    = !bus_granted && eng_rst_n;
  assign dben_oe  = !bus_granted && eng_rst_n;
  assign cbreq_oe = !bus_granted && eng_rst_n;

  // RESET is open drain: the output value is a constant zero and the enable is
  // what asserts it. HALT is an input only on the MC68030 (Table 5-2); the
  // halted state shows on STATUS instead (UM 7.5.4).
  assign reset_n_o  = 1'b0;
  assign reset_n_oe = rsto_q;

  // ==========================================================================
  // STATUS and REFILL -- UM 5.11.3, 5.11.4 and 12.7.1
  //
  // Table 12-4: STATUS asserted for one clock is an instruction boundary; for
  // two, a boundary at which a trace or interrupt exception is to be taken; for
  // three, the start of exception processing (reset, bus error, address error,
  // ...); continuously, "processor halted due to double bus fault". REFILL
  // "identifies when the microsequencer requests an instruction pipeline
  // refill". "Both STATUS and REFILL assert and negate from the falling edge of
  // the CLK signal" (UM 12.7.1, specifications 62 and 63), so both are
  // falling-edge registers, and neither is ever three-stated (Table 5-2).
  //
  // The sequencer reports each event as a one-clock pulse. A pulse is followed
  // by at least one clock negated, or two pulses would read as one longer one;
  // an event that arrives while one is being shown is held, and two that
  // arrive in that time are shown as the longer of them -- doc/divergences.md.
  // ==========================================================================
  logic [1:0] sts_code;       // what the sequencer reports this clock
  logic [1:0] sts_rem_q;      // clocks still to show after this one
  logic [1:0] sts_pend_q;     // an event waiting for its turn
  logic       sts_on_q;
  logic       refill_on_q;
  logic [1:0] sts_take;

  always_comb begin
    // ... and an ATC miss: "MMU address translation cache miss -- processor to
    // begin table search", the same three clocks (UM Table 8-2).
    if (sts_exc || mmu_sts) sts_code = 2'd3;
    else if (sts_pend)  sts_code = 2'd2;
    else if (sts_bound) sts_code = 2'd1;
    else                sts_code = 2'd0;
  end

  always_comb begin
    sts_take = (sts_pend_q > sts_code) ? sts_pend_q : sts_code;
  end

  always_ff @(negedge clk or negedge eng_rst_nn) begin
    if (!eng_rst_nn) begin
      sts_rem_q   <= 2'd0;
      sts_pend_q  <= 2'd0;
      sts_on_q    <= 1'b0;
      refill_on_q <= 1'b0;
    end else begin
      refill_on_q <= sts_refill;
      if (dbf) begin
        // UM 7.5.4: "continuously asserting the STATUS signal until the
        // processor is reset".
        sts_on_q   <= 1'b1;
        sts_rem_q  <= 2'd0;
        sts_pend_q <= 2'd0;
      end else if (sts_rem_q != 2'd0) begin
        sts_on_q   <= 1'b1;
        sts_rem_q  <= sts_rem_q - 2'd1;
        sts_pend_q <= sts_take;
      end else if (sts_on_q) begin
        // The clock negated between two indications.
        sts_on_q   <= 1'b0;
        sts_pend_q <= sts_take;
      end else if (sts_take != 2'd0) begin
        sts_on_q   <= 1'b1;
        sts_rem_q  <= sts_take - 2'd1;
        sts_pend_q <= 2'd0;
      end else begin
        sts_on_q   <= 1'b0;
      end
    end
  end

  assign status_n_o = ~sts_on_q;
  assign refill_n_o = ~refill_on_q;

  // ==========================================================================
  // Back to the sequencer
  // ==========================================================================
  assign req_rdata   = rdata_q;
  assign req_early   = early_q;
  assign fetch_rdata = frdata_q;

  // How the cycle ended, as the samples of Tables 7-8 and 7-9 leave it. This is
  // the live verdict, and it lives only until the rising edge that ends S5
  // consumes it.
  always_comb begin
    if (term_err)                            end_now = rd68031_pkg::CE_BERR;
    else if (term_rty)                       end_now = rd68031_pkg::CE_RETRY;
    else if (term_avc)                       end_now = rd68031_pkg::CE_AVEC;
    else if (term_hlt)                       end_now = rd68031_pkg::CE_HALT;
    else if (term_syn)                       end_now = rd68031_pkg::CE_STERM;
    else if (term_q)                         end_now = rd68031_pkg::CE_DSACK;
    else                                     end_now = rd68031_pkg::CE_NONE;
  end

  // What the SEQUENCER is told, which has to outlive it.
  //
  // The verdict is cleared on the falling edge that ends S5 and `req_ack` is
  // raised on the rising edge inside it, so the sequencer -- which retires on
  // the rising edge AFTER it sees the acknowledge -- looked at a verdict that
  // had already been cleared half a clock earlier. Every microword that tests
  // how a cycle ended read CE_NONE: the interrupt acknowledge never saw its
  // AVEC and never saw its bus error, so the autovector and the spurious
  // interrupt were both unreachable, and a vectored interrupt was the only
  // one that worked. Latched with `req_ack`, it holds until the next operand
  // finishes, which is what lets `exc_irq` test AVEC on one microword and BERR
  // on the next.
  assign req_end   = req_end_q;
  assign req_dsack = port_code;

  // ==========================================================================
  // The fault snapshot -- doc/ssw.md
  //
  // Everything the fault frame says about the faulted access, latched on the
  // clock the fault is recognised. It has to be a latch and not a view of the
  // live operand registers, because the frame is BUILT BY BUS CYCLES: by the
  // time the special status word reaches +$0A of the frame, four writes have
  // gone through the same operand engine and op_addr, op_rem and op_data are
  // about the last of them.
  //
  // The address is the operand's residual address -- the next byte still to
  // transfer -- and not the address the microword asked for. UM 8.2.2 has the
  // handler "transfer the properly sized data from the data output buffer (DOB)
  // on the stack frame to the location indicated by the data fault address", and
  // what is left to transfer is what it has to move.
  // ==========================================================================
  always_ff @(posedge clk or negedge eng_rst_n) begin
    if (!eng_rst_n) begin
      flt_addr  <= '0;
      flt_bytes <= 3'd0;
      flt_fc    <= 3'd0;
      flt_rw    <= 1'b1;
      flt_rmc   <= 1'b0;
      flt_dob   <= '0;
      flt_dib   <= '0;
    end else if (st_n == rd68031_pkg::ST_S5 && op_finishing && !op_isfetch
                 && !op_istw && term_err && !cyc_fill
                 && ((op_fc != rd68031_pkg::FC_CPU) || op_cpflt)) begin
      flt_addr  <= op_addr;
      flt_bytes <= op_rem;
      flt_fc    <= op_fc;
      flt_rw    <= op_rw;
      flt_rmc   <= op_rmc;
      // The two buffers are the same register, because an operand is either
      // being read or being written and op_data is whichever applies. They are
      // separate frame fields, and separate here, so that a handler reads the
      // one the direction makes meaningful and RTE writes back only that one.
      flt_dob   <= op_rw ? flt_dob : op_data[31:0];
      flt_dib   <= op_rw ? op_data[31:0] : flt_dib;
    end
  end

  // PRM 6 RESET: "asserts the RSTO signal for 512 clock periods, resetting all
  // external devices" -- and UM 7.8: "the processor drives the RESET signal for
  // 512 clock cycles". The counter is nine bits plus the state bit that
  // qualifies it, so the pin is driven on the 512 clocks in which rsto_q is set.
  //
  // `reset_busy` answers on the same clock the request arrives, before the
  // counter has started. Without that the sequencer's microword would see an
  // idle bus unit and retire immediately; with it, the one clock in which the
  // request is presented and the counter is not yet running is still a busy
  // one. The arming bit is the other half of the same problem at the other end:
  // the request is still asserted on the clock the count expires, and must not
  // be allowed to start it again.
  logic [8:0] rsto_cnt;
  logic       rsto_arm_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rsto_cnt   <= 9'd0;
      rsto_q     <= 1'b0;
      rsto_arm_q <= 1'b1;
    end else if (rsto_q) begin
      rsto_arm_q <= 1'b0;
      if (rsto_cnt == 9'd0) rsto_q   <= 1'b0;
      else                  rsto_cnt <= rsto_cnt - 9'd1;
    end else if (reset_req && rsto_arm_q) begin
      rsto_q     <= 1'b1;
      rsto_cnt   <= 9'd511;
      rsto_arm_q <= 1'b0;
    end else if (!reset_req) begin
      rsto_arm_q <= 1'b1;
    end
  end

  assign reset_busy  = rsto_q || (reset_req && rsto_arm_q);

  // ==========================================================================
  // The RESET pin as an input -- UM 7.8, 5.10.1 and 8.1.1
  //
  // "An external reset signal resets the MC68030 as well as all external
  // devices. A reset signal from the processor (asserted as part of the RESET
  // instruction) resets external devices only; the internal state of the
  // processor is not altered" (UM 5.10.1). RESET alone does it: the MC68030
  // asks for no HALT with it (UM 7.8, figure 7-64), and "asserting RESET for 10
  // clock periods is sufficient for resetting the processor logic".
  //
  // The pin is open drain and this processor drives it for the RESET
  // instruction, so its own pulse comes back on the input. UM 7.8: "an
  // external RESET signal that is asserted to the processor during execution
  // of a reset instruction must extend beyond the reset period of the
  // instruction by at least eight clock cycles to reset the processor". So the
  // input is not acted on while the instruction drives the pin, nor for
  // RSTO_TAIL clocks after -- the two falling-edge ranks of the synchroniser
  // and one rising edge, with a clock to spare -- and a RESET held on from
  // outside past that resets the processor, inside the eight clocks the manual
  // allows.
  //
  // crst_q is a rising-edge register, and is the reset the rest of the
  // processor sees (`crst`): held for as long as the pin is, and released on a
  // rising edge, after which reset exception processing begins exactly as it
  // does after rst_n (UM 8.1.1, figure 8-1). What it resets, and what it
  // leaves alone, is in rd68031_seq and doc/pinout.md.
  // ==========================================================================
  localparam logic [2:0] RSTO_TAIL = 3'd4;
  logic [2:0] rsto_tail_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rsto_tail_q <= 3'd0;
      crst_q      <= 1'b0;
    end else begin
      if (rsto_q)                   rsto_tail_q <= RSTO_TAIL;
      else if (rsto_tail_q != 3'd0) rsto_tail_q <= rsto_tail_q - 3'd1;
      crst_q <= !reset_sync_n && !rsto_q && (rsto_tail_q == 3'd0);
    end
  end

  always_ff @(negedge clk or negedge rst_n) begin
    if (!rst_n) crst_nq <= 1'b0;
    else        crst_nq <= crst_q;
  end

  assign crst = crst_q;

  // A write the MMU refused, as its refusal ends -- see the port.
  assign dc_inv       = op_finishing && cyc_abt && !op_rw
                     && (st_n == rd68031_pkg::ST_S5);
  assign dc_inv_addr  = op_addr;
  assign dc_inv_fc    = op_fc;
  assign dc_inv_bytes = op_rem;
  assign bus_idle    = bus_is_idle;

  // ==========================================================================
  // Inputs this unit does not consume yet. The list shrinks visibly as the design
  // fills in, which is why it is written out rather than waived wholesale.
  // ==========================================================================
  logic unused_biu;
  assign unused_biu = &{1'b1,
                        rst_op_valid, rst_addr, rst_bytes, rst_fc, rst_rw,
                        rst_rmc, rst_dob,
                        fetch_addr[1:0], bus_abort,
                        reset_req, dbf,
                        avec_n_i,
                        op_data[39:32],
                        cn_last[3:0], nx_addr[1:0]};

endmodule
