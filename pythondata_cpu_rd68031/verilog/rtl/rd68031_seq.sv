// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// Sequencer: the microcode engine, the 32-bit datapath, the register file, the
// three stack pointers, and (from M9) the marshalling of the checkpoint state
// into and out of a fault frame.
//
// ONE MICROWORD PER CLOCK, and the sequencer never counts clocks. A microword
// with a bus request stalls until req_ack, so wait states, a narrow port, a
// misaligned transfer and a retried cycle are all invisible here -- which is also
// what keeps dynamic bus sizing out of reach of the micro-address path.
//
// The store is read at the NEXT micro-address rather than the current one and is
// registered, so the microword arrives at the same time as the micro-address it
// belongs to: no clock lost, and a memory instead of logic.
//
// The instruction set is the MC68030's: the MC68020's, forked from RD68021, less
// CALLM and RTM, with the CpID-0 F-line words decoded as the MMU instructions
// rather than given to the coprocessor interface (tools/ucode/program.py), and
// CACR in the MC68030's layout. The MMU itself is rd68031_mmu: this unit hands
// it the instructions' operands and stalls while it searches.

module rd68031_seq #(
    parameter bit COPROCESSOR = 1'b0
) (
    input  logic        clk,
    input  logic        rst_n,
    // The RESET pin, from outside, registered by the bus unit -- UM 7.8, 8.1.1.
    // Everything here but the registers a reset leaves alone is held in its
    // reset state while it is asserted; reset exception processing starts when
    // it negates, as it does after rst_n.
    input  logic        crst,

    // Operand request to the bus unit ----------------------------------------
    output logic        req_valid,
    output logic        req_want,     // req_valid but for req_last
    output logic  [2:0] req_kind,
    output logic  [2:0] req_fc,
    output logic [31:0] req_addr,
    output logic  [2:0] req_bytes,
    output logic [39:0] req_wdata,
    output logic        req_rmc,
    output logic  [3:0] req_cpuspace,
    output logic  [7:0] req_cpuaddr,
    // A bus error on this CPU-space cycle is a bus error -- UM 10.5.2.8. True
    // for every coprocessor interface register access but the first.
    output logic        req_cpfault,
    // The address is the program's own, whatever the space -- MOVES, whose
    // SFC or DFC may name CPU space (PRM 6). Every other CPU-space request has
    // its address built by the bus unit from the type (UM figure 7-42).
    output logic        req_cpuraw,
    input  logic        req_ack,
    input  logic        req_start,
    input  logic        req_last,
    input  logic        req_early,
    input  logic [39:0] req_rdata,
    input  logic  [2:0] req_end,
    input  logic        req_fault,
    input  logic        req_fault_wr,
    input  logic  [1:0] req_dsack,
    // Posted writes -- UM 11.2.5.2, doc/checkpoint.md rule 9. This microword's
    // write is posted; the bus unit has taken it; a posted write is still
    // outstanding; and the fault being reported is on one.
    output logic        req_post,
    input  logic        req_taken,
    input  logic        post_busy,
    input  logic        req_fault_post,

    // The faulted operand's residual, and the way back in --------------------
    input  logic [31:0] flt_addr,
    input  logic  [2:0] flt_bytes,
    input  logic  [2:0] flt_fc,
    input  logic        flt_rw,
    input  logic        flt_rmc,
    input  logic [31:0] flt_dob,
    input  logic [31:0] flt_dib,
    output logic        rst_op_valid,
    output logic        rst_cancel,
    // ... and the operand handed back is a posted write, which the bus unit
    // reruns by itself rather than for the resumed microword's request.
    output logic        rst_post,
    output logic [31:0] rst_addr,
    output logic  [2:0] rst_bytes,
    output logic  [2:0] rst_fc,
    output logic        rst_rw,
    output logic        rst_rmc,
    output logic [31:0] rst_dob,

    // Instruction fetch unit --------------------------------------------------
    output logic  [1:0] pf_op,
    output logic [31:0] pf_addr,
    output logic        pf_super,
    input  logic        pf_ready,
    input  logic        pf_dvalid,
    input  logic [15:0] stg_d,
    input  logic [15:0] stg_c,
    input  logic [15:0] stg_b,
    input  logic        stg_c_fault,
    input  logic        stg_c_rerun,
    input  logic        stg_b_rerun,
    input  logic        pf_stuck,
    input  logic        pf_odd,
    input  logic        stg_b_fault,
    input  logic [31:0] pc_d,
    input  logic [31:0] stg_b_addr,
    output logic        ckpt_save,
    output logic        ckpt_wr,
    output logic  [2:0] ckpt_sel,
    output logic [31:0] ckpt_data,
    output logic        ckpt_load,
    output logic        ckpt_hold,
    input  logic [31:0] ckpt_pc_fetch,

    // Cache control -----------------------------------------------------------
    output logic [31:0] cacr,
    output logic [31:0] caar,
    output logic  [3:0] cach_op,

    // The MMU instructions -- UM 9.8, rd68031_mmu ----------------------------
    output logic  [2:0] mmu_cmd,     // rd68031_ucode_pkg::U_MMU_*
    output logic        mmu_go,      // commit it, or start the search
    output logic [15:0] mmu_xw,      // the command word
    output logic  [2:0] mmu_fc,      // the function code it names
    output logic [31:0] mmu_ea,      // the logical address <ea> names
    output logic [31:0] mmu_wd_hi,   // PMOVE's operand: T0
    output logic [31:0] mmu_wd_lo,   // ... and T1
    input  logic [31:0] mmu_rd_hi,
    input  logic [31:0] mmu_rd_lo,
    input  logic        mmu_cfg_bad,
    input  logic [31:0] mmu_da,
    input  logic        mmu_busy,
    input  logic        mmu_done,

    // Status -------------------------------------------------------------------
    input  logic  [2:0] ipl_sync_n,
    input  logic        reset_sync_n,
    input  logic        halt_sync_n,
    input  logic        bus_idle,
    input  logic        bus_granted,
    input  logic        reset_busy,
    output logic        reset_req,
    output logic        dbf,
    output logic        ipend_n_o,
    // What the STATUS and REFILL pins show -- UM 12.7.1, Table 12-4. Each is a
    // one-clock pulse, registered on the rising edge after the event; the bus
    // unit, which owns the pins, turns them into the clock counts the table
    // gives.
    output logic        sts_bound,    // an instruction boundary
    output logic        sts_pend,     // ... where a trace or interrupt is taken
    output logic        sts_exc,      // exception processing begins
    output logic        sts_refill    // the pipeline is to be refilled
);


  // ==========================================================================
  // The two resets
  //
  // `rst_n` is the power-on initialisation, and is not an MC68030 pin
  // (doc/pinout.md): every register takes its value from it. The RESET pin is
  // the architectural one, and UM 8.1.1 lists what it does: T1 and T0 cleared,
  // S set and M cleared, the mask at 7, VBR zero, CACR's enable, freeze, burst
  // and write-allocate bits cleared, both caches invalidated, and the MMU's
  // TC.E, TT0.E and TT1.E cleared (M11 builds those registers, and their reset
  // goes in the `crst` arm below with these). Then "reset exception processing"
  // -- the stack pointer and the program counter out of the vector table, which
  // is the microcode's `reset`. It does not say a reset changes D0-D7, A0-A6,
  // USP, MSP, SFC, DFC or CAAR, so the pin leaves them as they were; rst_n
  // still clears them, because ASIC has no power-on state.
  //
  // Every register in this unit that is not architectural is reset by both,
  // asynchronously, through core_rst_n. The one clocked process that holds
  // the architectural registers takes the pin as a synchronous `crst` arm
  // instead, which leaves the spared ones out of it.
  // ==========================================================================
  logic core_rst_n;
  assign core_rst_n = rst_n && !crst;

  // ==========================================================================
  // The microword
  // ==========================================================================
  `define UF(f) uw[rd68031_ucode_pkg::U_``f``_LSB +: rd68031_ucode_pkg::U_``f``_W]

  logic [rd68031_ucode_pkg::UW-1:0]    uw;
  logic [rd68031_ucode_pkg::UADDR-1:0] upc, upc_nxt;

  logic [rd68031_ucode_pkg::UADDR-1:0] dec_entry;
  logic                                dec_illegal;

  // The coprocessor response primitive being served -- UM 10.4. Held from the
  // read of the response CIR to the end of the service, because the primitive
  // decoder and the operand transfers read its fields, and checkpointed because
  // a bus error can land in the middle of one.
  logic [15:0] cprim_q;
  logic [rd68031_ucode_pkg::UADDR-1:0] cp_entry;
  // The effective address as the primitive decoder sees it -- assigned with
  // the addressing-mode signals below.
  logic [4:0] cp_ea;
  logic [15:0] cp_int;

  rd68031_ucode_rom u_urom (
      .clk (clk), .rst_n (core_rst_n), .addr (upc_nxt), .uw (uw));

  // The word to decode. An instruction ends with one microword that both
  // advances the pipe and decodes, so the opcode the decoder must look at is the
  // one ADV is about to move into stage D -- which is stage C. Decoding stage D
  // there decodes the instruction that has just finished, again.
  logic [15:0] dec_ir;
  assign dec_ir = (`UF(PF) == rd68031_ucode_pkg::U_PF_ADV) ? stg_c : stg_d;

  logic [rd68031_ucode_pkg::UADDR-1:0] dec_rom_entry;

  rd68031_decode_rom u_decode (
      .ir (dec_ir), .entry (dec_rom_entry), .illegal (dec_illegal));

  // With no coprocessor interface built, every F-line word with a CpID other
  // than zero is an F-line exception, which is also what a machine with no
  // coprocessor attached wants -- UM 10.5.2.2. CpID 0 is not the coprocessor
  // interface's on the MC68030 but the MMU's (UM 10.1), so it goes to the
  // decode table either way: the MMU instructions and the privilege check
  // that comes before them (UM 8.1.5, 9.8) are there whether or not a
  // coprocessor can be attached. The decode table is the same either way; the
  // parameter picks its output.
  generate
    if (COPROCESSOR) begin : g_cp_dec
      assign dec_entry = dec_rom_entry;
    end else begin : g_nocp_dec
      // ... except that cpSAVE and cpRESTORE (types 100 and 101) are
      // privileged first, and make the check with or without an interface to
      // talk to: UM 10.2.3.3 and 10.2.3.4. One whose effective address the
      // table accepts -- its entry is not the F-line one -- goes to
      // cp_nocp_priv, which takes the privilege violation in user mode and the
      // F-line exception in supervisor mode.
      logic nocp_fline, nocp_save_rest;
      assign nocp_fline     = (dec_ir[15:12] == 4'hF) && (dec_ir[11:9] != 3'b000);
      assign nocp_save_rest = (dec_ir[8:7] == 2'b10)
                              && (dec_rom_entry != rd68031_ucode_pkg::ENTRY_LINE_F);
      assign dec_entry = !nocp_fline     ? dec_rom_entry
                       : nocp_save_rest  ? rd68031_ucode_pkg::ENTRY_CP_NOCP_PRIV
                       :                   rd68031_ucode_pkg::ENTRY_LINE_F_NOCP;
    end
  endgenerate

  // The extension-word decoder. It reads stage C -- the extension word, latched
  // but not yet consumed -- plus the microword's own bit saying whether the base
  // is the program counter, which is in the opcode and not in the word.
  logic [rd68031_ucode_pkg::UADDR-1:0] ea_entry;
  logic                                ea_reserved;

  // The base bit goes through a named signal rather than straight into the port.
  // Quartus does not resolve a package-scoped part-select inside a port
  // connection: it reads U_EAPC_LSB and U_EAPC_W as undeclared identifiers,
  // creates implicit nets for them and builds a netlist that does not match the
  // source -- with a zero exit code. doc/coding-standard.md has the rule.
  logic ea_pc_base;
  assign ea_pc_base = `UF(EAPC);

  // The addressing-mode decoder. One opcode pattern per instruction instead of
  // one per instruction and mode.
  logic [rd68031_ucode_pkg::UADDR-1:0] eam_entry;
  logic                                eam_illegal;

  // PRM 8: MOVE writes its destination's mode and register into bits 8:6 and
  // 11:9, register first, which is the reverse of every other effective
  // address. Putting them back in the usual order costs a mux and saves a
  // second decoder.
  logic [5:0] eam_mr;
  assign eam_mr = `UF(EADST) ? {stg_d[8:6], stg_d[11:9]} : stg_d[5:0];

  rd68031_eamode_rom u_eamode (
      .mr (eam_mr), .entry (eam_entry), .illegal (eam_illegal));

  // Whether that mode's base is the program counter. PRM 2: modes 111/010 and
  // 111/011 are the program-counter-relative ones, and every access they make
  // is a program reference. Four gates off the instruction word, so it does not
  // need a column in the decoder.
  logic eam_pc_base;
  assign eam_pc_base = (eam_mr[5:3] == 3'b111) && (eam_mr[2:1] == 2'b01);

  rd68031_eadec_rom u_eadec (
      .pc_base (ea_pc_base), .xw (stg_c),
      .entry (ea_entry), .reserved (ea_reserved));

  // The response primitive decoder -- UM 10.4 and table 10-6. Its seventeenth bit
  // is the instruction's category, because what a primitive is allowed to do
  // depends on it: most of them with CA clear, and several at all, are protocol
  // violations inside a conditional instruction. Bits 8:6 of stage D say which
  // category it is -- 000 for cpGEN, anything else reaching the dialogue is a
  // conditional (UM figures 10-6 to 10-13).
  logic cp_cond;
  assign cp_cond = (stg_d[8:6] != 3'b000);

  rd68031_cpdec_rom u_cpdec (
      .cond_cat (cp_cond), .prim (cprim_q), .ea (cp_ea), .entry (cp_entry));

  // ==========================================================================
  // Architectural state
  //
  // A7 is not a register. It is whichever of USP, ISP and MSP the S and M bits
  // of the status register select (UM 4.1.1), so an exception switches stacks
  // without moving anything.
  // ==========================================================================
  logic [31:0] dreg [0:7];
  logic [31:0] areg [0:6];
  logic [31:0] usp_q, isp_q, msp_q;
  logic [15:0] sr_q;
  logic [31:0] vbr_q;
  logic  [2:0] sfc_q, dfc_q;
  logic [31:0] cacr_q, caar_q;

  // The checkpoint set -- doc/checkpoint.md.
  logic [31:0] t_q [0:3];

  // The faulted operand RTE hands back to the bus unit, and the double bus
  // fault -- both written further down, both read above where they are written.
  logic [31:0] rst_addr_q;
  logic [31:0] rst_data_q;
  logic  [2:0] rst_bytes_q;
  logic        dbf_q;

  // Declared here, above the logic that reads them, rather than beside the
  // logic that drives them: Quartus, Vivado and Questa all refuse a use above
  // the declaration, and tools/src_lint.py holds `make lint` to that.
  logic [31:0] a_bus, b_bus, y;
  logic [3:0] frame_code;
  logic [31:0] bf_offset;
  logic [31:0] bf_byteoff;
  logic [31:0] bf_field, bf_sxfield, bf_merged_reg;
  logic  [5:0] bf_ffo;
  logic [2:0] irq_taking_q;
  logic [31:0] pc_prev_q;   // the address of the instruction just finished
  logic [31:0] ea_save;
  logic        flt_odd_q;
  logic [rd68031_ucode_pkg::UADDR-1:0] flt_upc;
  // Where the read-modify-write under way starts again -- UM 8.2.3, frame +$52.
  logic [rd68031_ucode_pkg::UADDR-1:0] rmw_upc_q;
  // A fault is being taken this clock, and the step into an interrupt's entry
  // point -- both driven further down, read first by IPEND.
  logic fault_now;
  logic irq_enter;
  // The fault was on a posted write -- doc/checkpoint.md rule 9. Frame +$08.
  logic        post_flt_q;
  logic [15:0] ssw;
  logic [1:0] flt_siz;
  logic [15:0] int08;
  logic [15:0] int36;
  logic retire;
  logic [15:0] xw_q;
  logic [31:0] ea_q;


  // The return address of an effective-address routine. One level, because such
  // a routine is called from an instruction and calls nothing itself.
  logic [rd68031_ucode_pkg::UADDR-1:0] link_q;

  // Whether the base of the effective address under way is the program counter.
  // The microword's own `eapc` bit cannot answer that once the routine has been
  // entered: the brief format has one routine per base, but the twenty-one full
  // format routines are shared between the two, because the full extension word
  // behaves identically either way (PRM 2.5) and duplicating them would double
  // the table to carry a single bit. So the bit is latched out of the microword
  // that dispatched -- the only one that knows, since the base is named by the
  // opcode and not by the extension word -- and read back by EABASE.
  logic eapc_q;

  // Which field the effective address under way came out of. The shared
  // routines carry their own microword bits, so this has to be latched at the
  // dispatch for the same reason eapc_q is.
  logic eadst_q;

  logic super_mode;
  logic master_mode;
  // The status register as it WILL be once the microword now presented retires.
  //
  // A microword that writes the status register and decodes is one microword --
  // MOVE #imm,SR is exactly that -- and everything decided at that boundary is
  // decided by the register it has just written, not the one it replaced. UM
  // 8.1.7 traces the instruction AFTER one that turns tracing on, and UM 8.1.9
  // compares the interrupt level against the mask an instruction has just
  // lowered. Reading sr_q there is one instruction late in both cases, which a
  // MOVE #$8700,SR followed by a MOVEQ shows: the MOVEQ is the instruction the
  // manual traces, and it was the one after it that got traced.
  //
  // Only a microword that DECODES can make the difference, and only four shapes
  // of it exist -- MOVE to SR and STOP write T0, ANDI, ORI and EORI to SR combine
  // SR with T0 -- so the value is computed from those two registers here and
  // not taken off the result bus. assemble.py's check_live_shape holds every
  // SR-writing microword that decodes to one of the four. Reading the result
  // bus put the shifter and the bit-field unit, which no such microword uses,
  // on a timing path into the next micro-address (doc/critical-path.md).
  logic [15:0] sr_dec;
  always_comb begin
    unique case (`UF(ALU))
      rd68031_ucode_pkg::U_ALU_AND: sr_dec = sr_q & t_q[0][15:0];
      rd68031_ucode_pkg::U_ALU_OR:  sr_dec = sr_q | t_q[0][15:0];
      rd68031_ucode_pkg::U_ALU_EOR: sr_dec = sr_q ^ t_q[0][15:0];
      default:                      sr_dec = t_q[0][15:0];
    endcase
  end

  logic [15:0] sr_eff;
  assign sr_eff = (retire && (`UF(DST) == rd68031_ucode_pkg::U_DST_SR)
                   && (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_DECODE))
                  ? (sr_dec & rd68031_pkg::SR_IMPLEMENTED)
                  : sr_q;

  assign super_mode  = sr_q[rd68031_pkg::SR_S];
  assign master_mode = sr_q[rd68031_pkg::SR_M];

  logic [31:0] sp_read;
  always_comb begin
    if (!super_mode)     sp_read = usp_q;
    else if (master_mode) sp_read = msp_q;
    else                 sp_read = isp_q;
  end

  // ==========================================================================
  // Register selects. A convention rather than a microword field: an A source
  // reads the register bits 2:0 of the instruction word name, and a destination
  // writes the one bits 11:9 name. That is the direction MOVE and MOVEQ both go.
  // ==========================================================================
  // The register an effective address names. For MOVE's destination that is
  // bits 11:9, so it follows the same mux the decoder does -- otherwise (An)+
  // as a MOVE destination would step whichever register bits 2:0 happened to
  // name, which is the source's.
  //
  // A fast effective-address path (tools/ucode/program.py, Phase 2 of
  // doc/timing-divergences.md) addresses MOVE's destination without an EAMODE
  // dispatch to latch eadst_q, so its own microword's EADST bit steers too.
  logic [2:0] rsel, wsel;
  assign rsel = (eadst_q || `UF(EADST)) ? stg_d[11:9] : stg_d[2:0];
  assign wsel = stg_d[11:9];

  // The index register an extension word names, sized and scaled -- PRM 2.5 and
  // table 2-1. D/A is bit 15, the register number bits 14:12, W/L bit 11 (a
  // sign-extended word or a long word) and SCALE bits 10:9 (1, 2, 4 or 8).
  logic [2:0]  xw_ix;
  logic [31:0] xw_ixval;
  logic [31:0] xw_index;

  assign xw_ix = xw_q[14:12];

  always_comb begin
    if (xw_q[15]) xw_ixval = (xw_ix == 3'd7) ? sp_read : areg[xw_ix];
    else          xw_ixval = dreg[xw_ix];
  end

  // Bit 8 selects the format: 0 brief, 1 full. In a full extension word bit 6
  // is IS, which suppresses the index, and bit 7 is BS, which suppresses the
  // base -- PRM table 2-1. Both are done here rather than by having a separate
  // microcode routine for each, which would double the table for nothing.
  logic xw_full;
  assign xw_full = xw_q[8];

  always_comb begin
    logic [31:0] sized;
    sized = xw_q[11] ? xw_ixval : {{16{xw_ixval[15]}}, xw_ixval[15:0]};
    if (xw_full && xw_q[6]) xw_index = 32'd0;
    else                    xw_index = sized << xw_q[10:9];
  end

  // The base of an indexed effective address.
  logic [31:0] ea_base;
  always_comb begin
    if (xw_full && xw_q[7])      ea_base = 32'd0;
    else if (eapc_q)             ea_base = stg_b_addr - 32'd2;
    else if (rsel == 3'd7)       ea_base = sp_read;
    else                         ea_base = areg[rsel];
  end

  // ==========================================================================
  // The effective operand size
  //
  // Most instructions carry their size in the opcode, and an effective-address
  // routine that read the microword's own size field would have to exist once
  // per size. `szsel` says where to read it from instead, and every one of its
  // sources is a field of stage D -- a register, so nothing a bus request
  // depends on is being discovered here.
  // ==========================================================================
  logic [1:0] eff_size;
  logic [1:0] size_q;     // what the dispatching microword resolved
  always_comb begin
    unique case (`UF(SZSEL))
      // Bits 7:6, and the opmode field's low two bits, which are the same
      // encoding: 00 byte, 01 word, 10 long.
      rd68031_ucode_pkg::U_SZSEL_IR76,
      rd68031_ucode_pkg::U_SZSEL_IR86:
        unique case (stg_d[7:6])
          2'b00:   eff_size = rd68031_ucode_pkg::U_SIZE_BYTE;
          2'b01:   eff_size = rd68031_ucode_pkg::U_SIZE_WORD;
          default: eff_size = rd68031_ucode_pkg::U_SIZE_LONG;
        endcase
      // MOVE alone encodes its size in bits 13:12, and not in the same order as
      // anything else -- PRM 8.
      rd68031_ucode_pkg::U_SZSEL_MOVE:
        unique case (stg_d[13:12])
          2'b01:   eff_size = rd68031_ucode_pkg::U_SIZE_BYTE;
          2'b11:   eff_size = rd68031_ucode_pkg::U_SIZE_WORD;
          default: eff_size = rd68031_ucode_pkg::U_SIZE_LONG;
        endcase
      rd68031_ucode_pkg::U_SZSEL_IR6:
        eff_size = stg_d[6] ? rd68031_ucode_pkg::U_SIZE_LONG
                            : rd68031_ucode_pkg::U_SIZE_WORD;
      rd68031_ucode_pkg::U_SZSEL_IR8:
        eff_size = stg_d[8] ? rd68031_ucode_pkg::U_SIZE_LONG
                            : rd68031_ucode_pkg::U_SIZE_WORD;
      rd68031_ucode_pkg::U_SZSEL_LATCHED:
        eff_size = size_q;
      // PRM 8: CAS and CAS2 number the same two bits from one, not from zero,
      // and leave 00 unused.
      rd68031_ucode_pkg::U_SZSEL_CAS:
        unique case (stg_d[10:9])
          2'b01:   eff_size = rd68031_ucode_pkg::U_SIZE_BYTE;
          2'b10:   eff_size = rd68031_ucode_pkg::U_SIZE_WORD;
          default: eff_size = rd68031_ucode_pkg::U_SIZE_LONG;
        endcase
      // PRM 8 gives CMP2 and CHK2 their size in bits 10:9 numbered from zero.
      rd68031_ucode_pkg::U_SZSEL_IR109:
        unique case (stg_d[10:9])
          2'b00:   eff_size = rd68031_ucode_pkg::U_SIZE_BYTE;
          2'b01:   eff_size = rd68031_ucode_pkg::U_SIZE_WORD;
          default: eff_size = rd68031_ucode_pkg::U_SIZE_LONG;
        endcase
      // A bit-field access is as many bytes as the field touches, so the size
      // is not one of the three the rest of the machine uses. The two selectors
      // differ only in whether the field is in a register or in memory.
      rd68031_ucode_pkg::U_SZSEL_BFMEM,
      rd68031_ucode_pkg::U_SZSEL_BFREG:
        eff_size = rd68031_ucode_pkg::U_SIZE_LONG;
      rd68031_ucode_pkg::U_SZSEL_CHK:
        eff_size = stg_d[7] ? rd68031_ucode_pkg::U_SIZE_WORD
                            : rd68031_ucode_pkg::U_SIZE_LONG;
      // UM 10.4.9: a register operand is one, two or four bytes. Anything else
      // is refused before a microword with this selector is reached.
      rd68031_ucode_pkg::U_SZSEL_CPLEN:
        unique case (cprim_q[7:0])
          8'd1:    eff_size = rd68031_ucode_pkg::U_SIZE_BYTE;
          8'd2:    eff_size = rd68031_ucode_pkg::U_SIZE_WORD;
          default: eff_size = rd68031_ucode_pkg::U_SIZE_LONG;
        endcase
      default: eff_size = `UF(SIZE);
    endcase
  end

  // The operand size in bytes. PRM 2: a byte access through A7 steps it by two,
  // so that the stack pointer stays even.
  //
  // UM 2.4.4 and 2.4.5 tie that rule to the register being stepped, so there
  // are two of these: for the register the effective-address field names
  // (rsel), and for the one bits 11:9 name (wsel) -- Ax of the two-register
  // memory forms, -(Ay),-(Ax) and (Ay)+,(Ax)+ (doc/bugs-found.md, M6).
  logic [31:0] opsize_bytes, opsize_bytes_w;
  always_comb begin
    unique case (eff_size)
      rd68031_ucode_pkg::U_SIZE_BYTE: opsize_bytes = (rsel == 3'd7) ? 32'd2 : 32'd1;
      rd68031_ucode_pkg::U_SIZE_WORD: opsize_bytes = 32'd2;
      default:                        opsize_bytes = 32'd4;
    endcase
    unique case (eff_size)
      rd68031_ucode_pkg::U_SIZE_BYTE: opsize_bytes_w = (wsel == 3'd7) ? 32'd2 : 32'd1;
      rd68031_ucode_pkg::U_SIZE_WORD: opsize_bytes_w = 32'd2;
      default:                        opsize_bytes_w = 32'd4;
    endcase
  end

  // ==========================================================================
  // The datapath
  //
  // The source multiplexers below read signals that the units producing them
  // declare further down -- the register file MOVEM walks, the control
  // registers, the multiplier and the divider. iverilog, Verilator and yosys
  // accept a use before its declaration inside a module; Quartus creates an
  // IMPLICIT NET for it and builds a netlist that does not match the source,
  // and Questa refuses outright. So they are declared here, above their first
  // use, and driven where they belong. doc/coding-standard.md has the rule.
  // ==========================================================================
  logic [31:0] regn_val, regnr_val;
  logic [31:0] creg_read;
  logic [31:0] xreg_read;
  logic [31:0] cpreg_read;
  logic [63:0] mul_full;
  logic [31:0] div_q, div_r;
  logic [31:0] bit_mask;


  // What an address-register destination actually stores. PRM 2: the whole
  // register is written whatever the operation size, and a narrower result is
  // sign extended to get there.
  //
  // The byte case exists for one instruction. MOVEA has no byte form and
  // neither does ADDQ or SUBQ to an address register, so until MOVES arrived
  // nothing could put a byte in one -- PRM 6, "if the destination is an address
  // register, the source operand is sign-extended to 32 bits". Leaving the byte
  // case out put $0000009C where $FFFFFF9C belonged.
  logic [31:0] y_areg;
  always_comb begin
    unique case (eff_size)
      rd68031_ucode_pkg::U_SIZE_BYTE: y_areg = {{24{y[7]}},  y[7:0]};
      rd68031_ucode_pkg::U_SIZE_WORD: y_areg = {{16{y[15]}}, y[15:0]};
      default:                        y_areg = y;
    endcase
  end

  always_comb begin
    unique case (`UF(ASRC))
      rd68031_ucode_pkg::U_ASRC_ZERO:  a_bus = 32'd0;
      rd68031_ucode_pkg::U_ASRC_T0:    a_bus = t_q[0];
      rd68031_ucode_pkg::U_ASRC_T1:    a_bus = t_q[1];
      rd68031_ucode_pkg::U_ASRC_T2:    a_bus = t_q[2];
      rd68031_ucode_pkg::U_ASRC_T3:    a_bus = t_q[3];
      rd68031_ucode_pkg::U_ASRC_RDATA: a_bus = req_rdata[31:0];
      rd68031_ucode_pkg::U_ASRC_STG_D: a_bus = {16'd0, stg_d};
      rd68031_ucode_pkg::U_ASRC_STG_C: a_bus = {16'd0, stg_c};
      rd68031_ucode_pkg::U_ASRC_XW:    a_bus = {{16{xw_q[15]}}, xw_q};
      rd68031_ucode_pkg::U_ASRC_PC_D:  a_bus = pc_d;
      rd68031_ucode_pkg::U_ASRC_SR:    a_bus = {16'd0, sr_q};
      rd68031_ucode_pkg::U_ASRC_DREG:  a_bus = dreg[rsel];
      rd68031_ucode_pkg::U_ASRC_AREG:  a_bus = (rsel == 3'd7) ? sp_read
                                                              : areg[rsel];
      rd68031_ucode_pkg::U_ASRC_DREGW: a_bus = dreg[wsel];
      rd68031_ucode_pkg::U_ASRC_AREGW: a_bus = (wsel == 3'd7) ? sp_read
                                                              : areg[wsel];
      rd68031_ucode_pkg::U_ASRC_IMM8:  a_bus = {{24{stg_d[7]}}, stg_d[7:0]};
      rd68031_ucode_pkg::U_ASRC_DISP8: a_bus = {{24{stg_d[7]}}, stg_d[7:0]};
      rd68031_ucode_pkg::U_ASRC_SP:    a_bus = sp_read;
      rd68031_ucode_pkg::U_ASRC_USP:   a_bus = usp_q;
      rd68031_ucode_pkg::U_ASRC_CCRW:  a_bus = {27'd0, sr_q[4:0]};
      // UM 8.1 step four: "the processor multiplies the vector number by four
      // to determine the exception vector offset". The offset is what both the
      // format word and the vector address want, so it is what this gives.
      rd68031_ucode_pkg::U_ASRC_VECOFF:
        a_bus = {22'd0, `UF(VEC), 2'b00};
      // PRM 4: TRAP #n takes vector 32 + n, and n is bits 3:0 of the opcode.
      rd68031_ucode_pkg::U_ASRC_TRAPVEC:
        // Vector 32 + n, times four, which is 128 plus four times n.
        a_bus = 32'd128 + {26'd0, stg_d[3:0], 2'b00};
      // The format word at +$06 of every frame: the format in bits 15:12 and
      // the vector offset, which T0 is holding, in bits 11:0.
      rd68031_ucode_pkg::U_ASRC_FMTVEC:
        a_bus = {16'd0, frame_code, t_q[0][11:0]};
      // The same word, with the vector offset out of the microword rather than
      // out of T0: a fault frame has to name its format and its vector while
      // T0 still belongs to the instruction that faulted.
      // The vector a bus fault takes, as an offset, and the same thing packed
      // with the frame format. One builder serves the bus error and the address
      // error, and this is where they part.
      rd68031_ucode_pkg::U_ASRC_FLTVEC:
        a_bus = flt_odd_q ? 32'd12 : 32'd8;
      rd68031_ucode_pkg::U_ASRC_FLTFMT:
        a_bus = {16'd0, frame_code, flt_odd_q ? 12'h00C : 12'h008};
      rd68031_ucode_pkg::U_ASRC_FMTVECI:
        a_bus = {16'd0, frame_code, 2'b00, `UF(VEC), 2'b00};
      // The bit field's results are NOT on the A bus: see bf_a below.
      rd68031_ucode_pkg::U_ASRC_VBR:   a_bus = vbr_q;
      // The fault frame's fields -- doc/ssw.md and doc/checkpoint.md.
      rd68031_ucode_pkg::U_ASRC_SSW:        a_bus = {16'd0, ssw};
      rd68031_ucode_pkg::U_ASRC_DFA:        a_bus = flt_addr;
      rd68031_ucode_pkg::U_ASRC_DOB:        a_bus = flt_dob;
      rd68031_ucode_pkg::U_ASRC_DIB:        a_bus = flt_dib;
      // Stage C as a frame field. The same bits as U_ASRC_STG_C and a different
      // encoding on purpose: this one is not a USE of the word, so it neither
      // waits for the pipe nor takes the prefetch fault the word carries.
      rd68031_ucode_pkg::U_ASRC_STG_C_RAW:  a_bus = {16'd0, stg_c};
      rd68031_ucode_pkg::U_ASRC_STG_B:      a_bus = {16'd0, stg_b};
      rd68031_ucode_pkg::U_ASRC_STG_B_ADDR: a_bus = stg_b_addr;
      rd68031_ucode_pkg::U_ASRC_PC_FETCH:   a_bus = ckpt_pc_fetch;
      rd68031_ucode_pkg::U_ASRC_EA_SAVE:    a_bus = ea_save;
      rd68031_ucode_pkg::U_ASRC_LINK:
        a_bus = {{(32 - rd68031_ucode_pkg::UADDR){1'b0}}, link_q};
      rd68031_ucode_pkg::U_ASRC_UPC:
        a_bus = {{(32 - rd68031_ucode_pkg::UADDR){1'b0}}, flt_upc};
      rd68031_ucode_pkg::U_ASRC_INT08:      a_bus = {16'd0, int08};
      rd68031_ucode_pkg::U_ASRC_INT36:      a_bus = {16'd0, int36};
      rd68031_ucode_pkg::U_ASRC_RMWUPC:
        a_bus = {{(32 - rd68031_ucode_pkg::UADDR){1'b0}}, rmw_upc_q};
      rd68031_ucode_pkg::U_ASRC_PC_PREV: a_bus = pc_prev_q;
      rd68031_ucode_pkg::U_ASRC_IRQLEVEL: a_bus = {29'd0, irq_taking_q};
      // UM Table 8-1: the autovectors are 25 to 31 for levels 1 to 7, which is
      // 24 plus the level.
      rd68031_ucode_pkg::U_ASRC_AUTOVEC:
        a_bus = 32'd96 + {27'd0, irq_taking_q, 2'b00};
      // The vector a device returned on an acknowledge cycle is a BYTE, and
      // UM 8.1 multiplies it by four like any other.
      rd68031_ucode_pkg::U_ASRC_IRQVEC:
        a_bus = {22'd0, req_rdata[7:0], 2'b00};
      rd68031_ucode_pkg::U_ASRC_REGN:  a_bus = regn_val;
      rd68031_ucode_pkg::U_ASRC_REGNR: a_bus = regnr_val;
      // The product is not on the A bus either: see bf_a below.
      rd68031_ucode_pkg::U_ASRC_DIVQ:  a_bus = div_q;
      rd68031_ucode_pkg::U_ASRC_DIVR:  a_bus = div_r;
      // PRM 8 puts the long forms' register numbers in the extension word:
      // Dq or Dl in bits 14:12, Dr or Dh in bits 2:0.
      rd68031_ucode_pkg::U_ASRC_DREG_XQ: a_bus = dreg[xw_q[14:12]];
      rd68031_ucode_pkg::U_ASRC_DREG_XR: a_bus = dreg[xw_q[2:0]];
      rd68031_ucode_pkg::U_ASRC_DREG_XU: a_bus = dreg[xw_q[8:6]];
      rd68031_ucode_pkg::U_ASRC_CREG:  a_bus = creg_read;
      rd68031_ucode_pkg::U_ASRC_XREG:  a_bus = xreg_read;
      rd68031_ucode_pkg::U_ASRC_STG_C_HI: a_bus = {stg_c, 16'd0};
      rd68031_ucode_pkg::U_ASRC_XW_HI: a_bus = {xw_q, 16'd0};
      rd68031_ucode_pkg::U_ASRC_EA:    a_bus = ea_q;
      // PRM 2.5: "the value of the PC is the address of the extension word".
      rd68031_ucode_pkg::U_ASRC_PC_C:  a_bus = stg_b_addr - 32'd2;
      rd68031_ucode_pkg::U_ASRC_EABASE: a_bus = ea_base;
      // The coprocessor interface -- UM section 10.
      rd68031_ucode_pkg::U_ASRC_CPLEN: a_bus = {24'd0, cprim_q[7:0]};
      rd68031_ucode_pkg::U_ASRC_CPRIM: a_bus = {16'd0, cprim_q};
      rd68031_ucode_pkg::U_ASRC_CPVEC: a_bus = {22'd0, cprim_q[7:0], 2'b00};
      rd68031_ucode_pkg::U_ASRC_CPREG: a_bus = cpreg_read;
      rd68031_ucode_pkg::U_ASRC_CPINT: a_bus = {16'd0, cp_int};
      // UM figure 10-14: the format word, then a reserved word, at the head of
      // a coprocessor state frame. The reserved word is written as zero.
      rd68031_ucode_pkg::U_ASRC_FWLONG: a_bus = {t_q[0][15:0], 16'd0};
      rd68031_ucode_pkg::U_ASRC_FWLEN:  a_bus = {24'd0, t_q[0][7:0]};
      // The scanPC -- UM 10.4.1 -- is the address of stage C, "the word
      // following" whatever the instruction has consumed so far.
      rd68031_ucode_pkg::U_ASRC_PC_C_RAW: a_bus = stg_b_addr - 32'd2;
      // The instruction boundary RTE resumes at out of a short bus fault
      // frame, which carries no micro-address -- doc/checkpoint.md rule 1.
      rd68031_ucode_pkg::U_ASRC_UBOUND:
        a_bus = {{(32 - rd68031_ucode_pkg::UADDR){1'b0}},
                 rd68031_ucode_pkg::ENTRY_RTE_BOUNDARY};
      // PMOVE from an MMU register, and PTEST's descriptor address.
      rd68031_ucode_pkg::U_ASRC_MMUHI: a_bus = mmu_rd_hi;
      rd68031_ucode_pkg::U_ASRC_MMULO: a_bus = mmu_rd_lo;
      rd68031_ucode_pkg::U_ASRC_MMUDA: a_bus = mmu_da;
      default:                         a_bus = 32'd0;
    endcase
  end

  // The bit field's results -- PRM 4 -- and the multiplier's product, on a
  // multiplexer of their own. Every microword that reads one copies it (or, for
  // a bit field, complements it) into a register and does nothing else with it
  // (assemble.py's check_bf_shape and check_mul_shape), so it joins the result
  // only at the register destinations: the adder, the prefetch address, the
  // bus's write data and the checkpoint port never see it, and the multiply's
  // condition codes are taken from the product directly. On the A bus these put
  // the bit-field unit's and the multiplier's depth in front of all of them, on
  // paths no microword takes (doc/critical-path.md).
  logic        bf_on;
  logic [31:0] bf_a, y_reg;
  always_comb begin
    bf_on = 1'b1;
    unique case (`UF(ASRC))
      rd68031_ucode_pkg::U_ASRC_BF_FIELD:   bf_a = bf_field;
      rd68031_ucode_pkg::U_ASRC_BF_SXFIELD: bf_a = bf_sxfield;
      // BFFFO: "the bit offset in the instruction plus the offset of the first
      // one bit", and the field's width when there is none.
      //
      // The offset in that sum is the instruction's whole offset, for a field
      // in a data register as for one in memory: PRM 4 gives a register offset
      // the range -2^31 to 2^31-1 and reduces it modulo 32 only to locate the
      // field. RD68021 summed the reduced offset for a register field; that was
      // a reading of a gap in the manual, and doc/manual-contradictions.md
      // (entry 14) records why this one takes the literal reading instead.
      rd68031_ucode_pkg::U_ASRC_BF_FFO:
        bf_a = bf_offset + {26'd0, bf_ffo};
      rd68031_ucode_pkg::U_ASRC_BF_MERGED:  bf_a = bf_merged_reg;
      rd68031_ucode_pkg::U_ASRC_MULLO:      bf_a = mul_full[31:0];
      rd68031_ucode_pkg::U_ASRC_MULHI:      bf_a = mul_full[63:32];
      default: begin
        bf_on = 1'b0;
        bf_a  = 32'd0;
      end
    endcase
  end
  assign y_reg = !bf_on ? y
               : (`UF(ALU) == rd68031_ucode_pkg::U_ALU_NOT) ? ~bf_a : bf_a;

  always_comb begin
    unique case (`UF(BSRC))
      rd68031_ucode_pkg::U_BSRC_ZERO:  b_bus = 32'd0;
      rd68031_ucode_pkg::U_BSRC_TWO:   b_bus = 32'd2;
      rd68031_ucode_pkg::U_BSRC_FOUR:  b_bus = 32'd4;
      rd68031_ucode_pkg::U_BSRC_SIX:    b_bus = 32'd6;
      rd68031_ucode_pkg::U_BSRC_EIGHT:  b_bus = 32'd8;
      rd68031_ucode_pkg::U_BSRC_DREG_XR: b_bus = dreg[xw_q[2:0]];
      rd68031_ucode_pkg::U_BSRC_BF_BYTEOFF: b_bus = bf_byteoff;
      rd68031_ucode_pkg::U_BSRC_TWELVE: b_bus = 32'd12;
      // From rd68031_frame_pkg, which is generated from the same table the
      // frame is laid out by, so the step down to the frame base cannot drift
      // from the frame's size.
      rd68031_ucode_pkg::U_BSRC_FRAME_A_BYTES:
        b_bus = rd68031_frame_pkg::FRAME_A_BYTES;
      rd68031_ucode_pkg::U_BSRC_FRAME_B_BYTES:
        b_bus = rd68031_frame_pkg::FRAME_B_BYTES;
      rd68031_ucode_pkg::U_BSRC_BITMASK: b_bus = bit_mask;
      rd68031_ucode_pkg::U_BSRC_DIVQ:    b_bus = div_q;
      rd68031_ucode_pkg::U_BSRC_IRQLEVEL: b_bus = {29'd0, irq_taking_q};
      rd68031_ucode_pkg::U_BSRC_T0:    b_bus = t_q[0];
      rd68031_ucode_pkg::U_BSRC_T1:    b_bus = t_q[1];
      rd68031_ucode_pkg::U_BSRC_T2:    b_bus = t_q[2];
      rd68031_ucode_pkg::U_BSRC_T3:    b_bus = t_q[3];
      rd68031_ucode_pkg::U_BSRC_XW:    b_bus = {{16{xw_q[15]}}, xw_q};
      rd68031_ucode_pkg::U_BSRC_DISP8: b_bus = {{24{stg_d[7]}}, stg_d[7:0]};
      rd68031_ucode_pkg::U_BSRC_DREG:  b_bus = dreg[rsel];
      rd68031_ucode_pkg::U_BSRC_AREG:  b_bus = (rsel == 3'd7) ? sp_read
                                                              : areg[rsel];
      rd68031_ucode_pkg::U_BSRC_RDATA: b_bus = req_rdata[31:0];
      rd68031_ucode_pkg::U_BSRC_STG_C_U: b_bus = {16'd0, stg_c};
      rd68031_ucode_pkg::U_BSRC_STG_C_S: b_bus = {{16{stg_c[15]}}, stg_c};
      rd68031_ucode_pkg::U_BSRC_OPSIZE:  b_bus = opsize_bytes;
      rd68031_ucode_pkg::U_BSRC_OPSIZEW: b_bus = opsize_bytes_w;
      rd68031_ucode_pkg::U_BSRC_OPBYTES: b_bus = (eff_size == rd68031_ucode_pkg::U_SIZE_BYTE) ? 32'd1
                                               : (eff_size == rd68031_ucode_pkg::U_SIZE_WORD) ? 32'd2
                                               : 32'd4;
      rd68031_ucode_pkg::U_BSRC_INDEX:   b_bus = xw_index;
      rd68031_ucode_pkg::U_BSRC_XWDISP8: b_bus = {{24{xw_q[7]}}, xw_q[7:0]};
      rd68031_ucode_pkg::U_BSRC_EA:      b_bus = ea_q;
      rd68031_ucode_pkg::U_BSRC_DREGW:   b_bus = dreg[wsel];
      rd68031_ucode_pkg::U_BSRC_AREGW:   b_bus = (wsel == 3'd7) ? sp_read
                                                                : areg[wsel];
      rd68031_ucode_pkg::U_BSRC_ONE:     b_bus = 32'd1;
      // PRM 4: the quick forms take a value of one to eight, and zero in the
      // field means eight.
      rd68031_ucode_pkg::U_BSRC_IMMQ:    b_bus = (stg_d[11:9] == 3'd0)
                                                 ? 32'd8 : {29'd0, stg_d[11:9]};
      rd68031_ucode_pkg::U_BSRC_THREE:   b_bus = 32'd3;
      rd68031_ucode_pkg::U_BSRC_PCBIT:   b_bus = 32'h0000_4000;
      rd68031_ucode_pkg::U_BSRC_TWENTY:  b_bus = 32'd20;
      // UM 8.1.13: the long frame's version word, from rd68031_frame_pkg.
      rd68031_ucode_pkg::U_BSRC_VEROFF:  b_bus = rd68031_frame_pkg::OFF_VERSION;
      rd68031_ucode_pkg::U_BSRC_CPLEN:   b_bus = {24'd0, cprim_q[7:0]};
      // UM 10.4.9 and 10.4.12: a one-byte operand through A7 steps it by two.
      rd68031_ucode_pkg::U_BSRC_CPSTEP:
        b_bus = ((cprim_q[7:0] == 8'd1) && (rsel == 3'd7)) ? 32'd2
                                                           : {24'd0, cprim_q[7:0]};
      default:                           b_bus = 32'd0;
    endcase
  end

  // ==========================================================================
  // MOVEM's register list
  //
  // The register a transfer moves is the lowest set bit of what is left of the
  // list in T0, and the transfer clears it (cnt = CLRLOW) -- so the loop visits
  // the registers in the list and no others. REGN names them in the order
  // D0..D7, A0..A7; REGNR the other way, because PRM 4 reverses the mask for
  // the predecrement form -- "bit 0 selects A7".
  //
  // A priority encoder in the DATAPATH -- the register-file select -- and not
  // in the micro-address path: the loop branches on EMPTY and NOTEMPTY, a
  // sixteen-bit zero test on a register.
  // ==========================================================================
  logic  [3:0] regn, regnr;

  always_comb begin
    regn = 4'd0;
    for (int i = 15; i >= 0; i--) begin
      if (t_q[0][i]) regn = 4'(i);
    end
  end
  assign regnr = 4'd15 - regn;

  // Written out twice rather than as a function called from two continuous
  // assignments. doc/coding-standard.md's rule: a function that reads module
  // state is re-evaluated when its ARGUMENTS change and not when the state it
  // reads does, and the tools disagree about even that.
  //
  // It cost a real bug. MOVEM's first register is number zero, and the index
  // never changes from the value it starts at -- so the read of D0 was never
  // re-evaluated and every MOVEM stored a zero in place of it, while the other
  // eleven registers came out right because their index moved.
  always_comb begin
    if (!regn[3])               regn_val = dreg[regn[2:0]];
    else if (regn[2:0] == 3'd7) regn_val = sp_read;
    else                        regn_val = areg[regn[2:0]];
  end

  // REGNR's only reader is MOVEM to -(An), and there the addressing register
  // itself is the one exception to "the register's value". PRM 4: "For the
  // MC68020, MC68030, MC68040, and CPU32, if the addressing register is also
  // moved to memory, the value written is the initial register value
  // decremented by the size of the operation. The MC68000 and MC68010 write the
  // initial register value (not decremented)." The loop does not write An until
  // it is over, so the register still holds its initial value here, and the
  // operand size is IR6's two or four. RD68021 stored the MC68010's value
  // (doc/bugs-found.md, M6).
  logic [31:0] regnr_reg;
  always_comb begin
    if (!regnr[3])               regnr_reg = dreg[regnr[2:0]];
    else if (regnr[2:0] == 3'd7) regnr_reg = sp_read;
    else                         regnr_reg = areg[regnr[2:0]];
    if (regnr[3] && (regnr[2:0] == rsel)) regnr_val = regnr_reg - opsize_bytes;
    else                                  regnr_val = regnr_reg;
  end

  // Which of the two the destination names. Worked out HERE and not inside the
  // clocked block: a variable declared inside an always_ff and assigned with a
  // blocking assignment reads like a temporary and infers storage in yosys --
  // four flip-flops with no reset, which make_audit refuses.
  logic [3:0] movem_wn;
  assign movem_wn = (`UF(DST) == rd68031_ucode_pkg::U_DST_REGN) ? regn : regnr;

  // ==========================================================================
  // The bit instructions' mask
  //
  // PRM 4: the bit number is modulo 32 when the operand is a data register and
  // modulo 8 when it is a byte in memory. The operand size already says which,
  // so the mask follows it rather than the microword.
  // ==========================================================================
  logic [5:0]  bit_num;
  logic [4:0]  bit_sel;

  // A static bit number has been LATCHED by then, not left in stage C: the
  // effective address's extension words follow it, so the address routine has
  // already eaten past it. See the note in program.py's bitop.
  assign bit_num = `UF(BITIMM) ? xw_q[5:0] : dreg[wsel][5:0];
  assign bit_sel = (eff_size == rd68031_ucode_pkg::U_SIZE_BYTE)
                   ? {2'd0, bit_num[2:0]} : bit_num[4:0];
  assign bit_mask = 32'd1 << bit_sel;

  // ==========================================================================
  // The shifter
  //
  // PRM 8 lays the same instruction out two ways. The register forms put a
  // count of one to eight in bits 11:9, or the number of a data register whose
  // low six bits are the count, and the kind in bits 4:3. The memory forms
  // shift one bit of a word and put the kind in bits 10:9. Both are fields of
  // stage D, so the microword picks the layout and carries nothing.
  // ==========================================================================
  logic [5:0] sh_count;
  logic [1:0] sh_kind;
  logic [1:0] sh_size;
  logic       sh_left;
  logic       sh_by_reg;

  assign sh_by_reg = stg_d[5];

  always_comb begin
    if (`UF(SHOP) == rd68031_ucode_pkg::U_SHOP_MEM) begin
      sh_count = 6'd1;
      sh_kind  = stg_d[10:9];
      sh_size  = rd68031_ucode_pkg::U_SIZE_WORD;
      sh_left  = stg_d[8];
    end else begin
      // PRM 4: the count in a register is taken modulo 64, and the immediate
      // count is one to eight with zero meaning eight.
      sh_count = sh_by_reg ? dreg[stg_d[11:9]][5:0]
                           : ((stg_d[11:9] == 3'd0) ? 6'd8
                                                    : {3'd0, stg_d[11:9]});
      sh_kind  = stg_d[4:3];
      sh_size  = eff_size;
      sh_left  = stg_d[8];
    end
  end

  logic [31:0] sh_res;
  logic        sh_c, sh_v, sh_x, sh_xwr;

  // The extend bit goes through a named signal rather than straight into the
  // port. A PACKAGE-SCOPED name inside a port connection is read by Quartus as
  // an undeclared identifier: it creates an implicit net and builds a netlist
  // that does not match the source, with a zero exit code. The same trap took
  // `UF(EAPC)` in M6; the rule is in doc/coding-standard.md and this is the
  // second time it has been broken in this file.
  logic sh_x_in;
  assign sh_x_in = sr_q[rd68031_pkg::SR_X];

  // The shifter's operand is a data register or read data and nothing else --
  // assemble.py's check_shift_src -- so it has a two-way multiplexer of its own
  // rather than the A bus, which also carries the bit-field unit's results.
  logic [31:0] sh_op;
  assign sh_op = (`UF(ASRC) == rd68031_ucode_pkg::U_ASRC_RDATA) ? req_rdata[31:0]
                                                                  : dreg[rsel];

  rd68031_shifter u_shifter (
      .op (sh_op), .count (sh_count), .size (sh_size), .kind (sh_kind),
      .left (sh_left), .x_in (sh_x_in),
      .res (sh_res), .c_out (sh_c), .v_out (sh_v), .x_out (sh_x),
      .x_write (sh_xwr));

  // The four bits of the format word. UM Table 8-6.
  always_comb begin
    unique case (`UF(FRAME))
      rd68031_ucode_pkg::U_FRAME_F0: frame_code = 4'h0;
      rd68031_ucode_pkg::U_FRAME_F1: frame_code = 4'h1;
      rd68031_ucode_pkg::U_FRAME_F2: frame_code = 4'h2;
      rd68031_ucode_pkg::U_FRAME_F9: frame_code = 4'h9;
      rd68031_ucode_pkg::U_FRAME_FA: frame_code = 4'hA;
      default:                       frame_code = 4'hB;
    endcase
  end

  // ==========================================================================
  // MOVEC's control registers -- PRM 6
  //
  // "This is always a 32-bit transfer, even though the control register may be
  // implemented with fewer bits. Unimplemented bits are read as zeros." The
  // codes are the eight PRM 6 lists for the MC68020 and MC68030, which are the
  // same eight; the MC68040's are not here. The MC68030's MMU registers are not
  // control registers: PMOVE reaches them (UM 9.7).
  // Any other code is an illegal instruction -- UM 8.1.5, "a MOVEC instruction
  // with an undefined register specification field" -- and, through the
  // transfer-control-register primitive, a protocol violation (UM table 10-5).
  //
  // USP is the one that is NOT simply whichever stack pointer is active: MOVEC
  // names it explicitly, which is the whole point of the instruction in a
  // kernel that has to reach a user stack while running on its own.
  // ==========================================================================
  logic [11:0] creg_sel;
  assign creg_sel = xw_q[11:0];

  always_comb begin
    unique case (creg_sel)
      12'h000: creg_read = {29'd0, sfc_q};
      12'h001: creg_read = {29'd0, dfc_q};
      12'h002: creg_read = cacr_q;
      12'h800: creg_read = usp_q;
      12'h801: creg_read = vbr_q;
      12'h802: creg_read = caar_q;
      12'h803: creg_read = msp_q;
      12'h804: creg_read = isp_q;
      default: creg_read = 32'd0;
    endcase
  end

  logic creg_bad;
  always_comb begin
    unique case (creg_sel)
      12'h000, 12'h001, 12'h002,
      12'h800, 12'h801, 12'h802, 12'h803, 12'h804: creg_bad = 1'b0;
      default:                                    creg_bad = 1'b1;
    endcase
  end

  // The same test on stage C, for MOVEC: the extension word is latched into XW
  // by the microword that tests it, and a condition reads a register as it
  // stands, not as it is about to be.
  logic creg_bad_c;
  always_comb begin
    unique case (stg_c[11:0])
      12'h000, 12'h001, 12'h002,
      12'h800, 12'h801, 12'h802, 12'h803, 12'h804: creg_bad_c = 1'b0;
      default:                                    creg_bad_c = 1'b1;
    endcase
  end

  // ==========================================================================
  // The MMU instructions' command word -- UM 9.8, PRM 6 (the "MC68030 only"
  // pages) and PRM 8
  //
  // UM 8.1.5: "Instructions that have word patterns with bits [15:12] equal to
  // $F, bits [11:9] equal to $0, and defined word patterns for subsequent words
  // are legal MMU instructions. Instructions that have bits [15:12] of the
  // first words equal to $F, bits [11:9] equal to $0, and undefined patterns in
  // subsequent words are treated as unimplemented instructions with F-line
  // opcodes when execution is attempted in supervisor mode." The defined
  // patterns are PRM 6's:
  //
  //   PMOVE TT0/TT1    000 PPP R FD 00000000   PPP = 010 or 011
  //   PLOAD            001 000 R 0000 FFFFF
  //   PFLUSHA          001 001 0 0 000 00000
  //   PFLUSH FC,MASK   001 100 0 0 MMM FFFFF   (no <ea>)
  //   PFLUSH ..,<ea>   001 110 0 0 MMM FFFFF
  //   PMOVE TC/SRP/CRP 010 PPP R FD 00000000   PPP = 000, 010 or 011
  //   PMOVE MMUSR      011 000 R 0  00000000
  //   PTEST            100 LLL R A RRR FFFFF   level 0 only with A and RRR 0
  //
  // with FFFFF one of 10XXX (immediate), 01DDD (a data register), 00000 (SFC)
  // and 00001 (DFC). "Only control alterable addressing modes" for every <ea>
  // (UM 9.6, PRM 6) -- so not Dn and not An (manual-contradictions 12): the
  // decoder sends only those and the no-<ea> word $F000 here, and the word
  // with no <ea> is only PFLUSHA and PFLUSH FC,MASK. Tested on stage C, as
  // MOVEC's register is, on the microword that takes the word.
  // ==========================================================================
  logic mmu_eanone, mmu_fcok, mmu_bad_c;
  logic [2:0] mc_grp, mc_mid;
  assign mmu_eanone = (stg_d[5:0] == 6'd0);
  assign mmu_fcok   = (stg_c[4:3] == 2'b10) || (stg_c[4:3] == 2'b01)
                   || (stg_c[4:0] == 5'b00000) || (stg_c[4:0] == 5'b00001);
  assign mc_grp     = stg_c[15:13];
  assign mc_mid     = stg_c[12:10];
  always_comb begin
    unique case (mc_grp)
      3'b000:  mmu_bad_c = mmu_eanone || (stg_c[7:0] != 8'd0)
                        || ((mc_mid != 3'b010) && (mc_mid != 3'b011));
      3'b010:  mmu_bad_c = mmu_eanone || (stg_c[7:0] != 8'd0)
                        || ((mc_mid != 3'b000) && (mc_mid != 3'b010)
                            && (mc_mid != 3'b011));
      3'b011:  mmu_bad_c = mmu_eanone || (stg_c[8:0] != 9'd0) || (mc_mid != 3'b000);
      3'b001: begin
        unique case (mc_mid)
          3'b000:  mmu_bad_c = mmu_eanone || (stg_c[8:5] != 4'd0) || !mmu_fcok;
          3'b001:  mmu_bad_c = !mmu_eanone || (stg_c[9:0] != 10'd0);
          3'b100:  mmu_bad_c = !mmu_eanone || (stg_c[9:8] != 2'd0) || !mmu_fcok;
          3'b110:  mmu_bad_c = mmu_eanone || (stg_c[9:8] != 2'd0) || !mmu_fcok;
          default: mmu_bad_c = 1'b1;
        endcase
      end
      // PRM 6, PTEST: "When this field contains 0, the A field and the
      // register field must also be 0. The instruction takes an F-line
      // exception when the level field is 0 and the A field is not 0." And
      // "when the A field contains 0, this field must contain 0".
      3'b100:  mmu_bad_c = mmu_eanone || !mmu_fcok
                        || ((mc_mid == 3'b000) && stg_c[8])
                        || (!stg_c[8] && (stg_c[7:5] != 3'd0));
      default: mmu_bad_c = 1'b1;
    endcase
  end

  // The function code the command word names, for PTEST, PLOAD and PFLUSH --
  // PRM 6: "10XXX -- function code is specified as bits XXX. 01DDD -- function
  // code is specified as bits 2-0 of data register DDD. 00000 -- function code
  // is specified as SFC register. 00001 -- function code is specified as DFC
  // register."
  always_comb begin
    if (xw_q[4])      mmu_fc = xw_q[2:0];
    else if (xw_q[3]) mmu_fc = dreg[xw_q[2:0]][2:0];
    else if (xw_q[0]) mmu_fc = dfc_q;
    else              mmu_fc = sfc_q;
  end

  // The general register the extension word names: bit 15 says which file, bits
  // 14:12 which register.
  always_comb begin
    if (!xw_q[15])               xreg_read = dreg[xw_q[14:12]];
    else if (xw_q[14:12] == 3'd7) xreg_read = sp_read;
    else                         xreg_read = areg[xw_q[14:12]];
  end

  // ... and the one a transfer-single-register primitive names: D/A in bit 3,
  // the number in bits 2:0 -- UM figure 10-33.
  always_comb begin
    if (!cprim_q[3])             cpreg_read = dreg[cprim_q[2:0]];
    else if (cprim_q[2:0] == 3'd7) cpreg_read = sp_read;
    else                         cpreg_read = areg[cprim_q[2:0]];
  end

  // ==========================================================================
  // The bit field -- PRM 4, the eight BFxxx instructions
  //
  // Offset and width are not operands the microcode fetches; they are functions
  // of the extension word and, when it says so, of a data register. So they are
  // wires, and the microcode never has to move them anywhere.
  //
  // PRM 4: the offset is an unsigned five-bit immediate or the SIGNED
  // thirty-two-bit value in a data register; the width is an immediate or a
  // register modulo 32, and a zero means 32 in either case.
  // ==========================================================================
  logic  [4:0] bf_width_enc;
  logic  [5:0] bf_width;
  logic  [2:0] bf_nbytes;

  assign bf_offset    = xw_q[11] ? dreg[xw_q[8:6]] : {27'd0, xw_q[10:6]};
  assign bf_width_enc = xw_q[5]  ? dreg[xw_q[2:0]][4:0] : xw_q[4:0];
  assign bf_width     = (bf_width_enc == 5'd0) ? 6'd32 : {1'b0, bf_width_enc};

  // How far the base byte is from the effective address, and how many bytes the
  // field touches. A negative offset divides DOWNWARD -- an arithmetic shift --
  // and the remainder is still the non-negative one the low three bits give,
  // which is what makes the same two expressions right on both sides of zero.
  assign bf_byteoff   = {{3{bf_offset[31]}}, bf_offset[31:3]};
  assign bf_nbytes    = 3'(({3'd0, bf_offset[2:0]} + bf_width + 6'd7) >> 3);

  // The bytes the read returned, left justified: the bus unit hands them back
  // right justified, and the field is measured from the FIRST of them.
  logic [39:0] bf_window;
  always_comb begin
    unique case (bf_nbytes)
      3'd1:    bf_window = {req_rdata[7:0],  32'd0};
      3'd2:    bf_window = {req_rdata[15:0], 24'd0};
      3'd3:    bf_window = {req_rdata[23:0], 16'd0};
      3'd4:    bf_window = {req_rdata[31:0],  8'd0};
      default: bf_window = req_rdata[39:0];
    endcase
  end

  // The bit of the value being inserted that becomes the field's most
  // significant one. The width is one to thirty-two, so this is zero to
  // thirty-one and fits a register index exactly.
  logic  [4:0] bf_msb_ix;
  assign bf_msb_ix = 5'(bf_width - 6'd1);

  logic [39:0] bf_merged_mem;
  logic        bf_msb, bf_zero;

  // A named signal and not the expression in the port connection: Quartus reads
  // a package-scoped name there as an undeclared identifier -- coding standard.
  logic bf_is_reg;
  assign bf_is_reg = (`UF(SZSEL) == rd68031_ucode_pkg::U_SZSEL_BFREG);

  rd68031_bitfield u_bf (
      .is_reg     (bf_is_reg),
      .reg_data   (dreg[rsel]),
      .mem_data   (bf_window),
      .roff       (bf_offset[4:0]),
      .boff       (bf_offset[2:0]),
      .width      (bf_width),
      // What goes back in is always held in T3 first. It cannot be the ALU
      // result of the microword that writes it: that microword's result IS the
      // merge, so the two would depend on each other.
      .ins        (t_q[3]),
      .field      (bf_field),
      .sxfield    (bf_sxfield),
      .msb        (bf_msb),
      .zero       (bf_zero),
      .ffo        (bf_ffo),
      .merged_reg (bf_merged_reg),
      .merged_mem (bf_merged_mem));

  // ==========================================================================
  // The binary-coded decimal adjust -- PRM 4
  //
  // ABCD, SBCD and NBCD. A byte holds two decimal digits, and "store the result
  // in binary-coded decimal form" means: do the binary arithmetic, then correct
  // it. The correction is the textbook one, and it is applied to the WHOLE BYTE
  // rather than digit by digit:
  //
  //     add:       if the low digits summed past nine, add six;
  //                then if the byte passed $99, subtract $A0 and carry.
  //     subtract:  if the low digits borrowed, subtract six;
  //                then if the byte went below zero or past $99, add $A0 and
  //                borrow.
  //
  // Doing it per digit -- carry out of the low digit, then adjust the high one
  // -- is the obvious decomposition and is WRONG whenever a low-digit sum
  // exceeds fifteen, because the six then carries TWO into the digit above and
  // a one-bit carry cannot say so. That only happens when a digit is greater
  // than nine, which the manual calls an invalid operand and does not define;
  // the part still produces a definite answer and so does this.
  //
  // NBCD is SBCD with a zero minuend, which is why there is no third case.
  //
  // PRM 4 leaves N and V undefined for all three. This design sets N from the
  // result and clears V -- doc/divergences.md records it as the choice it is,
  // and the sweep does not compare either bit for these instructions.
  // ==========================================================================
  logic        bcd_sub;
  logic  [7:0] bcd_a, bcd_b;
  logic        bcd_x;
  logic  [4:0] bcd_lo;        // one digit, with room for the carry or borrow
  logic        bcd_six;
  logic  [9:0] bcd_raw, bcd_tmp;
  logic  [7:0] bcd_res;
  logic        bcd_c;

  assign bcd_sub = (`UF(ALU) == rd68031_ucode_pkg::U_ALU_SBCD);
  assign bcd_a   = a_bus[7:0];
  assign bcd_b   = b_bus[7:0];
  assign bcd_x   = sr_q[rd68031_pkg::SR_X];

  always_comb begin
    if (!bcd_sub) begin
      bcd_lo  = {1'b0, bcd_a[3:0]} + {1'b0, bcd_b[3:0]} + {4'd0, bcd_x};
      bcd_six = (bcd_lo > 5'd9);
      bcd_raw = {2'd0, bcd_a} + {2'd0, bcd_b} + {9'd0, bcd_x};
      bcd_tmp = bcd_raw + (bcd_six ? 10'd6 : 10'd0);
      bcd_c   = (bcd_tmp > 10'h099);
      bcd_res = bcd_c ? (bcd_tmp[7:0] - 8'hA0) : bcd_tmp[7:0];
    end else begin
      bcd_lo  = {1'b0, bcd_a[3:0]} - {1'b0, bcd_b[3:0]} - {4'd0, bcd_x};
      bcd_six = bcd_lo[4];                      // the low digit borrowed
      bcd_raw = {2'd0, bcd_a} - {2'd0, bcd_b} - {9'd0, bcd_x};
      bcd_tmp = bcd_raw - (bcd_six ? 10'd6 : 10'd0);
      // Below zero -- which shows as the top bits of a ten-bit two's complement
      // -- or above $99. Either is a decimal borrow.
      bcd_c   = bcd_tmp[9] || (bcd_tmp[8:0] > 9'h099);
      bcd_res = bcd_c ? (bcd_tmp[7:0] + 8'hA0) : bcd_tmp[7:0];
    end
  end

  // ==========================================================================
  // The multiplier and the divider -- PRM 4
  //
  // The signedness and the register numbers of the WORD forms are in the
  // opcode; of the LONG forms, in the extension word. `mdext` says which, and
  // the extension word has been latched into xw by then.
  //
  //     MULU.W  <ea>,Dn      16 x 16 -> 32
  //     MULU.L  <ea>,Dl      32 x 32 -> 32, V set if the product did not fit
  //     MULU.L  <ea>,Dh:Dl   32 x 32 -> 64
  //
  // One multiplier serves all of them: the operands are widened to thirty-two
  // bits according to the size and the product taken at sixty-four.
  // ==========================================================================
  logic md_signed;
  assign md_signed = `UF(MDEXT) ? xw_q[11] : stg_d[8];

  logic signed [32:0] mul_a, mul_b;
  logic signed [65:0] mul_full66;

  // The operands are T0 and T1, not the A and B buses. They cannot be the
  // buses: MULLO and MULHI are A-bus SOURCES, so a multiplier fed from the A
  // bus closes a combinational loop through the source mux. It is a false loop
  // -- only one arm is ever selected -- but it is a real one structurally, and
  // the linter is right to refuse it.
  // Always thirty-two by thirty-two. The word forms reach it with their operands
  // ALREADY widened -- the microcode does that with alu = XSZ, which extends the
  // way the instruction's own signedness says -- so the multiplier needs no size
  // input and a 16-by-16 product simply has a top half that is the sign
  // extension of its bottom, which is what makes the overflow test below work
  // for both forms without a special case.
  always_comb begin
    mul_a = md_signed ? {t_q[0][31], t_q[0]} : {1'b0, t_q[0]};
    mul_b = md_signed ? {t_q[1][31], t_q[1]} : {1'b0, t_q[1]};
  end

  assign mul_full66 = mul_a * mul_b;
  assign mul_full   = mul_full66[63:0];

  // PRM 4: MULx.L into a single register sets V when the product does not fit
  // in thirty-two bits -- which for a signed product means the top half is not
  // the sign extension of the bottom, and for an unsigned one that it is not
  // zero.
  logic mul_ovf;
  assign mul_ovf = md_signed ? (mul_full[63:32] != {32{mul_full[31]}})
                             : (mul_full[63:32] != 32'd0);

  logic        div_start, div_busy, div_zero, div_ovf_wide, div_qneg;
  logic [31:0] div_qmag;
  logic [63:0] div_num;

  // The dividend. A word form divides a long word; the long forms divide a long
  // word or a quad word, and the microcode has put the halves in T2 and T3.
  // The dividend and the divisor are registers for the same reason: DIVQ and
  // DIVR are A-bus sources. A word form divides the long word in T2; the long
  // forms divide the quad word in T3:T2.
  assign div_num = {t_q[3], t_q[2]};


  rd68031_divider u_divider (
      .clk (clk), .rst_n (core_rst_n),
      .start (div_start), .dividend (div_num), .divisor (t_q[1]),
      .is_signed (md_signed),
      .busy (div_busy), .quotient (div_q), .remainder (div_r),
      .div_zero (div_zero), .overflow (div_ovf_wide),
      .q_mag (div_qmag), .q_neg (div_qneg));

  // Whether the quotient fits where it is going. The divider only knows about
  // thirty-two bits; a word form has sixteen to put it in, and a signed long
  // has thirty-one and a sign.
  logic div_fit_ovf, div_ovf;
  always_comb begin
    if (eff_size == rd68031_ucode_pkg::U_SIZE_WORD) begin
      if (md_signed)
        // -32768 is representable and 32768 is not, which is why the negative
        // case is allowed one more.
        div_fit_ovf = div_qneg ? (div_qmag > 32'h0000_8000)
                               : (div_qmag > 32'h0000_7FFF);
      else
        div_fit_ovf = (div_qmag > 32'h0000_FFFF);
    end else if (md_signed) begin
      div_fit_ovf = div_qneg ? (div_qmag > 32'h8000_0000)
                             : (div_qmag > 32'h7FFF_FFFF);
    end else begin
      div_fit_ovf = 1'b0;
    end
  end
  assign div_ovf = div_ovf_wide | div_fit_ovf;

  // ==========================================================================
  // The adder
  //
  // ONE 33-bit adder serves ADD, ADDX, SUB, SUBX, CMP, NEG and NEGX at all
  // three sizes. Two things make that possible.
  //
  // Subtraction is a + ~b + 1. The carry out then means "no borrow", so C is
  // its complement, and the same overflow expression works for both.
  //
  // The byte and word carries and every overflow come out of the carry CHAIN
  // rather than out of two more adders: in a ripple adder the carry into bit n
  // is sum[n] ^ a[n] ^ b[n], so the carry out of bit 7 is sum[8]^a[8]^b[8], and
  // overflow at a width is the carry out of the sign bit exclusive-ored with
  // the carry into it. Three adders would have been the obvious way to write
  // this and would have cost three times the logic on the path that already
  // sets the clock.
  // ==========================================================================
  logic        alu_sub, alu_usex;
  logic [31:0] add_a, add_b;
  logic        add_cin;
  logic [32:0] add_sum;
  logic        cin7, cin15, cin31, cout7, cout15, cout31;
  logic        alu_cout, alu_v, alu_c;

  assign alu_sub  = (`UF(ALU) == rd68031_ucode_pkg::U_ALU_SUB)
                 || (`UF(ALU) == rd68031_ucode_pkg::U_ALU_SUBX);
  assign alu_usex = (`UF(ALU) == rd68031_ucode_pkg::U_ALU_ADDX)
                 || (`UF(ALU) == rd68031_ucode_pkg::U_ALU_SUBX);

  assign add_a   = a_bus;
  assign add_b   = alu_sub ? ~b_bus : b_bus;
  assign add_cin = alu_sub ? ~(alu_usex & sr_q[rd68031_pkg::SR_X])
                           :  (alu_usex & sr_q[rd68031_pkg::SR_X]);
  assign add_sum = {1'b0, add_a} + {1'b0, add_b} + {32'd0, add_cin};

  assign cin7   = add_sum[7]  ^ add_a[7]  ^ add_b[7];
  assign cout7  = add_sum[8]  ^ add_a[8]  ^ add_b[8];
  assign cin15  = add_sum[15] ^ add_a[15] ^ add_b[15];
  assign cout15 = add_sum[16] ^ add_a[16] ^ add_b[16];
  assign cin31  = add_sum[31] ^ add_a[31] ^ add_b[31];
  assign cout31 = add_sum[32];

  always_comb begin
    unique case (eff_size)
      rd68031_ucode_pkg::U_SIZE_BYTE: begin
        alu_cout = cout7;
        alu_v    = cout7 ^ cin7;
      end
      rd68031_ucode_pkg::U_SIZE_WORD: begin
        alu_cout = cout15;
        alu_v    = cout15 ^ cin15;
      end
      default: begin
        alu_cout = cout31;
        alu_v    = cout31 ^ cin31;
      end
    endcase
  end

  assign alu_c = alu_sub ? ~alu_cout : alu_cout;

  always_comb begin
    unique case (`UF(ALU))
      rd68031_ucode_pkg::U_ALU_A:   y = a_bus;
      rd68031_ucode_pkg::U_ALU_B:   y = b_bus;
      rd68031_ucode_pkg::U_ALU_ADD,
      rd68031_ucode_pkg::U_ALU_ADDX,
      rd68031_ucode_pkg::U_ALU_SUB,
      rd68031_ucode_pkg::U_ALU_SUBX: y = add_sum[31:0];
      rd68031_ucode_pkg::U_ALU_AND: y = a_bus & b_bus;
      rd68031_ucode_pkg::U_ALU_OR:  y = a_bus | b_bus;
      rd68031_ucode_pkg::U_ALU_EOR: y = a_bus ^ b_bus;
      rd68031_ucode_pkg::U_ALU_NOT: y = ~a_bus;
      rd68031_ucode_pkg::U_ALU_SWAP: y = {a_bus[15:0], a_bus[31:16]};
      rd68031_ucode_pkg::U_ALU_EXTW: y = {{24{a_bus[7]}},  a_bus[7:0]};
      rd68031_ucode_pkg::U_ALU_EXTL: y = {{16{a_bus[15]}}, a_bus[15:0]};
      rd68031_ucode_pkg::U_ALU_EXTB: y = {{24{a_bus[7]}},  a_bus[7:0]};
      rd68031_ucode_pkg::U_ALU_SHIFT:  y = sh_res;
      rd68031_ucode_pkg::U_ALU_ANDNOT: y = a_bus & ~b_bus;
      rd68031_ucode_pkg::U_ALU_LSR1:   y = {1'b0, a_bus[31:1]};
      // Widen to thirty-two bits the way the multiply or divide under way
      // says: sign extended for the signed forms, zero extended for the
      // unsigned ones. It is what lets one 32-by-32 multiplier and one 64-by-32
      // divider serve all eight instructions.
      rd68031_ucode_pkg::U_ALU_XSZ:
        unique case (eff_size)
          rd68031_ucode_pkg::U_SIZE_BYTE:
            y = md_signed ? {{24{a_bus[7]}},  a_bus[7:0]}  : {24'd0, a_bus[7:0]};
          rd68031_ucode_pkg::U_SIZE_WORD:
            y = md_signed ? {{16{a_bus[15]}}, a_bus[15:0]} : {16'd0, a_bus[15:0]};
          default: y = a_bus;
        endcase
      // The top half of that same widening. A 32-bit dividend reaches the
      // 64-bit divider as itself over this.
      rd68031_ucode_pkg::U_ALU_XSZHI: y = md_signed ? {32{a_bus[31]}} : 32'd0;
      // PRM 4: the word divide puts its remainder in the high word of the
      // destination and its quotient in the low one.
      rd68031_ucode_pkg::U_ALU_ABCD,
      rd68031_ucode_pkg::U_ALU_SBCD:    y = {24'd0, bcd_res};
      // MOVEP moves a word or a long word through every other byte of memory,
      // most significant first -- PRM 4 -- so it needs to walk a register a
      // byte at a time in each direction.
      rd68031_ucode_pkg::U_ALU_SHR8:    y = {8'd0, a_bus[31:8]};
      rd68031_ucode_pkg::U_ALU_SHL8OR:  y = {a_bus[23:0], b_bus[7:0]};
      rd68031_ucode_pkg::U_ALU_ROL8:    y = {a_bus[23:0], a_bus[31:24]};
      rd68031_ucode_pkg::U_ALU_SETB7:   y = a_bus | 32'h0000_0080;
      rd68031_ucode_pkg::U_ALU_SHL16:   y = {a_bus[15:0], 16'd0};
      rd68031_ucode_pkg::U_ALU_ORLOW16: y = {a_bus[31:16], b_bus[15:0]};
      // UM 8.1 step one, as one operation. Splitting it would leave a clock in
      // which the processor is at the supervisor level with tracing still on.
      // UM 8.1: the level replaces I2-I0 and nothing else moves.
      rd68031_ucode_pkg::U_ALU_SETMASK:
        y = (a_bus & ~(32'd7 << rd68031_pkg::SR_I0))
            | ({29'd0, b_bus[2:0]} << rd68031_pkg::SR_I0);
      rd68031_ucode_pkg::U_ALU_EXCSR:
        y = (a_bus | (32'd1 << rd68031_pkg::SR_S))
            & ~((32'd1 << rd68031_pkg::SR_T1) | (32'd1 << rd68031_pkg::SR_T0));
      // UM 8.1.9: clearing M is what moves the active supervisor stack from the
      // master one to the interrupt one, mid-exception.
      rd68031_ucode_pkg::U_ALU_CLRM:
        y = a_bus & ~(32'd1 << rd68031_pkg::SR_M);
      // ... and the throwaway frame's copy of the status register is the one
      // already stacked "except that the S-bit is set".
      rd68031_ucode_pkg::U_ALU_SETS:
        y = a_bus | (32'd1 << rd68031_pkg::SR_S);
      // PRM 4. Two bit shuffles and no arithmetic: the adjustment is added by
      // an ordinary ADD in the microword before, because the manual adds it to
      // the value BEFORE the nibbles are taken out for PACK and AFTER they are
      // spread out for UNPK, and those are different microwords either way.
      rd68031_ucode_pkg::U_ALU_PACK:
        y = {24'd0, a_bus[11:8], a_bus[3:0]};
      rd68031_ucode_pkg::U_ALU_UNPK:
        y = {16'd0, 4'd0, a_bus[7:4], 4'd0, a_bus[3:0]};
      rd68031_ucode_pkg::U_ALU_BYTEPAIR:
        y = {16'd0, a_bus[7:0], b_bus[7:0]};
      rd68031_ucode_pkg::U_ALU_SX:
        unique case (eff_size)
          rd68031_ucode_pkg::U_SIZE_BYTE: y = {{24{a_bus[7]}},  a_bus[7:0]};
          rd68031_ucode_pkg::U_SIZE_WORD: y = {{16{a_bus[15]}}, a_bus[15:0]};
          default:                        y = a_bus;
        endcase
      default:                      y = a_bus;
    endcase
  end

  // The result as the destination size sees it. PRM 3: a byte or word result
  // sets N and Z from that width, and a write to a data register leaves the rest
  // of the register alone.
  logic        res_n, res_z;
  always_comb begin
    unique case (eff_size)
      rd68031_ucode_pkg::U_SIZE_BYTE: begin
        res_n = y[7];
        res_z = (y[7:0] == 8'd0);
      end
      rd68031_ucode_pkg::U_SIZE_WORD: begin
        res_n = y[15];
        res_z = (y[15:0] == 16'd0);
      end
      default: begin
        res_n = y[31];
        res_z = (y == 32'd0);
      end
    endcase
  end

  // ==========================================================================
  // Running the divider
  //
  // A microword with mdop = DIV starts it on its first clock and stalls until
  // it is done, which is the same shape as a bus request and for the same
  // reason: the sequencer does not count clocks.
  // ==========================================================================
  logic div_req, div_go_q, div_fin_q;
  assign div_req = (`UF(MDOP) == rd68031_ucode_pkg::U_MDOP_DIV);

  assign div_start = div_req && !div_go_q && !div_fin_q;

  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      div_go_q  <= 1'b0;
      div_fin_q <= 1'b0;
    end else if (!div_req) begin
      div_go_q  <= 1'b0;
      div_fin_q <= 1'b0;
    end else if (div_start) begin
      div_go_q  <= 1'b1;
    end else if (div_go_q && !div_busy) begin
      div_go_q  <= 1'b0;
      div_fin_q <= 1'b1;
    end
  end

  logic div_stall;
  assign div_stall = div_req && !div_fin_q;


  // ==========================================================================
  // Interrupts -- UM 8.1.9 and figures 8-2 and 8-3
  //
  // IPL2-IPL0 are active low and carry the level inverted: all negated is level
  // zero, and the pins reading 001 mean level 6. The bus unit has already
  // synchronised them on consecutive falling edges, which is UM 8.1.9's "an
  // interrupt request that is the same for two consecutive falling clock edges
  // is considered a valid input".
  //
  // An interrupt becomes pending when its level is GREATER than the mask --
  // "the value in the interrupt mask is the highest priority level that the
  // processor ignores" -- or on a TRANSITION to level 7, which no mask value
  // can inhibit. Level 7 is therefore both level- and edge-sensitive, and
  // figure 6-3 spells out the difference: at level 6 a request lowered and
  // raised again is not a second interrupt, and at level 7 it is.
  // ==========================================================================
  logic [2:0] irq_level;
  logic [2:0] irq_prev_q;
  // The level this interrupt is being taken at, latched when the sequencer
  // dispatches to the handler. UM 8.1.9 requires the device to hold IPL until
  // the acknowledge, but the mask and the acknowledge address must both come
  // from ONE reading of the pins and not from two.
  logic       irq_nmi_edge;
  logic       irq_pending;
  logic       irq_ipend;

  assign irq_level = ~ipl_sync_n;

  // A transition TO level 7, which is what makes a second non-maskable
  // interrupt happen where a second level-6 one would not.
  assign irq_nmi_edge = (irq_level == 3'd7) && (irq_prev_q != 3'd7);

  // What the sequencer acts on at an instruction boundary, judged against the
  // mask the instruction now retiring may itself be writing.
  assign irq_pending = (irq_level > sr_eff[rd68031_pkg::SR_I0 +: 3])
                    || irq_nmi_edge;

  // ... and what the pin says, which is the same comparison against the
  // register as it stands. The pin is a statement about the outside world and
  // nothing is timed off it, so it is kept off the arithmetic result sr_eff
  // carries: UM 5.8.2 asks only that IPEND reflect a request the mask admits.
  assign irq_ipend   = (irq_level > sr_q[rd68031_pkg::SR_I0 +: 3])
                    || irq_nmi_edge;

  // UM 8.1.9: IPEND "signals to external devices that an interrupt exception
  // will be taken at an upcoming instruction boundary". It is a pin and is
  // never three-stated.
  //
  // A RISING-edge register. MC68030EC specification 6 times IPEND from "Clock
  // High", with the address, and figure 8 draws it there. The comparison it
  // registers reads the level from the input synchroniser, a falling-edge
  // register, and the mask from sr_q, a rising-edge one, so as a combinational
  // output it moved on both edges -- and, being a comparison of several bits
  // of each, could glitch on any edge that changed two of them, which is the
  // decode doc/coding-standard.md forbids on a pin (doc/bugs-found.md, M3).
  // Registered, it is one rising edge later than the comparison, which is
  // still before the boundary the sequencer acts at: the sequencer reads
  // irq_pending, not the pin.
  //
  // And it stays asserted until the acknowledge starts. UM 8.1.9: "if no
  // higher priority interrupt has been synchronized, the IPEND signal is
  // negated during state 0 (S0) of an interrupt acknowledge cycle". The
  // exception raises the mask, and drops the level-7 edge, a few microwords
  // before the acknowledge runs -- either would negate the comparison early
  // -- so iack_due_q holds the pin from the step into the interrupt entry to
  // the rising edge that starts the acknowledge's S0 (req_start). A fault
  // landing in between (a posted write's, doc/checkpoint.md rule 9) takes the
  // acknowledge out of this exception processing, and the hold with it.
  logic ipend_q;
  logic iack_due_q;
  logic iack_now;
  assign iack_now = req_start
                 && (`UF(CPUSPACE) == rd68031_ucode_pkg::U_CPUSPACE_IACK);
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      ipend_q    <= 1'b0;
      iack_due_q <= 1'b0;
    end else begin
      ipend_q    <= irq_ipend || (iack_due_q && !iack_now);
      if (fault_now || iack_now) iack_due_q <= 1'b0;
      else if (irq_enter)        iack_due_q <= 1'b1;
    end
  end

  assign ipend_n_o = ~ipend_q;

  // ==========================================================================
  // Tracing -- UM 8.1.7 and Table 8-3
  //
  //   T1 T0   what is traced
  //    0  0   nothing
  //    0  1   instructions that change the flow
  //    1  0   every instruction
  //    1  1   undefined, reserved
  //
  // "The state of these bits WHEN AN INSTRUCTION BEGINS EXECUTION determines
  // whether the instruction generates a trace exception after the instruction
  // completes." So the mode is latched at the boundary that starts each
  // instruction, and the instruction's own writes to the status register --
  // which is one of the things that counts as a change of flow -- cannot
  // change whether it is traced.
  //
  // A change of flow is a pipe FLUSH, or a write to the status register. The
  // manual counts the second because a real part "must re-prefetch instruction
  // words to fill the pipe again any time an instruction that can modify the
  // SR is executed"; this design does not have to, so it says so explicitly
  // rather than getting it for free.
  // ==========================================================================
  logic [1:0] trace_mode_q;
  logic       flow_q;
  logic       notrace_q;
  logic        pc_kept_q;   // ... and it was taken early, before a flush

  // ==========================================================================
  // The fault frame -- doc/ssw.md
  // ==========================================================================
  // The exception's own frame pointer. A bus fault is taken mid-instruction, so
  // the frame cannot be built with the address buffer the instruction is using:
  // ea_q is frame field +$38 and has to still be in it when RTE reads it back.

  // "A data fault has occurred and caused the exception." Set when the bus unit
  // reports a faulted operand, and cleared at every instruction boundary, so a
  // fault taken at a boundary -- which is a prefetch fault and never a data one
  // -- reports it clear. It needs no home in the frame beyond the SSW bit: a
  // second fault between this one and the finished frame is a double bus fault
  // and the processor halts, so there is no window in which it must survive.
  logic        df_q;

  // UM 8.1.2: "if a bus error occurs during the exception processing for a bus
  // error, address error, or reset ... a double bus fault occurs and the
  // processor enters the halted state". This is the window.
  logic        g0_q;

  // Which of the two group-0 vectors this fault takes: 2 for a bus error, 3 for
  // an address error -- UM 8.1.2 and 8.1.3. One frame builder serves both, and
  // this is the only thing that differs between them.


  // The faulted microword's own address. By the time the builder writes frame
  // field +$14 its own micro-address is deep inside itself, and the fault entry
  // replaced `upc` on the very edge the fault was reported, so this is the only
  // moment the value exists.

  // What RTE has read back so far. The special status word arrives several
  // microwords before the pipe's flags can be written, because the pipe needs
  // stage D's fault bit out of the OTHER packed word; these two hold the halves
  // until they can be put together. Neither survives the instruction.
  // doc/ssw.md: the word is assembled when a frame is built and taken apart
  // when RTE reads one back, and is never stored. These are the bits of it this
  // core acts on; the reserved ones and SIZE are not among them, because the
  // residual byte count comes out of the internal word, which can say five and
  // UM Table 7-2's two-bit field cannot.
  logic        rs_rc_q, rs_rb_q, rs_dv_q;
  logic        rs_df_q, rs_rm_q, rs_rw_q;
  logic  [2:0] rs_space_q;
  // ... and the micro-address to resume at.
  logic [rd68031_ucode_pkg::UADDR-1:0] rupc_q;

  // UM figure 8-9. Two halves about two different things, assembled here and
  // never stored: the pipe owns FC, FB, RC and RB, and the bus unit owns the
  // rest. UM 8.2.1: "the least significant half of the SSW applies to data
  // cycles only", so with no data fault it reads as zero rather than as the
  // leavings of the last one. Bits 11-9 and 3 are the MC68030's "for internal
  // use only"; this design keeps nothing there and writes them as zero
  // (rd68031_frame_pkg::SSW_INTERNAL, doc/ssw.md).
  always_comb begin
    ssw = 16'h0000;
    ssw[rd68031_frame_pkg::SSW_FC] = stg_c_fault;
    ssw[rd68031_frame_pkg::SSW_FB] = stg_b_fault;
    ssw[rd68031_frame_pkg::SSW_RC] = stg_c_rerun;
    ssw[rd68031_frame_pkg::SSW_RB] = stg_b_rerun;
    ssw[rd68031_frame_pkg::SSW_DF] = df_q;
    ssw[rd68031_frame_pkg::SSW_RM] = df_q & flt_rmc;
    ssw[rd68031_frame_pkg::SSW_RW] = df_q & flt_rw;
    ssw[rd68031_frame_pkg::SSW_SIZE_HI:rd68031_frame_pkg::SSW_SIZE_LO]
      = df_q ? flt_siz : 2'b00;
    ssw[rd68031_frame_pkg::SSW_FCODE_HI:rd68031_frame_pkg::SSW_FCODE_LO]
      = df_q ? flt_fc : 3'b000;
  end

  // UM Table 7-2 encodes the size of a transfer in two bits, and cannot say
  // five. Five is what a bit-field operand spanning five bytes leaves, so the
  // residual goes into the frame's internal word as well and SIZE carries what
  // it can -- doc/ssw.md.
  assign flt_siz = (flt_bytes >= 3'd4) ? 2'b00 : flt_bytes[1:0];

  // The two packed internal words of the fault frames. Every bit of them is a
  // row of doc/checkpoint.md, and rd68031_frame_pkg names the positions, so
  // they are written here in exactly the order that table is printed in.
  always_comb begin
    int08 = 16'd0;
    int08[rd68031_frame_pkg::I_BYTES_LO      +: 3] = flt_bytes;
    int08[rd68031_frame_pkg::I_NOTRACE_LO    +: 1] = notrace_q;
    int08[rd68031_frame_pkg::I_EAPC_LO       +: 1] = eapc_q;
    int08[rd68031_frame_pkg::I_OPSIZE_LO     +: 2] = size_q;
    int08[rd68031_frame_pkg::I_EADST_LO      +: 1] = eadst_q;
    int08[rd68031_frame_pkg::I_DVALID_LO     +: 1] = pf_dvalid;
    int08[rd68031_frame_pkg::I_POSTED_LO     +: 1] = post_flt_q;
    int08[rd68031_frame_pkg::I_IRQLVL_LO     +: 3] = irq_taking_q;
  end

  always_comb begin
    int36 = 16'd0;
    int36[rd68031_frame_pkg::I_TRMODE_LO     +: 2] = trace_mode_q;
    int36[rd68031_frame_pkg::I_FLOW_LO       +: 1] = flow_q;
    int36[rd68031_frame_pkg::I_PC_KEPT_LO    +: 1] = pc_kept_q;
    // UM 8.1.13 by way of doc/checkpoint.md: the version nibble is what makes
    // a private encoding of the internal words legitimate, because RTE refuses
    // any frame that does not carry this one.
    int36[rd68031_frame_pkg::VERSION_LO      +: 4] = rd68031_frame_pkg::FRAME_VERSION;
  end
  logic       trace_take;
  logic       flow_eff;

  // The same one-microword problem sr_eff has: MOVE #imm,SR writes the status
  // register on the microword that decodes, so the change of flow it is has to
  // count at that very boundary. flow_q alone is one instruction late, and
  // since the DECODE arm also clears it, one instruction late means never.
  assign flow_eff = flow_q
                 || (retire && ((`UF(PF) == rd68031_ucode_pkg::U_PF_FLUSH)
                                || (`UF(DST) == rd68031_ucode_pkg::U_DST_SR)
                                || (`UF(DST) == rd68031_ucode_pkg::U_DST_SCANPC)));

  // The coprocessor midinstruction frame's internal word at +$0C -- UM figure
  // 10-43. A coprocessor instruction interrupted in its dialogue resumes it
  // after RTE, so what it had decided about tracing has to come back with it:
  // UM 8.1.7 fixes the trace mode at the start of the instruction, and whether
  // it has changed the flow yet is what trace-on-change-of-flow asks.
  assign cp_int = {11'd0, notrace_q, pc_kept_q, flow_q, trace_mode_q};

  // UM table 8-3: T1 T0 = 10 traces every instruction, 01 a change of flow,
  // and 11 is "undefined, reserved". This core takes 11 as 10 -- T1 decides,
  // T0 matters only without it -- which is what the cputest corpus, generated
  // by a tester validated against real processors, records the part doing
  // (doc/divergences.md). RD68021 traced nothing under 11.
  //
  // Not at the end of exception processing for a bus error or an address
  // error, which is the boundary g0_q is still set at: UM 8.1.7, "if an
  // instruction does not complete due to a bus error or address error
  // exception, trace exception processing is deferred until after the
  // execution of the suspended instruction is resumed and the instruction
  // execution completes normally". The trace mode and the cancel bit go into
  // the frame as they were (+$36, +$08), and the RTE that resumes the
  // instruction brings them back for the boundary that ends it. An exception
  // an instruction forces as part of its own execution -- TRAP, CHK, a zero
  // divide -- is not one of these, and is traced at the end of its exception
  // processing (UM 8.1.12).
  assign trace_take = !notrace_q && !g0_q
                   && (trace_mode_q[1]
                       || ((trace_mode_q == 2'b01) && flow_eff));

  // The trace that holds a general coprocessor instruction's dialogue open --
  // UM 10.5.2.5: "when a trace exception is pending ... the main processor
  // continues to read the response CIR and to service the primitives until it
  // receives either a null, CA=0, PF=1 primitive" -- is the trace on
  // instruction execution, pending from the instruction's start. Not the trace
  // on change of flow, which the dialogue itself makes pending: "a trace
  // exception is taken for the instruction only when the coprocessor issues a
  // transfer status register and scanPC primitive with DR = 1 ... In this
  // case, it is possible that the coprocessor is still executing the cpGEN
  // instruction concurrently when the main processor begins execution of the
  // trace exception handler" -- which it could not be if that trace held the
  // dialogue until PF = 1. That trace is still taken at the boundary, by
  // trace_take; it only does not keep the processor reading.
  logic trace_hold;
  assign trace_hold = !notrace_q && !g0_q && trace_mode_q[1];

  // ==========================================================================
  // The addressing-mode classes -- PRM 2.2 and table 2-4, for the coprocessor
  // primitives that evaluate the effective address in the operation word and
  // must first check it is one the primitive allows (UM table 10-4).
  // ==========================================================================
  logic ea_dn, ea_an, ea_ind, ea_post, ea_pre, ea_d16, ea_idx, ea_abs;
  logic ea_pcrel, ea_imm, ea_valid;
  logic ea_ctl, ea_ctlalt, ea_dataalt, ea_memalt, ea_alt, ea_data, ea_mem;
  logic cp_ea_ok;
  assign ea_dn    = (stg_d[5:3] == 3'b000);
  assign ea_an    = (stg_d[5:3] == 3'b001);
  assign ea_ind   = (stg_d[5:3] == 3'b010);
  assign ea_post  = (stg_d[5:3] == 3'b011);
  assign ea_pre   = (stg_d[5:3] == 3'b100);
  assign ea_d16   = (stg_d[5:3] == 3'b101);
  assign ea_idx   = (stg_d[5:3] == 3'b110);
  assign ea_abs   = (stg_d[5:3] == 3'b111) && (stg_d[2:1] == 2'b00);
  assign ea_pcrel = (stg_d[5:3] == 3'b111) && (stg_d[2:1] == 2'b01);
  assign ea_imm   = (stg_d[5:0] == 6'b111100);
  assign ea_valid = !((stg_d[5:3] == 3'b111) && (stg_d[2:0] > 3'b100));
  assign ea_ctlalt  = ea_ind || ea_d16 || ea_idx || ea_abs;
  assign ea_ctl     = ea_ctlalt || ea_pcrel;
  assign ea_memalt  = ea_ctlalt || ea_post || ea_pre;
  assign ea_dataalt = ea_memalt || ea_dn;
  assign ea_alt     = ea_dataalt || ea_an;
  assign ea_mem     = ea_valid && !ea_dn && !ea_an;
  assign ea_data    = ea_valid && !ea_an;

  // UM table 10-4, the valid-EA field of the evaluate-and-transfer-data
  // primitive, bits 10:8.
  always_comb begin
    unique case (cprim_q[10:8])
      3'b000:  cp_ea_ok = ea_ctlalt;
      3'b001:  cp_ea_ok = ea_dataalt;
      3'b010:  cp_ea_ok = ea_memalt;
      3'b011:  cp_ea_ok = ea_alt;
      3'b100:  cp_ea_ok = ea_ctl;
      3'b101:  cp_ea_ok = ea_data;
      3'b110:  cp_ea_ok = ea_mem;
      default: cp_ea_ok = ea_valid;
    endcase
  end

  // ... and for the primitive decoder, so that a primitive that takes an
  // effective address goes straight to the handler for the kind it is, rather
  // than asking microword by microword: in the class (table 7-4); suitable for
  // multiple coprocessor registers (UM 10.4.16: to the coprocessor control or
  // (An)+, from it control alterable or -(An)); and which kind. Everything
  // here is a function of stage D and the primitive, both registers.
  logic       cp_mea_ok;
  logic [2:0] cp_eakind;
  assign cp_mea_ok = cprim_q[13] ? (ea_ctlalt || ea_pre) : (ea_ctl || ea_post);
  always_comb begin
    if      (ea_dn)   cp_eakind = 3'd0;
    else if (ea_an)   cp_eakind = 3'd1;
    else if (ea_imm)  cp_eakind = 3'd2;
    else if (ea_post) cp_eakind = 3'd3;
    else if (ea_pre)  cp_eakind = 3'd4;
    else              cp_eakind = 3'd5;
  end
  assign cp_ea = {cp_ea_ok, cp_mea_ok, cp_eakind};

  // ==========================================================================
  // The conditional tests -- PRM table 3-19
  //
  // Sixteen conditions on four flags, written out rather than factored: the
  // table is the specification, and every attempt to be clever about it loses
  // the ability to check it line by line against the manual.
  // ==========================================================================
  logic flag_n, flag_z, flag_v, flag_c;
  assign flag_n = sr_q[rd68031_pkg::SR_N];
  assign flag_z = sr_q[rd68031_pkg::SR_Z];
  assign flag_v = sr_q[rd68031_pkg::SR_V];
  assign flag_c = sr_q[rd68031_pkg::SR_C];

  // CHK's two tests, on the register and the bound themselves: the register is
  // negative, and the register is greater than the bound, signed, at the size
  // CHK names. See U_COND_RESNEG and U_COND_GTZ below.
  logic chk_neg, chk_gt;
  always_comb begin
    if (eff_size == rd68031_ucode_pkg::U_SIZE_WORD) begin
      chk_neg = dreg[wsel][15];
      chk_gt  = $signed(dreg[wsel][15:0]) > $signed(t_q[1][15:0]);
    end else begin
      chk_neg = dreg[wsel][31];
      chk_gt  = $signed(dreg[wsel]) > $signed(t_q[1]);
    end
  end

  logic cc_true;
  always_comb begin
    unique case (stg_d[11:8])
      4'b0000: cc_true = 1'b1;                          // T
      4'b0001: cc_true = 1'b0;                          // F
      4'b0010: cc_true = ~flag_c & ~flag_z;             // HI
      4'b0011: cc_true =  flag_c |  flag_z;             // LS
      4'b0100: cc_true = ~flag_c;                       // CC / HS
      4'b0101: cc_true =  flag_c;                       // CS / LO
      4'b0110: cc_true = ~flag_z;                       // NE
      4'b0111: cc_true =  flag_z;                       // EQ
      4'b1000: cc_true = ~flag_v;                       // VC
      4'b1001: cc_true =  flag_v;                       // VS
      4'b1010: cc_true = ~flag_n;                       // PL
      4'b1011: cc_true =  flag_n;                       // MI
      4'b1100: cc_true = ~(flag_n ^ flag_v);            // GE
      4'b1101: cc_true =  (flag_n ^ flag_v);            // LT
      4'b1110: cc_true = ~(flag_n ^ flag_v) & ~flag_z;  // GT
      default: cc_true =  (flag_n ^ flag_v) |  flag_z;  // LE
    endcase
  end

  logic cond_true;
  always_comb begin
    unique case (`UF(COND))
      rd68031_ucode_pkg::U_COND_CC:    cond_true = cc_true;
      rd68031_ucode_pkg::U_COND_NCC:   cond_true = ~cc_true;
      // The three conditions on a microword's own result are taken from the
      // registers it reads, not from the result bus: DBcc's counter minus one is
      // $FFFF exactly when the counter was zero, and CHK's two tests are a sign
      // and a signed comparison. assemble.py's check_live_shape holds each to
      // that one shape, and with it no result -- the shifter's, the bit-field
      // unit's -- has a path into the next micro-address (doc/critical-path.md).
      rd68031_ucode_pkg::U_COND_RESM1: cond_true = (dreg[rsel][15:0] == 16'h0000);
      rd68031_ucode_pkg::U_COND_EMPTY:    cond_true = (t_q[0][15:0] == 16'd0);
      rd68031_ucode_pkg::U_COND_NOTEMPTY: cond_true = (t_q[0][15:0] != 16'd0);
      // Bit 10 of the extension word: the long forms' 64-bit selector.
      rd68031_ucode_pkg::U_COND_XW10:  cond_true = xw_q[10];
      // RTE's format word -- UM 8.4 names the MC68030's six. Its only source
      // is read data, which is what doc/checkpoint.md's rule on bus-steering
      // conditions admits.
      rd68031_ucode_pkg::U_COND_FMT0:  cond_true = (xw_q[15:12] == 4'h0);
      rd68031_ucode_pkg::U_COND_FMT1:  cond_true = (xw_q[15:12] == 4'h1);
      rd68031_ucode_pkg::U_COND_FMT2:  cond_true = (xw_q[15:12] == 4'h2);
      rd68031_ucode_pkg::U_COND_FMTA:  cond_true = (xw_q[15:12] == 4'hA);
      rd68031_ucode_pkg::U_COND_FMTB:  cond_true = (xw_q[15:12] == 4'hB);
      rd68031_ucode_pkg::U_COND_FMT9:  cond_true = (xw_q[15:12] == 4'h9);
      // UM 8.1.8: a long frame whose version number is not this design's is a
      // format error. The word at +$36 is in xw.
      rd68031_ucode_pkg::U_COND_VERBAD:
        cond_true = (xw_q[rd68031_frame_pkg::VERSION_HI:rd68031_frame_pkg::VERSION_LO]
                     != rd68031_frame_pkg::FRAME_VERSION);
      // UM 8.2.2: "the only bits in the SSW that may be modified are DF, RB, and
      // RC", so these three are the whole of what a handler can tell RTE, and
      // doc/checkpoint.md's rule on bus-steering conditions admits them: every
      // one is a single bit of a register whose only source is read data.
      rd68031_ucode_pkg::U_COND_SSW_DF: cond_true = rs_df_q;
      rd68031_ucode_pkg::U_COND_SSW_RB: cond_true = rs_rb_q;
      rd68031_ucode_pkg::U_COND_SSW_RC: cond_true = rs_rc_q;
      rd68031_ucode_pkg::U_COND_SSW_RW: cond_true = rs_rw_q;
      rd68031_ucode_pkg::U_COND_SSW_RM: cond_true = rs_rm_q;
      rd68031_ucode_pkg::U_COND_MASTER: cond_true = master_mode;
      rd68031_ucode_pkg::U_COND_USER:  cond_true = ~super_mode;
      rd68031_ucode_pkg::U_COND_DIVZERO: cond_true = div_zero;
      rd68031_ucode_pkg::U_COND_ZSET:    cond_true = flag_z;
      rd68031_ucode_pkg::U_COND_CSET:    cond_true = flag_c;
      rd68031_ucode_pkg::U_COND_VSET:    cond_true = flag_v;
      // PRM 4: the extension word of CMP2 and CHK2 says which of the two this
      // is, and which register it checks.
      rd68031_ucode_pkg::U_COND_XW11:    cond_true = xw_q[11];
      rd68031_ucode_pkg::U_COND_XW15:    cond_true = xw_q[15];
      rd68031_ucode_pkg::U_COND_AVEC:
        cond_true = (req_end == rd68031_pkg::CE_AVEC);
      rd68031_ucode_pkg::U_COND_BERR:
        cond_true = (req_end == rd68031_pkg::CE_BERR);
      // Tested on the result the CURRENT microword is computing, not on the
      // status register, which it has not written yet.
      rd68031_ucode_pkg::U_COND_RESNEG: cond_true = chk_neg;
      rd68031_ucode_pkg::U_COND_GTZ:    cond_true = chk_gt;
      rd68031_ucode_pkg::U_COND_MDOVF: cond_true =
          (`UF(MDOP) == rd68031_ucode_pkg::U_MDOP_DIV) ? div_ovf : mul_ovf;
      // ---- The coprocessor interface -- UM section 10 --------------------
      // The primitive's own bits, figure 10-22.
      rd68031_ucode_pkg::U_COND_CPCA:  cond_true = cprim_q[15];
      rd68031_ucode_pkg::U_COND_CPPC:  cond_true = cprim_q[14];
      rd68031_ucode_pkg::U_COND_CPDR:  cond_true = cprim_q[13];
      rd68031_ucode_pkg::U_COND_CPB8:  cond_true = cprim_q[8];
      rd68031_ucode_pkg::U_COND_CPPF:  cond_true = cprim_q[1];
      rd68031_ucode_pkg::U_COND_CPTF:  cond_true = cprim_q[0];
      // Which instruction -- figures 7-6 to 7-13.
      rd68031_ucode_pkg::U_COND_CPGEN:  cond_true = !cp_cond;
      rd68031_ucode_pkg::U_COND_CPBCC:  cond_true = stg_d[7];
      rd68031_ucode_pkg::U_COND_CPDBCC: cond_true = (stg_d[5:3] == 3'b001);
      rd68031_ucode_pkg::U_COND_CPTRAP: cond_true = (stg_d[5:3] == 3'b111);
      rd68031_ucode_pkg::U_COND_IR0:    cond_true = stg_d[0];
      rd68031_ucode_pkg::U_COND_IR1:    cond_true = stg_d[1];
      rd68031_ucode_pkg::U_COND_IR6:    cond_true = stg_d[6];
      // The byte counter of an operand transfer.
      rd68031_ucode_pkg::U_COND_T1GE4:  cond_true = (t_q[1][31:2] != 30'd0);
      rd68031_ucode_pkg::U_COND_T1B1:   cond_true = t_q[1][1];
      rd68031_ucode_pkg::U_COND_T1B0:   cond_true = t_q[1][0];
      rd68031_ucode_pkg::U_COND_T1ZERO: cond_true = (t_q[1] == 32'd0);
      rd68031_ucode_pkg::U_COND_TRACEPEND: cond_true = trace_hold;
      rd68031_ucode_pkg::U_COND_CPAGAIN:   cond_true = cprim_q[15] || trace_hold;
      // Against the mask as it stands: nothing in the dialogue writes it on
      // the microword that asks.
      rd68031_ucode_pkg::U_COND_IRQPEND:   cond_true = irq_ipend;
      // The effective-address field of stage D.
      rd68031_ucode_pkg::U_COND_EADN:     cond_true = ea_dn;
      rd68031_ucode_pkg::U_COND_EAAN:     cond_true = ea_an;
      rd68031_ucode_pkg::U_COND_EAPOST:   cond_true = ea_post;
      rd68031_ucode_pkg::U_COND_EAPRE:    cond_true = ea_pre;
      rd68031_ucode_pkg::U_COND_EAIMM:    cond_true = ea_imm;
      rd68031_ucode_pkg::U_COND_EAUNALT:  cond_true = ea_imm || ea_pcrel;
      rd68031_ucode_pkg::U_COND_CPEAOK:   cond_true = cp_ea_ok;
      rd68031_ucode_pkg::U_COND_EACTLALT: cond_true = ea_ctlalt;
      // UM 10.4.16: to the coprocessor, control or (An)+; from it, control
      // alterable or -(An).
      rd68031_ucode_pkg::U_COND_CPMEAOK:  cond_true = cp_mea_ok;
      rd68031_ucode_pkg::U_COND_CPLEN124:
        cond_true = (cprim_q[7:0] == 8'd1) || (cprim_q[7:0] == 8'd2)
                 || (cprim_q[7:0] == 8'd4);
      // A coprocessor format word in T0 -- UM table 10-2.
      rd68031_ucode_pkg::U_COND_FWNOTRDY: cond_true = (t_q[0][15:8] == 8'h01);
      rd68031_ucode_pkg::U_COND_FWEMPTY:  cond_true = (t_q[0][15:8] == 8'h00);
      rd68031_ucode_pkg::U_COND_FWBAD:
        cond_true = (t_q[0][15:8] >= 8'h02) && (t_q[0][15:8] <= 8'h0F);
      rd68031_ucode_pkg::U_COND_FWLEN:    cond_true = (t_q[0][1:0] != 2'b00);
      rd68031_ucode_pkg::U_COND_CREGBAD:  cond_true = creg_bad;
      rd68031_ucode_pkg::U_COND_CREGBADC: cond_true = creg_bad_c;
      // ---- The MMU instructions, UM 9.8 --------------------------------
      rd68031_ucode_pkg::U_COND_MMUBADC:  cond_true = mmu_bad_c;
      rd68031_ucode_pkg::U_COND_MMUPTEST: cond_true = (xw_q[15:13] == 3'b100);
      rd68031_ucode_pkg::U_COND_MMUPLFL:  cond_true = (xw_q[15:13] == 3'b001);
      rd68031_ucode_pkg::U_COND_MMUSRG:   cond_true = (xw_q[15:13] == 3'b011);
      rd68031_ucode_pkg::U_COND_MMUQUAD:  cond_true = (xw_q[15:13] == 3'b010) && xw_q[11];
      rd68031_ucode_pkg::U_COND_XW12:     cond_true = xw_q[12];
      rd68031_ucode_pkg::U_COND_XW9:      cond_true = xw_q[9];
      rd68031_ucode_pkg::U_COND_XW8:      cond_true = xw_q[8];
      // UM 9.7.5.3: the register just written was refused -- rd68031_mmu
      // checks the operand, which is still in T0 and T1.
      rd68031_ucode_pkg::U_COND_MMUCFG:   cond_true = mmu_cfg_bad;
      default:                         cond_true = 1'b0;
    endcase
  end

  // ==========================================================================
  // Stalling
  //
  // A microword retires when nothing it asked for is outstanding. Everything
  // else -- the datapath write, the pipe operation, the micro-address -- is
  // conditioned on that one signal, so a stalled microword has no effect at all
  // and can be re-evaluated every clock without doing anything twice.
  // ==========================================================================
  logic bus_req;
  logic needs_c;
  logic stall;
  // PRM 6 STOP. While this is set the sequencer does nothing at all: no
  // microword retires, so nothing commits, no bus request is presented and the
  // pipe does not move. UM 4: "when the processor executes a STOP instruction, it
  // is in a special type of normal processing state, one without bus cycles. It
  // is stopped, not halted".
  logic stopped_q;

  assign bus_req = (`UF(BUS) != rd68031_ucode_pkg::U_BUS_NONE);
  assign needs_c = (`UF(PF) == rd68031_ucode_pkg::U_PF_ADV)
                || (`UF(PF) == rd68031_ucode_pkg::U_PF_CONSUME)
                || (`UF(ASRC) == rd68031_ucode_pkg::U_ASRC_STG_C)
                || (`UF(ASRC) == rd68031_ucode_pkg::U_ASRC_STG_C_HI)
                || (`UF(ASRC) == rd68031_ucode_pkg::U_ASRC_PC_C)
                || (`UF(ASRC) == rd68031_ucode_pkg::U_ASRC_EABASE)
                || (`UF(BSRC) == rd68031_ucode_pkg::U_BSRC_STG_C_U)
                || (`UF(BSRC) == rd68031_ucode_pkg::U_BSRC_STG_C_S)
                || (`UF(SEQ)  == rd68031_ucode_pkg::U_SEQ_EADEC);
  // Not EAMODE: the addressing-mode decoder reads stage D alone, and every
  // effective-address routine that takes a word from stage C asks for it on its
  // own microword -- and takes its prefetch fault there. Waiting at the dispatch
  // cost a clock on every memory operand whenever the last pipe advance had left
  // the queue empty. doc/timing-divergences.md.

  // Everything a microword waits for EXCEPT the bus. The distinction matters:
  // the bus request may not be presented while any of these holds, because the
  // bus unit takes an operand the moment it can and latches its address and its
  // write data then -- so a microword that asks for a bus cycle AND reads
  // something not yet valid would send the stale value.
  //
  // JSR (d16,An) is what found it. The push reads PC_C, which is only right
  // once the pipe has refilled past the displacement the address routine ate;
  // the microword stalled for exactly that reason, and the write had already
  // gone out carrying the address of the displacement instead of the address of
  // the next instruction.
  // A posted write is outstanding and this microword may not run until it is
  // done -- either one before it, or (wait_own) its own. Assigned below, with
  // the pipe-fault terms they read.
  logic wait_prev, wait_own;
  logic other_stall;
  // An MMU instruction waits for a table search under way to finish, and PTEST
  // and PLOAD for their own -- rd68031_mmu. Assigned below.
  logic mmu_stall;
  assign other_stall = wait_prev
                    || mmu_stall
                    || div_stall
                    || dbf_q
                    || stopped_q
                    || (`UF(RSTO) && reset_busy)
                    || (needs_c && !pf_ready)
                    || ((`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_DECODE)
                        && (`UF(PF) != rd68031_ucode_pkg::U_PF_ADV)
                        && !pf_dvalid);

  // The early retire. A microword the assembler marked `early` retires on the
  // edge that ends S5, when the bus unit has seen its operand finish cleanly,
  // instead of a clock later on the registered acknowledge -- which then arrives
  // in the NEXT microword's first clock and belongs to nothing, so early_q
  // discards it. A fault never retires early: the operand ends with a bus error
  // or a retry, req_early stays low, and the ordinary handshake takes it.
  logic early_hit, early_q, bus_done;
  assign early_hit = `UF(EARLY) && req_early;
  // A posted write is done with as far as this microword is concerned when the
  // bus unit has taken it: own_q holds that across the clocks it may still wait.
  logic own_q, post_own;
  assign post_own  = `UF(POST) && (req_taken || own_q);
  assign bus_done  = `UF(POST) ? post_own
                               : ((req_ack && !early_q) || early_hit);

  assign stall = (bus_req && !bus_done) || other_stall || wait_own;

  assign retire = !stall;

  // ==========================================================================
  // The MMU -- UM 9.8, rd68031_mmu
  //
  // PTEST and PLOAD search the tables, and the microword stalls for the search
  // the way it stalls for the divider: the search starts on its first clock and
  // the microword retires a clock after it ends. No fault can land on it while
  // it waits -- it is marked to wait for posted writes first, it asks for no
  // bus cycle of its own and uses no pipe word -- so the search is never
  // abandoned half done (doc/checkpoint.md rule 11), and a bus error on one of
  // its cycles is recorded in MMUSR or the ATC entry, not taken (UM 9.5.2).
  //
  // The other MMU microwords commit on their own edge, and every one waits
  // first for a search already under way -- an instruction prefetch's or an
  // operand's -- so that the ATC and the registers a search reads are not
  // changed under it.
  // ==========================================================================
  logic mmu_walk, mmu_inst, mmu_start, mmu_go_q, mmu_fin_q;
  assign mmu_walk  = (`UF(MMU) == rd68031_ucode_pkg::U_MMU_PLOAD)
                  || (`UF(MMU) == rd68031_ucode_pkg::U_MMU_PTEST);
  assign mmu_inst  = (`UF(MMU) != rd68031_ucode_pkg::U_MMU_NONE) && !mmu_walk;
  assign mmu_start = mmu_walk && !mmu_go_q && !mmu_fin_q && !mmu_busy
                  && !wait_prev && !dbf_q && !stopped_q;

  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      mmu_go_q  <= 1'b0;
      mmu_fin_q <= 1'b0;
    end else if (!mmu_walk) begin
      mmu_go_q  <= 1'b0;
      mmu_fin_q <= 1'b0;
    end else if (mmu_start) begin
      mmu_go_q  <= 1'b1;
    end else if (mmu_go_q && mmu_done) begin
      mmu_go_q  <= 1'b0;
      mmu_fin_q <= 1'b1;
    end
  end

  assign mmu_stall = (mmu_walk && !mmu_fin_q) || (mmu_inst && mmu_busy);

  // ==========================================================================
  // Instruction-stream faults -- UM 8.1.2, 8.1.3 and doc/ssw.md
  //
  // "If the aborted bus cycle is an instruction prefetch, the processor may
  // delay taking the exception until it attempts to use the prefetched
  // information." There are two ways to attempt to use it and they are both
  // here.
  //
  // The first is to USE the word: advance it into stage D, read it as an
  // extension word, or hand it to the extension-word decoder. That microword
  // retires -- the queue has a word, it is simply a bad one -- so the fault is
  // taken on the retirement and nothing commits. Reading the ADDRESS of stage C
  // is not a use: PC_C and EABASE are right whatever the word is.
  //
  // The second is to WAIT for a word that will never come: stage D is empty and
  // the front of the queue is faulted, so nothing may be loaded into it, or the
  // fetch address is odd and no cycle will be run at all. That one is the
  // address error as well, and the pipe says which.
  //
  // Either way the frame is the long one: the microword that could not have its
  // word is re-executed after RTE, and it may depend on any working register.
  // ==========================================================================
  logic uses_c_word;
  assign uses_c_word = (`UF(PF)   == rd68031_ucode_pkg::U_PF_ADV)
                    || (`UF(ASRC) == rd68031_ucode_pkg::U_ASRC_STG_C)
                    || (`UF(ASRC) == rd68031_ucode_pkg::U_ASRC_STG_C_HI)
                    || (`UF(BSRC) == rd68031_ucode_pkg::U_BSRC_STG_C_U)
                    || (`UF(BSRC) == rd68031_ucode_pkg::U_BSRC_STG_C_S)
                    || (`UF(SEQ)  == rd68031_ucode_pkg::U_SEQ_EADEC);

  logic at_decode;
  assign at_decode = (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_DECODE);

  logic pipe_wait;
  assign pipe_wait = (needs_c && !pf_ready)
                  || (at_decode && (`UF(PF) != rd68031_ucode_pkg::U_PF_ADV)
                      && !pf_dvalid);

  // ==========================================================================
  // Posted writes -- UM 11.2.5.2, doc/checkpoint.md rule 9
  //
  // A posted write's fault is taken at whatever microword is running when it
  // arrives, and that microword is re-executed after RTE. So a microword that
  // must not be one waits for the write instead:
  //   - `sync`: it reads state a frame does not carry (assemble.py, mark_sync),
  //     or it is NOP, which PRM 4 makes wait for the bus;
  //   - one about to take a prefetch fault: the write came first, so its fault
  //     does -- and a prefetch fault taken first would make the write's a
  //     double bus fault;
  // A microword whose OWN posted write is outstanding waits too, without
  // presenting it again; a fault then is an ordinary one on its own request.
  // ==========================================================================
  // `sync` is about state the microword READS, so it waits only for a write
  // posted before it: one it posts itself has its data already. A prefetch
  // fault is about where a fault lands, so it waits for its own as well. The
  // interrupt being taken needs no wait: its level is in the frame.
  logic post_hold, own_hold, pipe_flt_raw;
  assign pipe_flt_raw = (uses_c_word && stg_c_fault) || (pipe_wait && pf_stuck);
  assign own_hold  = pipe_flt_raw;
  assign post_hold = `UF(SYNC) || own_hold;
  assign wait_prev = post_hold && post_busy && !post_own;
  assign wait_own  = own_hold  && post_busy &&  post_own;

  logic pipe_fault;
  assign pipe_fault = ((retire && uses_c_word && stg_c_fault)
                       || (pipe_wait && pf_stuck))
                   && !post_busy;

  // Every fault takes the long frame -- doc/divergences.md. In this design a
  // prefetch fault is taken by the instruction's own last microword, which RTE
  // re-executes, and that microword may read any of the working registers; the
  // short frame carries none of them. A CMPM at the end of a page lost its
  // source operand to RTE's own frame pointer, and SunOS's ps -U died of it.
  // doc/bugs-found.md.

  // Any of the three, and only the data one sets DF.
  assign fault_now = req_fault || pipe_fault;

  // doc/checkpoint.md rule 2: A FAULTED MICROWORD ENDS BUT COMMITS NOTHING --
  // no register write, no pipe advance, no address-register update, no
  // condition code. The state the frame records is therefore the state at the
  // START of the faulted access, and resuming at the saved micro-address
  // re-executes the microword and reissues exactly the same request. There is
  // no separate restart path to get wrong, which is the whole reason the rule
  // is written this way round.
  //
  // The microword still ENDS: `retire` is what the micro-address arm reads, and
  // it goes to the fault entry. Only the commits are withheld.
  logic commit;
  assign commit = retire && !fault_now;

  // ==========================================================================
  // The bus request
  // ==========================================================================
  // The fast paths' addresses: the register the effective address names, and
  // that register stepped back by the operand size for -(An), which is what the
  // same microword writes back into it.
  logic [31:0] areg_ea;
  assign areg_ea = (rsel == 3'd7) ? sp_read : areg[rsel];

  logic [31:0] req_addr_sel;
  always_comb begin
    unique case (`UF(ASEL))
      rd68031_ucode_pkg::U_ASEL_ZERO: req_addr_sel = 32'd0;
      rd68031_ucode_pkg::U_ASEL_FOUR: req_addr_sel = 32'd4;
      rd68031_ucode_pkg::U_ASEL_T0:   req_addr_sel = t_q[0];
      rd68031_ucode_pkg::U_ASEL_T1:   req_addr_sel = t_q[1];
      rd68031_ucode_pkg::U_ASEL_T2:   req_addr_sel = t_q[2];
      rd68031_ucode_pkg::U_ASEL_T3:   req_addr_sel = t_q[3];
      rd68031_ucode_pkg::U_ASEL_SP:   req_addr_sel = sp_read;
      rd68031_ucode_pkg::U_ASEL_EA:   req_addr_sel = ea_q;
      // A bus fault is taken mid-instruction, so the frame is built with a
      // pointer of its own and the instruction's address buffer is left alone:
      // it is frame field +$38 and RTE puts it back.
      rd68031_ucode_pkg::U_ASEL_EA_SAVE: req_addr_sel = ea_save;
      rd68031_ucode_pkg::U_ASEL_PC_D: req_addr_sel = pc_d;
      rd68031_ucode_pkg::U_ASEL_AREG: req_addr_sel = areg_ea;
      rd68031_ucode_pkg::U_ASEL_AREG_PRE: req_addr_sel = areg_ea - opsize_bytes;
      default:                        req_addr_sel = 32'd0;
    endcase
  end

  always_comb begin
    unique case (`UF(FC))
      rd68031_ucode_pkg::U_FC_PROG: req_fc = super_mode
                                             ? rd68031_pkg::FC_SUPER_PROG
                                             : rd68031_pkg::FC_USER_PROG;
      rd68031_ucode_pkg::U_FC_CPU:  req_fc = rd68031_pkg::FC_CPU;
      // The space of the effective address under way. PRM 2 classifies every
      // program-counter-relative access as a program reference, and the same
      // latched bit that chose the base chooses the space.
      // PRM 6: MOVES names its own space, and the two control registers are what
      // it names it with.
      rd68031_ucode_pkg::U_FC_SFC:  req_fc = sfc_q[2:0];
      rd68031_ucode_pkg::U_FC_DFC:  req_fc = dfc_q[2:0];
      rd68031_ucode_pkg::U_FC_EASP: req_fc = super_mode
                                             ? (eapc_q ? rd68031_pkg::FC_SUPER_PROG
                                                       : rd68031_pkg::FC_SUPER_DATA)
                                             : (eapc_q ? rd68031_pkg::FC_USER_PROG
                                                       : rd68031_pkg::FC_USER_DATA);
      // An exception frame is on the supervisor stack whatever mode the
      // exception came from -- isa.py's SDATA.
      rd68031_ucode_pkg::U_FC_SDATA: req_fc = rd68031_pkg::FC_SUPER_DATA;
      // Program space at the privilege level of the status register a short
      // bus fault frame carries, which RTE holds in xw -- UM 8.2.1.
      rd68031_ucode_pkg::U_FC_FRAMEPROG:
        req_fc = xw_q[rd68031_pkg::SR_S] ? rd68031_pkg::FC_SUPER_PROG
                                         : rd68031_pkg::FC_USER_PROG;
      default:                      req_fc = super_mode
                                             ? rd68031_pkg::FC_SUPER_DATA
                                             : rd68031_pkg::FC_USER_DATA;
    endcase
  end

  // Drop the request as soon as it has been taken, and it takes BOTH terms.
  //
  // The bus unit accepts a new request on the very edge the previous operand
  // finishes -- that is what makes back-to-back cycles possible at all -- so a
  // request still asserted at that edge is taken a second time. req_last covers
  // that edge. But this sequencer waits for req_ack rather than presenting its
  // next request in that half clock, so the microword is still current for one
  // more clock after the operand completes, with req_last back low: without the
  // second term the same read runs twice, which is how the reset vectors came
  // back as the stack pointer twice over.
  assign req_valid    = req_want && !req_last;
  // ... and the request without the req_last term, for the bus unit's ATC
  // lookup, which must not wait for the operand under way to finish (M15
  // phase B): while this sequencer's own operand is under way, the request it
  // presents is that operand, and the bus unit looks up something else.
  assign req_want     = bus_req && !other_stall
                     && !(req_ack && !early_q) && !post_own
                     // A posted write's fault is reported while some other
                     // microword is presented, and that one is about to be
                     // abandoned -- its request must not start.
                     && !req_fault;
  assign req_post     = `UF(POST);
  assign req_kind     = (`UF(BUS) == rd68031_ucode_pkg::U_BUS_WRITE)
                        ? rd68031_pkg::CT_WRITE : rd68031_pkg::CT_READ;
  assign req_addr     = req_addr_sel;
  // A `bytes` of zero on a microword that asks for a bus cycle means "the
  // effective operand size". Without it every effective-address routine that
  // moves an operand would exist once per size, which is the duplication szsel
  // is here to remove.
  logic [2:0] size_bytes;
  always_comb begin
    unique case (eff_size)
      rd68031_ucode_pkg::U_SIZE_BYTE: size_bytes = 3'd1;
      rd68031_ucode_pkg::U_SIZE_WORD: size_bytes = 3'd2;
      default:                        size_bytes = 3'd4;
    endcase
  end

  assign req_bytes    = (`UF(SZSEL) == rd68031_ucode_pkg::U_SZSEL_BFMEM)
                        ? bf_nbytes
                        : (`UF(BYTES) == 3'd0) ? size_bytes : `UF(BYTES);
  // A bit-field write puts back all the bytes the field touches, up to five of
  // them, with only the field itself changed -- and RIGHT justified, because
  // that is how the bus unit takes an operand of any length. The window the
  // field is measured in is left justified, so this is that undone.
  logic [39:0] bf_wdata;
  always_comb begin
    unique case (bf_nbytes)
      3'd1:    bf_wdata = {32'd0, bf_merged_mem[39:32]};
      3'd2:    bf_wdata = {24'd0, bf_merged_mem[39:24]};
      3'd3:    bf_wdata = {16'd0, bf_merged_mem[39:16]};
      3'd4:    bf_wdata = { 8'd0, bf_merged_mem[39:8]};
      default: bf_wdata = bf_merged_mem;
    endcase
  end

  assign req_wdata    = (`UF(SZSEL) == rd68031_ucode_pkg::U_SZSEL_BFMEM)
                        ? bf_wdata : {8'd0, y};
  // UM 7.3.3 and 7.5.2: RMC is held across the whole read-modify-write, and each
  // cycle inside it is an ordinary one that is retried on its own. TAS, CAS and
  // CAS2 assert it.
  assign req_rmc      = `UF(RMC);
  // UM figure 7-42 lays out the CPU address spaces, and "on the MC68030, three
  // of the encodings are implemented": the breakpoint acknowledge, the
  // coprocessor, and the interrupt acknowledge.
  always_comb begin
    unique case (`UF(CPUSPACE))
      rd68031_ucode_pkg::U_CPUSPACE_BKPT:   req_cpuspace = rd68031_pkg::CPUS_BKPT;
      rd68031_ucode_pkg::U_CPUSPACE_COPROC,
      rd68031_ucode_pkg::U_CPUSPACE_CPINIT: req_cpuspace = rd68031_pkg::CPUS_COPROC;
      default:                              req_cpuspace = rd68031_pkg::CPUS_IACK;
    endcase
  end

  // What goes in the CPU-space address besides the type. An interrupt
  // acknowledge puts the level on A3-A1, and a breakpoint acknowledge puts the
  // breakpoint number on A4-A2 -- UM 7.4.1 and 7.4.2, figure 7-42. The bus unit
  // builds the rest of the address from the type.
  always_comb begin
    unique case (`UF(CPUSPACE))
      rd68031_ucode_pkg::U_CPUSPACE_BKPT:   req_cpuaddr = {5'd0, stg_d[2:0]};
      // UM figure 10-3: the CpID from bits 11:9 of the F-line operation word on
      // A15-A13, and the interface register on A4-A0, out of `vec`.
      rd68031_ucode_pkg::U_CPUSPACE_COPROC,
      rd68031_ucode_pkg::U_CPUSPACE_CPINIT:
        req_cpuaddr = {stg_d[11:9], uw[rd68031_ucode_pkg::U_VEC_LSB +: 5]};
      default:                              req_cpuaddr = {5'd0, irq_taking_q};
    endcase
  end

  // UM 10.5.2.8: "if a bus error occurs during the CIR access that is used to
  // initiate a coprocessor instruction, the main processor assumes that the
  // coprocessor is not present and takes an F-line emulator exception", and on
  // any other coprocessor access "the main processor performs bus error
  // exception processing". The first is read off the end code, as the other
  // CPU-space cycles are; the second has to raise the fault.
  assign req_cpfault = (`UF(CPUSPACE) == rd68031_ucode_pkg::U_CPUSPACE_COPROC);

  // PRM 6, MOVES: "moves the byte, word, or long operand ... to or from the
  // specified address space", the address being the effective address. With
  // SFC or DFC at 7 that is a CPU-space cycle at that address -- not one whose
  // address the bus unit builds from a type field, which is what every other
  // CPU-space request here is. doc/bugs-found.md.
  assign req_cpuraw  = (`UF(FC) == rd68031_ucode_pkg::U_FC_SFC)
                    || (`UF(FC) == rd68031_ucode_pkg::U_FC_DFC);

  // ==========================================================================
  // The instruction pipe
  // ==========================================================================
  assign pf_op    = commit ? `UF(PF) : rd68031_ucode_pkg::U_PF_NONE;
  assign pf_addr  = y;
  assign pf_super = super_mode;

  // ==========================================================================
  // The next micro-address
  // ==========================================================================
  always_comb begin
    // The only ways out of the stopped state are an interrupt above the mask
    // STOP wrote and a reset -- PRM 6. The decode that stopped the processor
    // has already put the next instruction's entry in `upc`, so leaving by way
    // of an interrupt means overriding it here: the interrupt is taken instead
    // of that instruction, not after it.
    // A data fault outranks every other reason to go somewhere: UM 8.1.2, "if
    // the aborted bus cycle is a data access, the processor immediately begins
    // exception processing".
    if (dbf_q) begin
      upc_nxt = upc;
    end else if (fault_now && !g0_q) begin
      upc_nxt = rd68031_ucode_pkg::ENTRY_FAULT_LONG;
    end else if (stopped_q) begin
      upc_nxt = irq_pending ? rd68031_ucode_pkg::ENTRY_IRQ : upc;
    end else if (!retire) begin
      upc_nxt = upc;
    end else begin
      unique case (`UF(SEQ))
        // The decode arm, with the trace exception in front of it. UM 8.1.7:
        // "the exception processing for a trace starts at the end of normal
        // processing for the traced instruction and BEFORE THE START OF THE
        // NEXT INSTRUCTION", which is exactly this point and no other.
        // UM 8.1.7: "if an interrupt is pending at the completion of an
        // instruction, the trace exception processing occurs BEFORE the
        // interrupt exception processing starts". So the trace comes first and
        // the interrupt is taken at the boundary the trace handler's own first
        // instruction ends on.
        rd68031_ucode_pkg::U_SEQ_DECODE:
          upc_nxt = trace_take   ? rd68031_ucode_pkg::ENTRY_TRACE
                  : irq_pending  ? rd68031_ucode_pkg::ENTRY_IRQ
                                 : dec_entry;
        rd68031_ucode_pkg::U_SEQ_EADEC:  upc_nxt = ea_entry;
        rd68031_ucode_pkg::U_SEQ_EAMODE: upc_nxt = eam_entry;
        rd68031_ucode_pkg::U_SEQ_CPDEC:  upc_nxt = cp_entry;
        rd68031_ucode_pkg::U_SEQ_RET:    upc_nxt = link_q;
        // doc/checkpoint.md rule 2 is what makes this a jump and nothing else:
        // the microword that faulted committed nothing, so re-executing it
        // reissues exactly the same request.
        //
        // ... except for a read-modify-write, which is not resumed at its
        // faulted access at all. UM 8.2.2: with RM set and DF cleared, "the RTE
        // instruction expects the entire operation to have been completed",
        // "even if the fault occurred on the first read cycle" -- so the
        // instruction ends (rte_rmw_done). UM 8.2.3: with DF set, "the rerun
        // operation ... reruns the entire instruction" -- so it starts again
        // at the first microword of the locked sequence, from the frame's
        // +$52, and runs every one of its cycles afresh.
        rd68031_ucode_pkg::U_SEQ_RESUME:
          upc_nxt = !rs_rm_q ? rupc_q
                  : rs_df_q  ? rmw_upc_q
                  :            rd68031_ucode_pkg::ENTRY_RTE_RMW_DONE;
        // Branch when the condition holds, fall through when it does not.
        rd68031_ucode_pkg::U_SEQ_COND:   upc_nxt = cond_true ? `UF(NEXT)
                                                             : upc + 1'b1;
        default:                         upc_nxt = `UF(NEXT);
      endcase
    end
  end

  // ==========================================================================
  // The one clocked process
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      upc    <= rd68031_ucode_pkg::ENTRY_RESET;
      // UM 4.1.1: after reset the processor is at the supervisor level in
      // interrupt mode, and UM 8.1.1 puts the mask at 7. UM 6.2: reset clears
      // the enable, burst-enable and freeze bits of both caches, and WA.
      sr_q   <= rd68031_pkg::SR_RESET;
      vbr_q  <= '0;
      sfc_q  <= '0;
      dfc_q  <= '0;
      cacr_q <= '0;
      caar_q <= '0;
      usp_q  <= '0;
      isp_q  <= '0;
      msp_q  <= '0;
      xw_q   <= '0;
      ea_q   <= '0;
      cprim_q <= '0;
      link_q <= '0;
      eapc_q  <= 1'b0;
      eadst_q <= 1'b0;
      trace_mode_q <= 2'b00;
      flow_q       <= 1'b0;
      notrace_q    <= 1'b0;
      pc_prev_q    <= '0;
      pc_kept_q    <= 1'b0;
      rs_rc_q      <= 1'b0;
      rs_rb_q      <= 1'b0;
      rs_dv_q      <= 1'b0;
      rs_df_q      <= 1'b0;
      rs_rm_q      <= 1'b0;
      rs_rw_q      <= 1'b1;
      rs_space_q   <= 3'd0;
      rupc_q       <= '0;
      rst_addr_q   <= '0;
      rst_data_q   <= '0;
      rst_bytes_q  <= 3'd0;
      size_q  <= rd68031_ucode_pkg::U_SIZE_LONG;
      // Loop indices local to the loop: a module-level one is a variable the
      // reset branch writes and the other branch does not, and Quartus infers
      // a latch for it (Warning 10240).
      for (int i = 0; i < 8; i++) dreg[i] <= '0;
      for (int i = 0; i < 7; i++) areg[i] <= '0;
      for (int i = 0; i < 4; i++) t_q[i]  <= '0;
    end else if (crst) begin
      // The RESET pin -- see "The two resets" above. The same values as
      // rst_n gives them, less the registers UM 8.1.1 does not name.
      upc    <= rd68031_ucode_pkg::ENTRY_RESET;
      // UM 4.1.1: after reset the processor is at the supervisor level in
      // interrupt mode, and UM 8.1.1 puts the mask at 7. UM 6.2: reset clears
      // the enable, burst-enable and freeze bits of both caches, and WA.
      sr_q   <= rd68031_pkg::SR_RESET;
      vbr_q  <= '0;
      cacr_q <= '0;
      xw_q   <= '0;
      ea_q   <= '0;
      cprim_q <= '0;
      link_q <= '0;
      eapc_q  <= 1'b0;
      eadst_q <= 1'b0;
      trace_mode_q <= 2'b00;
      flow_q       <= 1'b0;
      notrace_q    <= 1'b0;
      pc_prev_q    <= '0;
      pc_kept_q    <= 1'b0;
      rs_rc_q      <= 1'b0;
      rs_rb_q      <= 1'b0;
      rs_dv_q      <= 1'b0;
      rs_df_q      <= 1'b0;
      rs_rm_q      <= 1'b0;
      rs_rw_q      <= 1'b1;
      rs_space_q   <= 3'd0;
      rupc_q       <= '0;
      rst_addr_q   <= '0;
      rst_data_q   <= '0;
      rst_bytes_q  <= 3'd0;
      size_q  <= rd68031_ucode_pkg::U_SIZE_LONG;
      // Written out, not looped: a loop index in this arm, which is not the
      // asynchronous reset's, is a variable yosys gives storage of its own --
      // thirty-two flip-flops with no reset, which `make audit` counts
      // (doc/coding-standard.md).
      t_q[0] <= '0;
      t_q[1] <= '0;
      t_q[2] <= '0;
      t_q[3] <= '0;
    end else begin
      upc <= upc_nxt;

      if (commit) begin
        // `call` latches the microword after this one, which is where seq = RET
        // comes back to.
        if (`UF(CALL)) link_q <= upc + 1'b1;

        // MOVEM's list: the register just named leaves it. The assembler
        // refuses CLRLOW on a microword that also writes T0 -- check_cond_dst.
        if (`UF(CNT) == rd68031_ucode_pkg::U_CNT_CLRLOW)
          t_q[0] <= t_q[0] & (t_q[0] - 32'd1);

        // The dispatch into an extension-word routine is the one place the base
        // is still known.
        if (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_EADEC)  eapc_q <= `UF(EAPC);
        if (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_EAMODE) begin
          eapc_q  <= eam_pc_base;
          eadst_q <= `UF(EADST);
          size_q  <= eff_size;
        end
        // ... and they do not outlive the instruction that set them. eadst_q
        // steers `rsel`, so a MOVE to memory that left it set made the NEXT
        // instruction read the register bits 11:9 name wherever it meant bits
        // 2:0 -- CMPA.L A1,A0 compared A0 with itself. The latches are
        // per-effective-address and an instruction starts with none under way.
        if (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_DECODE) begin
          eapc_q  <= 1'b0;
          eadst_q <= 1'b0;
          // The instruction that is about to start. Its trace mode is the one
          // the status register holds NOW -- UM 8.1.7 -- and it has not
          // changed the flow yet.
          trace_mode_q <= {sr_eff[rd68031_pkg::SR_T1], sr_eff[rd68031_pkg::SR_T0]};
          flow_q       <= 1'b0;
          // ... unless what starts is not an instruction at all but the trace
          // or the interrupt exception the decode arm puts in front of one.
          // Exception processing is not traced: the trace exception "clears
          // the T1 and T0 bits" and the handler's first instruction is the
          // first that could be (UM 8.1.7). Without this the end of a trace
          // exception's own processing was judged against the trace mode of
          // the instruction it traced, and with T1 set it took a second trace
          // exception with the handler's address in the frame -- found by
          // core_status_tb counting STATUS indications (doc/bugs-found.md).
          notrace_q    <= trace_take || irq_pending;
          // And the address of the instruction that has just finished, which
          // is what a trace frame carries at +$08. pc_d is still it at this
          // edge: the pipe advance that moves it on is a non-blocking write on
          // this same edge.
          //
          // Unless the instruction FLUSHED the pipe, which moves pc_d to the
          // new stream several microwords before the instruction ends. A
          // traced BRA stacked the address of its own target at +$08, which
          // reads as a trace of the instruction that had not run yet. The
          // value is therefore taken at the flush when there was one, and
          // pc_kept_q says so.
          if (!pc_kept_q) pc_prev_q <= pc_d;
          pc_kept_q    <= 1'b0;
        end else begin
          // The last moment at which pc_d is still this instruction's own
          // address. Only the first flush of an instruction counts: RTE
          // flushes once, and nothing flushes twice, but the guard costs
          // nothing and makes that an assumption the design does not rest on.
          if ((`UF(PF) == rd68031_ucode_pkg::U_PF_FLUSH) && !pc_kept_q) begin
            pc_prev_q <= pc_d;
            pc_kept_q <= 1'b1;
          end
          if (`UF(PF) == rd68031_ucode_pkg::U_PF_FLUSH
              || `UF(DST) == rd68031_ucode_pkg::U_DST_SR
              || `UF(DST) == rd68031_ucode_pkg::U_DST_SCANPC) flow_q <= 1'b1;
          if (`UF(NOTRACE)) notrace_q <= 1'b1;
        end

        unique case (`UF(DST))
          rd68031_ucode_pkg::U_DST_T0: t_q[0] <= y_reg;
          rd68031_ucode_pkg::U_DST_T1: t_q[1] <= y_reg;
          rd68031_ucode_pkg::U_DST_T2: t_q[2] <= y_reg;
          rd68031_ucode_pkg::U_DST_T3: t_q[3] <= y_reg;
          rd68031_ucode_pkg::U_DST_XW: xw_q   <= y[15:0];
          rd68031_ucode_pkg::U_DST_CPRIM: cprim_q <= y[15:0];
          // The midinstruction frame's internal word, put back -- the same
          // bits cp_int packs.
          rd68031_ucode_pkg::U_DST_CPINT: begin
            trace_mode_q <= y[1:0];
            flow_q       <= y[2];
            pc_kept_q    <= y[3];
            notrace_q    <= y[4];
          end
          // ------------------------------------------------------------------
          // RTE putting a fault frame back. The two packed words are unpacked
          // into exactly the registers they were packed from, and the positions
          // come from rd68031_frame_pkg at both ends, so the pair cannot drift.
          // ------------------------------------------------------------------
          rd68031_ucode_pkg::U_DST_INT08: begin
            notrace_q    <= y[rd68031_frame_pkg::I_NOTRACE_LO];
            eapc_q       <= y[rd68031_frame_pkg::I_EAPC_LO];
            size_q       <= y[rd68031_frame_pkg::I_OPSIZE_LO +: 2];
            eadst_q      <= y[rd68031_frame_pkg::I_EADST_LO];
            // The residual byte count, which UM Table 7-2's SIZE field in the
            // special status word cannot hold when it is five -- doc/ssw.md.
            rst_bytes_q  <= y[rd68031_frame_pkg::I_BYTES_LO +: 3];
            rs_dv_q      <= y[rd68031_frame_pkg::I_DVALID_LO];
          end
          rd68031_ucode_pkg::U_DST_INT36: begin
            trace_mode_q <= y[rd68031_frame_pkg::I_TRMODE_LO +: 2];
            flow_q       <= y[rd68031_frame_pkg::I_FLOW_LO];
            pc_kept_q    <= y[rd68031_frame_pkg::I_PC_KEPT_LO];
          end
          // UM figure 8-9, taken apart. FC and FB are what the frame tells the
          // HANDLER and the internal bits are nobody's; neither comes back.
          rd68031_ucode_pkg::U_DST_SSW: begin
            rs_rc_q    <= y[rd68031_frame_pkg::SSW_RC];
            rs_rb_q    <= y[rd68031_frame_pkg::SSW_RB];
            rs_df_q    <= y[rd68031_frame_pkg::SSW_DF];
            rs_rm_q    <= y[rd68031_frame_pkg::SSW_RM];
            rs_rw_q    <= y[rd68031_frame_pkg::SSW_RW];
            rs_space_q <= y[rd68031_frame_pkg::SSW_FCODE_HI:rd68031_frame_pkg::SSW_FCODE_LO];
          end
          // The same word out of a SHORT frame -- doc/checkpoint.md rule 1. The
          // short frame has no internal word this design can read, so what the
          // long one's +$08 would say is what an instruction boundary implies:
          // stage D holds a word (RTE reads it from memory), nothing is
          // cancelled, and the residual is the whole operand SIZE names, UM
          // Table 7-2. DF stands only for a write -- UM 8.2.2, "data read
          // faults only generate the long bus fault frame" -- and the post
          // bit, in the fault block below, makes that write the bus unit's own.
          rd68031_ucode_pkg::U_DST_SSWA: begin
            rs_rc_q    <= y[rd68031_frame_pkg::SSW_RC];
            rs_rb_q    <= y[rd68031_frame_pkg::SSW_RB];
            rs_df_q    <= y[rd68031_frame_pkg::SSW_DF] & ~y[rd68031_frame_pkg::SSW_RW];
            // A short frame is taken at an instruction boundary, where no
            // read-modify-write can be under way: RM in one is not acted on,
            // or a RESUME would end the instruction at the boundary as well.
            rs_rm_q    <= 1'b0;
            rs_rw_q    <= y[rd68031_frame_pkg::SSW_RW];
            rs_space_q <= y[rd68031_frame_pkg::SSW_FCODE_HI:rd68031_frame_pkg::SSW_FCODE_LO];
            rs_dv_q    <= 1'b1;
            notrace_q  <= 1'b0;
            rst_bytes_q <= (y[rd68031_frame_pkg::SSW_SIZE_HI:rd68031_frame_pkg::SSW_SIZE_LO] == 2'b00)
                           ? 3'd4
                           : {1'b0, y[rd68031_frame_pkg::SSW_SIZE_HI:rd68031_frame_pkg::SSW_SIZE_LO]};
          end
          rd68031_ucode_pkg::U_DST_DFA:     rst_addr_q <= y;
          // The two buffers are one register in the bus unit, and which of
          // them the frame means is the direction of the faulted access. Both
          // are read back so that the walk needs no conditional step; the one
          // that matters is the one that lands.
          rd68031_ucode_pkg::U_DST_DOB: if (!rs_rw_q) rst_data_q <= y;
          rd68031_ucode_pkg::U_DST_DIB: if ( rs_rw_q) rst_data_q <= y;
          rd68031_ucode_pkg::U_DST_LINK:    link_q    <= y[rd68031_ucode_pkg::UADDR-1:0];
          rd68031_ucode_pkg::U_DST_PC_PREV: pc_prev_q <= y;
          rd68031_ucode_pkg::U_DST_RUPC:    rupc_q    <= y[rd68031_ucode_pkg::UADDR-1:0];
          rd68031_ucode_pkg::U_DST_EA: ea_q   <= y;
          rd68031_ucode_pkg::U_DST_SR: sr_q   <= y[15:0]
                                                 & rd68031_pkg::SR_IMPLEMENTED;
          rd68031_ucode_pkg::U_DST_DREG: begin
            // A byte or word write leaves the rest of the data register alone.
            unique case (eff_size)
              rd68031_ucode_pkg::U_SIZE_BYTE: dreg[wsel][7:0]  <= y_reg[7:0];
              rd68031_ucode_pkg::U_SIZE_WORD: dreg[wsel][15:0] <= y_reg[15:0];
              default:                        dreg[wsel]       <= y_reg;
            endcase
          end
          rd68031_ucode_pkg::U_DST_DREG_R: begin
            // The same, addressed by the effective-address register field: the
            // destination of a one-operand instruction whose <ea> is a register.
            unique case (eff_size)
              rd68031_ucode_pkg::U_SIZE_BYTE: dreg[rsel][7:0]  <= y_reg[7:0];
              rd68031_ucode_pkg::U_SIZE_WORD: dreg[rsel][15:0] <= y_reg[15:0];
              default:                        dreg[rsel]       <= y_reg;
            endcase
          end
          // MOVEM writing a register back. A word transfer is sign extended
          // into the whole register whatever it is -- PRM 4, "the MOVEM
          // instruction sign-extends a word to a long word in a data register
          // as well as an address register", which is the one place a data
          // register is written full width by a word operation.
          rd68031_ucode_pkg::U_DST_REGN,
          rd68031_ucode_pkg::U_DST_REGNR: begin
            if (!movem_wn[3]) begin
              dreg[movem_wn[2:0]] <= y_areg;
            end else if (movem_wn[2:0] == 3'd7) begin
              if (!super_mode)      usp_q <= y_areg;
              else if (master_mode) msp_q <= y_areg;
              else                  isp_q <= y_areg;
            end else begin
              areg[movem_wn[2:0]] <= y_areg;
            end
          end
          // MOVEC writing a control register. SFC and DFC are three bits, and
          // CACR keeps the seven of its thirty-two that UM figure 6-14 does not
          // reserve or make read as zero; PRM 6 says the transfer is thirty-two
          // bits wide either way and the rest read back as zero, which is what
          // narrow registers give. CAAR keeps all thirty-two: "all 32 bits of
          // the register are implemented" (UM 6.3.2).
          // The user stack pointer by name. MOVE USP reaches it while running
          // at the supervisor level, which is the only way a kernel can build
          // a frame on a user stack -- PRM 6.
          rd68031_ucode_pkg::U_DST_USP: usp_q <= y;
          rd68031_ucode_pkg::U_DST_CREG: begin
            unique case (creg_sel)
              12'h000: sfc_q  <= y[2:0];
              12'h001: dfc_q  <= y[2:0];
              12'h002: cacr_q <= y & rd68031_pkg::CACR_IMPLEMENTED;
              12'h800: usp_q  <= y;
              12'h801: vbr_q  <= y;
              12'h802: caar_q <= y;
              12'h803: msp_q  <= y;
              12'h804: isp_q  <= y;
              default: ;
            endcase
          end
          rd68031_ucode_pkg::U_DST_XREG: begin
            if (!xw_q[15]) begin
              dreg[xw_q[14:12]] <= y;
            end else if (xw_q[14:12] == 3'd7) begin
              if (!super_mode)      usp_q <= y;
              else if (master_mode) msp_q <= y;
              else                  isp_q <= y;
            end else begin
              areg[xw_q[14:12]] <= y;
            end
          end
          // PRM 6, MOVES: a byte or a word replaces "the corresponding
          // low-order bits" of a data register, and is sign-extended into the
          // whole of an address register -- which y_areg already is.
          rd68031_ucode_pkg::U_DST_XREG_SZ: begin
            if (!xw_q[15]) begin
              unique case (eff_size)
                rd68031_ucode_pkg::U_SIZE_BYTE: dreg[xw_q[14:12]][7:0]  <= y[7:0];
                rd68031_ucode_pkg::U_SIZE_WORD: dreg[xw_q[14:12]][15:0] <= y[15:0];
                default:                        dreg[xw_q[14:12]]       <= y;
              endcase
            end else if (xw_q[14:12] == 3'd7) begin
              if (!super_mode)      usp_q <= y_areg;
              else if (master_mode) msp_q <= y_areg;
              else                  isp_q <= y_areg;
            end else begin
              areg[xw_q[14:12]] <= y_areg;
            end
          end
          rd68031_ucode_pkg::U_DST_DREG_XQ: dreg[xw_q[14:12]] <= y_reg;
          // PRM 6, PTEST: the descriptor address "loads into the address
          // register" bits 7:5 of the command word name.
          rd68031_ucode_pkg::U_DST_PTEST_AN: begin
            if (xw_q[7:5] == 3'd7) begin
              if (!super_mode)      usp_q <= y;
              else if (master_mode) msp_q <= y;
              else                  isp_q <= y;
            end else begin
              areg[xw_q[7:5]] <= y;
            end
          end
          // At the operand size. CAS and CAS2 are the byte and word users:
          // PRM 4, "Destination -> Compare Operand", and a byte or word
          // operation on a data register changes only its low-order part (UM
          // 2.2.1). Writing the whole register lost the compare register's
          // upper bits whenever the comparison failed (doc/bugs-found.md, M6).
          rd68031_ucode_pkg::U_DST_DREG_XR: begin
            unique case (eff_size)
              rd68031_ucode_pkg::U_SIZE_BYTE: dreg[xw_q[2:0]][7:0]  <= y_reg[7:0];
              rd68031_ucode_pkg::U_SIZE_WORD: dreg[xw_q[2:0]][15:0] <= y_reg[15:0];
              default:                        dreg[xw_q[2:0]]       <= y_reg;
            endcase
          end
          rd68031_ucode_pkg::U_DST_CCR:
            sr_q[4:0] <= y[4:0];
          rd68031_ucode_pkg::U_DST_AREG: begin
            // An address register is always written full width, sign extended
            // from a word -- PRM 2, "the entire destination address register is
            // used regardless of the operation size". MOVEA.W and ADDA.W are
            // the instructions that depend on it.
            if (wsel == 3'd7) begin
              if (!super_mode)      usp_q <= y_areg;
              else if (master_mode) msp_q <= y_areg;
              else                  isp_q <= y_areg;
            end else begin
              areg[wsel] <= y_areg;
            end
          end
          rd68031_ucode_pkg::U_DST_SP: begin
            if (!super_mode)      usp_q <= y;
            else if (master_mode) msp_q <= y;
            else                  isp_q <= y;
          end
          // An ADDRESS, not a data operand: the whole register, never extended.
          // PRM 2 sign-extends a word into an address register because the word
          // is a VALUE; the address a postincrement leaves behind is already
          // thirty-two bits and extending it at the operand size truncates it.
          rd68031_ucode_pkg::U_DST_AREG_ADDR: begin
            if (wsel == 3'd7) begin
              if (!super_mode)      usp_q <= y;
              else if (master_mode) msp_q <= y;
              else                  isp_q <= y;
            end else begin
              areg[wsel] <= y;
            end
          end
          rd68031_ucode_pkg::U_DST_AREG_EA_ADDR: begin
            if (rsel == 3'd7) begin
              if (!super_mode)      usp_q <= y;
              else if (master_mode) msp_q <= y;
              else                  isp_q <= y;
            end else begin
              areg[rsel] <= y;
            end
          end
          // UM 10.4.13: a long word into the register the primitive names.
          rd68031_ucode_pkg::U_DST_CPREG: begin
            if (!cprim_q[3]) begin
              dreg[cprim_q[2:0]] <= y;
            end else if (cprim_q[2:0] == 3'd7) begin
              if (!super_mode)      usp_q <= y;
              else if (master_mode) msp_q <= y;
              else                  isp_q <= y;
            end else begin
              areg[cprim_q[2:0]] <= y;
            end
          end
          // UM 10.4.9: "the MC68030 sign-extends a byte or word-sized operand
          // to a long-word value when it is transferred to an address register
          // ... using this primitive with the register direct effective
          // addressing mode" -- the register the effective-address field names.
          rd68031_ucode_pkg::U_DST_AREG_R: begin
            if (rsel == 3'd7) begin
              if (!super_mode)      usp_q <= y_areg;
              else if (master_mode) msp_q <= y_areg;
              else                  isp_q <= y_areg;
            end else begin
              areg[rsel] <= y_areg;
            end
          end
          rd68031_ucode_pkg::U_DST_AREG_EA: begin
            // The register the effective address field names, for (An)+ and
            // -(An), which step the register they address through.
            if (rsel == 3'd7) begin
              if (!super_mode)      usp_q <= y;
              else if (master_mode) msp_q <= y;
              else                  isp_q <= y;
            end else begin
              areg[rsel] <= y;
            end
          end
          default: ;
        endcase

        // The condition codes -- PRM 3.3 and table 3-18. The field names the
        // WAY the codes are set, not the instruction, because there are eighty
        // instructions and about eight ways.
        unique case (`UF(CCR))
          rd68031_ucode_pkg::U_CCR_LOGIC: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            sr_q[rd68031_pkg::SR_Z] <= res_z;
            sr_q[rd68031_pkg::SR_V] <= 1'b0;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          // PRM 4, CHK: N is "cleared if the compared value is greater than the
          // upper bound". That is the reason for the trap, not the sign of the
          // difference -- a negative bound makes the subtraction overflow and
          // the two disagree.
          // PRM 4, CMP2 and CHK2. Z is only ever SET: the microword before this
          // one cleared it and set it if the register equalled the LOWER bound,
          // and this one adds the upper. C is the borrow, which is out of
          // bounds. N and V are undefined and are left alone.
          rd68031_ucode_pkg::U_CCR_CMP2: begin
            if (res_z) sr_q[rd68031_pkg::SR_Z] <= 1'b1;
            sr_q[rd68031_pkg::SR_C] <= alu_c;
          end
          // PRM 4, every bit-field instruction: "N -- set if the most
          // significant bit of the field is set. Z -- set if all bits of the
          // field are zero. V -- always cleared. C -- always cleared."
          rd68031_ucode_pkg::U_CCR_BF: begin
            sr_q[rd68031_pkg::SR_N] <= bf_msb;
            sr_q[rd68031_pkg::SR_Z] <= bf_zero;
            sr_q[rd68031_pkg::SR_V] <= 1'b0;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          // ... except BFINS, whose page says "the instruction sets the
          // condition codes according to the INSERTED value" -- which is the
          // field only after the write, and is in T3 before it.
          // doc/manual-contradictions.md.
          rd68031_ucode_pkg::U_CCR_BFINS: begin
            sr_q[rd68031_pkg::SR_N] <= t_q[3][bf_msb_ix];
            sr_q[rd68031_pkg::SR_Z] <=
                ((t_q[3] & ~(32'hFFFF_FFFF << bf_width)) == 32'd0);
            sr_q[rd68031_pkg::SR_V] <= 1'b0;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          rd68031_ucode_pkg::U_CCR_CLRNZVC: begin
            sr_q[rd68031_pkg::SR_N] <= 1'b0;
            sr_q[rd68031_pkg::SR_Z] <= 1'b0;
            sr_q[rd68031_pkg::SR_V] <= 1'b0;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          rd68031_ucode_pkg::U_CCR_ADD,
          rd68031_ucode_pkg::U_CCR_SUB: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            sr_q[rd68031_pkg::SR_Z] <= res_z;
            sr_q[rd68031_pkg::SR_V] <= alu_v;
            sr_q[rd68031_pkg::SR_C] <= alu_c;
            sr_q[rd68031_pkg::SR_X] <= alu_c;
          end
          // "Z is cleared if the result is non-zero; unchanged otherwise", so
          // that Z ends up meaning that every part of a multi-precision result
          // was zero. Writing res_z here instead is the classic ADDX bug.
          rd68031_ucode_pkg::U_CCR_ADDX,
          rd68031_ucode_pkg::U_CCR_SUBX: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            if (!res_z) sr_q[rd68031_pkg::SR_Z] <= 1'b0;
            sr_q[rd68031_pkg::SR_V] <= alu_v;
            sr_q[rd68031_pkg::SR_C] <= alu_c;
            sr_q[rd68031_pkg::SR_X] <= alu_c;
          end
          // CMP and TST are subtractions that set no extend bit.
          rd68031_ucode_pkg::U_CCR_CMP: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            sr_q[rd68031_pkg::SR_Z] <= res_z;
            sr_q[rd68031_pkg::SR_V] <= alu_v;
            sr_q[rd68031_pkg::SR_C] <= alu_c;
          end
          rd68031_ucode_pkg::U_CCR_ZN: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            sr_q[rd68031_pkg::SR_Z] <= res_z;
          end
          // The bit instructions move Z and nothing else at all.
          rd68031_ucode_pkg::U_CCR_ZBIT:
            sr_q[rd68031_pkg::SR_Z] <= res_z;
          // The shifter works out its own C, V and X, and whether X moves at
          // all -- a rotate never touches it, and a count of zero never does.
          rd68031_ucode_pkg::U_CCR_SHIFT: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            sr_q[rd68031_pkg::SR_Z] <= res_z;
            sr_q[rd68031_pkg::SR_V] <= sh_v;
            sr_q[rd68031_pkg::SR_C] <= sh_c;
            if (sh_xwr) sr_q[rd68031_pkg::SR_X] <= sh_x;
          end
          // PRM 4 for the multiplies and divides. C is always cleared, X is
          // never touched, and a 64-bit product takes its N and Z from all
          // sixty-four bits rather than from the register the low half lands in.
          // PRM 4 for ABCD, SBCD and NBCD: X and C are the decimal carry, Z is
          // only ever cleared -- so that it means every byte of a
          // multi-precision result was zero -- and N and V are undefined.
          rd68031_ucode_pkg::U_CCR_BCD: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            if (!res_z) sr_q[rd68031_pkg::SR_Z] <= 1'b0;
            sr_q[rd68031_pkg::SR_V] <= 1'b0;
            sr_q[rd68031_pkg::SR_C] <= bcd_c;
            sr_q[rd68031_pkg::SR_X] <= bcd_c;
          end
          rd68031_ucode_pkg::U_CCR_MUL32: begin
            // From the product, not the result bus: check_mul_shape holds
            // every microword with these codes to a long copy of MULLO.
            sr_q[rd68031_pkg::SR_N] <= mul_full[31];
            sr_q[rd68031_pkg::SR_Z] <= (mul_full[31:0] == 32'd0);
            sr_q[rd68031_pkg::SR_V] <= mul_ovf;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          rd68031_ucode_pkg::U_CCR_MUL64: begin
            sr_q[rd68031_pkg::SR_N] <= mul_full[63];
            sr_q[rd68031_pkg::SR_Z] <= (mul_full == 64'd0);
            sr_q[rd68031_pkg::SR_V] <= 1'b0;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          rd68031_ucode_pkg::U_CCR_DIV: begin
            sr_q[rd68031_pkg::SR_N] <= res_n;
            sr_q[rd68031_pkg::SR_Z] <= res_z;
            sr_q[rd68031_pkg::SR_V] <= 1'b0;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          // On overflow PRM 4 leaves the operands alone and sets V. N and Z are
          // undefined there and this design leaves them alone too, which
          // doc/divergences.md records as the choice it is.
          rd68031_ucode_pkg::U_CCR_DIVV: begin
            sr_q[rd68031_pkg::SR_V] <= 1'b1;
            sr_q[rd68031_pkg::SR_C] <= 1'b0;
          end
          rd68031_ucode_pkg::U_CCR_ALL:
            sr_q[4:0] <= y[4:0];
          default: ;
        endcase
      end
    end
  end

  // A microword retired early, so the acknowledge the bus unit registers on the
  // same edge is its, not the next one's.
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n) early_q <= 1'b0;
    else        early_q <= retire && bus_req && early_hit;
  end

  // Entered by the decode arm, once the trace and the interrupt have had their
  // chance at that same boundary, and left only by an interrupt or by reset.
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n)
      stopped_q <= 1'b0;
    else if (irq_pending)
      stopped_q <= 1'b0;
    else if (retire && (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_DECODE)
             && `UF(STOP) && !trace_take)
      stopped_q <= 1'b1;
  end

  // PRM 6 RESET. The bus unit holds the pin for 512 clocks and says so; this
  // microword stalls until it lets go.
  assign reset_req = `UF(RSTO);

  // The fault itself. Everything here is latched on the one clock the bus unit
  // reports it, because the microcode that follows runs bus cycles of its own.
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      df_q      <= 1'b0;
      flt_odd_q <= 1'b0;
      // Reset exception processing is a group-0 window from its first clock:
      // UM 8.1.1, figure 8-1, and UM 12.7.1 -- "the processor also halts if it
      // receives a bus error or address error during the vector table read
      // operations or the prefetch for the first instruction after an
      // external reset".
      g0_q      <= 1'b1;
      rmw_upc_q <= '0;
      ea_save   <= '0;
      flt_upc   <= '0;
      post_flt_q <= 1'b0;
      own_q     <= 1'b0;
    end else begin
      // The microword's posted write has been taken and it has not retired.
      own_q <= post_own && !retire && !fault_now;
      if (fault_now) begin
        // A fault on a write posted by a microword that has since retired.
        // One on the write of the microword still waiting is its own.
        post_flt_q <= req_fault_post && !post_own;
        // "The least significant half of the SSW applies to data cycles only",
        // so only a data fault sets DF -- doc/ssw.md.
        df_q    <= req_fault;
        // UM 8.2.1: an address error sets the rerun bits and not the fault
        // bits, and it is the vector that tells the two apart -- 3 against 2.
        flt_odd_q <= !req_fault && pf_odd;
        g0_q    <= 1'b1;
        flt_upc <= upc;
        // The frame is built with a pointer of its own -- ea_q belongs to the
        // instruction and is frame +$38 -- and it starts at the ACTIVE
        // SUPERVISOR stack, which is where UM 8.1 step three puts every frame.
        // Not `sp_read`: the fault may have happened in user mode, where that
        // is the user stack, and the exception has not set S yet.
        ea_save <= master_mode ? msp_q : isp_q;
      end else if (commit && (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_DECODE)) begin
        // A fault taken at an instruction boundary is a prefetch fault, never
        // a data one, and must say so -- doc/ssw.md. And the first instruction
        // of a handler is beginning, so exception processing is over (UM 4,
        // "it has completed when execution of the first instruction of the
        // exception handler routine begins").
        df_q    <= 1'b0;
        g0_q    <= 1'b0;
      end else if (commit && (`UF(DST) == rd68031_ucode_pkg::U_DST_EA_SAVE)) begin
        ea_save <= y;
      end else if (commit && (`UF(DST) == rd68031_ucode_pkg::U_DST_INT08)) begin
        post_flt_q <= y[rd68031_frame_pkg::I_POSTED_LO];
      end else if (commit && (`UF(DST) == rd68031_ucode_pkg::U_DST_SSWA)) begin
        // A short frame's data fault is a write its finished instruction
        // posted: RTE hands it to the bus unit to run on its own -- rule 9.
        post_flt_q <= y[rd68031_frame_pkg::SSW_DF] & ~y[rd68031_frame_pkg::SSW_RW];
      end
      // RTE begins to load a validated bus fault frame -- UM 8.1.13 -- and from
      // here to its RESUME a bus error is a double bus fault. The microword
      // that marks it writes ea_save as well, which the chain above does.
      if (commit && (`UF(MARK) == rd68031_ucode_pkg::U_MARK_LOAD))
        g0_q <= 1'b1;
      // ... and has loaded it. A fault on the rerun that follows is an
      // ordinary one: UM 8.2.3, "the processor creates a new stack frame on
      // the supervisor stack after deallocating the previous frame".
      if (commit && (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_RESUME))
        g0_q <= 1'b0;
      // Where a read-modify-write starts -- UM 8.2.3 -- and RTE putting it
      // back out of the frame. Latched whenever the marked microword is
      // presented, and not only when it commits: for CAS and TAS it is the
      // locked read itself, and a fault on that read is one the frame has to
      // carry it for.
      if (`UF(MARK) == rd68031_ucode_pkg::U_MARK_RMW)
        rmw_upc_q <= upc;
      else if (commit && (`UF(DST) == rd68031_ucode_pkg::U_DST_RMWUPC))
        rmw_upc_q <= y[rd68031_ucode_pkg::UADDR-1:0];
    end
  end

  // The level this acknowledge cycle is for. Latched on the step INTO the
  // interrupt entry point, wherever that step comes from -- the decode arm or
  // the stopped state -- and not while sitting on it, which a stalled first
  // microword would otherwise do once per clock with whatever the pins then
  // said.
  // The coprocessor's midinstruction interrupt is the same acknowledge reached
  // from inside an instruction -- UM 10.5.2.6 -- and latches the same way.
  assign irq_enter = ((upc != rd68031_ucode_pkg::ENTRY_IRQ)
                      && (upc_nxt == rd68031_ucode_pkg::ENTRY_IRQ))
                  || ((upc != rd68031_ucode_pkg::ENTRY_CP_IRQ)
                      && (upc_nxt == rd68031_ucode_pkg::ENTRY_CP_IRQ));

  // ... but RTE's RESUME into the entry point is not a recognition. A posted
  // write's fault taken on the entry's first microword -- doc/checkpoint.md
  // rule 9 -- leaves that microword as the frame's resume point, so RTE steps
  // INTO the entry again, and the level being taken is the frame's, which the
  // INT08 restore has already put back. UM 8.1.9 recognises the level once and
  // the acknowledge carries that one: latching the pins here acknowledged
  // whatever they said by the end of the bus error handler -- level 0 under
  // NetBSD/sun3x, and vector 0 taken (doc/bugs-found.md). The level-7 edge is
  // the same: it was consumed at the recognition, and a transition to 7 since
  // then is for the next boundary to see. IPEND's hold, which the fault
  // cleared, is re-armed: the acknowledge is upcoming again.
  logic irq_recognise;
  assign irq_recognise = irq_enter
                      && (`UF(SEQ) != rd68031_ucode_pkg::U_SEQ_RESUME);

  // The level as it was at the last boundary, for the level-7 edge -- and at
  // every step into an interrupt, which is where a transition is CONSUMED. Two
  // of those steps retire no decode: leaving STOP, and the coprocessor's
  // midinstruction interrupt, entered by the microcode from inside the
  // dialogue. Without them the handler's first boundary judged level 7 against
  // the level from before: a device holds its request until the handler clears
  // it, so that was a fresh "transition", and the non-maskable interrupt was
  // taken a second time, nested (doc/bugs-found.md).
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n)                                        irq_prev_q <= 3'd0;
    else if ((retire && (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_DECODE))
             || irq_recognise)
                                                       irq_prev_q <= irq_level;
  end

  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n)          irq_taking_q <= 3'd0;
    else if (irq_recognise) irq_taking_q <= irq_level;
    // ... and put back by RTE: a posted write's fault may have been taken
    // between the dispatch and the acknowledge -- doc/checkpoint.md rule 9.
    else if (commit && (`UF(DST) == rd68031_ucode_pkg::U_DST_INT08))
      irq_taking_q <= y[rd68031_frame_pkg::I_IRQLVL_LO +: 3];
  end

  assign cacr      = cacr_q;
  assign caar      = caar_q;

  assign mmu_cmd   = `UF(MMU);
  assign mmu_go    = mmu_start || (commit && mmu_inst);
  assign mmu_xw    = xw_q;
  assign mmu_ea    = ea_q;
  assign mmu_wd_hi = t_q[0];
  assign mmu_wd_lo = t_q[1];
  // MOVEC setting CACR's CI, CEI, CD or CED -- UM 6.3.1.8, 6.3.1.9, 6.3.1.3
  // and 6.3.1.4. All four "always read as a zero", so they are never stored:
  // the write is an order to the cache, carried out on the edge that commits
  // it, "at the time a MOVEC instruction loads a one" into the bit. The two
  // caches are independent -- "a similar operation can be performed for both
  // caches by a single MOVEC instruction" (UM 6.3.1) -- so each pair is passed
  // on as it stands; each cache lets its clear-all cover its clear-entry.
  logic cacr_wr;
  assign cacr_wr = commit && (`UF(DST) == rd68031_ucode_pkg::U_DST_CREG)
                && (creg_sel == 12'h002);
  assign cach_op = cacr_wr ? {y[rd68031_pkg::CACR_CED], y[rd68031_pkg::CACR_CD],
                              y[rd68031_pkg::CACR_CEI], y[rd68031_pkg::CACR_CI]}
                           : 4'b0000;
  // ==========================================================================
  // RTE putting a fault frame back -- UM 8.2.3, doc/ssw.md
  //
  // One field per microword: six go straight out to the pipe over the
  // checkpoint port, and the rest land here. Nothing is buffered, because the
  // frame is read in an order in which each field's register is dead by the
  // time it is written.
  // ==========================================================================
  assign ckpt_save = 1'b0;

  logic ckpt_is_pipe;
  always_comb begin
    ckpt_is_pipe = 1'b1;
    unique case (`UF(DST))
      rd68031_ucode_pkg::U_DST_STG_D:  ckpt_sel = rd68031_pkg::CK_STG_D;
      rd68031_ucode_pkg::U_DST_STG_C:  ckpt_sel = rd68031_pkg::CK_STG_C;
      rd68031_ucode_pkg::U_DST_STG_B:  ckpt_sel = rd68031_pkg::CK_STG_B;
      rd68031_ucode_pkg::U_DST_PC_D:   ckpt_sel = rd68031_pkg::CK_PC_D;
      rd68031_ucode_pkg::U_DST_FILL:   ckpt_sel = rd68031_pkg::CK_FILL;
      rd68031_ucode_pkg::U_DST_PIPE_F: ckpt_sel = rd68031_pkg::CK_FLAGS;
      rd68031_ucode_pkg::U_DST_SCANPC: ckpt_sel = rd68031_pkg::CK_SCAN;
      default: begin
        ckpt_sel     = rd68031_pkg::CK_STG_D;
        ckpt_is_pipe = 1'b0;
      end
    endcase
  end

  assign ckpt_wr = commit && ckpt_is_pipe;
  // The pipe's fault bits and its depth come from two frame fields that arrive
  // in different microwords, so the one that completes them is the one that
  // says the pipe is whole -- doc/ssw.md.
  // The pipe is whole when its last field has been written, which is the fill
  // point in both frame formats: the words and the depth all come before it.
  // ... and the coprocessor's scanPC, which is the last thing RTE puts back out
  // of a midinstruction frame and which empties and refills the queue itself.
  // The pipe is whole once the fill point is written, but the status register
  // the frame carries is written after it -- it has to be, because writing it
  // moves A7 (UM 8.1.13) -- and the refill must not start before: UM 8.2.1, the
  // rerun prefetch is in "the program space for the privilege level indicated
  // in the copy of the status register on the stack". So the pipe fetches
  // nothing from the fill point to the RESUME, which comes after the status
  // register. A fault in between ends the hold with the RTE.
  logic ckpt_hold_q;
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n)
      ckpt_hold_q <= 1'b0;
    else if (fault_now)
      ckpt_hold_q <= 1'b0;
    else if (commit && (`UF(DST) == rd68031_ucode_pkg::U_DST_FILL))
      ckpt_hold_q <= 1'b1;
    else if (commit && (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_RESUME))
      ckpt_hold_q <= 1'b0;
  end
  assign ckpt_hold = ckpt_hold_q;

  assign ckpt_load = commit && ((`UF(DST) == rd68031_ucode_pkg::U_DST_FILL)
                                || (`UF(DST) == rd68031_ucode_pkg::U_DST_SCANPC));
  // {D valid, RB, RC}, as rd68031_ifu takes them -- the first out of the
  // internal word at +$08, the others out of the SSW, both read by now because
  // +$0A is where the walk puts the depth. FC and FB are not among them: they are
  // what the frame tells the HANDLER, and this core writes them and never reads
  // them back -- doc/ssw.md.
  assign ckpt_data = (`UF(DST) == rd68031_ucode_pkg::U_DST_PIPE_F)
                     ? {29'd0, rs_dv_q, rs_rb_q, rs_rc_q}
                     : y;

  // ==========================================================================
  // Handing the faulted operand back to the bus unit -- UM 8.2.3
  //
  // Everything but the data comes out of the special status word and the packed
  // internal word; the data is whichever buffer the direction makes meaningful,
  // and the microcode reads the one RW names.
  // ==========================================================================
  // rst_addr_q, rst_data_q and rst_bytes_q are declared with the checkpoint set
  // at the top of the module: the register-write block uses them first.

  // Nothing is handed back for a read-modify-write: it is either finished or
  // started again from its first cycle -- the RESUME arm above.
  assign rst_op_valid = commit && `UF(RSTOP) && !rs_rm_q;
  assign rst_addr     = rst_addr_q;
  // UM 8.2.3 reruns the faulted access when DF is still set, and UM 8.2.2 says
  // the handler did it when DF is clear. Both hand the operand back; the only
  // difference is how much of it is left, so the microcode has one path and
  // this is where the two part.
  assign rst_bytes    = rs_df_q ? rst_bytes_q : 3'd0;
  assign rst_fc       = rs_space_q;
  assign rst_rw       = rs_rw_q;
  assign rst_rmc      = rs_rm_q;
  assign rst_dob      = rst_data_q;
  assign rst_post     = post_flt_q;

  // The microword RESUME jumped to is the one retiring. If it asked for an
  // operand it has taken the hand-back by now; if it did not, the fault was a
  // prefetch at a boundary and the hand-back is nobody's. Either way it is over.
  logic resumed_q;
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n)      resumed_q <= 1'b0;
    else if (retire) resumed_q <= (`UF(SEQ) == rd68031_ucode_pkg::U_SEQ_RESUME);
  end
  assign rst_cancel = resumed_q && retire;

  // UM 8.1.2: "if a bus error occurs during the exception processing for a bus
  // error, address error, or reset ... a double bus fault occurs and the
  // processor enters the halted state. In this case, the processor does not
  // attempt to alter the current state of memory. Only an external RESET can
  // restart a processor halted by a double bus fault."
  //
  // The window is g0_q: exception processing for a bus error, an address error
  // or reset, up to the first instruction of the handler -- UM 12.7.1, "any
  // additional bus error exception occurring before the execution of the
  // first instruction of the bus error handler routine constitutes a double
  // bus fault", which takes in the vector read and the handler's own first
  // prefetch, faulted or odd (UM 8.1.3) -- and RTE loading a bus fault frame,
  // UM 8.1.13. Any fault in it halts: a data fault, and a prefetch one.
  //
  // Once set it stays set: the bus unit shows it on STATUS, asserted
  // continuously (UM 7.5.4 -- HALT is an input only on the MC68030), nothing
  // here retires again, and only a reset clears it -- rst_n, or the RESET pin,
  // which is the manual's "only an external RESET". Declared at the top of the
  // module, because the stall logic reads it first.
  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n)                 dbf_q <= 1'b0;
    else if (fault_now && g0_q)      dbf_q <= 1'b1;
  end
  assign dbf = dbf_q;

  // ==========================================================================
  // STATUS and REFILL -- UM 12.7.1
  //
  // Table 12-4. One clock: "sequencer at instruction boundary will begin
  // execution of next instruction". Two: at a boundary, "but will not begin next
  // instruction immediately due to pending trace exception OR pending interrupt
  // exception" -- which is exactly the decode arm choosing ENTRY_TRACE or
  // ENTRY_IRQ. Three: "exception processing to begin for reset OR bus error OR
  // address error OR ...": the first clock out of reset, and the fault entry.
  // The table's other three-clock cases -- an ATC miss, a spurious or
  // autovectored interrupt, an F-line with no coprocessor -- are not told apart
  // here yet; doc/divergences.md.
  //
  // REFILL: "the microsequencer requests an instruction pipeline refill" --
  // a pipe flush, which is what every branch, jump, trap, return and status
  // register write does here.
  // ==========================================================================
  logic boot_q;
  logic sts_bound_q, sts_pend_q, sts_exc_q, sts_refill_q;
  logic at_boundary;
  assign at_boundary = retire && at_decode && !fault_now && !dbf_q;

  always_ff @(posedge clk or negedge core_rst_n) begin
    if (!core_rst_n) begin
      boot_q       <= 1'b1;
      sts_bound_q  <= 1'b0;
      sts_pend_q   <= 1'b0;
      sts_exc_q    <= 1'b0;
      sts_refill_q <= 1'b0;
    end else begin
      boot_q       <= 1'b0;
      sts_bound_q  <= at_boundary;
      sts_pend_q   <= at_boundary && (trace_take || irq_pending);
      sts_exc_q    <= boot_q || (fault_now && !g0_q && !dbf_q)
                   || (commit && (`UF(MARK) == rd68031_ucode_pkg::U_MARK_EXC));
      sts_refill_q <= (pf_op == rd68031_ucode_pkg::U_PF_FLUSH);
    end
  end

  assign sts_bound  = sts_bound_q;
  assign sts_pend   = sts_pend_q;
  assign sts_exc    = sts_exc_q;
  assign sts_refill = sts_refill_q;

  // ==========================================================================
  // Not consumed yet.
  // ==========================================================================
  logic unused_seq;
  assign unused_seq = &{1'b1,
                        req_last, req_end, req_fault, req_fault_wr, req_dsack,
                        eam_illegal, mul_full66[65:64], req_rdata[39:8],
                        // The bit number is taken modulo 32 at most, so its
                        // top bit is never part of an answer -- PRM 4.
                        bit_num[5],
                        req_rdata[39:32],
                        flt_addr, flt_bytes, flt_fc, flt_rw, flt_rmc, flt_dob,
                        flt_dib,
                        stg_b, stg_c_fault, stg_b_fault,
                        stg_c_rerun, stg_b_rerun, pf_stuck, pf_odd,
                        stg_b_addr, ckpt_pc_fetch,
                        ipl_sync_n, reset_sync_n, halt_sync_n, bus_idle,
                        bus_granted, reset_busy,
                        dec_illegal, ea_reserved, vbr_q, sfc_q, dfc_q,
                        `UF(COND),
                        COPROCESSOR};

  `undef UF

endmodule
