// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68884

// RD68884 - SystemVerilog MC68881 floating-point coprocessor
//
// The microsequencer and its datapath. One microinstruction per clock; every
// action reads the state as it was at the start of the clock (doc/microcode.md).
//
// tools/iss/core.py is the executable definition of every field and this is
// its transcription; tools/iss/lockstep.py runs the two side by side on the
// same BIU signals and compares them clock by clock. The microcode is
// tools/ucode/program.py, the field table tools/ucode/fields.py, and both
// rd68884_ucode_pkg and rd68884_ucode_rom are generated from them.
//
// The datapath: the 32-bit transfer bus, the memory image XI, the working
// values A and B, the register file, FPCR and FPSR, the FMOVEM mask and the
// predicate evaluator, and the arithmetic of doc/microcode.md -- the mantissa
// adders, one right shifter shared with normalisation, the rounder, the
// 64x16 multiplier, and the divide and square-root steps.

module rd68884_seq #(
    parameter int MODEL = 68881            // 68881 (RD68884) or 68882 (RD68885)
) (
    input  logic        clk,
    input  logic        rst_n,

    // ---- from the BIU --------------------------------------------------------
    input  logic        arch_reset_i,
    input  logic        cmd_pend_i,
    input  logic        cmd_cond_i,
    input  logic [15:0] cmd_word_i,
    input  logic        opw_valid_i,
    input  logic [31:0] opw_data_i,
    input  logic        opr_valid_i,
    input  logic        save_req_i,
    input  logic        restore_req_i,
    input  logic [15:0] restore_word_i,
    input  logic [31:0] fpiar_i,
    input  logic        pv_i,
    input  logic        resp_read_i,
    input  logic        rsel_read_i,
    input  logic        save_read_i,
    input  logic        abort_i,

    // ---- to the BIU ------------------------------------------------------------
    output logic        resp_we_o,
    output logic [15:0] resp_o,
    output logic        resp_oneshot_o,
    output logic [2:0]  expect_o,
    output logic        resp_cond_o,
    output logic        cmd_ack_o,
    output logic        opw_ack_o,
    output logic        opr_we_o,
    output logic [31:0] opr_o,
    output logic        rsel_we_o,
    output logic [7:0]  rsel_o,
    output logic        rsel_dir_o,
    output logic        save_we_o,
    output logic [15:0] save_o,
    output logic [5:0]  save_xfer_o,
    output logic        restore_we_o,
    output logic [15:0] restore_o,
    output logic [5:0]  restore_xfer_o,
    output logic        fpiar_we_o,
    output logic [31:0] fpiar_o,
    output logic        clear_o,

    // ---- RD68885: the conversion unit (rd68884_biu, doc/rd68885.md) ------
    input  logic        abort_ab_i,
    input  logic        cu_ready_i,
    input  logic        cu_valid_i,
    input  logic        cu_mid_i,
    input  logic [15:0] cu_word_i,
    input  logic [31:0] cu_d0_i,
    input  logic [31:0] cu_d1_i,
    input  logic [31:0] cu_d2_i,
    input  logic [31:0] cu_w0_i,
    input  logic [31:0] cu_w1_i,
    output logic        apu_run_o,
    output logic        pcen_o,
    output logic [1:0]  resp_xfer_o,
    output logic        cu_take_o,
    output logic        cu_load_o,
    output logic [2:0]  cu_idx_o,
    output logic [31:0] cu_data_o,
    output logic        cu_resume_o,
    output logic        relatch_o,
    output logic [15:0] relatch_word_o,
    output logic        relatch_cond_o,
    // The conversion unit's port into the register file, and the FPSR
    // effects of an FMOVE it completed (doc/rd68885.md, stage B).
    input  logic        rfb_we_i,
    input  logic [2:0]  rfb_wa_i,
    input  logic [90:0] rfb_wd_i,
    input  logic        rfb_re_i,
    input  logic [2:0]  rfb_ra_i,
    output logic [90:0] rfb_q_o,
    output logic        prec_x_o,
    input  logic        dcc_v_i,
    input  logic        dcc_cc_v_i,
    input  logic [3:0]  dcc_cc_i,
    output logic        dcc_ack_o
);

  // ==========================================================================
  // The microword
  // ==========================================================================
  logic [rd68884_ucode_pkg::UA-1:0] upc;
  logic [rd68884_ucode_pkg::UA-1:0] addr_nxt;
  logic [rd68884_ucode_pkg::UW-1:0] uw;

  // The microcode store and the entry points of the model's program.
  localparam logic [rd68884_ucode_pkg::UA-1:0] E_RESET =
      (MODEL == 68882) ? rd68884_ucode_pkg::ENTRY82_RESET : rd68884_ucode_pkg::ENTRY_RESET;
  localparam logic [rd68884_ucode_pkg::UA-1:0] E_ABORT =
      (MODEL == 68882) ? rd68884_ucode_pkg::ENTRY82_ABORT : rd68884_ucode_pkg::ENTRY_ABORT;
  localparam logic [rd68884_ucode_pkg::UA-1:0] E_RESTORE =
      (MODEL == 68882) ? rd68884_ucode_pkg::ENTRY82_RESTORE : rd68884_ucode_pkg::ENTRY_RESTORE;
  // The MC68882's AB (FPU 7.2.2): the abort that keeps a pending exception.
  localparam logic [rd68884_ucode_pkg::UA-1:0] E_ABORT_AB =
      (MODEL == 68882) ? rd68884_ucode_pkg::ENTRY82_ABORT_AB : rd68884_ucode_pkg::ENTRY_ABORT;
  localparam logic [15:0] F_IDLE = (MODEL == 68882) ? rd68884_pkg::FRAME82_IDLE : rd68884_pkg::FRAME_IDLE;
  localparam logic [15:0] F_BUSY = (MODEL == 68882) ? rd68884_pkg::FRAME82_BUSY : rd68884_pkg::FRAME_BUSY;

  generate
    if (MODEL == 68882) begin : g_rom82
      rd68885_ucode_rom u_rom (
          .clk  (clk),
          .rst_n(rst_n),
          .addr (addr_nxt),
          .uw   (uw)
      );
    end else begin : g_rom81
      rd68884_ucode_rom u_rom (
          .clk  (clk),
          .rst_n(rst_n),
          .addr (addr_nxt),
          .uw   (uw)
      );
    end
  endgenerate

  logic [rd68884_ucode_pkg::F_SEQ_W-1:0] f_seq;
  logic [rd68884_ucode_pkg::F_NEG_W-1:0] f_neg;
  logic [rd68884_ucode_pkg::F_COND_W-1:0] f_cond;
  logic [rd68884_ucode_pkg::F_IDX_W-1:0] f_idx;
  logic [rd68884_ucode_pkg::F_TGT_W-1:0] f_tgt;
  logic [rd68884_ucode_pkg::F_IMM_W-1:0] f_imm;
  logic [rd68884_ucode_pkg::F_RESP_W-1:0] f_resp;
  logic [rd68884_ucode_pkg::F_ORS_W-1:0] f_ors;
  logic [rd68884_ucode_pkg::F_ONESHOT_W-1:0] f_oneshot;
  logic [rd68884_ucode_pkg::F_EXPECT_W-1:0] f_expect;
  logic [rd68884_ucode_pkg::F_BIU_W-1:0] f_biu;
  logic [rd68884_ucode_pkg::F_XFER_W-1:0] f_xfer;
  logic [rd68884_ucode_pkg::F_TSRC_W-1:0] f_tsrc;
  logic [rd68884_ucode_pkg::F_TDST_W-1:0] f_tdst;
  logic [rd68884_ucode_pkg::F_RF_W-1:0] f_rf;
  logic [rd68884_ucode_pkg::F_RFA_W-1:0] f_rfa;
  logic [rd68884_ucode_pkg::F_ASRC_W-1:0] f_asrc;
  logic [rd68884_ucode_pkg::F_BSRC_W-1:0] f_bsrc;
  logic [rd68884_ucode_pkg::F_XOP_W-1:0] f_xop;
  logic [rd68884_ucode_pkg::F_FPSR_W-1:0] f_fpsr;
  logic [rd68884_ucode_pkg::F_EXCSET_W-1:0] f_excset;
  logic [rd68884_ucode_pkg::F_MOP_W-1:0] f_mop;
  logic [rd68884_ucode_pkg::F_EOP_W-1:0] f_eop;
  logic [rd68884_ucode_pkg::F_SGN_W-1:0] f_sgn;
  logic [rd68884_ucode_pkg::F_PSR_W-1:0] f_psr;
  logic [rd68884_ucode_pkg::F_RMR_W-1:0] f_rmr;
  logic [rd68884_ucode_pkg::F_CTR_W-1:0] f_ctr;
  logic [rd68884_ucode_pkg::F_MASK_W-1:0] f_mask;
  logic [rd68884_ucode_pkg::F_FLAG_W-1:0] f_flag;

  always_comb begin
    f_seq      = uw[rd68884_ucode_pkg::F_SEQ_LSB +: rd68884_ucode_pkg::F_SEQ_W];
    f_neg      = uw[rd68884_ucode_pkg::F_NEG_LSB +: rd68884_ucode_pkg::F_NEG_W];
    f_cond     = uw[rd68884_ucode_pkg::F_COND_LSB +: rd68884_ucode_pkg::F_COND_W];
    f_idx      = uw[rd68884_ucode_pkg::F_IDX_LSB +: rd68884_ucode_pkg::F_IDX_W];
    f_tgt      = uw[rd68884_ucode_pkg::F_TGT_LSB +: rd68884_ucode_pkg::F_TGT_W];
    f_imm      = uw[rd68884_ucode_pkg::F_IMM_LSB +: rd68884_ucode_pkg::F_IMM_W];
    f_resp     = uw[rd68884_ucode_pkg::F_RESP_LSB +: rd68884_ucode_pkg::F_RESP_W];
    f_ors      = uw[rd68884_ucode_pkg::F_ORS_LSB +: rd68884_ucode_pkg::F_ORS_W];
    f_oneshot  = uw[rd68884_ucode_pkg::F_ONESHOT_LSB +: rd68884_ucode_pkg::F_ONESHOT_W];
    f_expect   = uw[rd68884_ucode_pkg::F_EXPECT_LSB +: rd68884_ucode_pkg::F_EXPECT_W];
    f_biu      = uw[rd68884_ucode_pkg::F_BIU_LSB +: rd68884_ucode_pkg::F_BIU_W];
    f_xfer     = uw[rd68884_ucode_pkg::F_XFER_LSB +: rd68884_ucode_pkg::F_XFER_W];
    f_tsrc     = uw[rd68884_ucode_pkg::F_TSRC_LSB +: rd68884_ucode_pkg::F_TSRC_W];
    f_tdst     = uw[rd68884_ucode_pkg::F_TDST_LSB +: rd68884_ucode_pkg::F_TDST_W];
    f_rf       = uw[rd68884_ucode_pkg::F_RF_LSB +: rd68884_ucode_pkg::F_RF_W];
    f_rfa      = uw[rd68884_ucode_pkg::F_RFA_LSB +: rd68884_ucode_pkg::F_RFA_W];
    f_asrc     = uw[rd68884_ucode_pkg::F_ASRC_LSB +: rd68884_ucode_pkg::F_ASRC_W];
    f_bsrc     = uw[rd68884_ucode_pkg::F_BSRC_LSB +: rd68884_ucode_pkg::F_BSRC_W];
    f_xop      = uw[rd68884_ucode_pkg::F_XOP_LSB +: rd68884_ucode_pkg::F_XOP_W];
    f_fpsr     = uw[rd68884_ucode_pkg::F_FPSR_LSB +: rd68884_ucode_pkg::F_FPSR_W];
    f_excset   = uw[rd68884_ucode_pkg::F_EXCSET_LSB +: rd68884_ucode_pkg::F_EXCSET_W];
    f_mop      = uw[rd68884_ucode_pkg::F_MOP_LSB +: rd68884_ucode_pkg::F_MOP_W];
    f_eop      = uw[rd68884_ucode_pkg::F_EOP_LSB +: rd68884_ucode_pkg::F_EOP_W];
    f_sgn      = uw[rd68884_ucode_pkg::F_SGN_LSB +: rd68884_ucode_pkg::F_SGN_W];
    f_psr      = uw[rd68884_ucode_pkg::F_PSR_LSB +: rd68884_ucode_pkg::F_PSR_W];
    f_rmr      = uw[rd68884_ucode_pkg::F_RMR_LSB +: rd68884_ucode_pkg::F_RMR_W];
    f_ctr      = uw[rd68884_ucode_pkg::F_CTR_LSB +: rd68884_ucode_pkg::F_CTR_W];
    f_mask     = uw[rd68884_ucode_pkg::F_MASK_LSB +: rd68884_ucode_pkg::F_MASK_W];
    f_flag     = uw[rd68884_ucode_pkg::F_FLAG_LSB +: rd68884_ucode_pkg::F_FLAG_W];
  end

  // ==========================================================================
  // State
  // ==========================================================================
  logic [rd68884_ucode_pkg::UA-1:0] stack0, stack1, stack2, stack3;
  logic [1:0]  sp;
  logic [15:0] ctr;
  logic [31:0] t;
  logic [7:0]  mask;
  logic [2:0]  rn;
  logic [15:0] cmd;
  logic        is_cond;
  logic [15:0] fpcr;
  logic [31:0] fpsr;
  logic [31:0] xi0, xi1, xi2;
  logic        a_sign;
  logic [17:0] a_exp;
  logic [71:0] a_mant;
  logic        b_sign;
  logic [17:0] b_exp;
  logic [71:0] b_mant;
  logic [87:0] c;                // the multiplier's accumulator, the quotient, the root
  logic [3:0]  rxq;              // the mantissa's carry, borrow, or remainder bits above A
  logic        stk;              // the sticky bit below A
  logic        stkb;             // B's copy of it
  logic [6:0]  sa;               // the shift amount
  logic        rinex;            // the last rounding was inexact
  logic [1:0]  psr;              // the precision (P_X, P_S, P_D, P_SGLX)
  logic [1:0]  rmr;              // the rounding mode (FPU 2.2.2)
  logic        exc_pend;
  logic        null_state;
  logic        pcode;
  logic        run;              // RD68885: a released instruction computes
  logic        ev_resp_q, ev_rsel_q, ev_save_q;
  logic        restore_req_q;

  logic [90:0] rfq;              // the register file's or the constant ROM's read register
  logic        q_crom;           // RFQ holds a constant
  logic        qstk;             // that constant's sticky bit

  // ==========================================================================
  // Traps and conditions
  // ==========================================================================
  logic restore_rise;
  logic trap;
  logic [rd68884_ucode_pkg::UA-1:0] trap_addr;
  always_comb begin
    restore_rise = restore_req_i & ~restore_req_q;
    trap      = arch_reset_i | abort_i | abort_ab_i | restore_rise;
    trap_addr = arch_reset_i ? E_RESET :
                abort_i      ? E_ABORT :
                abort_ab_i   ? E_ABORT_AB :
                               E_RESTORE;
  end

  logic [rd68884_ucode_pkg::UA-1:0] stack_top;
  always_comb begin
    case (sp - 2'd1)
      2'd0:    stack_top = stack0;
      2'd1:    stack_top = stack1;
      2'd2:    stack_top = stack2;
      default: stack_top = stack3;
    endcase
  end

  logic ev_resp, ev_rsel, ev_save;
  assign ev_resp = ev_resp_q | resp_read_i;
  assign ev_rsel = ev_rsel_q | rsel_read_i;
  assign ev_save = ev_save_q | save_read_i;

  logic [2:0] opclass, rx;
  assign opclass = cmd[15:13];
  assign rx      = cmd[12:10];

  // FPU 4.4: the predicate. Table 4-20 note 3: bit 5 is ignored. The 01xxxx
  // predicates set BSUN when NAN is set.
  logic cc_n, cc_z, cc_nan;
  logic eq, gt, lt, un;
  logic tf, bsun;
  always_comb begin
    cc_n   = fpsr[27];
    cc_z   = fpsr[26];
    cc_nan = fpsr[24];
    eq = cc_z;
    gt = ~(cc_nan | cc_z | cc_n);
    lt = cc_n & ~(cc_nan | cc_z);
    un = cc_nan;
    case (cmd[3:0])
      4'h0: tf = 1'b0;
      4'h1: tf = eq;
      4'h2: tf = gt;
      4'h3: tf = gt | eq;
      4'h4: tf = lt;
      4'h5: tf = lt | eq;
      4'h6: tf = gt | lt;
      4'h7: tf = ~un;
      4'h8: tf = un;
      4'h9: tf = un | eq;
      4'hA: tf = un | gt;
      4'hB: tf = un | gt | eq;
      4'hC: tf = un | lt;
      4'hD: tf = un | lt | eq;
      4'hE: tf = ~eq;
      default: tf = 1'b1;
    endcase
    bsun = cmd[4] & cc_nan;
  end

  // FPU 6.1.9 / 6.1.10: the vector of the highest enabled exception, 49 if
  // none (an exception made pending by software, FPU 6.4.2.2).
  logic [7:0] exc, en;
  logic [7:0] vec;
  always_comb begin
    exc = fpsr[15:8];
    en  = fpcr[15:8];
    if      (exc[7] & en[7]) vec = 8'd48;
    else if (exc[6] & en[6]) vec = 8'd54;
    else if (exc[5] & en[5]) vec = 8'd52;
    else if (exc[4] & en[4]) vec = 8'd53;
    else if (exc[3] & en[3]) vec = 8'd51;
    else if (exc[2] & en[2]) vec = 8'd50;
    else                     vec = 8'd49;
  end

  // The inexact bits need no test of their own for the vector: below DZ
  // every case, INEX1, INEX2, an untrapped overflow or none at all, is 49.
  // TRAP: is any exception enabled, the inexact trap on an overflow included.
  logic trap_any;
  assign trap_any = (|(exc & en & 8'hFD)) | (|(exc & 8'h12) & en[1]);

  logic pcen;
  assign pcen = |fpcr[14:8];


  // ==========================================================================
  // The transfer bus
  // ==========================================================================
  logic [31:0] tbus;
  logic [2:0]  frame_code;
  always_comb begin
    frame_code = pcode ? (is_cond ? 3'd1 : 3'd3) : 3'd7;
    case (f_tsrc)
      rd68884_ucode_pkg::TSRC_T:     tbus = t;
      rd68884_ucode_pkg::TSRC_OPW:   tbus = opw_data_i;
      rd68884_ucode_pkg::TSRC_IMM:   tbus = {16'd0, f_imm};
      rd68884_ucode_pkg::TSRC_FPCR:  tbus = {16'd0, fpcr};
      rd68884_ucode_pkg::TSRC_FPSR:  tbus = fpsr;
      rd68884_ucode_pkg::TSRC_FPIAR: tbus = fpiar_i;
      rd68884_ucode_pkg::TSRC_CMDW:  tbus = pcode ? {cmd, 16'hFFFF} : 32'hFFFF_FFFF;
      rd68884_ucode_pkg::TSRC_XI0:   tbus = xi0;
      rd68884_ucode_pkg::TSRC_XI1:   tbus = xi1;
      rd68884_ucode_pkg::TSRC_XI2:   tbus = xi2;
      // FPU figure 6-6 / table 6-4: the BIU flags of the idle frame.
      rd68884_ucode_pkg::TSRC_FLAGS: tbus = {pv_i, frame_code, ~exc_pend, 1'b1, 10'd0, 16'hFFFF};
      rd68884_ucode_pkg::TSRC_RESTW: tbus = {16'd0, restore_word_i};
      // A busy frame's sequencer word (doc/microcode.md).
      rd68884_ucode_pkg::TSRC_SEQST: tbus = {stack_top, mask, rn, is_cond, exc_pend,
                                             ev_resp, ev_rsel, 5'd0};
      // RD68885: the conversion unit's frame word IMM[2:0] (doc/model.md).
      rd68884_ucode_pkg::TSRC_CU:    tbus = (f_imm[2:0] == 3'd0) ? cu_w0_i :
                                            (f_imm[2:0] == 3'd1) ? cu_w1_i :
                                            (f_imm[2:0] == 3'd2) ? cu_d0_i :
                                            (f_imm[2:0] == 3'd3) ? cu_d1_i :
                                            (f_imm[2:0] == 3'd4) ? cu_d2_i : 32'hFFFF_FFFF;
      default:                       tbus = 32'hFFFF_FFFF;
    endcase
  end

  // ==========================================================================
  // The datapath (doc/microcode.md; tools/iss/core.py is the definition)
  // ==========================================================================
  localparam logic [17:0] EXP_SPECIAL = 18'd16384;     // infinity and NaN
  localparam logic [17:0] EXP_ZERO    = 18'h3C001;     // -16383
  localparam logic [1:0]  P_X = 2'd0, P_S = 2'd1, P_D = 2'd2, P_SGLX = 2'd3;
  localparam logic [1:0]  RN = 2'd0, RZ = 2'd1, RM = 2'd2, RP = 2'd3;

  // ---- classification, FPU table 3-3 -----------------------------------------
  logic a_special, a_nan, a_inf, a_zero, a_snan;
  logic b_special, b_nan, b_inf, b_zero, b_snan;
  logic [3:0] a_cc;
  always_comb begin
    a_special = (a_exp == EXP_SPECIAL);
    a_inf  = a_special & (a_mant[70:0] == 71'd0);
    a_nan  = a_special & ~a_inf;
    a_snan = a_nan & ~a_mant[70];
    a_zero = ~a_special & (a_mant == 72'd0);
    b_special = (b_exp == EXP_SPECIAL);
    b_inf  = b_special & (b_mant[70:0] == 71'd0);
    b_nan  = b_special & ~b_inf;
    b_snan = b_nan & ~b_mant[70];
    b_zero = ~b_special & (b_mant == 72'd0);
    // FPU table 2-1: the condition codes look at the extended mantissa only.
    a_cc = {a_sign, ~a_special & (a_mant[71:8] == 64'd0), a_inf, a_nan};
  end

  // ---- the precision: the exponent range ---------------------------------
  logic [17:0] emin, emax;
  always_comb begin
    case (psr)
      P_S:     begin emin = -18'sd126;   emax = 18'd127;   end
      P_D:     begin emin = -18'sd1022;  emax = 18'd1023;  end
      default: begin emin = -18'sd16383; emax = 18'd16383; end
    endcase
  end

  // ---- exponent comparisons and the shift amounts ---------------------------------
  logic [17:0] imm_s;           // IMM sign-extended
  assign imm_s = {{2{f_imm[15]}}, f_imm};

  logic ae_lt_emin, ae_gt_emax, ae_lt_xmin, ae_lt_b, ae_ge_imm;
  always_comb begin
    ae_lt_emin = $signed(a_exp) <  $signed(emin);
    ae_gt_emax = $signed(a_exp) >  $signed(emax);
    ae_lt_xmin = $signed(a_exp) <  $signed(EXP_ZERO);
    ae_lt_b    = $signed(a_exp) <  $signed(b_exp);
    ae_ge_imm  = $signed(a_exp) >= $signed(imm_s);
  end

  // SA <= x - y, clamped to 0..127.
  logic [17:0] sa_x, sa_y;
  logic [18:0] sa_d;
  logic [6:0]  sa_clamped;
  always_comb begin
    case (f_eop)
      rd68884_ucode_pkg::EOP_SA_AB: begin sa_x = a_exp; sa_y = b_exp; end
      rd68884_ucode_pkg::EOP_SA_IA: begin sa_x = imm_s; sa_y = a_exp; end
      default:                      begin sa_x = emin;  sa_y = a_exp; end
    endcase
    sa_d = {sa_x[17], sa_x} - {sa_y[17], sa_y};
    if (sa_d[18])                 sa_clamped = 7'd0;
    else if (sa_d[17:7] != 11'd0) sa_clamped = 7'd127;
    else                          sa_clamped = sa_d[6:0];
  end

  // ---- integers: the width of the command word's format (FPU table 4-15) ------
  logic [1:0] ifmt;             // 0: long, 1: word, 2: byte
  always_comb begin
    case (rx)
      3'd4:    ifmt = 2'd1;
      3'd6:    ifmt = 2'd2;
      default: ifmt = 2'd0;
    endcase
  end

  // INT_OVF: |A| beyond the format's range for A's sign.
  logic [63:0] am64;
  logic        int_ovf;
  always_comb begin
    am64 = a_mant[71:8];
    case (ifmt)
      2'd1:    int_ovf = (am64[63:15] != 49'd0) & ~(a_sign & (am64 == 64'h8000));
      2'd2:    int_ovf = (am64[63:7]  != 57'd0) & ~(a_sign & (am64 == 64'h80));
      default: int_ovf = (am64[63:31] != 33'd0) & ~(a_sign & (am64 == 64'h8000_0000));
    endcase
  end

  // ---- unpacking XI (FPU tables 3-1 to 3-3) --------------------------------------
  logic [17:0] ux_exp;
  logic        us_sign, ud_sign, ui_sign;
  logic [17:0] us_exp, ud_exp, ui_exp;
  logic [71:0] us_mant, ud_mant, ui_mant;
  logic [31:0] ui_v, ui_abs;
  always_comb begin
    ux_exp = {3'b000, xi0[30:16]} - 18'd16383;

    // Single: a denormal keeps the minimum exponent and J = 0.
    us_sign = xi0[31];
    if (xi0[30:23] == 8'hFF) begin
      us_exp  = EXP_SPECIAL;
      us_mant = (xi0[22:0] != 23'd0) ? {1'b1, xi0[22:0], 48'd0} : 72'd0;
    end else if (xi0[30:23] == 8'd0) begin
      us_exp  = (xi0[22:0] != 23'd0) ? -18'sd126 : EXP_ZERO;
      us_mant = {1'b0, xi0[22:0], 48'd0};
    end else begin
      us_exp  = {10'd0, xi0[30:23]} - 18'd127;
      us_mant = {1'b1, xi0[22:0], 48'd0};
    end

    ud_sign = xi0[31];
    if (xi0[30:20] == 11'h7FF) begin
      ud_exp  = EXP_SPECIAL;
      ud_mant = ({xi0[19:0], xi1} != 52'd0) ? {1'b1, xi0[19:0], xi1, 19'd0} : 72'd0;
    end else if (xi0[30:20] == 11'd0) begin
      ud_exp  = ({xi0[19:0], xi1} != 52'd0) ? -18'sd1022 : EXP_ZERO;
      ud_mant = {1'b0, xi0[19:0], xi1, 19'd0};
    end else begin
      ud_exp  = {7'd0, xi0[30:20]} - 18'd1023;
      ud_mant = {1'b1, xi0[19:0], xi1, 19'd0};
    end

    // Integers, left-justified in 32 bits whatever their width.
    case (f_asrc)
      rd68884_ucode_pkg::ASRC_UNPACKW: begin ui_v = {xi0[31:16], 16'd0}; ui_exp = 18'd15; end
      rd68884_ucode_pkg::ASRC_UNPACKB: begin ui_v = {xi0[31:24], 24'd0}; ui_exp = 18'd7;  end
      default:                         begin ui_v = xi0;                 ui_exp = 18'd31; end
    endcase
    ui_abs  = ui_v[31] ? (32'd0 - ui_v) : ui_v;
    ui_sign = ui_v[31];
    ui_mant = {ui_abs, 40'd0};
    if (ui_v == 32'd0) begin
      ui_sign = 1'b0;
      ui_exp  = EXP_ZERO;
    end
  end

  // ---- packing A into XI ------------------------------------------------------------
  logic [14:0] px_be;
  logic [7:0]  ps_e;
  logic [10:0] pd_e;
  logic [31:0] pi_v, pi_q, pni_q, psat_q;
  always_comb begin
    px_be = a_exp[14:0] + 15'd16383;
    if (a_special)       ps_e = 8'hFF;
    else if (~a_mant[71]) ps_e = 8'd0;
    else                 ps_e = a_exp[7:0] + 8'd127;
    if (a_special)       pd_e = 11'h7FF;
    else if (~a_mant[71]) pd_e = 11'd0;
    else                 pd_e = a_exp[10:0] + 11'd1023;
    // FMOVE to an integer: the low bits of the two's complement of A.
    pi_v = a_sign ? (32'd0 - am64[31:0]) : am64[31:0];
    case (ifmt)
      2'd1: begin
        pi_q   = {pi_v[15:0], 16'd0};
        pni_q  = {a_mant[71:56], 16'd0};
        psat_q = a_sign ? 32'h8000_0000 : 32'h7FFF_0000;
      end
      2'd2: begin
        pi_q   = {pi_v[7:0], 24'd0};
        pni_q  = {a_mant[71:64], 24'd0};
        psat_q = a_sign ? 32'h8000_0000 : 32'h7F00_0000;
      end
      default: begin
        pi_q   = pi_v;
        pni_q  = a_mant[71:40];
        psat_q = a_sign ? 32'h8000_0000 : 32'h7FFF_FFFF;
      end
    endcase
  end

  // ---- the shifter: right by SA (SHRA, SHRB), or left by the leading zeros
  // (NORM), as a right shift of the bit-reversed mantissa ----------------------
  logic [71:0] a_rev;
  logic [71:0] sh_in, sh_out, sh_lost;
  logic [6:0]  sh_amt;
  logic [6:0]  lz;
  logic [8:0]  grp_nz;
  logic [26:0] grp_lz;          // three bits a byte
  logic [31:0] lz_i;
  logic        sh_norm;
  logic [71:0] norm_mant;
  always_comb begin
    for (int i = 0; i < 72; i++) a_rev[i] = a_mant[71 - i];

    // Leading zeros: per byte, then the highest non-zero byte.
    for (int g = 0; g < 9; g++) begin
      grp_nz[g] = (a_mant[8 * g +: 8] != 8'd0);
      if      (a_mant[8 * g + 7]) grp_lz[3 * g +: 3] = 3'd0;
      else if (a_mant[8 * g + 6]) grp_lz[3 * g +: 3] = 3'd1;
      else if (a_mant[8 * g + 5]) grp_lz[3 * g +: 3] = 3'd2;
      else if (a_mant[8 * g + 4]) grp_lz[3 * g +: 3] = 3'd3;
      else if (a_mant[8 * g + 3]) grp_lz[3 * g +: 3] = 3'd4;
      else if (a_mant[8 * g + 2]) grp_lz[3 * g +: 3] = 3'd5;
      else if (a_mant[8 * g + 1]) grp_lz[3 * g +: 3] = 3'd6;
      else                        grp_lz[3 * g +: 3] = 3'd7;
    end
    lz_i = 32'd0;
    for (int g = 0; g < 9; g++) begin
      if (grp_nz[g]) lz_i = 64 - 8 * g + {29'd0, grp_lz[3 * g +: 3]};
    end
    lz = lz_i[6:0];

    sh_norm = (f_mop == rd68884_ucode_pkg::MOP_NORM);
    if (sh_norm)                                    sh_in = a_rev;
    else if (f_mop == rd68884_ucode_pkg::MOP_SHRB)  sh_in = b_mant;
    else                                            sh_in = a_mant;
    sh_amt  = sh_norm ? lz : sa;
    sh_out  = sh_in >> sh_amt;
    sh_lost = sh_in & ~({72{1'b1}} << sh_amt);
    for (int i = 0; i < 72; i++) norm_mant[i] = sh_out[71 - i];
  end

  // ---- the rounder (FPU figure 6-3): at the precision's LSB, or extended's ------
  logic        rnd_x;
  logic [71:0] rnd_low, rnd_lsb, rnd_half;
  logic        rnd_g, rnd_rest, rnd_inexact, rnd_qlsb, rnd_inc;
  always_comb begin
    rnd_x = (f_mop == rd68884_ucode_pkg::MOP_ROUNDX) | (psr == P_X);
    if (rnd_x) begin
      rnd_low  = {64'd0, 8'hFF};
      rnd_lsb  = {63'd0, 1'b1, 8'd0};
      rnd_half = {64'd0, 8'h80};
    end else if (psr == P_D) begin
      rnd_low  = {53'd0, {19{1'b1}}};
      rnd_lsb  = {52'd0, 1'b1, 19'd0};
      rnd_half = {53'd0, 1'b1, 18'd0};
    end else begin
      rnd_low  = {24'd0, {48{1'b1}}};
      rnd_lsb  = {23'd0, 1'b1, 48'd0};
      rnd_half = {24'd0, 1'b1, 47'd0};
    end
    rnd_g       = |(a_mant & rnd_half);
    rnd_rest    = |(a_mant & rnd_low & ~rnd_half) | stk;
    rnd_inexact = |(a_mant & rnd_low) | stk;
    rnd_qlsb    = |(a_mant & rnd_lsb);
    case (rmr)
      RN:      rnd_inc = rnd_g & (rnd_rest | rnd_qlsb);
      RM:      rnd_inc = rnd_inexact & a_sign;
      RP:      rnd_inc = rnd_inexact & ~a_sign;
      default: rnd_inc = 1'b0;
    endcase
  end

  // ---- the adder, one for every mantissa sum: X + Y + CIN, 79 bits ------------
  //   ADD      A + B                    SUB      A - B - STK
  //   NEG      0 - A                    ROUND    (A without its low bits) + LSB
  //   DIVSTEP  {RX, A} - B              SQSTEP   (4R + the radicand's next two
  //                                              bits) - (4 root + 1)
  //   MUL10    8A + 2A                  ADDDIG   A + the digit at XI0[3:0]
  //   CMPM     A - B, for its sign and whether it is zero
  // A subtraction's sign is bit 78; ADD's and ROUND's carry, SUB's borrow,
  // bit 72.
  logic [78:0] add_x, add_y, add_s;
  logic        add_cin;
  logic [77:0] sq_r2;
  always_comb begin
    sq_r2 = {rxq, a_mant, b_mant[71:70]};
    add_x   = {7'd0, a_mant};
    add_y   = {7'd0, b_mant};
    add_cin = 1'b0;
    case (f_mop)
      rd68884_ucode_pkg::MOP_SUB: begin
        add_y = ~{7'd0, b_mant};  add_cin = ~stk;
      end
      rd68884_ucode_pkg::MOP_NEG: begin
        add_x = 79'd0;  add_y = ~{7'd0, a_mant};  add_cin = 1'b1;
      end
      rd68884_ucode_pkg::MOP_ROUND,
      rd68884_ucode_pkg::MOP_ROUNDX: begin
        add_x = {7'd0, a_mant & ~rnd_low};
        add_y = rnd_inc ? {7'd0, rnd_lsb} : 79'd0;
      end
      rd68884_ucode_pkg::MOP_DIVSTEP: begin
        add_x = {3'd0, rxq, a_mant};  add_y = ~{7'd0, b_mant};  add_cin = 1'b1;
      end
      rd68884_ucode_pkg::MOP_SQSTEP: begin
        add_x = {1'b0, sq_r2};  add_y = ~{1'b0, c[75:0], 2'b01};  add_cin = 1'b1;
      end
      rd68884_ucode_pkg::MOP_CMPM: begin
        add_y = ~{7'd0, b_mant};  add_cin = 1'b1;
      end
      rd68884_ucode_pkg::MOP_MUL10: begin
        add_x = {7'd0, a_mant[68:0], 3'd0};  add_y = {7'd0, a_mant[70:0], 1'b0};
      end
      rd68884_ucode_pkg::MOP_ADDDIG: add_y = {75'd0, xi0[3:0]};
      default: ;
    endcase
    add_s = add_x + add_y + {78'd0, add_cin};
  end

  // Restoring division and square root keep the remainder if the trial
  // subtraction went negative.
  logic [75:0] div_rr;
  logic        div_q;
  logic [77:0] sq_rr;
  logic        sq_q;
  logic        r_nz;
  always_comb begin
    div_q  = ~add_s[78];
    div_rr = div_q ? add_s[75:0] : {rxq, a_mant};
    sq_q   = (c[87:76] == 12'd0) & ~add_s[78];
    sq_rr  = sq_q ? add_s[77:0] : sq_r2;
    r_nz   = (rxq != 4'd0) | (a_mant != 72'd0);
  end

  // ---- the multiplier: 72 x 16 a clock, accumulated in C ---------------------------
  logic [87:0] mul_prod, mul_acc;
  always_comb begin
    mul_prod = a_mant * b_mant[15:0];
    mul_acc  = {16'd0, c[87:16]} + mul_prod;
  end

  // The bits the arithmetic computes and never reads: the loop counter's
  // top, DIVSTEP's remainder bit 75 (always zero after a step, a remainder
  // is below twice the divisor), SQSTEP's two (zero likewise).
  logic unused_x;
  assign unused_x = &{1'b1, lz_i[31:7], div_rr[75], sq_rr[77:76]};

  // ---- packed decimal ----------------------------------------------------------
  // LOG10: floor(E log10 2) = (E x $4D104D42) >> 32, exact over the range
  // (doc/microcode.md). The digit a division by ten leaves: {RX[0], A[71:69]}.
  logic signed [49:0] log10_p;
  logic [17:0] log10_e;
  logic [3:0]  dig;
  logic [67:0] mdig;            // the mantissa digits {XI0[3:0], XI1, XI2}
  always_comb begin
    log10_p = $signed(a_exp) * $signed(32'h4D104D42);
    log10_e = log10_p[49:32];
    dig     = {rxq[0], a_mant[71:69]};
    mdig    = {xi0[3:0], xi1, xi2};
  end

  // The product's fraction, and the digit DIGR shifts out.
  logic unused_p;
  assign unused_p = &{1'b1, log10_p[31:0], mdig[3:0]};

  // ---- FGETEXP: the exponent as a value ------------------------------------------
  logic [17:0] expf_abs;
  assign expf_abs = a_exp[17] ? (18'd0 - a_exp) : a_exp;

  // ==========================================================================
  // The microcode's conditions
  // ==========================================================================
  logic cond_raw;
  logic cond;
  always_comb begin
    case (f_cond)
      rd68884_ucode_pkg::COND_TRUE:       cond_raw = 1'b1;
      rd68884_ucode_pkg::COND_CMD_PEND:   cond_raw = cmd_pend_i;
      rd68884_ucode_pkg::COND_CMD_COND:   cond_raw = cmd_cond_i;
      rd68884_ucode_pkg::COND_OPW_VALID:  cond_raw = opw_valid_i;
      rd68884_ucode_pkg::COND_OPR_VALID:  cond_raw = opr_valid_i;
      rd68884_ucode_pkg::COND_RESP_READ:  cond_raw = ev_resp;
      rd68884_ucode_pkg::COND_RSEL_READ:  cond_raw = ev_rsel;
      rd68884_ucode_pkg::COND_SAVE_REQ:   cond_raw = save_req_i;
      rd68884_ucode_pkg::COND_SAVE_READ:  cond_raw = ev_save;
      rd68884_ucode_pkg::COND_EXC_PEND:   cond_raw = exc_pend;
      rd68884_ucode_pkg::COND_NULL_STATE: cond_raw = null_state;
      rd68884_ucode_pkg::COND_CTR_ZERO:   cond_raw = (ctr == 16'd0);
      rd68884_ucode_pkg::COND_MASK_ZERO:  cond_raw = (mask == 8'd0);
      rd68884_ucode_pkg::COND_TF:         cond_raw = tf;
      rd68884_ucode_pkg::COND_BSUN:       cond_raw = bsun;
      rd68884_ucode_pkg::COND_BSUN_EN:    cond_raw = fpcr[15];
      rd68884_ucode_pkg::COND_REST_NULL:  cond_raw = (restore_word_i[15:8] == 8'h00);
      rd68884_ucode_pkg::COND_REST_IDLE:  cond_raw = (restore_word_i == F_IDLE);
      rd68884_ucode_pkg::COND_REST_BUSY:  cond_raw = (restore_word_i == F_BUSY);
      rd68884_ucode_pkg::COND_FLINE:      cond_raw = (opclass == 3'd1) |
                                                     (((opclass == 3'd0) | ((opclass == 3'd2) & (rx != 3'd7))) & cmd[6]);
      rd68884_ucode_pkg::COND_REPORTS:    cond_raw = (opclass == 3'd0) | (opclass == 3'd2) | (opclass == 3'd3);
      rd68884_ucode_pkg::COND_PCEN:       cond_raw = pcen;
      rd68884_ucode_pkg::COND_LST_CR:     cond_raw = cmd[12];
      rd68884_ucode_pkg::COND_LST_SR:     cond_raw = cmd[11];
      rd68884_ucode_pkg::COND_LST_IAR:    cond_raw = cmd[10] | (rx == 3'd0);
      rd68884_ucode_pkg::COND_DYN_LIST:   cond_raw = cmd[11];
      rd68884_ucode_pkg::COND_PV:         cond_raw = pv_i;
      rd68884_ucode_pkg::COND_PEND_GEN:   cond_raw = (t[30:28] == 3'd3);
      rd68884_ucode_pkg::COND_PEND_COND:  cond_raw = (t[30:28] == 3'd1);
      rd68884_ucode_pkg::COND_IS_COND:    cond_raw = is_cond;
      rd68884_ucode_pkg::COND_CMD_FMOVE:  cond_raw = (cmd[6:0] == 7'd0);
      rd68884_ucode_pkg::COND_A_ZERO:     cond_raw = a_zero;
      rd68884_ucode_pkg::COND_A_INF:      cond_raw = a_inf;
      rd68884_ucode_pkg::COND_A_NAN:      cond_raw = a_nan;
      rd68884_ucode_pkg::COND_A_SNAN:     cond_raw = a_snan;
      rd68884_ucode_pkg::COND_A_SIGN:     cond_raw = a_sign;
      rd68884_ucode_pkg::COND_B_ZERO:     cond_raw = b_zero;
      rd68884_ucode_pkg::COND_B_INF:      cond_raw = b_inf;
      rd68884_ucode_pkg::COND_B_NAN:      cond_raw = b_nan;
      rd68884_ucode_pkg::COND_B_SNAN:     cond_raw = b_snan;
      rd68884_ucode_pkg::COND_B_SIGN:     cond_raw = b_sign;
      rd68884_ucode_pkg::COND_A_J:        cond_raw = a_mant[71];
      rd68884_ucode_pkg::COND_AE_LT_EMIN: cond_raw = ae_lt_emin;
      rd68884_ucode_pkg::COND_AE_GT_EMAX: cond_raw = ae_gt_emax;
      rd68884_ucode_pkg::COND_AE_LT_XMIN: cond_raw = ae_lt_xmin;
      rd68884_ucode_pkg::COND_AE_LT_B:    cond_raw = ae_lt_b;
      rd68884_ucode_pkg::COND_AE_GE_IMM:  cond_raw = ae_ge_imm;
      rd68884_ucode_pkg::COND_AE_ODD:     cond_raw = a_exp[0];
      rd68884_ucode_pkg::COND_RINEX:      cond_raw = rinex;
      rd68884_ucode_pkg::COND_RX0:        cond_raw = rxq[0];
      rd68884_ucode_pkg::COND_OVF_INF:    cond_raw = (rmr == RN) | ((rmr == RM) & a_sign) |
                                                     ((rmr == RP) & ~a_sign);
      rd68884_ucode_pkg::COND_INT_OVF:    cond_raw = int_ovf;
      rd68884_ucode_pkg::COND_AE_EQ_B:    cond_raw = (a_exp == b_exp);
      rd68884_ucode_pkg::COND_RX1:        cond_raw = rxq[1];
      rd68884_ucode_pkg::COND_TRAP:       cond_raw = trap_any;
      rd68884_ucode_pkg::COND_SUPPRESS:   cond_raw = |(exc & en & 8'h64);
      rd68884_ucode_pkg::COND_PREC_X:     cond_raw = (psr == P_X);
      rd68884_ucode_pkg::COND_SIGN_XOR:   cond_raw = a_sign ^ b_sign;
      rd68884_ucode_pkg::COND_RM_MODE:    cond_raw = (rmr == RM);
      rd68884_ucode_pkg::COND_PREC_SGLX:  cond_raw = (psr == P_SGLX);
      rd68884_ucode_pkg::COND_P_SPECIAL:  cond_raw = (xi0[30:28] == 3'b111) & (xi0[27:16] == 12'hFFF);
      rd68884_ucode_pkg::COND_P_SE:       cond_raw = xi0[30];
      rd68884_ucode_pkg::COND_K_GT17:     cond_raw = ~mask[6] & (mask[5:0] > 6'd17);
      rd68884_ucode_pkg::COND_K_POS:      cond_raw = ~mask[6] & (mask[5:0] != 6'd0);
      rd68884_ucode_pkg::COND_Q_ODD:      cond_raw = c[0];
      rd68884_ucode_pkg::COND_A_POW2:     cond_raw = (a_mant == {1'b1, 71'd0});
      rd68884_ucode_pkg::COND_PCODE:      cond_raw = pcode;
      rd68884_ucode_pkg::COND_CU_READY:   cond_raw = cu_ready_i;
      rd68884_ucode_pkg::COND_CU_VALID:   cond_raw = cu_valid_i;
      rd68884_ucode_pkg::COND_CU_MID:     cond_raw = cu_mid_i;
      default:                            cond_raw = 1'b0;
    endcase
    cond = cond_raw ^ f_neg;
  end

  // The FMOVEM mask: the highest set bit, and the register it names.
  logic [2:0] mbit;
  always_comb begin
    casez (mask)
      8'b1???????: mbit = 3'd7;
      8'b01??????: mbit = 3'd6;
      8'b001?????: mbit = 3'd5;
      8'b0001????: mbit = 3'd4;
      8'b00001???: mbit = 3'd3;
      8'b000001??: mbit = 3'd2;
      8'b0000001?: mbit = 3'd1;
      default:     mbit = 3'd0;
    endcase
  end

  // ==========================================================================
  // The register file
  // ==========================================================================
  logic [6:0] rf_addr;
  always_comb begin
    case (f_rfa)
      rd68884_ucode_pkg::RFA_IMM:   rf_addr = f_imm[6:0];
      rd68884_ucode_pkg::RFA_RX:    rf_addr = {4'd0, rx};
      rd68884_ucode_pkg::RFA_RY:    rf_addr = {4'd0, cmd[9:7]};
      rd68884_ucode_pkg::RFA_RN:    rf_addr = {4'd0, rn};
      rd68884_ucode_pkg::RFA_ETEMP: rf_addr = 7'd8;
      rd68884_ucode_pkg::RFA_FPC:   rf_addr = {4'd0, cmd[2:0]};
      default:                      rf_addr = ctr[6:0];
    endcase
  end

  logic rf_we, rf_re, cr_re;
  assign rf_we = ~trap & (f_rf == rd68884_ucode_pkg::RF_WRITE);
  assign rf_re = ~trap & (f_rf == rd68884_ucode_pkg::RF_READ);
  assign cr_re = ~trap & (f_rf == rd68884_ucode_pkg::RF_CROM);

  logic [90:0] rf_q;
  rd68884_regfile #(.PORT_B(MODEL == 68882 ? 1 : 0)) u_rf (
      .clk(clk),
      .we (rf_we),
      .re (rf_re),
      .a  (rf_addr),
      .wd ({a_sign, a_exp, a_mant}),
      .q  (rf_q),
      .web(rfb_we_i),
      .reb(rfb_re_i),
      .ab ({4'd0, rfb_we_i ? rfb_wa_i : rfb_ra_i}),   // never both in a clock
      .wdb(rfb_wd_i),
      .qb (rfb_q_o)
  );

  // The constant ROM, at IMM + an offset (tools/ucode/fields.py, RF CROM).
  logic [9:0]  cr_off;
  logic [10:0] cr_addr;
  always_comb begin
    case (f_rfa)
      rd68884_ucode_pkg::RFA_CMD: cr_off = {3'd0, cmd[6:0]};
      rd68884_ucode_pkg::RFA_ELO: cr_off = {4'd0, a_exp[5:0]};
      rd68884_ucode_pkg::RFA_EHI: cr_off = {3'd0, a_exp[12:6]};
      rd68884_ucode_pkg::RFA_EXP: cr_off = a_exp[9:0];
      rd68884_ucode_pkg::RFA_PSR: cr_off = {8'd0, psr};
      default:                    cr_off = 10'd0;
    endcase
    cr_addr = f_imm[10:0] + {1'b0, cr_off};
  end

  logic [91:0] cr_q;
  rd68884_crom u_crom (
      .clk (clk),
      .re  (cr_re),
      .addr(cr_addr),
      .q   (cr_q)
  );

  assign rfq  = q_crom ? cr_q[90:0] : rf_q;
  assign qstk = q_crom & cr_q[91];

  // ==========================================================================
  // To the BIU
  // ==========================================================================
  logic [15:0] ors;
  always_comb begin
    ors = 16'd0;
    case (f_ors)
      rd68884_ucode_pkg::ORS_PC:   ors = {1'b0, pcen, 14'd0};
      rd68884_ucode_pkg::ORS_TF:   ors = {15'd0, tf};
      rd68884_ucode_pkg::ORS_VEC:  ors = {8'd0, vec};
      rd68884_ucode_pkg::ORS_DN:   ors = {13'd0, cmd[6:4]};
      rd68884_ucode_pkg::ORS_DNPC: ors = {1'b0, pcen, 11'd0, cmd[6:4]};
      default:                     ors = 16'd0;
    endcase
  end

  always_comb begin
    resp_we_o      = ~trap & (f_resp != rd68884_ucode_pkg::RESP_NONE);
    resp_o         = f_imm | ors;
    resp_oneshot_o = f_oneshot;
    expect_o       = f_expect;
    resp_cond_o    = (f_resp == rd68884_ucode_pkg::RESP_WRC);
    cmd_ack_o      = ~trap & (f_biu == rd68884_ucode_pkg::BIU_CMD_ACK);
    opw_ack_o      = ~trap & (f_biu == rd68884_ucode_pkg::BIU_OPW_ACK);
    opr_we_o       = ~trap & (f_biu == rd68884_ucode_pkg::BIU_OPR_WR);
    opr_o          = tbus;
    rsel_we_o      = ~trap & (f_biu == rd68884_ucode_pkg::BIU_RSEL_WR);
    rsel_o         = mask;
    rsel_dir_o     = cmd[13];
    save_we_o      = ~trap & (f_biu == rd68884_ucode_pkg::BIU_SAVE_WR);
    save_o         = tbus[15:0];
    save_xfer_o    = f_xfer;
    restore_we_o   = ~trap & (f_biu == rd68884_ucode_pkg::BIU_RESTORE_WR);
    restore_o      = tbus[15:0];
    restore_xfer_o = f_xfer;
    fpiar_we_o     = ~trap & (f_biu == rd68884_ucode_pkg::BIU_FPIAR_WR);
    fpiar_o        = tbus;
    clear_o        = ~trap & (f_biu == rd68884_ucode_pkg::BIU_CLEAR);
    apu_run_o      = run;
    pcen_o         = pcen;
    resp_xfer_o    = f_xfer[1:0];
    cu_take_o      = ~trap & (f_biu == rd68884_ucode_pkg::BIU_CU_TAKE);
    cu_resume_o    = ~trap & (f_biu == rd68884_ucode_pkg::BIU_CU_RESUME);
    relatch_o      = ~trap & (f_biu == rd68884_ucode_pkg::BIU_RELATCH);
    relatch_word_o = cmd;
    relatch_cond_o = is_cond;
    cu_load_o      = ~trap & (f_tdst == rd68884_ucode_pkg::TDST_CU);
    cu_idx_o       = f_imm[2:0];
    cu_data_o      = tbus;
    prec_x_o       = (fpcr[7:6] == 2'b00) | (fpcr[7:6] == 2'b11);
    dcc_ack_o      = ~trap & dcc_v_i & ~run;
  end

  // ==========================================================================
  // The next micro-address
  // ==========================================================================
  logic [rd68884_ucode_pkg::UA-1:0] upc_inc;
  logic [rd68884_ucode_pkg::UA-1:0] seq_nxt;
  logic [6:0]  idx;
  always_comb begin
    upc_inc = upc + 1'b1;
    case (f_idx)
      rd68884_ucode_pkg::IDX_OPCLASS: idx = {4'd0, opclass};
      rd68884_ucode_pkg::IDX_RX:      idx = {4'd0, rx};
      default:                        idx = cmd[6:0];
    endcase
    case (f_seq)
      rd68884_ucode_pkg::SEQ_JUMP: seq_nxt = f_tgt;
      rd68884_ucode_pkg::SEQ_CALL: seq_nxt = f_tgt;
      rd68884_ucode_pkg::SEQ_RET: begin
        case (sp - 2'd1)
          2'd0:    seq_nxt = stack0;
          2'd1:    seq_nxt = stack1;
          2'd2:    seq_nxt = stack2;
          default: seq_nxt = stack3;
        endcase
      end
      rd68884_ucode_pkg::SEQ_BR:   seq_nxt = cond ? f_tgt : upc_inc;
      rd68884_ucode_pkg::SEQ_WAIT: seq_nxt = cond ? upc_inc : upc;
      rd68884_ucode_pkg::SEQ_DISP: seq_nxt = f_tgt | {{(rd68884_ucode_pkg::UA-7){1'b0}}, idx};
      rd68884_ucode_pkg::SEQ_LOOP: seq_nxt = (ctr != 16'd0) ? f_tgt : upc_inc;
      default:                     seq_nxt = upc_inc;
    endcase
    addr_nxt = trap ? trap_addr : seq_nxt;
  end

  // ==========================================================================
  // The arithmetic registers' next state. Where fields write the same
  // register in one clock the later wins, in the order of tools/iss/core.py:
  // ASRC/BSRC, MOP, EOP, SGN.
  // ==========================================================================
  logic        an_sign, bn_sign;
  logic [17:0] an_exp, bn_exp;
  logic [71:0] an_mant, bn_mant;
  logic [87:0] cn;
  logic [3:0]  rxn;
  logic        stkn, stkbn, rinexn;
  logic [6:0]  san;
  logic [1:0]  psrn, rmrn;
  always_comb begin
    an_sign = a_sign;  an_exp = a_exp;  an_mant = a_mant;
    bn_sign = b_sign;  bn_exp = b_exp;  bn_mant = b_mant;
    cn = c;  rxn = rxq;  stkn = stk;  stkbn = stkb;
    san = sa;  rinexn = rinex;  psrn = psr;  rmrn = rmr;

    case (f_asrc)
      rd68884_ucode_pkg::ASRC_RFQ: begin
        an_sign = rfq[90];  an_exp = rfq[89:72];  an_mant = rfq[71:0];  stkn = stk | qstk;
      end
      rd68884_ucode_pkg::ASRC_UNPACKX: begin
        an_sign = xi0[31];  an_exp = ux_exp;  an_mant = {xi1, xi2, 8'd0};
      end
      rd68884_ucode_pkg::ASRC_NAN: begin
        an_sign = 1'b0;  an_exp = EXP_SPECIAL;  an_mant = {64'hFFFF_FFFF_FFFF_FFFF, 8'd0};
      end
      rd68884_ucode_pkg::ASRC_ZERO: begin
        an_sign = 1'b0;  an_exp = EXP_ZERO;  an_mant = 72'd0;
      end
      rd68884_ucode_pkg::ASRC_B: begin
        an_sign = b_sign;  an_exp = b_exp;  an_mant = b_mant;  stkn = stkb;
      end
      rd68884_ucode_pkg::ASRC_UNPACKS: begin
        an_sign = us_sign;  an_exp = us_exp;  an_mant = us_mant;
      end
      rd68884_ucode_pkg::ASRC_UNPACKD: begin
        an_sign = ud_sign;  an_exp = ud_exp;  an_mant = ud_mant;
      end
      rd68884_ucode_pkg::ASRC_UNPACKL,
      rd68884_ucode_pkg::ASRC_UNPACKW,
      rd68884_ucode_pkg::ASRC_UNPACKB: begin
        an_sign = ui_sign;  an_exp = ui_exp;  an_mant = ui_mant;
      end
      default: ;
    endcase
    case (f_bsrc)
      rd68884_ucode_pkg::BSRC_RFQ: begin
        bn_sign = rfq[90];  bn_exp = rfq[89:72];  bn_mant = rfq[71:0];
      end
      rd68884_ucode_pkg::BSRC_A: begin
        bn_sign = a_sign;  bn_exp = a_exp;  bn_mant = a_mant;  stkbn = stk;
      end
      default: ;
    endcase

    case (f_mop)
      rd68884_ucode_pkg::MOP_ADD,
      rd68884_ucode_pkg::MOP_SUB: begin
        an_mant = add_s[71:0];  rxn = {3'd0, add_s[72]};
      end
      rd68884_ucode_pkg::MOP_NEG: begin
        an_mant = add_s[71:0];  rxn = 4'd0;
      end
      rd68884_ucode_pkg::MOP_SHRA: begin
        an_mant = sh_out;  stkn = stk | (|sh_lost);
      end
      rd68884_ucode_pkg::MOP_SHRB: begin
        bn_mant = sh_out;  stkn = stk | (|sh_lost);
      end
      rd68884_ucode_pkg::MOP_NORM: begin
        if (a_mant != 72'd0) begin
          an_mant = norm_mant;  an_exp = a_exp - {11'd0, lz};
        end
      end
      rd68884_ucode_pkg::MOP_RSH1: begin
        if (rxq[0]) begin
          stkn = stk | a_mant[0];
          an_mant = {1'b1, a_mant[71:1]};
          an_exp = a_exp + 18'd1;
          rxn = 4'd0;
        end
      end
      rd68884_ucode_pkg::MOP_ROUND,
      rd68884_ucode_pkg::MOP_ROUNDX: begin
        an_mant = add_s[72] ? {1'b1, 71'd0} : add_s[71:0];
        an_exp  = a_exp + {17'd0, add_s[72]};
        rinexn  = rnd_inexact;
        stkn    = 1'b0;
      end
      rd68884_ucode_pkg::MOP_CMPM:   rxn = {2'd0, (add_s[71:0] == 72'd0), add_s[78]};
      rd68884_ucode_pkg::MOP_CLRQ: begin
        cn = 88'd0;  rxn = 4'd0;  stkn = 1'b0;
      end
      rd68884_ucode_pkg::MOP_MULSTEP: begin
        cn = mul_acc;
        stkn = stk | (c[15:0] != 16'd0);
        bn_mant = {c[15:0], b_mant[71:16]};    // the dropped bits (doc/microcode.md)
      end
      rd68884_ucode_pkg::MOP_MUL10,
      rd68884_ucode_pkg::MOP_ADDDIG: an_mant = add_s[71:0];
      rd68884_ucode_pkg::MOP_QINC:   cn = {c[87:7], c[6:0] + 7'd1};
      rd68884_ucode_pkg::MOP_MULFIN: begin              // a NORM follows (doc/microcode.md)
        an_mant = c[79:8];  an_exp = a_exp + 18'd1;  stkn = stk | (c[7:0] != 8'd0);
      end
      rd68884_ucode_pkg::MOP_DIVSTEP: begin
        cn = {8'd0, c[78:0], div_q};
        an_mant = {div_rr[70:0], 1'b0};
        rxn = div_rr[74:71];
      end
      rd68884_ucode_pkg::MOP_DIVFIN: begin              // a NORM follows
        an_mant = c[73:2];  stkn = stk | (c[1:0] != 2'd0) | r_nz;
        rxn = 4'd0;
      end
      rd68884_ucode_pkg::MOP_SQSTEP: begin
        bn_mant = {b_mant[69:0], 2'b00};
        cn = {8'd0, c[78:0], sq_q};
        an_mant = sq_rr[71:0];
        rxn = sq_rr[75:72];
      end
      rd68884_ucode_pkg::MOP_SQFIN: begin
        an_mant = c[71:0];  stkn = stk | r_nz;  rxn = 4'd0;
      end
      rd68884_ucode_pkg::MOP_EXPF: begin
        if (a_exp == 18'd0) begin
          an_sign = 1'b0;  an_exp = EXP_ZERO;  an_mant = 72'd0;
        end else begin
          an_sign = a_exp[17];  an_exp = 18'd17;  an_mant = {expf_abs, 54'd0};
        end
      end
      rd68884_ucode_pkg::MOP_QUIET: an_mant = a_mant | {2'b01, 70'd0};
      rd68884_ucode_pkg::MOP_INFA: begin
        an_exp = EXP_SPECIAL;  an_mant = 72'd0;
      end
      rd68884_ucode_pkg::MOP_ZEROM: begin
        an_exp = EXP_ZERO;  an_mant = 72'd0;
      end
      default: ;
    endcase

    case (f_eop)
      rd68884_ucode_pkg::EOP_ADDB:    an_exp = a_exp + b_exp;
      rd68884_ucode_pkg::EOP_SUBB:    an_exp = a_exp - b_exp;
      rd68884_ucode_pkg::EOP_ADDI:    an_exp = a_exp + imm_s;
      rd68884_ucode_pkg::EOP_LDI:     an_exp = imm_s;
      rd68884_ucode_pkg::EOP_SA_AB,
      rd68884_ucode_pkg::EOP_SA_IA,
      rd68884_ucode_pkg::EOP_SA_EMIN: san = sa_clamped;
      rd68884_ucode_pkg::EOP_SA_IMM:  san = f_imm[6:0];
      rd68884_ucode_pkg::EOP_LDEMIN:  an_exp = emin;
      rd68884_ucode_pkg::EOP_HALF:    an_exp = {a_exp[17], a_exp[17:1]};
      rd68884_ucode_pkg::EOP_LDB:     an_exp = b_exp;
      rd68884_ucode_pkg::EOP_ADDBI:   an_exp = b_sign ? (a_exp - {2'd0, b_mant[23:8]})
                                                      : (a_exp + {2'd0, b_mant[23:8]});
      rd68884_ucode_pkg::EOP_NEGE:    an_exp = 18'd0 - a_exp;
      rd68884_ucode_pkg::EOP_EXP10:   an_exp = {a_exp[14:0], 3'd0} + {a_exp[16:0], 1'b0}
                                               + {14'd0, xi0[27:24]};
      rd68884_ucode_pkg::EOP_LOG10:   an_exp = log10_e;
      rd68884_ucode_pkg::EOP_LDK:     an_exp = {{11{mask[6]}}, mask[6:0]};
      rd68884_ucode_pkg::EOP_SUBK:    an_exp = a_exp - {{11{mask[6]}}, mask[6:0]};
      rd68884_ucode_pkg::EOP_LDM:     an_exp = a_sign ? (18'd0 - a_mant[25:8]) : a_mant[25:8];
      rd68884_ucode_pkg::EOP_LO6:     an_exp = {12'd0, a_exp[5:0]};
      default: ;
    endcase

    case (f_sgn)
      rd68884_ucode_pkg::SGN_NEG: an_sign = ~a_sign;
      rd68884_ucode_pkg::SGN_ABS: an_sign = 1'b0;
      rd68884_ucode_pkg::SGN_XOR: an_sign = a_sign ^ b_sign;
      rd68884_ucode_pkg::SGN_RMZ: an_sign = (rmr == RM);
      rd68884_ucode_pkg::SGN_XI0: an_sign = xi0[31];
      default: ;
    endcase

    // FPU figure 2-3: precision 11 is undefined; extended (doc/model.md).
    case (f_psr)
      rd68884_ucode_pkg::PSR_FPCR: psrn = (fpcr[7:6] == 2'd3) ? P_X : fpcr[7:6];
      rd68884_ucode_pkg::PSR_X:    psrn = P_X;
      rd68884_ucode_pkg::PSR_S:    psrn = P_S;
      rd68884_ucode_pkg::PSR_D:    psrn = P_D;
      rd68884_ucode_pkg::PSR_SGLX: psrn = P_SGLX;
      default: ;
    endcase
    case (f_rmr)
      rd68884_ucode_pkg::RMR_FPCR: rmrn = fpcr[5:4];
      rd68884_ucode_pkg::RMR_RZ:   rmrn = RZ;
      rd68884_ucode_pkg::RMR_RN:   rmrn = RN;
      rd68884_ucode_pkg::RMR_RM:   rmrn = RM;
      default: ;
    endcase

    if (f_flag == rd68884_ucode_pkg::FLAG_SET_STK) stkn = 1'b1;
    if (f_flag == rd68884_ucode_pkg::FLAG_CLR_STK) stkn = 1'b0;
  end

  // FPSR: the bus, then the condition codes and the EXC clear, then the flags
  // that OR into it -- the order of tools/iss/core.py.
  logic [31:0] fpsr_nxt;
  always_comb begin
    fpsr_nxt = fpsr;
    if (f_tdst == rd68884_ucode_pkg::TDST_FPSR) fpsr_nxt = tbus & 32'h0FFF_FFF8;
    if (f_fpsr == rd68884_ucode_pkg::FPSR_CC || f_fpsr == rd68884_ucode_pkg::FPSR_CC_CLREXC)
      fpsr_nxt[27:24] = a_cc;
    if (f_fpsr == rd68884_ucode_pkg::FPSR_CLREXC || f_fpsr == rd68884_ucode_pkg::FPSR_CC_CLREXC)
      fpsr_nxt[15:8] = 8'd0;
    // FPU 2.3.4 / 6.1.10: the accrued byte from the exception byte.
    if (f_fpsr == rd68884_ucode_pkg::FPSR_ACCRUE)
      fpsr_nxt[7:3] = fpsr_nxt[7:3] | {|exc[7:5], exc[4], exc[3] & exc[1], exc[2],
                                       exc[4] | exc[1] | exc[0]};
    if (f_fpsr == rd68884_ucode_pkg::FPSR_CCIMM) fpsr_nxt[27:24] = f_imm[3:0];
    if (f_fpsr == rd68884_ucode_pkg::FPSR_QSIGN) fpsr_nxt[23]    = a_sign ^ b_sign;
    if (f_fpsr == rd68884_ucode_pkg::FPSR_QBITS) fpsr_nxt[22:16] = c[6:0];
    fpsr_nxt[15:8] = fpsr_nxt[15:8] | f_excset;
    if (f_flag == rd68884_ucode_pkg::FLAG_BSUN)  fpsr_nxt = fpsr_nxt | 32'h0000_8080;
    if (f_flag == rd68884_ucode_pkg::FLAG_RESET) fpsr_nxt = 32'd0;
    // RD68885: an FMOVE the CU completed while the APU computed lands once
    // no released instruction computes, in program order: its exception
    // byte cleared, its condition codes (tools/iss/core.py).
    if (dcc_v_i && !run) begin
      fpsr_nxt[15:8] = 8'd0;
      if (dcc_cc_v_i) fpsr_nxt[27:24] = dcc_cc_i;
    end
  end

  // ==========================================================================
  // The clock
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      upc           <= E_RESET;
      stack0        <= '0;
      stack1        <= '0;
      stack2        <= '0;
      stack3        <= '0;
      sp            <= 2'd0;
      ctr           <= 16'd0;
      t             <= 32'd0;
      mask          <= 8'd0;
      rn            <= 3'd0;
      cmd           <= 16'd0;
      is_cond       <= 1'b0;
      fpcr          <= 16'd0;
      fpsr          <= 32'd0;
      xi0           <= 32'd0;
      xi1           <= 32'd0;
      xi2           <= 32'd0;
      a_sign        <= 1'b0;
      a_exp         <= -18'sd16383;
      a_mant        <= 72'd0;
      b_sign        <= 1'b0;
      b_exp         <= -18'sd16383;
      b_mant        <= 72'd0;
      c             <= 88'd0;
      q_crom        <= 1'b0;
      rxq           <= 4'd0;
      stk           <= 1'b0;
      stkb          <= 1'b0;
      sa            <= 7'd0;
      rinex         <= 1'b0;
      psr           <= 2'd0;
      rmr           <= 2'd0;
      exc_pend      <= 1'b0;
      null_state    <= 1'b0;
      pcode         <= 1'b0;
      run           <= 1'b0;
      ev_resp_q     <= 1'b0;
      ev_rsel_q     <= 1'b0;
      ev_save_q     <= 1'b0;
      restore_req_q <= 1'b0;
    end else begin
      restore_req_q <= restore_req_i;
      upc           <= addr_nxt;
      if (trap) begin
        ev_resp_q <= ev_resp;
        ev_rsel_q <= ev_rsel;
        ev_save_q <= ev_save;
        run       <= 1'b0;
      end else begin
        // The sticky events, re-armed by the action that awaits the next.
        ev_resp_q <= ev_resp & (f_resp == rd68884_ucode_pkg::RESP_NONE);
        ev_rsel_q <= ev_rsel & (f_biu != rd68884_ucode_pkg::BIU_RSEL_WR);
        if (f_tdst == rd68884_ucode_pkg::TDST_SEQST) begin
          ev_resp_q <= tbus[6];
          ev_rsel_q <= tbus[5];
        end
        ev_save_q <= ev_save & (f_biu != rd68884_ucode_pkg::BIU_SAVE_WR);

        t <= tbus;

        // ---- the transfer bus's destination ----------------------------
        case (f_tdst)
          rd68884_ucode_pkg::TDST_FPCR:  fpcr <= tbus[15:0];
          rd68884_ucode_pkg::TDST_MASK:  mask <= tbus[7:0];
          rd68884_ucode_pkg::TDST_XI0:   xi0  <= tbus;
          rd68884_ucode_pkg::TDST_XI1:   xi1  <= tbus;
          rd68884_ucode_pkg::TDST_XI2:   xi2  <= tbus;
          rd68884_ucode_pkg::TDST_CMD:   cmd  <= tbus[31:16];
          rd68884_ucode_pkg::TDST_FLAGS: exc_pend <= ~tbus[27];
          rd68884_ucode_pkg::TDST_SEQST: begin
            mask      <= tbus[19:12];
            rn        <= tbus[11:9];
            is_cond   <= tbus[8];
            exc_pend  <= tbus[7];
          end
          default: ;
        endcase

        fpsr <= fpsr_nxt;

        // ---- the BIU's side effects in here -----------------------------
        if (f_biu == rd68884_ucode_pkg::BIU_CMD_ACK) begin
          cmd     <= cmd_word_i;
          is_cond <= cmd_cond_i;
        end
        if (f_biu == rd68884_ucode_pkg::BIU_CU_TAKE) begin
          // The CU's instruction: its command word and its operand.
          cmd     <= cu_word_i;
          is_cond <= 1'b0;
          xi0     <= cu_d0_i;
          xi1     <= cu_d1_i;
          xi2     <= cu_d2_i;
        end

        // ---- the arithmetic registers ---------------------------------------
        a_sign <= an_sign;
        a_exp  <= an_exp;
        a_mant <= an_mant;
        b_sign <= bn_sign;
        b_exp  <= bn_exp;
        b_mant <= bn_mant;
        c      <= cn;
        rxq    <= rxn;
        stk    <= stkn;
        if (cr_re)      q_crom <= 1'b1;
        else if (rf_re) q_crom <= 1'b0;
        stkb   <= stkbn;
        sa     <= san;
        rinex  <= rinexn;
        psr    <= psrn;
        rmr    <= rmrn;

        // ---- XI <= A packed, after the bus's write -------------------------
        case (f_xop)
          rd68884_ucode_pkg::XOP_PACKX: begin
            xi0 <= {a_sign, px_be, 16'd0};
            xi1 <= a_mant[71:40];
            xi2 <= a_mant[39:8];
          end
          rd68884_ucode_pkg::XOP_PACKS:   xi0 <= {a_sign, ps_e, a_mant[70:48]};
          rd68884_ucode_pkg::XOP_PACKD: begin
            xi0 <= {a_sign, pd_e, a_mant[70:51]};
            xi1 <= a_mant[50:19];
          end
          rd68884_ucode_pkg::XOP_PACKI:   xi0 <= pi_q;
          rd68884_ucode_pkg::XOP_PACKNI:  xi0 <= pni_q;
          rd68884_ucode_pkg::XOP_PACKSAT: xi0 <= psat_q;
          rd68884_ucode_pkg::XOP_DIGL: begin
            xi0[3:0] <= xi1[31:28];
            xi1      <= {xi1[27:0], xi2[31:28]};
            xi2      <= {xi2[27:0], 4'd0};
          end
          rd68884_ucode_pkg::XOP_EDIGL:   xi0[27:16] <= {xi0[23:16], 4'd0};
          rd68884_ucode_pkg::XOP_DIGR: begin
            xi0[3:0] <= dig;
            xi1      <= mdig[67:36];
            xi2      <= mdig[35:4];
          end
          rd68884_ucode_pkg::XOP_EDIGR:   xi0[27:16] <= {dig, xi0[27:20]};
          rd68884_ucode_pkg::XOP_EDIG3:   xi0[15:12] <= dig;
          rd68884_ucode_pkg::XOP_PINIT: begin
            xi0 <= {a_sign, a_exp[17], 30'd0};
            xi1 <= 32'd0;
            xi2 <= 32'd0;
          end
          default: ;
        endcase

        // ---- counters, mask, flags ------------------------------------------
        if (f_ctr == rd68884_ucode_pkg::CTR_LOAD) begin
          ctr <= f_imm;
        end else if (f_ctr == rd68884_ucode_pkg::CTR_DEC) begin
          ctr <= ctr - 16'd1;
        end else if (f_ctr == rd68884_ucode_pkg::CTR_LOADE) begin
          ctr <= a_exp[15:0];
        end
        if (f_seq == rd68884_ucode_pkg::SEQ_LOOP && ctr != 16'd0) begin
          ctr <= ctr - 16'd1;
        end
        if (f_mask == rd68884_ucode_pkg::MASK_LOAD) begin
          mask <= cmd[7:0];
        end else if (f_mask == rd68884_ucode_pkg::MASK_NEXT && mask != 8'd0) begin
          rn         <= cmd[12] ? (3'd7 - mbit) : mbit;
          mask[mbit] <= 1'b0;
        end
        case (f_flag)
          rd68884_ucode_pkg::FLAG_SET_EXC:   exc_pend   <= 1'b1;
          rd68884_ucode_pkg::FLAG_CLR_EXC:   exc_pend   <= 1'b0;
          rd68884_ucode_pkg::FLAG_SET_NULL:  null_state <= 1'b1;
          rd68884_ucode_pkg::FLAG_CLR_NULL:  null_state <= 1'b0;
          rd68884_ucode_pkg::FLAG_RESET: begin
            fpcr       <= 16'd0;
            exc_pend   <= 1'b0;
            null_state <= 1'b1;
          end
          rd68884_ucode_pkg::FLAG_PEND_NONE: pcode   <= 1'b0;
          rd68884_ucode_pkg::FLAG_PEND_CMD:  pcode   <= 1'b1;
          rd68884_ucode_pkg::FLAG_SET_COND:  is_cond <= 1'b1;
          rd68884_ucode_pkg::FLAG_CLR_COND:  is_cond <= 1'b0;
          rd68884_ucode_pkg::FLAG_SET_RUN:   run     <= 1'b1;
          rd68884_ucode_pkg::FLAG_CLR_RUN:   run     <= 1'b0;
          default: ;
        endcase

        // ---- the return stack ------------------------------------------------
        if (f_tdst == rd68884_ucode_pkg::TDST_SEQST) begin
          // A busy frame's resume address (never with a CALL or RET).
          case (sp)
            2'd0:    stack0 <= tbus[31:20];
            2'd1:    stack1 <= tbus[31:20];
            2'd2:    stack2 <= tbus[31:20];
            default: stack3 <= tbus[31:20];
          endcase
          sp <= sp + 2'd1;
        end else if (f_seq == rd68884_ucode_pkg::SEQ_CALL) begin
          case (sp)
            2'd0:    stack0 <= upc_inc;
            2'd1:    stack1 <= upc_inc;
            2'd2:    stack2 <= upc_inc;
            default: stack3 <= upc_inc;
          endcase
          sp <= sp + 2'd1;
        end else if (f_seq == rd68884_ucode_pkg::SEQ_RET) begin
          sp <= sp - 2'd1;
        end
      end
    end
  end

endmodule
