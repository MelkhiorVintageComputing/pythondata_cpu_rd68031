// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68884

// RD68884 - SystemVerilog MC68881 floating-point coprocessor
//
// Top level. The port list is the MC68881's pin list (FPU section 9, table 9-5),
// with the project's conventions (doc/pinout.md):
//
//   - three-state pins are split into _i / _o / _oe, _oe high = this chip drives;
//   - _n means active low at the pin;
//   - clk is the core clock, NOT the MC68881's CLK pin. The bus is asynchronous
//     (FPU 10.4), so the core's clock needs no relation to the main processor's;
//     any frequency works, a slow one only adding wait states (doc/bus-timing.md);
//     50 MHz from a board PLL is the target, set by the datapath;
//   - rst_n is the hardware initialisation input every register resets from. The
//     MC68881's RESET pin is reset_n_i, the architectural reset (FPU 9.9).
//   - SENSE is a wire to ground on the board (FPU 9.11), not a core port.
//
// The bus interface unit (rd68884_biu) and the microsequencer with its
// datapath (rd68884_seq). M4: every dialog; the arithmetic arrives in M5
// (doc/architecture.md).
//
// BUS_SYNC selects the BIU's front end (doc/bus-timing.md): 0, the default,
// takes the bus as asynchronous and clk as any clock; 1 requires clk to be the
// main processor's CLK and answers on its edges, with BUS_SYNC_WAIT wait states
// (0 or 1).

module rd68884_top #(
    parameter int BUS_SYNC      = 0,
    parameter int BUS_SYNC_WAIT = 0,
    parameter int RESP_HOLD     = 20,    // doc/bus-timing.md, "Response hold-off"
    parameter int DSACK_NEGATE  = 0,     // 1: DSACK negated before release (FPU 9.8)
    parameter int MODEL         = 68881  // 68881 (RD68884) or 68882 (RD68885): doc/rd68885.md
) (
    input  logic        clk,
    input  logic        rst_n,

    // FPU 9.9: RESET.
    input  logic        reset_n_i,

    // FPU 9.1-9.7: the bus inputs. a_i[0] doubles as a byte address on an 8-bit
    // port; on 16- and 32-bit ports it and size_n_i are strapped (table 9-2).
    input  logic        cs_n_i,
    input  logic        as_n_i,
    input  logic        ds_n_i,
    input  logic        rw_i,
    input  logic        size_n_i,
    input  logic [4:0]  a_i,

    // FPU 9.2: D31-D0. One enable per byte lane: on 8- and 16-bit ports the
    // board ties lanes together, so only the lane in use may be driven.
    input  logic [31:0] d_i,
    output logic [31:0] d_o,
    output logic [3:0]  d_oe,

    // FPU 9.8: DSACK1/DSACK0, bit 1 = DSACK1, driven together while an access
    // is acknowledged and floated the moment it ends (doc/divergences.md).
    output logic [1:0]  dsack_n_o,
    output logic        dsack_oe
);

  // BIU <-> sequencer (the port lists of rd68884_biu and rd68884_seq).
  logic        arch_reset, cmd_pend, cmd_cond, opw_valid, opr_valid, save_req;
  logic        restore_req, resp_read, rsel_read, save_read, abort, pv;
  logic [15:0] cmd_word, restore_word;
  logic [31:0] opw_data, fpiar;
  logic        resp_we, resp_oneshot, resp_cond, cmd_ack, opw_ack, opr_we;
  logic        rsel_we, rsel_dir, save_we, restore_we, fpiar_we, clear;
  logic [15:0] resp, save_v, restore_v;
  logic [2:0]  expect_v;
  logic [31:0] opr, fpiar_v;
  logic [7:0]  rsel;
  logic [5:0]  save_xfer, restore_xfer;
  // RD68885's conversion unit (inert when MODEL = 68881).
  logic        abort_ab, cu_ready, cu_valid, cu_mid, apu_run, pcen, cu_take, cu_load;
  logic        cu_resume, relatch, relatch_cond;
  logic [15:0] cu_word, relatch_word;
  logic [31:0] cu_d0, cu_d1, cu_d2, cu_w0, cu_w1, cu_data;
  logic [2:0]  cu_idx;
  logic [1:0]  resp_xfer;
  logic        prec_x, rfb_we, rfb_re, dcc_v, dcc_cc_v, dcc_ack;
  logic [2:0]  rfb_wa, rfb_ra;
  logic [90:0] rfb_wd, rfb_q;
  logic [3:0]  dcc_cc;

  rd68884_biu #(
      .BUS_SYNC     (BUS_SYNC),
      .BUS_SYNC_WAIT(BUS_SYNC_WAIT),
      .RESP_HOLD    (RESP_HOLD),
      .DSACK_NEGATE (DSACK_NEGATE),
      .MODEL        (MODEL)
  ) u_biu (
      .clk           (clk),
      .rst_n         (rst_n),
      .reset_n_i     (reset_n_i),
      .cs_n_i        (cs_n_i),
      .as_n_i        (as_n_i),
      .ds_n_i        (ds_n_i),
      .rw_i          (rw_i),
      .size_n_i      (size_n_i),
      .a_i           (a_i),
      .d_i           (d_i),
      .d_o           (d_o),
      .d_oe          (d_oe),
      .dsack_n_o     (dsack_n_o),
      .dsack_oe      (dsack_oe),
      .arch_reset_o  (arch_reset),
      .resp_we_i     (resp_we),
      .resp_i        (resp),
      .resp_oneshot_i(resp_oneshot),
      .expect_i      (expect_v),
      .resp_cond_i   (resp_cond),
      .cmd_pend_o    (cmd_pend),
      .cmd_cond_o    (cmd_cond),
      .cmd_word_o    (cmd_word),
      .cmd_ack_i     (cmd_ack),
      .opw_valid_o   (opw_valid),
      .opw_data_o    (opw_data),
      .opw_ack_i     (opw_ack),
      .opr_we_i      (opr_we),
      .opr_i         (opr),
      .opr_valid_o   (opr_valid),
      .rsel_we_i     (rsel_we),
      .rsel_i        (rsel),
      .rsel_dir_i    (rsel_dir),
      .save_we_i     (save_we),
      .save_i        (save_v),
      .save_xfer_i   (save_xfer),
      .save_req_o    (save_req),
      .restore_req_o (restore_req),
      .restore_word_o(restore_word),
      .restore_we_i  (restore_we),
      .restore_i     (restore_v),
      .restore_xfer_i(restore_xfer),
      .clear_i       (clear),
      .fpiar_o       (fpiar),
      .fpiar_we_i    (fpiar_we),
      .fpiar_i       (fpiar_v),
      .resp_read_o   (resp_read),
      .rsel_read_o   (rsel_read),
      .save_read_o   (save_read),
      .abort_o       (abort),
      .pv_o          (pv),
      .apu_run_i     (apu_run),
      .pcen_i        (pcen),
      .resp_xfer_i   (resp_xfer),
      .cu_take_i     (cu_take),
      .cu_load_i     (cu_load),
      .cu_idx_i      (cu_idx),
      .cu_data_i     (cu_data),
      .cu_resume_i   (cu_resume),
      .relatch_i     (relatch),
      .relatch_word_i(relatch_word),
      .relatch_cond_i(relatch_cond),
      .abort_ab_o    (abort_ab),
      .cu_ready_o    (cu_ready),
      .cu_valid_o    (cu_valid),
      .cu_mid_o      (cu_mid),
      .cu_word_o     (cu_word),
      .cu_d0_o       (cu_d0),
      .cu_d1_o       (cu_d1),
      .cu_d2_o       (cu_d2),
      .cu_w0_o       (cu_w0),
      .cu_w1_o       (cu_w1),
      .prec_x_i      (prec_x),
      .rfb_q_i       (rfb_q),
      .rfb_we_o      (rfb_we),
      .rfb_wa_o      (rfb_wa),
      .rfb_wd_o      (rfb_wd),
      .rfb_re_o      (rfb_re),
      .rfb_ra_o      (rfb_ra),
      .dcc_v_o       (dcc_v),
      .dcc_cc_v_o    (dcc_cc_v),
      .dcc_cc_o      (dcc_cc),
      .dcc_ack_i     (dcc_ack)
  );

  rd68884_seq #(
      .MODEL(MODEL)
  ) u_seq (
      .clk           (clk),
      .rst_n         (rst_n),
      .arch_reset_i  (arch_reset),
      .cmd_pend_i    (cmd_pend),
      .cmd_cond_i    (cmd_cond),
      .cmd_word_i    (cmd_word),
      .opw_valid_i   (opw_valid),
      .opw_data_i    (opw_data),
      .opr_valid_i   (opr_valid),
      .save_req_i    (save_req),
      .restore_req_i (restore_req),
      .restore_word_i(restore_word),
      .fpiar_i       (fpiar),
      .pv_i          (pv),
      .resp_read_i   (resp_read),
      .rsel_read_i   (rsel_read),
      .save_read_i   (save_read),
      .abort_i       (abort),
      .resp_we_o     (resp_we),
      .resp_o        (resp),
      .resp_oneshot_o(resp_oneshot),
      .expect_o      (expect_v),
      .resp_cond_o   (resp_cond),
      .cmd_ack_o     (cmd_ack),
      .opw_ack_o     (opw_ack),
      .opr_we_o      (opr_we),
      .opr_o         (opr),
      .rsel_we_o     (rsel_we),
      .rsel_o        (rsel),
      .rsel_dir_o    (rsel_dir),
      .save_we_o     (save_we),
      .save_o        (save_v),
      .save_xfer_o   (save_xfer),
      .restore_we_o  (restore_we),
      .restore_o     (restore_v),
      .restore_xfer_o(restore_xfer),
      .fpiar_we_o    (fpiar_we),
      .fpiar_o       (fpiar_v),
      .clear_o       (clear),
      .abort_ab_i    (abort_ab),
      .cu_ready_i    (cu_ready),
      .cu_valid_i    (cu_valid),
      .cu_mid_i      (cu_mid),
      .cu_word_i     (cu_word),
      .cu_d0_i       (cu_d0),
      .cu_d1_i       (cu_d1),
      .cu_d2_i       (cu_d2),
      .cu_w0_i       (cu_w0),
      .cu_w1_i       (cu_w1),
      .apu_run_o     (apu_run),
      .pcen_o        (pcen),
      .resp_xfer_o   (resp_xfer),
      .cu_take_o     (cu_take),
      .cu_load_o     (cu_load),
      .cu_idx_o      (cu_idx),
      .cu_data_o     (cu_data),
      .cu_resume_o   (cu_resume),
      .relatch_o     (relatch),
      .relatch_word_o(relatch_word),
      .relatch_cond_o(relatch_cond),
      .rfb_we_i      (rfb_we),
      .rfb_wa_i      (rfb_wa),
      .rfb_wd_i      (rfb_wd),
      .rfb_re_i      (rfb_re),
      .rfb_ra_i      (rfb_ra),
      .rfb_q_o       (rfb_q),
      .prec_x_o      (prec_x),
      .dcc_v_i       (dcc_v),
      .dcc_cc_v_i    (dcc_cc_v),
      .dcc_cc_i      (dcc_cc),
      .dcc_ack_o     (dcc_ack)
  );

endmodule
