// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright 2026 Romain Dolbeau
// Source location: https://github.com/MelkhiorVintageComputing/RD68031

// RD68031 - SystemVerilog MC68030
//
// Architectural constants, transcribed from the manual. Nothing here is a design
// decision; everything is a number the part already has, with the citation that
// fixes it. The bus constants have been checked against the MC68030 manual (UM);
// the rest are still the MC68020's (UM20) until the milestone that revisits them.
//
// Referred to with full scope everywhere -- rd68031_pkg::FC_CPU -- because yosys 0.52
// supports no form of `import`. See doc/coding-standard.md.

`ifndef RD68031_PKG_SV
`define RD68031_PKG_SV

package rd68031_pkg;

  // ==========================================================================
  // Address spaces -- UM Table 4-1
  // ==========================================================================
  localparam logic [2:0] FC_RESERVED_0  = 3'b000;  // undefined, reserved
  localparam logic [2:0] FC_USER_DATA   = 3'b001;
  localparam logic [2:0] FC_USER_PROG   = 3'b010;
  localparam logic [2:0] FC_RESERVED_3  = 3'b011;  // undefined, reserved
  localparam logic [2:0] FC_RESERVED_4  = 3'b100;  // undefined, reserved
  localparam logic [2:0] FC_SUPER_DATA  = 3'b101;
  localparam logic [2:0] FC_SUPER_PROG  = 3'b110;
  localparam logic [2:0] FC_CPU         = 3'b111;

  // MOVES drives the function code from SFC/DFC, so all eight encodings -- including
  // the three reserved ones -- can appear on the pins.

  // ==========================================================================
  // CPU space types -- UM figure 7-42, encoded on A19-A16 when FC = FC_CPU
  // ==========================================================================
  // Which field of the instruction pipe RTE is putting back -- doc/checkpoint.md
  // and rd68031_ifu. CK_FLAGS carries {D valid, RB, RC} in its low three bits,
  // and the queue depth comes out of the two rerun bits.
  localparam logic [2:0] CK_STG_D = 3'd0;
  localparam logic [2:0] CK_STG_C = 3'd1;
  localparam logic [2:0] CK_STG_B = 3'd2;
  localparam logic [2:0] CK_PC_D  = 3'd3;
  localparam logic [2:0] CK_FILL  = 3'd4;
  localparam logic [2:0] CK_FLAGS = 3'd5;
  // Not a restore: the coprocessor writing the scanPC -- UM 10.4.17. The queue
  // behind stage D is emptied and refilled from the address, and stage D and
  // the program counter stay: they are the coprocessor instruction's own.
  localparam logic [2:0] CK_SCAN  = 3'd6;

  localparam logic [3:0] CPUS_BKPT      = 4'h0;  // breakpoint acknowledge
  localparam logic [3:0] CPUS_COPROC    = 4'h2;  // coprocessor communication
  localparam logic [3:0] CPUS_IACK      = 4'hF;  // interrupt acknowledge

  // ==========================================================================
  // Transfer size -- UM Table 7-2
  //
  // SIZ1/SIZ0 are the number of bytes REMAINING to be transferred (UM 7.2.1), not
  // the size of the original operand, so a multi-cycle transfer changes them as it
  // goes. The encoding is simply the count modulo four.
  // ==========================================================================
  localparam logic [1:0] SIZ_LONG  = 2'b00;
  localparam logic [1:0] SIZ_BYTE  = 2'b01;
  localparam logic [1:0] SIZ_WORD  = 2'b10;
  localparam logic [1:0] SIZ_3BYTE = 2'b11;

  // The request interface counts bytes rather than encoding SIZ, because a bit-field
  // operand can span five bytes and SIZ cannot say so. The BIU converts.
  localparam int unsigned REQ_BYTES_W = 3;
  localparam logic [2:0]  BYTES_MAX   = 3'd5;

  // Operand size as the instruction set means it.
  typedef enum logic [1:0] { SZ_BYTE, SZ_WORD, SZ_LONG, SZ_3BYTE } opsize_e;

  // ==========================================================================
  // Port size reported by DSACK1/DSACK0 -- UM Table 7-1
  //
  // The pins are active low. These are the values of {dsack_n_i[1], dsack_n_i[0]}
  // sampled at the falling edge entering S3; both must be captured by the same flop
  // pair on the same edge and decoded afterwards, because specification 31A allows
  // skew between them. A cycle STERM ends is a 32-bit port and reports DSACK_32.
  // ==========================================================================
  localparam logic [1:0] DSACK_WAIT = 2'b11;  // neither asserted: insert wait states
  localparam logic [1:0] DSACK_8    = 2'b10;  // DSACK0 only: 8-bit port
  localparam logic [1:0] DSACK_16   = 2'b01;  // DSACK1 only: 16-bit port
  localparam logic [1:0] DSACK_32   = 2'b00;  // both: 32-bit port

  typedef enum logic [1:0] { PORT_8, PORT_16, PORT_32 } port_size_e;

  // ==========================================================================
  // Bus states -- UM 7.3, one state per CLK half period
  //
  // Even states begin on a rising edge, odd states on a falling edge, so an
  // asynchronous bus cycle with no wait states is three clocks and a synchronous
  // one two. ST_WH/ST_WL are the wait pair inserted between S3 and S4 (or, for a
  // synchronous cycle, after S2); the rest are not the manual's states but this
  // design's, and are named so. ST_S5 is the half clock a cycle ends in, which
  // for a synchronous cycle is the manual's S3 (rd68031_biu).
  // ==========================================================================
  typedef enum logic [3:0] {
    ST_S0, ST_S1, ST_S2, ST_S3, ST_S4, ST_S5,
    ST_WH, ST_WL,          // wait: one whole clock, re-sampling at ST_WL and ST_WH
    ST_IDLE,               // no cycle in progress
    ST_HALT,               // HALT asserted: no new cycle until it negates
    ST_RETRY,              // BERR+HALT: wait for both to negate, then rerun
    ST_BL,                 // burst: a long word latched, AS held (UM 7.3.7 S3/S5/S7)
    ST_BH,                 // burst: STERM tested for the next long word (S4/S6/S8)
    ST_BW,                 // burst: a wait state between two long words
    // A cycle the MMU refuses (M11): UM 9.2.1, "the MMU causes a bus cycle to
    // abort before the assertion of address strobe when ... an invalid access
    // is attempted". No pin moves; the falling edge after it ends the cycle
    // in S5 with a bus error, which the operand engine reports as it reports
    // BERR -- doc/mmu.md.
    ST_ABT
  } bus_state_e;

  // ==========================================================================
  // Bus arbitration -- UM figure 7-61 and 7.7.4
  //
  // The figure's seven states, numbered as the figure numbers them. Its
  // overbars do not survive the manual's text layer, but they do survive a
  // rendering of the page, and that is what these are read from -- the
  // transitions are in rd68031_biu. G is BG, T the three-state control.
  // ==========================================================================
  typedef enum logic [2:0] {
    ARB_IDLE,    // state 0: G and T negated, the processor is bus master
    ARB_GRANT,   // state 1: G and T asserted
    ARB_WAIT,    // state 2: held, until A is asserted or R is negated
    ARB_DROP,    // state 3: G negated, T held
    ARB_HELD,    // state 4: G negated, T held -- an external master has the bus
    ARB_REGRANT, // state 5: G asserted again for a request pending in state 4
    ARB_REWAIT   // state 6: held while the old master still asserts A
  } arb_state_e;

  // ==========================================================================
  // Cycle kinds and terminations
  // ==========================================================================
  typedef enum logic [2:0] {
    CT_READ,     // data read
    CT_WRITE,    // data write
    CT_IFETCH,   // instruction prefetch: always a long word from a long-word address
    CT_IACK,     // interrupt acknowledge, CPU space $F
    CT_BKPT,     // breakpoint acknowledge, CPU space $0
    CT_CPU       // other CPU-space access: coprocessor ($2), access level ($1)
  } cycle_kind_e;

  // There is no CT_RMW. UM 7.5.2 retries each read and each write of a
  // read-modify-write separately with RMC held throughout, so RMC is a qualifier
  // across a run of ordinary cycles rather than one indivisible cycle -- which is
  // also the only way CAS2's four transfers are expressible.

  typedef enum logic [2:0] {
    CE_NONE,     // no cycle
    CE_DSACK,    // normal termination
    CE_BERR,     // bus error
    CE_RETRY,    // BERR + HALT
    CE_AVEC,     // autovector, interrupt acknowledge only
    CE_HALT,     // HALT at or before DSACK or STERM: cycle completed, then halted
    CE_STERM     // normal synchronous termination -- UM 7.1.7
  } cycle_end_e;

  // ==========================================================================
  // Status register -- UM Figure 1-4
  // ==========================================================================
  localparam int SR_C  = 0;
  localparam int SR_V  = 1;
  localparam int SR_Z  = 2;
  localparam int SR_N  = 3;
  localparam int SR_X  = 4;
  localparam int SR_I0 = 8;   // I2-I0 are bits 10:8
  localparam int SR_M  = 12;  // master/interrupt mode: selects MSP or ISP
  localparam int SR_S  = 13;
  localparam int SR_T0 = 14;  // trace on change of flow
  localparam int SR_T1 = 15;  // trace on any instruction

  // Bits 11, 7, 6 and 5 read as zero and are ignored when written.
  localparam logic [15:0] SR_IMPLEMENTED = 16'b1111_0111_0001_1111;

  // Reset leaves the processor in the interrupt mode of the supervisor level with the
  // mask at 7 (UM 4.1.1: "the processor is in this mode after a reset operation";
  // UM 8.1.1, steps 1-3).
  localparam logic [15:0] SR_RESET = 16'h2700;

  // ==========================================================================
  // Exception stack frames -- UM Table 8-6
  // ==========================================================================
  localparam logic [3:0] FMT_SHORT     = 4'h0;  // four words
  localparam logic [3:0] FMT_THROWAWAY = 4'h1;  // four words, interrupt stack switch
  localparam logic [3:0] FMT_SIX       = 4'h2;  // six words
  localparam logic [3:0] FMT_COPROC    = 4'h9;  // ten words, coprocessor midinstruction
  localparam logic [3:0] FMT_FAULT_S   = 4'hA;  // sixteen words, short bus fault
  localparam logic [3:0] FMT_FAULT_L   = 4'hB;  // forty-six words, long bus fault

  localparam int FRAME_WORDS_SHORT     = 4;
  localparam int FRAME_WORDS_THROWAWAY = 4;
  localparam int FRAME_WORDS_SIX       = 6;
  localparam int FRAME_WORDS_COPROC    = 10;
  localparam int FRAME_WORDS_FAULT_S   = 16;
  localparam int FRAME_WORDS_FAULT_L   = 46;

  // UM 8.1.13: RTE compares the version number in bits 15-12 of the word at SP+$36 of
  // a long frame against its own, and takes a format error if they differ. That is
  // what makes this design's private encoding of the internal words legitimate, and
  // it is also why format $A -- which has no version field -- may carry no private
  // state. See doc/manual-contradictions.md for why the offset is $36 and not $38.
  localparam int         FRAME_VERSION_OFF = 'h36;
  localparam logic [3:0] FRAME_VERSION     = 4'h1;

  // ==========================================================================
  // Special status word -- UM Figure 8-9, at offset $0A of both fault frames
  // ==========================================================================
  localparam int SSW_FC   = 15;  // fault on stage C
  localparam int SSW_FB   = 14;  // fault on stage B
  localparam int SSW_RC   = 13;  // rerun stage C prefetch
  localparam int SSW_RB   = 12;  // rerun stage B prefetch
  localparam int SSW_DF   = 8;   // data fault / rerun the data access
  localparam int SSW_RM   = 7;   // the data cycle was part of a read-modify-write
  localparam int SSW_RW   = 6;   // 1 = read, 0 = write
  localparam int SSW_SIZE = 4;   // SIZE is bits 5:4
  // Bits 2:0 are the function code of the data cycle; bits 11:9 and 3 are reserved.

  // UM 8.2.2: "The only bits in the SSW that may be modified are DF, RB, and RC."
  localparam logic [15:0] SSW_HANDLER_MASK = (16'b1 << SSW_DF)
                                           | (16'b1 << SSW_RB)
                                           | (16'b1 << SSW_RC);

  // ==========================================================================
  // Vector numbers -- PRM Appendix B
  // ==========================================================================
  localparam logic [7:0] VEC_RESET_ISP    = 8'd0;
  localparam logic [7:0] VEC_RESET_PC     = 8'd1;
  localparam logic [7:0] VEC_ACCESS_FAULT = 8'd2;   // bus error
  localparam logic [7:0] VEC_ADDRESS_ERR  = 8'd3;
  localparam logic [7:0] VEC_ILLEGAL      = 8'd4;
  localparam logic [7:0] VEC_DIV_ZERO     = 8'd5;
  localparam logic [7:0] VEC_CHK          = 8'd6;   // CHK, CHK2
  localparam logic [7:0] VEC_TRAPCC       = 8'd7;   // TRAPcc, TRAPV, cpTRAPcc
  localparam logic [7:0] VEC_PRIVILEGE    = 8'd8;
  localparam logic [7:0] VEC_TRACE        = 8'd9;
  localparam logic [7:0] VEC_LINE_A       = 8'd10;
  localparam logic [7:0] VEC_LINE_F       = 8'd11;
  localparam logic [7:0] VEC_COPROC_PROTO = 8'd13;
  localparam logic [7:0] VEC_FORMAT_ERR   = 8'd14;
  localparam logic [7:0] VEC_UNINIT_INT   = 8'd15;
  localparam logic [7:0] VEC_SPURIOUS     = 8'd24;
  localparam logic [7:0] VEC_AUTOVEC_1    = 8'd25;  // levels 1-7 are 25-31
  localparam logic [7:0] VEC_TRAP_0       = 8'd32;  // TRAP #n is 32 + n

  // ==========================================================================
  // The caches -- UM 6.1, figures 6-2 and 6-3
  //
  // Each 256 bytes: 16 direct-mapped lines (A7-A4) of four long-word entries
  // (A3-A2), with a valid bit per entry and a tag per line. The instruction
  // cache's tag is A31-A8 and FC2, the data cache's A31-A8 and FC2-FC0.
  // rd68031_icache.sv and rd68031_dcache.sv; doc/caches.md.
  // ==========================================================================
  localparam int CACHE_LINES    = 16;
  localparam int ICACHE_INDEX_W = 6;   // A7-A2
  localparam int ICACHE_TAG_W   = 25;  // A31-A8 and FC2
  localparam int DCACHE_TAG_W   = 27;  // A31-A8 and FC2-FC0

  // CACR -- UM 6.3.1 and figure 6-14. "Bits 31-14 and 7-5 are reserved ...
  // They are currently read as zeros and are ignored when written", and the
  // four clear bits are orders that are "always read as a zero". The low four
  // bits are where the MC68020's E, F, CE and C were (UM20 figure 4-2), with
  // the same meanings for the instruction cache.
  localparam int CACR_EI  = 0;   // enable the instruction cache
  localparam int CACR_FI  = 1;   // freeze it: a miss does not replace
  localparam int CACR_CEI = 2;   // clear the entry CAAR names; reads zero
  localparam int CACR_CI  = 3;   // clear all of it; reads zero
  localparam int CACR_IBE = 4;   // instruction burst enable
  localparam int CACR_ED  = 8;   // enable the data cache
  localparam int CACR_FD  = 9;   // freeze it
  localparam int CACR_CED = 10;  // clear the data-cache entry CAAR names; reads zero
  localparam int CACR_CD  = 11;  // clear all of the data cache; reads zero
  localparam int CACR_DBE = 12;  // data burst enable
  localparam int CACR_WA  = 13;  // write allocate
  // What a MOVEC to CACR keeps: WA, DBE, FD, ED, IBE, FI and EI, all of which
  // act (doc/caches.md); CD, CED, CI and CEI are orders, never stored.
  localparam logic [31:0] CACR_IMPLEMENTED = 32'h0000_3313;

  // ==========================================================================
  // RESET -- UM 7.8
  // ==========================================================================
  localparam int RESET_INSN_CLKS = 512;  // the RESET instruction drives the pin this long

endpackage

`endif
