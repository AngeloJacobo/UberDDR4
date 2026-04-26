////////////////////////////////////////////////////////////////////////////////
`default_nettype none
//
// Filename: ddr4_model_wrapper.sv
// Project:  UberDDR4 - An Open Source DDR4 Controller
//
// Purpose:  Wraps two Micron DDR4 x8 behavioral model instances to match
//  UberDDR4's PHY output signals. Each instance handles one byte lane.
//  Command/address signals are shared; data signals are split per lane.
//
// Engineer: Angelo C. Jacobo
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2025  Angelo Jacobo
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

`timescale 1ps / 1ps

module ddr4_model_wrapper #(
    parameter DQ_BITS    = 8,
    parameter BYTE_LANES = 2
) (
    input  wire                          i_ddr4_ck_p,
    input  wire                          i_ddr4_ck_n,
    input  wire                          i_ddr4_reset_n,
    input  wire                          i_ddr4_cke,
    input  wire                          i_ddr4_cs_n,
    input  wire                          i_ddr4_act_n,
    input  wire [16:0]                   i_ddr4_addr,
    input  wire [1:0]                    i_ddr4_ba,
    input  wire [1:0]                    i_ddr4_bg,
    input  wire                          i_ddr4_odt,
    inout  wire [DQ_BITS*BYTE_LANES-1:0] io_ddr4_dq,
    inout  wire [BYTE_LANES-1:0]         io_ddr4_dqs_p,
    inout  wire [BYTE_LANES-1:0]         io_ddr4_dqs_n,
    inout  wire [BYTE_LANES-1:0]         io_ddr4_dm_n
);

    import arch_package::*;

    // ═══════════════════════════════════════════════════════════════════
    // Per-device DDR4 interfaces (one per byte lane)
    // ═══════════════════════════════════════════════════════════════════
    DDR4_if #(.CONFIGURED_DQ_BITS(DQ_BITS)) iDDR4_0();
    DDR4_if #(.CONFIGURED_DQ_BITS(DQ_BITS)) iDDR4_1();

    // ═══════════════════════════════════════════════════════════════════
    // Command/address fan-out (shared across both devices)
    // CK[1]=CK_t (true), CK[0]=CK_c (complement) per Micron interface
    // A[16]=RAS_n_A16, A[15]=CAS_n_A15, A[14]=WE_n_A14 per DDR4 pin mux
    // ═══════════════════════════════════════════════════════════════════
    assign iDDR4_0.CK        = {i_ddr4_ck_p, i_ddr4_ck_n};
    assign iDDR4_0.RESET_n   = i_ddr4_reset_n;
    assign iDDR4_0.CKE       = i_ddr4_cke;
    assign iDDR4_0.CS_n      = i_ddr4_cs_n;
    assign iDDR4_0.ACT_n     = i_ddr4_act_n;
    assign iDDR4_0.RAS_n_A16 = i_ddr4_addr[16];
    assign iDDR4_0.CAS_n_A15 = i_ddr4_addr[15];
    assign iDDR4_0.WE_n_A14  = i_ddr4_addr[14];
    assign iDDR4_0.ADDR      = i_ddr4_addr[13:0];
    assign iDDR4_0.BA        = i_ddr4_ba;
    assign iDDR4_0.BG        = i_ddr4_bg;
    assign iDDR4_0.ODT       = i_ddr4_odt;
    assign iDDR4_0.ADDR_17   = 1'b0;
    assign iDDR4_0.C         = '0;
    assign iDDR4_0.TEN       = 1'b0;
    assign iDDR4_0.PARITY    = 1'b0;
    assign iDDR4_0.ZQ        = 1'b1;
    assign iDDR4_0.PWR       = 1'b1;
    assign iDDR4_0.VREF_CA   = 1'b1;
    assign iDDR4_0.VREF_DQ   = 1'b1;

    assign iDDR4_1.CK        = {i_ddr4_ck_p, i_ddr4_ck_n};
    assign iDDR4_1.RESET_n   = i_ddr4_reset_n;
    assign iDDR4_1.CKE       = i_ddr4_cke;
    assign iDDR4_1.CS_n      = i_ddr4_cs_n;
    assign iDDR4_1.ACT_n     = i_ddr4_act_n;
    assign iDDR4_1.RAS_n_A16 = i_ddr4_addr[16];
    assign iDDR4_1.CAS_n_A15 = i_ddr4_addr[15];
    assign iDDR4_1.WE_n_A14  = i_ddr4_addr[14];
    assign iDDR4_1.ADDR      = i_ddr4_addr[13:0];
    assign iDDR4_1.BA        = i_ddr4_ba;
    assign iDDR4_1.BG        = i_ddr4_bg;
    assign iDDR4_1.ODT       = i_ddr4_odt;
    assign iDDR4_1.ADDR_17   = 1'b0;
    assign iDDR4_1.C         = '0;
    assign iDDR4_1.TEN       = 1'b0;
    assign iDDR4_1.PARITY    = 1'b0;
    assign iDDR4_1.ZQ        = 1'b1;
    assign iDDR4_1.PWR       = 1'b1;
    assign iDDR4_1.VREF_CA   = 1'b1;
    assign iDDR4_1.VREF_DQ   = 1'b1;

    // ═══════════════════════════════════════════════════════════════════
    // Bidirectional data path — dual assign creates transparent
    // connection between external wires and per-device interface wires.
    // Both sides resolve via wire strength rules (hi-Z yields to driven).
    // ═══════════════════════════════════════════════════════════════════
    assign iDDR4_0.DQ    = io_ddr4_dq[DQ_BITS-1:0];
    assign iDDR4_1.DQ    = io_ddr4_dq[2*DQ_BITS-1:DQ_BITS];
    assign io_ddr4_dq[DQ_BITS-1:0]       = iDDR4_0.DQ;
    assign io_ddr4_dq[2*DQ_BITS-1:DQ_BITS] = iDDR4_1.DQ;

    assign iDDR4_0.DQS_t = io_ddr4_dqs_p[0];
    assign iDDR4_0.DQS_c = io_ddr4_dqs_n[0];
    assign iDDR4_1.DQS_t = io_ddr4_dqs_p[1];
    assign iDDR4_1.DQS_c = io_ddr4_dqs_n[1];
    assign io_ddr4_dqs_p[0] = iDDR4_0.DQS_t;
    assign io_ddr4_dqs_n[0] = iDDR4_0.DQS_c;
    assign io_ddr4_dqs_p[1] = iDDR4_1.DQS_t;
    assign io_ddr4_dqs_n[1] = iDDR4_1.DQS_c;

    assign iDDR4_0.DM_n  = io_ddr4_dm_n[0];
    assign iDDR4_1.DM_n  = io_ddr4_dm_n[1];
    assign io_ddr4_dm_n[0] = iDDR4_0.DM_n;
    assign io_ddr4_dm_n[1] = iDDR4_1.DM_n;

    // ═══════════════════════════════════════════════════════════════════
    // Micron DDR4 x8 model instances (one per byte lane)
    // model_enable is inout on the Micron model — use a wire
    // ═══════════════════════════════════════════════════════════════════
    wire model_en;
    assign model_en = 1'b1;

    ddr4_model #(
        .CONFIGURED_DQ_BITS (DQ_BITS),
        .CONFIGURED_DENSITY (_8G),
        .CONFIGURED_RANKS   (1)
    ) u_ddr4_0 (
        .model_enable (model_en),
        .iDDR4        (iDDR4_0)
    );

    ddr4_model #(
        .CONFIGURED_DQ_BITS (DQ_BITS),
        .CONFIGURED_DENSITY (_8G),
        .CONFIGURED_RANKS   (1)
    ) u_ddr4_1 (
        .model_enable (model_en),
        .iDDR4        (iDDR4_1)
    );

endmodule
