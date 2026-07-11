// ============================================================================
//  mb88.sv  —  drop-in wrapper matching darfpga rtl/cpu/mb88.vhd's entity, so it
//  swaps into Kangaroo_CPU.sv's `mb88 mcu (...)` instantiation with no other edits.
//  Wraps the combinational-RAM mb88_core (mb88_sv/mb88_core.sv), which is validated
//  against MAME's kangaroo_ape.trace and fixes the protection-MCU hang the deployed
//  mb88.vhd suffered (BRAM data-RAM read-after-write hazard in the K-debounce).
//
//  Kangaroo ties r_in<-r_out (read_r = output latch); O port = protrom address.
// ============================================================================

module mb88
(
    input  wire        clock,
    input  wire        ena,
    input  wire        ena_timer,
    input  wire        reset_n,

    input  wire [3:0]  r0_port_in,  r1_port_in,  r2_port_in,  r3_port_in,
    output wire [3:0]  r0_port_out, r1_port_out, r2_port_out, r3_port_out,
    input  wire [3:0]  k_port_in,
    output wire [3:0]  ol_port_out, oh_port_out,
    output wire [3:0]  p_port_out,

    input  wire        stby_n,
    input  wire        tc_n,
    input  wire        irq_n,
    input  wire        sc_in_n,
    input  wire        si_n,
    output wire        sc_out_n,
    output wire        so_n,
    output wire        to_n,

    output wire [10:0] rom_addr,
    input  wire [7:0]  rom_data
);
    wire [15:0]  r_in  = {r3_port_in, r2_port_in, r1_port_in, r0_port_in};
    wire [15:0]  r_out;
    wire [7:0]   o_out;

    assign {r3_port_out, r2_port_out, r1_port_out, r0_port_out} = r_out;
    assign ol_port_out = o_out[3:0];
    assign oh_port_out = o_out[7:4];

    // serial/handshake outputs unused by Kangaroo — hold inactive (active-low)
    assign sc_out_n = 1'b1;
    assign so_n     = 1'b1;
    assign to_n     = 1'b1;

    mb88_core core
    (
        .clk       (clock),
        .ce        (ena),
        .ena_timer (ena_timer),
        .reset_n   (reset_n),

        .prog_addr (rom_addr),
        .prog_data (rom_data),

        .k_in      (k_port_in),
        .r_in      (r_in),
        .r_out     (r_out),
        .p_out     (p_port_out),
        .o_out     (o_out),
        .si_in     (si_n),
        .so_out    (),
        .irq_n     (irq_n),
        .tc_in     (tc_n),

        .int_req   (1'b0),        // trace injection off on hardware
        .int_vec   (6'd0),

        .dbg_pc(), .dbg_fetch_pc(), .dbg_a(), .dbg_x(), .dbg_y(),
        .dbg_st(), .dbg_zf(), .dbg_cf(), .dbg_retire(), .dbg_int_ack(),
        .dbg_illegal(), .dbg_ram()
    );

    // silence unused inputs (stby_n / sc_in_n not modeled)
    // verilator lint_off UNUSED
    wire _wunused = &{1'b0, stby_n, sc_in_n};
    // verilator lint_on UNUSED
endmodule
