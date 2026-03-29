//============================================================================
//
//  Kangaroo Main CPU Board (TVG-1-CPU-B)
//  Based on MAME kangaroo.cpp by Ville Laitinen, Aaron Giles
//
//============================================================================

module Kangaroo_CPU
(
    input         reset,
    input         clk_10m,           // 10 MHz master clock (matches real XTAL)

    // Video outputs
    output  [2:0] video_r, video_g,  // BGR 3-bit palette
    output  [1:0] video_b,
    output        video_hsync, video_vsync,
    output        video_hblank, video_vblank,
    output        ce_pix,

    // Inputs
    input   [7:0] dsw0,             // 8-bit DIP switch
    input   [4:0] in0,              // IN0: service, start1, start2, coin_l, coin_r
    input   [4:0] in1,              // IN1: P1 right, left, up, down, punch
    input   [4:0] in2,              // IN2: P2 right, left, up, down, punch

    // Sound interface
    output  [7:0] sound_latch,      // Data to sound CPU
    output        sound_latch_wr,   // Strobe: sound latch written

    // Blitter ROMs — directly memory-mapped, loaded via index 2
    input         blit0_cs_i, blit1_cs_i, blit2_cs_i, blit3_cs_i,
    // Main program ROMs — loaded via index 0
    input         rom0_cs_i, rom1_cs_i, rom2_cs_i,
    input         rom3_cs_i, rom4_cs_i, rom5_cs_i,
    input  [24:0] ioctl_addr,
    input   [7:0] ioctl_data,
    input         ioctl_wr,

    input         pause,

    // Hiscore interface (active high, stubbed for now)
    input  [15:0] hs_address,
    input   [7:0] hs_data_in,
    output  [7:0] hs_data_out,
    input         hs_write
);

//------------------------------------------------------- Clock Enables -------------------------------------------------------//

// Generate clock enables from 10 MHz master
// cen_5m  = 10/2 = 5 MHz (pixel clock)
// cen_2m5 = 10/4 = 2.5 MHz (Z80 clock)
reg [1:0] div = 2'd0;
always_ff @(posedge clk_10m) begin
    div <= div + 2'd1;
end
wire cen_5m  = (div[0] == 1'b0);     // Every 2nd clock
wire cen_2m5 = (div == 2'd0);         // Every 4th clock

assign ce_pix = cen_5m;

//------------------------------------------------------------ CPU -------------------------------------------------------------//

// Main CPU — Zilog Z80 (T80s soft core)
wire [15:0] cpu_A;
wire  [7:0] cpu_Dout;
wire n_m1, n_mreq, n_iorq, n_rd, n_wr, n_rfsh;

T80s #(.Mode(0), .T2Write(1), .IOWait(1)) main_cpu
(
    .RESET_n(reset),
    .CLK(clk_10m),
    .CEN(cen_2m5 & ~pause),
    .INT_n(n_irq),
    .NMI_n(n_nmi),
    .BUSRQ_n(1'b1),
    .M1_n(n_m1),
    .MREQ_n(n_mreq),
    .IORQ_n(n_iorq),
    .RD_n(n_rd),
    .WR_n(n_wr),
    .RFSH_n(n_rfsh),
    .A(cpu_A),
    .DI(cpu_Din),
    .DO(cpu_Dout)
);

//------------------------------------------------------ Address Decoding ------------------------------------------------------//

// Active-low signals for memory regions
wire mem_access = ~n_mreq & n_rfsh;

// ROM: 0x0000-0x5FFF (read only)
wire cs_rom = mem_access & (cpu_A[15:14] == 2'b00) & ~cpu_A[13]; // 0x0000-0x1FFF
wire cs_rom_hi = mem_access & (cpu_A[15:13] == 3'b001);           // 0x2000-0x3FFF
wire cs_rom_top = mem_access & (cpu_A[15:13] == 3'b010);          // 0x4000-0x5FFF
wire cs_any_rom = cs_rom | cs_rom_hi | cs_rom_top;

// Video RAM: 0x8000-0xBFFF (write only from CPU perspective)
wire cs_videoram = mem_access & (cpu_A[15:14] == 2'b10);          // 0x8000-0xBFFF

// Banked blitter ROM: 0xC000-0xDFFF (read only)
wire cs_blitbank = mem_access & (cpu_A[15:13] == 3'b110);         // 0xC000-0xDFFF

// Work RAM: 0xE000-0xE3FF
wire cs_workram = mem_access & (cpu_A[15:10] == 6'b111000);       // 0xE000-0xE3FF

// DSW: 0xE400 (read, mirrored across 0xE400-0xE7FF)
wire cs_dsw = mem_access & (cpu_A[15:10] == 6'b111001);           // 0xE400-0xE7FF

// Video control: 0xE800-0xE80A (write, mirrored with 0x03F0)
wire cs_vidctrl = mem_access & (cpu_A[15:10] == 6'b111010);       // 0xE800-0xEBFF

// IN0 read / soundlatch write: 0xEC00
wire cs_in0 = mem_access & (cpu_A[15:8] == 8'hEC);

// IN1 read / coin counter write: 0xED00
wire cs_in1 = mem_access & (cpu_A[15:8] == 8'hED);

// IN2 read: 0xEE00
wire cs_in2 = mem_access & (cpu_A[15:8] == 8'hEE);

// MCU: 0xEF00 (stubbed for bootleg — no MCU)
wire cs_mcu = mem_access & (cpu_A[15:8] == 8'hEF);

//--------------------------------------------------------- CPU Data Mux -------------------------------------------------------//

wire [7:0] rom_D;
wire [7:0] blitbank_D;
wire [7:0] workram_D;

wire [7:0] cpu_Din =
    cs_any_rom     ? rom_D :
    cs_blitbank    ? blitbank_D :
    (cs_workram & ~n_rd) ? workram_D :
    cs_dsw         ? dsw0 :
    cs_in0         ? {3'b000, in0} :
    cs_in1         ? {3'b000, in1} :
    cs_in2         ? {3'b000, in2} :
    cs_mcu         ? 8'h00 :       // Bootleg: MCU reads return 0
    8'hFF;

//-------------------------------------------------------- Program ROMs --------------------------------------------------------//

wire [7:0] rom0_D, rom1_D, rom2_D, rom3_D, rom4_D, rom5_D;

assign rom_D = (cpu_A[15:12] == 4'h0) ? rom0_D :
               (cpu_A[15:12] == 4'h1) ? rom1_D :
               (cpu_A[15:12] == 4'h2) ? rom2_D :
               (cpu_A[15:12] == 4'h3) ? rom3_D :
               (cpu_A[15:12] == 4'h4) ? rom4_D :
               (cpu_A[15:12] == 4'h5) ? rom5_D :
               8'hFF;

eprom_4k rom0 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(rom0_D),
               .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
               .CS_DL(rom0_cs_i), .WR(ioctl_wr));
eprom_4k rom1 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(rom1_D),
               .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
               .CS_DL(rom1_cs_i), .WR(ioctl_wr));
eprom_4k rom2 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(rom2_D),
               .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
               .CS_DL(rom2_cs_i), .WR(ioctl_wr));
eprom_4k rom3 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(rom3_D),
               .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
               .CS_DL(rom3_cs_i), .WR(ioctl_wr));
eprom_4k rom4 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(rom4_D),
               .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
               .CS_DL(rom4_cs_i), .WR(ioctl_wr));
eprom_4k rom5 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(rom5_D),
               .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
               .CS_DL(rom5_cs_i), .WR(ioctl_wr));

//------------------------------------------------------- Blitter ROMs --------------------------------------------------------//

// Blitter has 4 x 4KB ROMs, banked into 0xC000-0xDFFF via bank select (video_control[8])
// Bank 0: blit0 + blit2 (when bit 0 or 2 of bank select is 0)
// Bank 1: blit1 + blit3 (when bit 0 or 2 of bank select is set)
// MAME: m_blitbank->set_entry((data & 0x05) ? 1 : 0)
// Bank 0 maps: C000-CFFF=blit0(v0), D000-DFFF=blit2(v1)
// Bank 1 maps: C000-CFFF=blit1(v2), D000-DFFF=blit3(v3)

wire [7:0] blit0_D, blit1_D, blit2_D, blit3_D;
wire blit_bank_sel = (video_control[8] & 8'h05) != 0;  // MAME: (data & 0x05) ? 1 : 0

assign blitbank_D = cpu_A[12] ?
    (blit_bank_sel ? blit3_D : blit2_D) :
    (blit_bank_sel ? blit1_D : blit0_D);

eprom_4k blit0 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(blit0_D),
                .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
                .CS_DL(blit0_cs_i), .WR(ioctl_wr));
eprom_4k blit1 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(blit1_D),
                .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
                .CS_DL(blit1_cs_i), .WR(ioctl_wr));
eprom_4k blit2 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(blit2_D),
                .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
                .CS_DL(blit2_cs_i), .WR(ioctl_wr));
eprom_4k blit3 (.ADDR(cpu_A[11:0]), .CLK(clk_10m), .DATA(blit3_D),
                .ADDR_DL(ioctl_addr), .CLK_DL(clk_10m), .DATA_IN(ioctl_data),
                .CS_DL(blit3_cs_i), .WR(ioctl_wr));

//------------------------------------------------------------ RAM ------------------------------------------------------------//

// Work RAM (0xE000-0xE3FF, 1KB)
dpram_dc #(.widthad_a(10)) workram
(
    .clock_a(clk_10m),
    .wren_a(cs_workram & ~n_wr),
    .address_a(cpu_A[9:0]),
    .data_a(cpu_Dout),
    .q_a(workram_D),

    .clock_b(clk_10m),
    .wren_b(hs_write),
    .address_b(hs_address[9:0]),
    .data_b(hs_data_in),
    .q_b(hs_data_out)
);

//--------------------------------------------------- Video Control Registers --------------------------------------------------//

// MAME: m_video_control[0..10], written at 0xE800-0xE80A
// Only bits [3:0] of the address select the register (mirrored with 0x03F0)
reg [7:0] video_control [0:10];
integer vc_i;
initial begin
    for (vc_i = 0; vc_i < 11; vc_i = vc_i + 1)
        video_control[vc_i] = 8'd0;
end

// Trigger for blitter execution (directly from Step 4)
reg blitter_start = 0;

always_ff @(posedge clk_10m) begin
    blitter_start <= 0;
    if(cs_vidctrl & ~n_wr) begin
        if(cpu_A[3:0] <= 4'd10)
            video_control[cpu_A[3:0]] <= cpu_Dout;
        if(cpu_A[3:0] == 4'd5)
            blitter_start <= 1;  // Writing to reg 5 triggers DMA blit
    end
end

//------------------------------------------------------- Sound Latch ---------------------------------------------------------//

reg [7:0] slatch = 8'd0;
reg       slatch_wr_pulse = 0;
always_ff @(posedge clk_10m) begin
    slatch_wr_pulse <= 0;
    if(cs_in0 & ~n_wr) begin
        slatch <= cpu_Dout;
        slatch_wr_pulse <= 1;
    end
end
assign sound_latch = slatch;
assign sound_latch_wr = slatch_wr_pulse;

//------------------------------------------------------- Bootleg NMI ---------------------------------------------------------//

// The bootleg has no MCU. It pulses NMI at reset to make the game boot.
// MAME: m_maincpu->pulse_input_line(INPUT_LINE_NMI, attotime::zero);
reg [3:0] nmi_boot_cnt = 4'd0;
reg n_nmi = 1'b1;
always_ff @(posedge clk_10m) begin
    if(!reset) begin
        nmi_boot_cnt <= 4'd15;
        n_nmi <= 1'b1;
    end
    else if(nmi_boot_cnt > 0) begin
        nmi_boot_cnt <= nmi_boot_cnt - 4'd1;
        if(nmi_boot_cnt == 4'd8)
            n_nmi <= 1'b0;  // Assert NMI
        else if(nmi_boot_cnt == 4'd4)
            n_nmi <= 1'b1;  // Release NMI
    end
end

//-------------------------------------------------------- VBlank IRQ ----------------------------------------------------------//

// MAME: standard IM1 interrupt, RST 38h every vblank
// IRQ fires on vblank rising edge
reg n_irq = 1'b1;
reg vblank_last = 0;
always_ff @(posedge clk_10m) begin
    if(!reset) begin
        n_irq <= 1'b1;
        vblank_last <= 0;
    end
    else begin
        vblank_last <= video_vblank;
        // Assert IRQ on rising edge of vblank
        if(video_vblank & ~vblank_last)
            n_irq <= 1'b0;
        // Auto-clear: Z80 IM1 acknowledges via M1+IORQ
        if(~n_m1 & ~n_iorq)
            n_irq <= 1'b1;
    end
end

//-------------------------------------------------------- Video Timing --------------------------------------------------------//

// Kangaroo video timing from MAME:
// screen.set_raw(10_MHz_XTAL, 320*2, 0*2, 256*2, 260, 8, 248);
// Total: 640 pixel clocks horizontal (at 5 MHz that's 320 positions at 10 MHz)
// But the screen uses 10 MHz as raw pixel clock with 640 total, 512 visible
// Vertical: 260 lines total, visible 8-248 (240 lines)
//
// We run the pixel counter at 10 MHz (ce_pix = cen_5m for output,
// but the counters tick every clk_10m)

reg [9:0] h_cnt = 10'd0;  // 0-639
reg [8:0] v_cnt = 9'd0;   // 0-259

always_ff @(posedge clk_10m) begin
    if(!reset) begin
        h_cnt <= 0;
        v_cnt <= 0;
    end
    else begin
        if(h_cnt == 10'd639) begin
            h_cnt <= 0;
            if(v_cnt == 9'd259)
                v_cnt <= 0;
            else
                v_cnt <= v_cnt + 9'd1;
        end
        else
            h_cnt <= h_cnt + 10'd1;
    end
end

// Sync and blank generation
// MAME visible: x = 0*2 to 256*2-1 = 0 to 511, y = 8 to 247
assign video_hblank = (h_cnt >= 10'd512);
assign video_vblank = (v_cnt < 9'd8) | (v_cnt >= 9'd248);
assign video_hsync  = (h_cnt >= 10'd560) & (h_cnt < 10'd624);  // ~64 clocks
assign video_vsync  = (v_cnt >= 9'd252) & (v_cnt < 9'd256);     // ~4 lines

//--------------------------------------------------------- Video RAM ----------------------------------------------------------//

// Kangaroo VRAM: 16384 addresses × 32 bits (4 bytes per word, 2 planes × 4 pixels)
// Split into two 16-bit-wide dpram_dc instances (lo=bytes 0,1  hi=bytes 2,3)
// Port A = CPU/blitter read-modify-write
// Port B = video scanout (read-only)

wire [15:0] vram_lo_qa, vram_hi_qa;   // Port A read data (CPU/blitter side)
wire [15:0] vram_lo_qb, vram_hi_qb;   // Port B read data (scanout side)
reg  [13:0] vram_addr_a;
reg  [15:0] vram_lo_da, vram_hi_da;
reg         vram_we_a;
wire [13:0] vram_addr_b;               // Scanout address (active accent accent driven by compositing logic)

dpram_dc #(.widthad_a(14), .width_a(16)) vram_lo
(
    .clock_a(clk_10m),
    .address_a(vram_addr_a),
    .data_a(vram_lo_da),
    .wren_a(vram_we_a),
    .q_a(vram_lo_qa),

    .clock_b(clk_10m),
    .address_b(vram_addr_b),
    .data_b(16'd0),
    .wren_b(1'b0),
    .q_b(vram_lo_qb)
);

dpram_dc #(.widthad_a(14), .width_a(16)) vram_hi
(
    .clock_a(clk_10m),
    .address_a(vram_addr_a),
    .data_a(vram_hi_da),
    .wren_a(vram_we_a),
    .q_a(vram_hi_qa),

    .clock_b(clk_10m),
    .address_b(vram_addr_b),
    .data_b(16'd0),
    .wren_b(1'b0),
    .q_b(vram_hi_qb)
);

//------------------------------------------------ VRAM Expand/Mask Functions --------------------------------------------------//

// MAME videoram_write expand logic — pure combinational functions
// Expand 8-bit CPU data to 32-bit (DCBADCBA → 4 bytes)
function [31:0] expand_data;
    input [7:0] data;
    reg [31:0] e;
    begin
        e = 32'd0;
        if (data[0]) e = e | 32'h00000055;
        if (data[4]) e = e | 32'h000000aa;
        if (data[1]) e = e | 32'h00005500;
        if (data[5]) e = e | 32'h0000aa00;
        if (data[2]) e = e | 32'h00550000;
        if (data[6]) e = e | 32'h00aa0000;
        if (data[3]) e = e | 32'h55000000;
        if (data[7]) e = e | 32'haa000000;
        expand_data = e;
    end
endfunction

// Build layer mask from 4-bit mask value
function [31:0] build_layermask;
    input [3:0] mask;
    reg [31:0] m;
    begin
        m = 32'd0;
        if (mask[3]) m = m | 32'h30303030;
        if (mask[2]) m = m | 32'hc0c0c0c0;
        if (mask[1]) m = m | 32'h03030303;
        if (mask[0]) m = m | 32'h0c0c0c0c;
        build_layermask = m;
    end
endfunction

//------------------------------------------------------ Blitter ROM -----------------------------------------------------------//

// 16KB blitter ROM — single dpram_dc, port A = blitter read, port B = ioctl download
wire [7:0] blitrom_qa;
reg  [13:0] blitrom_addr_a;

// Compute ioctl write address for blitter ROM download
// blit0 → 0x0000-0x0FFF, blit1 → 0x1000-0x1FFF, blit2 → 0x2000-0x2FFF, blit3 → 0x3000-0x3FFF
wire [13:0] blitrom_dl_addr = blit0_cs_i ? {2'b00, ioctl_addr[11:0]} :
                               blit1_cs_i ? {2'b01, ioctl_addr[11:0]} :
                               blit2_cs_i ? {2'b10, ioctl_addr[11:0]} :
                               blit3_cs_i ? {2'b11, ioctl_addr[11:0]} :
                               14'd0;
wire blitrom_dl_wr = ioctl_wr & (blit0_cs_i | blit1_cs_i | blit2_cs_i | blit3_cs_i);

dpram_dc #(.widthad_a(14), .width_a(8)) blitrom_mem
(
    .clock_a(clk_10m),
    .address_a(blitrom_addr_a),
    .data_a(8'd0),
    .wren_a(1'b0),
    .q_a(blitrom_qa),

    .clock_b(clk_10m),
    .address_b(blitrom_dl_addr),
    .data_b(ioctl_data),
    .wren_b(blitrom_dl_wr),
    .q_b()
);

//------------------------------------------------------ DMA Blitter -----------------------------------------------------------//

// Pipelined read-modify-write state machine:
//   IDLE     — wait for blitter_start or CPU VRAM write
//   RMW_READ — present address to VRAM port A, wait 1 cycle for read data
//   RMW_WRITE— compute new value, write back to VRAM port A
//   BLIT_SETUP — load blitter parameters
//   BLIT_READ  — present blitrom address, wait for ROM data
//   BLIT_RMW_RD — present VRAM dest address, wait for old data
//   BLIT_RMW_WR_LO — write low-half blit result to VRAM
//   BLIT_RMW_RD2 — re-read VRAM for high-half blit
//   BLIT_RMW_WR_HI — write high-half blit result, advance to next pixel

localparam ST_IDLE        = 4'd0;
localparam ST_CPU_RMW_RD  = 4'd1;
localparam ST_CPU_RMW_WR  = 4'd2;
localparam ST_BLIT_SETUP  = 4'd3;
localparam ST_BLIT_ROMRD  = 4'd4;
localparam ST_BLIT_RMW_RD = 4'd5;
localparam ST_BLIT_RMW_WR_LO = 4'd6;
localparam ST_BLIT_RMW_RD2   = 4'd7;
localparam ST_BLIT_RMW_WR_HI = 4'd8;
localparam ST_BLIT_NEXT   = 4'd9;
localparam ST_CPU_RMW_RD2 = 4'd10;

reg [3:0]  vram_state = ST_IDLE;
reg [7:0]  blit_width;
reg [7:0]  blit_height;
reg [7:0]  blit_x_cnt;
reg [7:0]  blit_y_cnt;
reg [15:0] blit_cur_src;
reg [15:0] blit_cur_dst;
reg [7:0]  blit_adj_mask;
reg [7:0]  blit_rom_data_lo;
reg [7:0]  blit_rom_data_hi;

// Registered copies of CPU write params (captured in IDLE)
reg [13:0] cpu_wr_addr;
reg [7:0]  cpu_wr_data;
reg [3:0]  cpu_wr_mask;

// Read-modify-write intermediates
reg [31:0] rmw_old_word;
reg [31:0] rmw_new_word;
reg [31:0] rmw_expdata;
reg [31:0] rmw_layermask;

always_ff @(posedge clk_10m) begin
    if (!reset) begin
        vram_state <= ST_IDLE;
        vram_we_a <= 0;
    end
    else begin
        vram_we_a <= 0;  // Default: no write

        case (vram_state)
            ST_IDLE: begin
                if (blitter_start) begin
                    // Latch blitter params
                    blit_width   <= video_control[4];
                    blit_height  <= video_control[5];
                    blit_cur_src <= {video_control[1], video_control[0]};
                    blit_cur_dst <= {video_control[3], video_control[2]};
                    blit_x_cnt  <= 0;
                    blit_y_cnt  <= 0;
                    // Compute adjusted mask
                    blit_adj_mask <= video_control[8];
                    vram_state <= ST_BLIT_SETUP;
                end
                else if (cs_videoram & ~n_wr) begin
                    // CPU VRAM write — start RMW cycle
                    cpu_wr_addr <= cpu_A[13:0];
                    cpu_wr_data <= cpu_Dout;
                    cpu_wr_mask <= video_control[8][3:0];
                    vram_addr_a <= cpu_A[13:0];  // Present read address
                    vram_state <= ST_CPU_RMW_RD;
                end
            end

            //--- CPU VRAM write (3-cycle RMW: addr → wait → read+compute → write) ---
            ST_CPU_RMW_RD: begin
                // Address was presented last cycle. dpram output will be valid NEXT cycle.
                // Pre-compute expand/mask while waiting for RAM.
                rmw_expdata  <= expand_data(cpu_wr_data);
                rmw_layermask <= build_layermask(cpu_wr_mask);
                vram_state <= ST_CPU_RMW_RD2;
            end

            ST_CPU_RMW_RD2: begin
                // NOW the dpram output is valid — latch it
                rmw_old_word <= {vram_hi_qa, vram_lo_qa};
                vram_state <= ST_CPU_RMW_WR;
            end

            ST_CPU_RMW_WR: begin
                rmw_new_word = (rmw_old_word & ~rmw_layermask) | (rmw_expdata & rmw_layermask);
                vram_addr_a <= cpu_wr_addr;
                vram_lo_da <= rmw_new_word[15:0];
                vram_hi_da <= rmw_new_word[31:16];
                vram_we_a  <= 1;
                vram_state <= ST_IDLE;
            end

            //--- Blitter DMA ---
            ST_BLIT_SETUP: begin
                // Adjust mask per MAME: OR top/bottom 2-bit pairs during DMA
                if (blit_adj_mask[3:2] != 0) blit_adj_mask[3:2] <= 2'b11;
                if (blit_adj_mask[1:0] != 0) blit_adj_mask[1:0] <= 2'b11;
                // Start first pixel: read low-half ROM
                blitrom_addr_a <= {1'b0, blit_cur_src[12:0]};
                vram_state <= ST_BLIT_ROMRD;
            end

            ST_BLIT_ROMRD: begin
                // Capture low-half ROM data, now read high-half
                blit_rom_data_lo <= blitrom_qa;
                blitrom_addr_a <= {1'b1, blit_cur_src[12:0]};
                // Simultaneously present VRAM read address for RMW
                vram_addr_a <= (blit_cur_dst[13:0] + {6'd0, blit_x_cnt}) & 14'h3FFF;
                vram_state <= ST_BLIT_RMW_RD;
            end

            ST_BLIT_RMW_RD: begin
                // Capture high-half ROM data (blitrom has 1-cycle latency, presented last cycle)
                blit_rom_data_hi <= blitrom_qa;
                // VRAM address was presented last cycle — output valid NEXT cycle
                vram_state <= ST_BLIT_RMW_RD2;
            end

            ST_BLIT_RMW_RD2: begin
                // NOW VRAM output is valid — latch it
                rmw_old_word <= {vram_hi_qa, vram_lo_qa};
                vram_state <= ST_BLIT_RMW_WR_LO;
            end

            ST_BLIT_RMW_WR_LO: begin
                // Apply low-half blit (mask & 0x05)
                rmw_expdata   = expand_data(blit_rom_data_lo);
                rmw_layermask = build_layermask(blit_adj_mask[3:0] & 4'b0101);
                rmw_new_word  = (rmw_old_word & ~rmw_layermask) | (rmw_expdata & rmw_layermask);
                // Now apply high-half blit (mask & 0x0A) on top of that
                rmw_expdata   = expand_data(blit_rom_data_hi);
                rmw_layermask = build_layermask(blit_adj_mask[3:0] & 4'b1010);
                rmw_new_word  = (rmw_new_word & ~rmw_layermask) | (rmw_expdata & rmw_layermask);
                // Write back
                vram_addr_a <= (blit_cur_dst[13:0] + {6'd0, blit_x_cnt}) & 14'h3FFF;
                vram_lo_da  <= rmw_new_word[15:0];
                vram_hi_da  <= rmw_new_word[31:16];
                vram_we_a   <= 1;
                vram_state  <= ST_BLIT_NEXT;
            end

            ST_BLIT_NEXT: begin
                // Advance to next pixel
                blit_cur_src <= blit_cur_src + 16'd1;
                if (blit_x_cnt == blit_width) begin
                    blit_x_cnt <= 0;
                    if (blit_y_cnt == blit_height) begin
                        vram_state <= ST_IDLE;  // Done
                    end
                    else begin
                        blit_y_cnt  <= blit_y_cnt + 8'd1;
                        blit_cur_dst <= blit_cur_dst + 16'd256;
                        // Start next row: read ROM for first pixel
                        blitrom_addr_a <= {1'b0, (blit_cur_src + 16'd1) & 16'h1FFF};
                        vram_state <= ST_BLIT_ROMRD;
                    end
                end
                else begin
                    blit_x_cnt <= blit_x_cnt + 8'd1;
                    // Start next pixel: read ROM
                    blitrom_addr_a <= {1'b0, (blit_cur_src + 16'd1) & 16'h1FFF};
                    vram_state <= ST_BLIT_ROMRD;
                end
            end

            default: vram_state <= ST_IDLE;
        endcase
    end
end

//----------------------------------------------- Pixel Compositing (screen_update) --------------------------------------------//

// MAME screen_update variables derived from video_control registers
wire [7:0] scrolly = video_control[6];
wire [7:0] scrollx = video_control[7];
wire [2:0] maska = (video_control[10] & 8'h28) >> 3;   // MAME exact
wire [2:0] maskb =  video_control[10][2:0];
wire [7:0] xora = video_control[9][5] ? 8'hFF : 8'h00;
wire [7:0] xorb = video_control[9][4] ? 8'hFF : 8'h00;
wire       enaa = video_control[9][3];
wire       enab = video_control[9][2];
wire       pria = ~video_control[9][1];
wire       prib = ~video_control[9][0];

// Current scanout position (used for address)
wire [8:0] scan_x = h_cnt[9:1];
wire [7:0] scan_y = v_cnt[7:0];

// Plane A / B coordinates (combo for address generation)
wire [7:0] effxa = scrollx + (scan_x[7:0] ^ xora);
wire [7:0] effya = scrolly + (scan_y ^ xora);
wire [7:0] effxb = scan_x[7:0] ^ xorb;
wire [7:0] effyb = scan_y ^ xorb;

// Scanout pipeline — locked to pixel clock (h_cnt[0])
// h_cnt[0]=0: present plane A address → dpram latches internally
// h_cnt[0]=1: plane A data valid, present plane B address
// Next h_cnt[0]=0: plane B data valid, latch both, present next plane A address
//
// This gives us both plane A and B data valid at each pixel boundary.

reg [31:0] scan_word_a, scan_word_b;
reg [1:0]  scan_effxa_slice, scan_effxb_slice;

wire [13:0] scan_addr_a_w = {effxa[7:2], effya};
wire [13:0] scan_addr_b_w = {effxb[7:2], effyb};

// Mux port B address: plane A on even h_cnt, plane B on odd h_cnt
assign vram_addr_b = h_cnt[0] ? scan_addr_b_w : scan_addr_a_w;

always_ff @(posedge clk_10m) begin
    if (h_cnt[0]) begin
        // Odd clock: plane A data now valid from address presented on even clock
        scan_word_a <= {vram_hi_qb, vram_lo_qb};
        scan_effxa_slice <= effxa[1:0];
    end
    else begin
        // Even clock: plane B data now valid from address presented on odd clock
        scan_word_b <= {vram_hi_qb, vram_lo_qb};
        scan_effxb_slice <= effxb[1:0];
    end
end

// Extract pixel bytes from latched 32-bit words
wire [7:0] vram_slice_a = (scan_effxa_slice == 2'd0) ? scan_word_a[7:0] :
                          (scan_effxa_slice == 2'd1) ? scan_word_a[15:8] :
                          (scan_effxa_slice == 2'd2) ? scan_word_a[23:16] :
                                                       scan_word_a[31:24];

wire [7:0] vram_slice_b = (scan_effxb_slice == 2'd0) ? scan_word_b[7:0] :
                          (scan_effxb_slice == 2'd1) ? scan_word_b[15:8] :
                          (scan_effxb_slice == 2'd2) ? scan_word_b[23:16] :
                                                       scan_word_b[31:24];

wire [3:0] pixa_raw = vram_slice_a[3:0];   // Plane A = low nibble
wire [3:0] pixb_raw = vram_slice_b[7:4];   // Plane B = high nibble

// Priority compositing (MAME logic)
// Even pixels (first of pair): full brightness, no KOS1 masking
// Odd pixels (second of pair): apply KOS1 color mask for Z=0 pixels
wire is_odd_pixel = h_cnt[0];

wire [3:0] pixa_masked = (is_odd_pixel && !(pixa_raw[3])) ? (pixa_raw & {1'b0, maska}) : pixa_raw;
wire [3:0] pixb_masked = (is_odd_pixel && !(pixb_raw[3])) ? (pixb_raw & {1'b0, maskb}) : pixb_raw;

wire [3:0] pixa_final = is_odd_pixel ? pixa_masked : pixa_raw;
wire [3:0] pixb_final = is_odd_pixel ? pixb_masked : pixb_raw;

reg [2:0] final_color;
always_comb begin
    final_color = 3'd0;
    if (enaa && (pria || pixb_final == 0))
        final_color = final_color | pixa_final[2:0];
    if (enab && (prib || pixa_final == 0))
        final_color = final_color | pixb_final[2:0];
end

// Output — BGR 3-bit palette (MAME: PALETTE(config, m_palette, palette_device::BGR_3BIT))
wire active_video = ~video_hblank & ~video_vblank;
assign video_r = active_video ? {final_color[0], final_color[0], final_color[0]} : 3'd0;
assign video_g = active_video ? {final_color[1], final_color[1], final_color[1]} : 3'd0;
assign video_b = active_video ? {final_color[2], final_color[2]} : 2'd0;

endmodule
