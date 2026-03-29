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

// Kangaroo VRAM: 256 rows × 64 columns × 32 bits = 16384 addresses × 32 bits
// MAME: memory_share_creator<uint32_t> m_videoram("videoram", 256*64*4, ENDIANNESS_LITTLE)
// Each 32-bit word holds 4 pixels × 2 planes (A low nibble, B high nibble) × 4 bytes
//
// For FPGA: use 4 × 8-bit dpram banks to form 32-bit words.
// Address = 14 bits (16384 entries), each entry is 4 bytes.
// CPU writes go through videoram_write expand logic.
// Scanout reads 32-bit words for pixel extraction.

reg [7:0] vram_byte0 [0:16383];
reg [7:0] vram_byte1 [0:16383];
reg [7:0] vram_byte2 [0:16383];
reg [7:0] vram_byte3 [0:16383];

initial begin
    integer vi;
    for (vi = 0; vi < 16384; vi = vi + 1) begin
        vram_byte0[vi] = 8'd0;
        vram_byte1[vi] = 8'd0;
        vram_byte2[vi] = 8'd0;
        vram_byte3[vi] = 8'd0;
    end
end

// Read 32-bit word from VRAM (for scanout and CPU read-back)
function [31:0] vram_read;
    input [13:0] addr;
    vram_read = {vram_byte3[addr], vram_byte2[addr], vram_byte1[addr], vram_byte0[addr]};
endfunction

//-------------------------------------------------- VRAM Write Logic ----------------------------------------------------------//

// MAME videoram_write: expand 8-bit CPU data into 32-bit with layer masks
// data contains 4 2-bit values packed as DCBADCBA
// expands into 4 bytes, each byte holding 2 bits per plane

task automatic vram_write_word;
    input [13:0] addr;
    input [7:0]  data;
    input [3:0]  mask;
    reg [31:0] expdata;
    reg [31:0] layermask;
    reg [31:0] old_val;
    begin
        expdata = 32'd0;
        if (data[0]) expdata = expdata | 32'h00000055;
        if (data[4]) expdata = expdata | 32'h000000aa;
        if (data[1]) expdata = expdata | 32'h00005500;
        if (data[5]) expdata = expdata | 32'h0000aa00;
        if (data[2]) expdata = expdata | 32'h00550000;
        if (data[6]) expdata = expdata | 32'h00aa0000;
        if (data[3]) expdata = expdata | 32'h55000000;
        if (data[7]) expdata = expdata | 32'haa000000;

        layermask = 32'd0;
        if (mask[3]) layermask = layermask | 32'h30303030;
        if (mask[2]) layermask = layermask | 32'hc0c0c0c0;
        if (mask[1]) layermask = layermask | 32'h03030303;
        if (mask[0]) layermask = layermask | 32'h0c0c0c0c;

        old_val = {vram_byte3[addr], vram_byte2[addr], vram_byte1[addr], vram_byte0[addr]};
        old_val = (old_val & ~layermask) | (expdata & layermask);

        vram_byte0[addr] = old_val[7:0];
        vram_byte1[addr] = old_val[15:8];
        vram_byte2[addr] = old_val[23:16];
        vram_byte3[addr] = old_val[31:24];
    end
endtask

//------------------------------------------------------ DMA Blitter -----------------------------------------------------------//

// MAME blitter_execute — triggered when video_control[5] is written
// Reads from blitter ROM, writes to VRAM through videoram_write logic
// This runs instantaneously in MAME; in FPGA we execute it over multiple clocks.

// Blitter ROM: 4 x 4KB = 16KB, split into two halves of 8KB each (gfxhalfsize = 0x2000)
// Half 0: blit0 + blit2 (low bank), Half 1: blit1 + blit3 (high bank)
// Read address for blitter: use dedicated read ports

// For blitter ROM reads, we need a second address into the blit ROMs.
// We'll read from the ROMs using a blitter-specific address bus.
// This requires the blit ROMs to have a second read port — use the dpram_dc port B.
//
// SIMPLIFICATION: Since the blitter runs during CPU time (CPU is effectively stalled
// in real hardware during DMA), we time-multiplex the ROM address bus.
// The blitter runs at 1 word/clock, completing in (width+1)*(height+1) clocks.

reg blit_active = 0;
reg [15:0] blit_src;
reg [15:0] blit_dst;
reg [7:0]  blit_width;
reg [7:0]  blit_height;
reg [7:0]  blit_mask;
reg [7:0]  blit_x_cnt;
reg [7:0]  blit_y_cnt;
reg [15:0] blit_cur_src;
reg [15:0] blit_cur_dst;

// Blitter state machine
localparam BLIT_IDLE = 2'd0;
localparam BLIT_READ = 2'd1;
localparam BLIT_WRITE = 2'd2;
reg [1:0] blit_state = BLIT_IDLE;

// Blitter ROM read (direct array access using src address)
// gfxhalfsize = 0x2000
// blit_rom_data_lo = blitrom[0*0x2000 + (src & 0x1FFF)] — from blit0/blit2
// blit_rom_data_hi = blitrom[1*0x2000 + (src & 0x1FFF)] — from blit1/blit3
wire [12:0] blit_rom_addr = blit_cur_src[12:0];  // & 0x1FFF
wire [7:0] blit_rom_lo, blit_rom_hi;

// We need dedicated blitter read ports on the blit ROMs.
// blit0/blit2 form the low half, blit1/blit3 form the high half.
// addr[12] selects blit0 vs blit2 (or blit1 vs blit3).
// BUT: gfxhalfsize = total_blit_size/2 = 0x2000, so src wraps at 13 bits.
// Low half: address 0x0000-0x0FFF = blit0, 0x1000-0x1FFF = blit2
// High half: address 0x0000-0x0FFF = blit1, 0x1000-0x1FFF = blit3

// For now, build a combined blitter ROM read using a simple registered approach.
// The blit ROMs are already instantiated as eprom_4k with port A = CPU, port B = download.
// We need a THIRD read for the blitter.
//
// PRACTICAL SOLUTION: Use registered shadow copies loaded at download time,
// OR add the blitter as a combinational read from the same eprom_4k instances
// by time-multiplexing. Since blitter runs when CPU is NOT accessing blit ROMs
// (blitter writes to VRAM, not reads from blit bank), we can safely share port A.
//
// Actually simpler: just declare blitter ROM as a separate 16KB block RAM loaded at download.

reg [7:0] blitrom [0:16383];  // 16KB blitter ROM flat
initial begin
    integer bi;
    for (bi = 0; bi < 16384; bi = bi + 1)
        blitrom[bi] = 8'd0;
end

// Load blitter ROM from ioctl (index 2)
always_ff @(posedge clk_10m) begin
    if(ioctl_wr) begin
        if(blit0_cs_i) blitrom[{2'b00, ioctl_addr[11:0]}] <= ioctl_data;
        if(blit1_cs_i) blitrom[{2'b01, ioctl_addr[11:0]}] <= ioctl_data;
        if(blit2_cs_i) blitrom[{2'b10, ioctl_addr[11:0]}] <= ioctl_data;
        if(blit3_cs_i) blitrom[{2'b11, ioctl_addr[11:0]}] <= ioctl_data;
    end
end

// Blitter execution state machine
always_ff @(posedge clk_10m) begin
    if(!reset) begin
        blit_active <= 0;
        blit_state <= BLIT_IDLE;
    end
    else begin
        case(blit_state)
            BLIT_IDLE: begin
                // CPU VRAM writes allowed only when blitter is idle (matches real HW bus-stall)
                if(cs_videoram & ~n_wr) begin
                    vram_write_word(cpu_A[13:0], cpu_Dout, video_control[8][3:0]);
                end
                if(blitter_start) begin
                    blit_src <= {video_control[1], video_control[0]};
                    blit_dst <= {video_control[3], video_control[2]};
                    blit_width <= video_control[4];
                    blit_height <= video_control[5];
                    blit_mask <= video_control[8];
                    // Adjust mask per MAME: OR top/bottom 2 bits during DMA
                    blit_x_cnt <= 0;
                    blit_y_cnt <= 0;
                    blit_cur_src <= {video_control[1], video_control[0]};
                    blit_cur_dst <= {video_control[3], video_control[2]};
                    blit_active <= 1;
                    blit_state <= BLIT_WRITE;
                end
            end

            BLIT_WRITE: begin
                // Compute effective addresses
                // effdst = (dst + x) & 0x3FFF
                // effsrc = src & (gfxhalfsize-1) = src & 0x1FFF
                reg [13:0] effdst;
                reg [12:0] effsrc;
                reg [7:0] adj_mask;
                effdst = (blit_cur_dst + {8'd0, blit_x_cnt}) & 14'h3FFF;
                effsrc = blit_cur_src[12:0];

                adj_mask = blit_mask;
                if (adj_mask[3:2] != 0) adj_mask[3:2] = 2'b11;
                if (adj_mask[1:0] != 0) adj_mask[1:0] = 2'b11;

                // Write low half (mask & 0x05)
                vram_write_word(effdst, blitrom[{1'b0, effsrc}], adj_mask[3:0] & 4'b0101);
                // Write high half (mask & 0x0A)
                vram_write_word(effdst, blitrom[{1'b1, effsrc}], adj_mask[3:0] & 4'b1010);

                blit_cur_src <= blit_cur_src + 16'd1;

                if(blit_x_cnt == blit_width) begin
                    blit_x_cnt <= 0;
                    if(blit_y_cnt == blit_height) begin
                        blit_active <= 0;
                        blit_state <= BLIT_IDLE;
                    end
                    else begin
                        blit_y_cnt <= blit_y_cnt + 8'd1;
                        blit_cur_dst <= blit_cur_dst + 16'd256;
                    end
                end
                else begin
                    blit_x_cnt <= blit_x_cnt + 8'd1;
                end
            end

            default: blit_state <= BLIT_IDLE;
        endcase
    end
end

//----------------------------------------------- Pixel Compositing (screen_update) --------------------------------------------//

// MAME screen_update variables derived from video_control registers
wire [7:0] scrolly = video_control[6];
wire [7:0] scrollx = video_control[7];
wire [2:0] maska = {video_control[10][5], video_control[10][3], 1'b0}; // (vc[10] & 0x28) >> 3
wire [2:0] maskb = video_control[10][2:0];                              // (vc[10] & 0x07)
wire [7:0] xora = video_control[9][5] ? 8'hFF : 8'h00;
wire [7:0] xorb = video_control[9][4] ? 8'hFF : 8'h00;
wire       enaa = video_control[9][3];
wire       enab = video_control[9][2];
wire       pria = ~video_control[9][1];
wire       prib = ~video_control[9][0];

// Current scanout position (from video timing counters)
// h_cnt runs 0-639 at 10MHz. Visible pixels are 0-511.
// MAME iterates x in steps of 2 (x, x+1), so each MAME x maps to h_cnt/2.
// For the FPGA, we compute pixel values on the fly during scanout.

wire [8:0] scan_x = h_cnt[9:1];  // 0-319, but visible 0-255
wire [7:0] scan_y = v_cnt[7:0];  // 0-259, visible 8-247

// Plane A coordinates (with scroll and flip)
wire [7:0] effxa = scrollx + (scan_x[7:0] ^ xora);
wire [7:0] effya = scrolly + (scan_y ^ xora);

// Plane B coordinates (no scroll, just flip)
wire [7:0] effxb = scan_x[7:0] ^ xorb;
wire [7:0] effyb = scan_y ^ xorb;

// VRAM read for plane A
// Address = effya + 256 * (effxa / 4), byte select = effxa % 4
wire [13:0] vram_addr_a = {effxa[7:2], effya};
wire [31:0] vram_word_a = {vram_byte3[vram_addr_a], vram_byte2[vram_addr_a],
                           vram_byte1[vram_addr_a], vram_byte0[vram_addr_a]};
wire [7:0]  vram_slice_a = (effxa[1:0] == 2'd0) ? vram_word_a[7:0] :
                           (effxa[1:0] == 2'd1) ? vram_word_a[15:8] :
                           (effxa[1:0] == 2'd2) ? vram_word_a[23:16] :
                                                   vram_word_a[31:24];
wire [3:0] pixa_raw = vram_slice_a[3:0];  // Plane A = low nibble

// VRAM read for plane B
wire [13:0] vram_addr_b = {effxb[7:2], effyb};
wire [31:0] vram_word_b = {vram_byte3[vram_addr_b], vram_byte2[vram_addr_b],
                           vram_byte1[vram_addr_b], vram_byte0[vram_addr_b]};
wire [7:0]  vram_slice_b = (effxb[1:0] == 2'd0) ? vram_word_b[7:0] :
                           (effxb[1:0] == 2'd1) ? vram_word_b[15:8] :
                           (effxb[1:0] == 2'd2) ? vram_word_b[23:16] :
                                                   vram_word_b[31:24];
wire [3:0] pixb_raw = vram_slice_b[7:4];  // Plane B = high nibble

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
