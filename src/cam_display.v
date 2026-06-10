`timescale 1ns / 1ps

module cam_display (
    input clk,
    input resetn,
    input test_mode, // 1: procedural test pattern, 0: BRAM camera data
    input [9:0] max_line_cnt,
    input [11:0] max_pixel_cnt,

    // RAM read interface
    output [15:0] rd_addr,
    input [3:0] rd_data,

    // AXI-Stream output
    output reg out_axis_tvalid,
    input out_axis_tready,
    output reg [23:0] out_axis_tdata,
    output reg [0:0] out_axis_tuser
);
    parameter SVO_HOR_PIXELS = 640;
    parameter SVO_VER_PIXELS = 480;
    parameter CAM_HOR_PIXELS = 320;
    parameter CAM_VER_PIXELS = 140; // Exactly 560 camera lines downsampled by 4

    reg [9:0] hcursor; // 0 to 639
    reg [9:0] vcursor; // 0 to 479

    // Check if we should advance the generator coordinates
    wire advance = !out_axis_tvalid || out_axis_tready;

    reg [9:0] next_hcursor;
    reg [9:0] next_vcursor;

    always @(*) begin
        if (hcursor == SVO_HOR_PIXELS - 1) begin
            next_hcursor = 10'd0;
            if (vcursor == SVO_VER_PIXELS - 1) begin
                next_vcursor = 10'd0;
            end else begin
                next_vcursor = vcursor + 10'd1;
            end
        end else begin
            next_hcursor = hcursor + 10'd1;
            next_vcursor = vcursor;
        end
    end

    // Center the 320x140 camera image vertically on 640x480 screen.
    // 320x140 scaled up by 2 is 640x280.
    // Vertical active range for camera: from line 100 to line 379 (inclusive)
    // Horizontal range is 0 to 639 (scaled 2x from 320 pixels)
    wire [9:0] v_offset = (next_vcursor >= 10'd100) ? (next_vcursor - 10'd100) : 10'd0;
    wire [8:0] v_index = v_offset[9:1];      // (next_vcursor - 100) / 2
    wire [8:0] h_index = next_hcursor[9:1];   // next_hcursor / 2

    // Check if next coordinate is in the active camera display window (lines 100 to 379)
    wire next_active = (next_vcursor >= 10'd100 && next_vcursor < 10'd380);

    // RAM address: v_index * 320 + h_index
    // Uses shift-and-add: 320 = 256 + 64
    assign rd_addr = {v_index, 8'b0} + {v_index, 6'b0} + h_index;

    // Registers to pipeline the control signals to align with RAM's 1-cycle read latency
    reg active_q;
    reg sof_q;

    // Delayed version of cursors for procedural test pattern generation
    reg [9:0] hcursor_q;
    reg [9:0] vcursor_q;

    // Resource-friendly division by 3: (max_pixel_cnt * 341) >> 10
    wire [21:0] pclk_mult = {10'd0, max_pixel_cnt} * 12'd341;
    wire [9:0] pclk_scaled = pclk_mult[19:10];
    
    // Division by 2 for line count
    wire [9:0] line_scaled = max_line_cnt[9:1];

    // Debug screen color generation
    wire is_pclk_bar  = (vcursor_q >= 10'd150 && vcursor_q < 10'd200) && (hcursor_q < pclk_scaled);
    wire is_line_bar  = (vcursor_q >= 10'd270 && vcursor_q < 10'd320) && (hcursor_q < line_scaled);
    
    // Scale tick lines (vertical dashed lines)
    // PCLK axis ticks at 958 (x=319), 1280 (x=426), 1384 (x=460)
    wire is_tick_958  = (hcursor_q == 10'd319) && (vcursor_q >= 10'd130 && vcursor_q < 10'd220) && (vcursor_q[2] == 1'b0);
    wire is_tick_1280 = (hcursor_q == 10'd426) && (vcursor_q >= 10'd130 && vcursor_q < 10'd220) && (vcursor_q[2] == 1'b0);
    wire is_tick_1384 = (hcursor_q == 10'd460) && (vcursor_q >= 10'd130 && vcursor_q < 10'd220) && (vcursor_q[2] == 1'b0);
    
    // Line axis ticks at 400 (x=200), 800 (x=400)
    wire is_tick_400  = (hcursor_q == 10'd200) && (vcursor_q >= 10'd250 && vcursor_q < 10'd340) && (vcursor_q[2] == 1'b0);
    wire is_tick_800  = (hcursor_q == 10'd400) && (vcursor_q >= 10'd250 && vcursor_q < 10'd340) && (vcursor_q[2] == 1'b0);
    
    // Color mapping corrected for BGR24 format: {B[7:0], G[7:0], R[7:0]}
    wire [23:0] debug_pixel = is_pclk_bar  ? 24'h00FF00 : // Green bar (B=00, G=FF, R=00)
                              is_line_bar  ? 24'hFFFF00 : // Cyan bar (B=FF, G=FF, R=00)
                              is_tick_958  ? 24'h0080FF : // Orange tick (B=00, G=80, R=FF)
                              is_tick_1280 ? 24'h0000FF : // Red tick (B=00, G=00, R=FF)
                              is_tick_1384 ? 24'hFF00FF : // Magenta tick (B=FF, G=00, R=FF)
                              is_tick_400  ? 24'h00FFFF : // Yellow tick (B=00, G=FF, R=FF)
                              is_tick_800  ? 24'hFF00FF : // Magenta tick (B=FF, G=00, R=FF)
                                             24'h2D1E14;  // Slate blue background (B=45, G=30, R=20)

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            hcursor <= 10'd0;
            vcursor <= 10'd0;
            out_axis_tvalid <= 1'b0;
            out_axis_tdata <= 24'd0;
            out_axis_tuser <= 1'b0;
            active_q <= 1'b0;
            sof_q <= 1'b0;
            hcursor_q <= 10'd0;
            vcursor_q <= 10'd0;
        end else if (advance) begin
            hcursor <= next_hcursor;
            vcursor <= next_vcursor;

            active_q <= next_active;
            sof_q <= (next_hcursor == 10'd0 && next_vcursor == 10'd0);
            
            hcursor_q <= hcursor;
            vcursor_q <= vcursor;

            out_axis_tvalid <= 1'b1;
            
            if (test_mode) begin
                out_axis_tdata <= debug_pixel;
            end else if (active_q) begin
                // Map 4-bit BRAM grayscale pixel data to 24-bit RGB (R=G=B)
                out_axis_tdata <= { {rd_data, rd_data}, {rd_data, rd_data}, {rd_data, rd_data} };
            end else begin
                out_axis_tdata <= 24'd0; // Black border
            end
            
            out_axis_tuser[0] <= sof_q;
        end
    end
endmodule
