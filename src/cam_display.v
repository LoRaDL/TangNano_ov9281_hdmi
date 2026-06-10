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
            
            if (active_q) begin
                // Map 4-bit BRAM grayscale pixel data to 24-bit RGB (R=G=B)
                out_axis_tdata <= { {rd_data, rd_data}, {rd_data, rd_data}, {rd_data, rd_data} };
            end else begin
                out_axis_tdata <= 24'd0; // Black border
            end
            
            out_axis_tuser[0] <= sof_q;
        end
    end
endmodule
