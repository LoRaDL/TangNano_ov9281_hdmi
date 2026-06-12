module top (
    input sys_clk,          // 27MHz crystal input
    input sys_rst_n,        // Reset button S1 (active low, Pin 4)
    input button,           // Button S2 (active low, Pin 3)
    output [5:0] led,       // Onboard LEDs (active low, pins 10, 11, 13, 14, 15, 16)
    
    // Camera pins (connected to 3.3V GPIOs on Header J1 via level translator)
    output cam_xclk,        // Pin 29 (Header Pin 9) -> 13.5MHz clock output
    output cam_pwdn,        // Pin 28 (Header Pin 8) -> PWDN output (active high)
    output cam_rst_n,       // Pin 27 (Header Pin 7) -> RESET output (active low)
    output cam_scl,         // Pin 25 (Header Pin 5) -> SCL output
    inout cam_sda,          // Pin 26 (Header Pin 6) -> SDA bidirectional
    
    // DVP Parallel interface pins
    input cam_pclk,         // Pin 30 -> Pixel clock
    input cam_vsync,        // Pin 33 -> Frame Sync
    input cam_href,         // Pin 34 -> Line Sync / Data Enable
    input [7:0] cam_data,   // Pins 40, 35, 41, 42, 51, 53, 36, 39 -> D9-D2 (8-bit RAW data)

    // HDMI pins
    output       tmds_clk_n,
    output       tmds_clk_p,
    output [2:0] tmds_d_n,
    output [2:0] tmds_d_p,

    // PSRAM pins
    output [0:0] O_psram_ck,
    output [0:0] O_psram_ck_n,
    inout [7:0] IO_psram_dq,
    inout [0:0] IO_psram_rwds,
    output [0:0] O_psram_cs_n,
    output [0:0] O_psram_reset_n
);

// Camera System Clock generation: Divide 27MHz by 2 to get 13.5MHz (safe for wiring)
reg xclk_reg = 0;
always @(posedge sys_clk) begin
    xclk_reg <= ~xclk_reg;
end
assign cam_xclk = xclk_reg;

// 1. Camera power-up and reset sequence (with XCLK active)
reg [23:0] startup_cnt;
reg rst_cam_n;
reg pwdn_cam;
reg start_i2c;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        startup_cnt <= 24'd0;
        rst_cam_n <= 1'b0; // hold camera in reset
        pwdn_cam <= 1'b1;  // hold camera in power down
        start_i2c <= 1'b0;
    end else begin
        if (startup_cnt < 24'd12_000_000) begin // ~444ms startup delay
            startup_cnt <= startup_cnt + 24'd1;
            start_i2c <= 1'b0;
            if (startup_cnt < 24'd2_700_000) begin // 0 - 100ms
                rst_cam_n <= 1'b0;
                pwdn_cam <= 1'b1;
            end else if (startup_cnt < 24'd5_400_000) begin // 100ms - 200ms
                rst_cam_n <= 1'b0;
                pwdn_cam <= 1'b0; // Wake up PWDN
            end else begin // 200ms - 444ms
                rst_cam_n <= 1'b1; // Release Reset (RST_N -> High)
                pwdn_cam <= 1'b0;
            end
        end else begin
            start_i2c <= 1'b1; // Trigger I2C Read
        end
    end
end

assign cam_rst_n = rst_cam_n;
assign cam_pwdn = pwdn_cam;

// 2. I2C / SCCB Master Clock Divider (400kHz tick rate for 100kHz I2C SCL)
reg [6:0] tick_cnt;
reg tick;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        tick_cnt <= 7'd0;
        tick <= 1'b0;
    end else if (tick_cnt == 7'd67) begin // 27,000,000 / 68 ≈ 397 kHz
        tick_cnt <= 7'd0;
        tick <= 1'b1;
    end else begin
        tick_cnt <= tick_cnt + 7'd1;
        tick <= 1'b0;
    end
end

// 3. I2C / SCCB State Machine
reg [3:0] state;
reg [1:0] quad_cnt;
reg [3:0] bit_cnt;
reg [2:0] byte_cnt;
reg [7:0] shift_reg;
reg [7:0] read_data;
reg scl_out;
reg sda_out;
reg sda_oe;
reg i2c_error;
reg [4:0] trans_idx;
reg stream_enabled;

localparam STATE_IDLE      = 4'd0;
localparam STATE_START     = 4'd1;
localparam STATE_WRITE     = 4'd2;
localparam STATE_ACK       = 4'd3;
localparam STATE_REP_START = 4'd4;
localparam STATE_READ      = 4'd5;
localparam STATE_SEND_NACK = 4'd6;
localparam STATE_STOP      = 4'd7;
localparam STATE_DONE      = 4'd8;
localparam STATE_ERROR     = 4'd9;

// Bidirectional SDA Driver
assign cam_sda = sda_oe ? sda_out : 1'bZ;
wire sda_in = cam_sda;
assign cam_scl = scl_out;

// Multiplexer for sequential writes
reg [7:0] next_write_msb;
reg [7:0] next_write_lsb;
reg [7:0] next_write_data;

always @(*) begin
    case (trans_idx)
        5'd1: begin // Write 0x00 to 0x3014 (Disable MIPI, Enable DVP)
            next_write_msb  = 8'h30;
            next_write_lsb  = 8'h14;
            next_write_data = 8'h00;
        end
        5'd2: begin // Write 0x22 to 0x3039 (Disable MIPI, Enable DVP)
            next_write_msb  = 8'h30;
            next_write_lsb  = 8'h39;
            next_write_data = 8'h22;
        end
        5'd3: begin // Write 0x01 to 0x4317 (Enable DVP option)
            next_write_msb  = 8'h43;
            next_write_lsb  = 8'h17;
            next_write_data = 8'h01;
        end
        5'd4: begin // Write 0x01 to 0x4701 (Output standard VSYNC)
            next_write_msb  = 8'h47;
            next_write_lsb  = 8'h01;
            next_write_data = 8'h01;
        end
        5'd5: begin // Write 0x01 to 0x300D (Enable DVP clock gclk_dvp)
            next_write_msb  = 8'h30;
            next_write_lsb  = 8'h0D;
            next_write_data = 8'h01;
        end
        5'd6: begin // Write 0xFF to 0x3004 (Enable output pads)
            next_write_msb  = 8'h30;
            next_write_lsb  = 8'h04;
            next_write_data = 8'hFF;
        end
        5'd7: begin // Write 0xFF to 0x3005 (Enable output pads)
            next_write_msb  = 8'h30;
            next_write_lsb  = 8'h05;
            next_write_data = 8'hFF;
        end
        5'd8: begin // Write 0xFF to 0x3006 (Enable output pads)
            next_write_msb  = 8'h30;
            next_write_lsb  = 8'h06;
            next_write_data = 8'hFF;
        end
        5'd9: begin // Write 0x00 to 0x3802 (TIMING_Y_ADDR_START H = 0)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h02;
            next_write_data = 8'h00;
        end
        5'd10: begin // Write 0x00 to 0x3803 (TIMING_Y_ADDR_START L = 0)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h03;
            next_write_data = 8'h00;
        end
        5'd11: begin // Write 0x03 to 0x3806 (TIMING_Y_ADDR_END H = 815)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h06;
            next_write_data = 8'h03;
        end
        5'd12: begin // Write 0x2F to 0x3807 (TIMING_Y_ADDR_END L)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h07;
            next_write_data = 8'h2F;
        end
        5'd13: begin // Write 0x02 to 0x3808 (TIMING_X_OUTPUT_SIZE H = 640)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h08;
            next_write_data = 8'h02;
        end
        5'd14: begin // Write 0x80 to 0x3809 (TIMING_X_OUTPUT_SIZE L = 640)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h09;
            next_write_data = 8'h80;
        end
        5'd15: begin // Write 0x01 to 0x380A (TIMING_Y_OUTPUT_SIZE H = 400)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h0A;
            next_write_data = 8'h01;
        end
        5'd16: begin // Write 0x90 to 0x380B (TIMING_Y_OUTPUT_SIZE L = 400)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h0B;
            next_write_data = 8'h90;
        end
        5'd17: begin // Write 0x02 to 0x380E (TIMING_VTS H = 520)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h0E;
            next_write_data = 8'h02;
        end
        5'd18: begin // Write 0x08 to 0x380F (TIMING_VTS L = 520)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h0F;
            next_write_data = 8'h08;
        end
        5'd19: begin // Write 0x31 to 0x3814 (TIMING_X_INC = 2x subsampling)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h14;
            next_write_data = 8'h31;
        end
        5'd20: begin // Write 0x22 to 0x3815 (TIMING_Y_INC = 2x subsampling)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h15;
            next_write_data = 8'h22;
        end
        5'd21: begin // Write 0x60 to 0x3820 (TIMING_FORMAT1 = subsampling)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h20;
            next_write_data = 8'h60;
        end
        5'd22: begin // Write 0x01 to 0x3821 (TIMING_FORMAT2 = subsampling)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h21;
            next_write_data = 8'h01;
        end
        5'd23: begin // Write 0x80 to 0x5E00 (Enable Test Pattern Bar)
            next_write_msb  = 8'h5E;
            next_write_lsb  = 8'h00;
            next_write_data = 8'h80;
        end
        5'd24: begin // Write 0x03 to 0x3503 (Enable Manual AEC/AGC)
            next_write_msb  = 8'h35;
            next_write_lsb  = 8'h03;
            next_write_data = 8'h03;
        end
        5'd25: begin // Write 0x19 to 0x3501 (Long Exposure MSB = 400 lines)
            next_write_msb  = 8'h35;
            next_write_lsb  = 8'h01;
            next_write_data = 8'h19;
        end
        5'd26: begin // Write 0x00 to 0x3502 (Long Exposure LSB)
            next_write_msb  = 8'h35;
            next_write_lsb  = 8'h02;
            next_write_data = 8'h00;
        end
        5'd27: begin // Write 0x02 to 0x3508 (Analog Gain MSB = 4x Gain)
            next_write_msb  = 8'h35;
            next_write_lsb  = 8'h08;
            next_write_data = 8'h02;
        end
        5'd28: begin // Write 0x00 to 0x3509 (Analog Gain LSB)
            next_write_msb  = 8'h35;
            next_write_lsb  = 8'h09;
            next_write_data = 8'h00;
        end
        5'd29: begin // Write 0x01 to 0x0100 (Streaming Enable)
            next_write_msb  = 8'h01;
            next_write_lsb  = 8'h00;
            next_write_data = 8'h01;
        end
        default: begin
            next_write_msb  = 8'h00;
            next_write_lsb  = 8'h00;
            next_write_data = 8'h00;
        end
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= STATE_IDLE;
        quad_cnt <= 2'd0;
        bit_cnt <= 4'd0;
        byte_cnt <= 3'd0;
        shift_reg <= 8'd0;
        read_data <= 8'd0;
        scl_out <= 1'b1;
        sda_out <= 1'b1;
        sda_oe <= 1'b1;
        i2c_error <= 1'b0;
        trans_idx <= 5'd0;
        stream_enabled <= 1'b0;
    end else if (tick) begin
        case (state)
            STATE_IDLE: begin
                scl_out <= 1'b1;
                sda_out <= 1'b1;
                sda_oe <= 1'b1;
                quad_cnt <= 2'd0;
                i2c_error <= 1'b0;
                if (start_i2c) begin
                    state <= STATE_START;
                end
            end
            
            STATE_START: begin
                quad_cnt <= quad_cnt + 2'd1;
                if (quad_cnt == 2'd0) begin
                    sda_out <= 1'b0; // SDA goes low
                    sda_oe <= 1'b1;
                end else if (quad_cnt == 2'd2) begin
                    scl_out <= 1'b0; // SCL goes low
                end else if (quad_cnt == 2'd3) begin
                    state <= STATE_WRITE;
                    bit_cnt <= 4'd0;
                    byte_cnt <= 3'd0;
                    shift_reg <= 8'hC0; // 0x60 7-bit addr << 1 | Write(0) = 0xC0
                end
            end
            
            STATE_WRITE: begin
                quad_cnt <= quad_cnt + 2'd1;
                if (quad_cnt == 2'd0) begin
                    scl_out <= 1'b0;
                    sda_out <= shift_reg[7];
                    sda_oe <= 1'b1;
                end else if (quad_cnt == 2'd2) begin
                    scl_out <= 1'b1;
                end else if (quad_cnt == 2'd3) begin
                    shift_reg <= {shift_reg[6:0], 1'b0};
                    if (bit_cnt == 4'd7) begin
                        state <= STATE_ACK;
                    end else begin
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                end
            end
            
            STATE_ACK: begin
                quad_cnt <= quad_cnt + 2'd1;
                if (quad_cnt == 2'd0) begin
                    scl_out <= 1'b0;
                    sda_oe <= 1'b0; // release SDA
                end else if (quad_cnt == 2'd2) begin
                    scl_out <= 1'b1;
                    if (sda_in == 1'b1) begin
                        i2c_error <= 1'b1; // NACK received
                    end
                end else if (quad_cnt == 2'd3) begin
                    scl_out <= 1'b0;
                    sda_oe <= 1'b1;
                    if (i2c_error) begin
                        state <= STATE_ERROR;
                    end else begin
                        if (trans_idx == 5'd0) begin
                            case (byte_cnt)
                                3'd0: begin
                                    byte_cnt <= 3'd1;
                                    shift_reg <= 8'h30; // Chip ID Register MSB (0x30)
                                    bit_cnt <= 4'd0;
                                    state <= STATE_WRITE;
                                end
                                3'd1: begin
                                    byte_cnt <= 3'd2;
                                    shift_reg <= 8'h0A; // Chip ID Register LSB (0x0A)
                                    bit_cnt <= 4'd0;
                                    state <= STATE_WRITE;
                                end
                                3'd2: begin
                                    byte_cnt <= 3'd3;
                                    state <= STATE_REP_START;
                                end
                                3'd3: begin
                                    byte_cnt <= 3'd4;
                                    bit_cnt <= 4'd0;
                                    state <= STATE_READ;
                                end
                                default: state <= STATE_ERROR;
                            endcase
                        end else begin
                            case (byte_cnt)
                                3'd0: begin
                                    byte_cnt <= 3'd1;
                                    shift_reg <= next_write_msb;
                                    bit_cnt <= 4'd0;
                                    state <= STATE_WRITE;
                                end
                                3'd1: begin
                                    byte_cnt <= 3'd2;
                                    shift_reg <= next_write_lsb;
                                    bit_cnt <= 4'd0;
                                    state <= STATE_WRITE;
                                end
                                3'd2: begin
                                    byte_cnt <= 3'd3;
                                    shift_reg <= next_write_data;
                                    bit_cnt <= 4'd0;
                                    state <= STATE_WRITE;
                                end
                                3'd3: begin
                                    byte_cnt <= 3'd4;
                                    state <= STATE_STOP;  // Finished writing data, go to STOP
                                end
                                default: state <= STATE_ERROR;
                            endcase
                        end
                    end
                end
            end
            
            STATE_REP_START: begin
                quad_cnt <= quad_cnt + 2'd1;
                if (quad_cnt == 2'd0) begin
                    sda_out <= 1'b1;
                    sda_oe <= 1'b1;
                    scl_out <= 1'b0;
                end else if (quad_cnt == 2'd1) begin
                    scl_out <= 1'b1;
                end else if (quad_cnt == 2'd2) begin
                    sda_out <= 1'b0; // SDA goes low while SCL high
                end else if (quad_cnt == 2'd3) begin
                    scl_out <= 1'b0;
                    shift_reg <= 8'hC1; // 0x60 7-bit addr << 1 | Read(1) = 0xC1
                    bit_cnt <= 4'd0;
                    state <= STATE_WRITE;
                end
            end
            
            STATE_READ: begin
                quad_cnt <= quad_cnt + 2'd1;
                if (quad_cnt == 2'd0) begin
                    scl_out <= 1'b0;
                    sda_oe <= 1'b0; // release SDA
                end else if (quad_cnt == 2'd2) begin
                    scl_out <= 1'b1;
                    read_data <= {read_data[6:0], sda_in};
                end else if (quad_cnt == 2'd3) begin
                    if (bit_cnt == 4'd7) begin
                        state <= STATE_SEND_NACK;
                    end else begin
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                end
            end
            
            STATE_SEND_NACK: begin
                quad_cnt <= quad_cnt + 2'd1;
                if (quad_cnt == 2'd0) begin
                    scl_out <= 1'b0;
                    sda_out <= 1'b1; // NACK (keep SDA high)
                    sda_oe <= 1'b1;
                end else if (quad_cnt == 2'd2) begin
                    scl_out <= 1'b1;
                end else if (quad_cnt == 2'd3) begin
                    scl_out <= 1'b0;
                    state <= STATE_STOP;
                end
            end
            
            STATE_STOP: begin
                quad_cnt <= quad_cnt + 2'd1;
                if (quad_cnt == 2'd0) begin
                    sda_out <= 1'b0;
                    sda_oe <= 1'b1;
                    scl_out <= 1'b0;
                end else if (quad_cnt == 2'd1) begin
                    scl_out <= 1'b1;
                end else if (quad_cnt == 2'd2) begin
                    sda_out <= 1'b1; // SDA goes high while SCL high
                end else if (quad_cnt == 2'd3) begin
                    state <= STATE_DONE;
                end
            end
            
            STATE_DONE: begin
                scl_out <= 1'b1;
                sda_out <= 1'b1;
                sda_oe <= 1'b1;
                if (trans_idx < 5'd29) begin
                    if (trans_idx == 5'd0) begin
                        if (read_data == 8'h92) begin
                            trans_idx <= trans_idx + 5'd1;
                            state <= STATE_IDLE; // Trigger next transaction
                        end
                    end else begin
                        trans_idx <= trans_idx + 5'd1;
                        state <= STATE_IDLE; // Trigger next transaction
                    end
                end else if (trans_idx == 5'd29) begin
                    stream_enabled <= 1'b1;
                end
            end
            
            STATE_ERROR: begin
                scl_out <= 1'b1;
                sda_out <= 1'b1;
                sda_oe <= 1'b1;
            end
            
            default: state <= STATE_IDLE;
        endcase
    end
end

// 4. Heartbeat counter for LED 0 blinking (at ~1Hz)
reg [24:0] heartbeat_cnt;
reg heartbeat;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        heartbeat_cnt <= 25'd0;
        heartbeat <= 1'b0;
    end else begin
        if (heartbeat_cnt == 25'd13_500_000) begin
            heartbeat_cnt <= 25'd0;
            heartbeat <= ~heartbeat;
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 25'd1;
        end
    end
end

// 5. DVP Parallel Capture Logic (runs on cam_pclk clock domain)
// Direct single-register capture of raw DVP signals (no filtering).

reg vsync_r;
reg href_r;
reg [7:0] data_r;

always @(negedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        vsync_r <= 1'b0;
        href_r  <= 1'b0;
        data_r  <= 8'd0;
    end else begin
        vsync_r <= cam_vsync;
        href_r  <= cam_href;
        data_r  <= cam_data;
    end
end

reg [7:0] sample_p0;
reg [7:0] sample_p1;
reg [7:0] sample_p2;
reg [7:0] sample_p3;
reg [7:0] sample_p4;
reg [7:0] sample_p5;

reg [11:0] pixel_cnt;
reg [9:0] line_cnt;
reg synced;

reg href_prev;
always @(negedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        href_prev <= 1'b0;
    end else begin
        href_prev <= href_r;
    end
end

wire href_rose = href_r && !href_prev;


// Level-sensitive reset using vsync_r is extremely robust against clock glitches
// during vertical blanking. Holding counters at 0 during VSYNC high ensures alignment.
always @(negedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pixel_cnt <= 12'd0;
        line_cnt <= 10'd0;
        synced <= 1'b0;
    end else if (vsync_r) begin
        pixel_cnt <= 12'd0;
        line_cnt <= 10'd0;
        synced <= 1'b0;
    end else begin
        if (!synced) begin
            // Sync to the very first HREF rising edge of the frame
            if (href_rose) begin
                synced <= 1'b1;
                pixel_cnt <= 12'd0;
                line_cnt <= 10'd0;
            end
        end else begin
            // Standard and raw DVP capture logic
            if (href_r) begin
                pixel_cnt <= pixel_cnt + 12'd1;
            end else begin
                pixel_cnt <= 12'd0;
                if (href_prev) begin // HREF falling edge (line end)
                    line_cnt <= line_cnt + 10'd1;
                end
            end
        end
    end
end

// Sample 6 pixels on row 400 (evenly spaced across 1280 pixels)
always @(negedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        sample_p0 <= 8'd0;
        sample_p1 <= 8'd0;
        sample_p2 <= 8'd0;
        sample_p3 <= 8'd0;
        sample_p4 <= 8'd0;
        sample_p5 <= 8'd0;
    end else if (line_cnt == 10'd400 && href_r) begin
        case (pixel_cnt)
            12'd200:  sample_p0 <= data_r;
            12'd400:  sample_p1 <= data_r;
            12'd600:  sample_p2 <= data_r;
            12'd800:  sample_p3 <= data_r;
            12'd1000: sample_p4 <= data_r;
            12'd1200: sample_p5 <= data_r;
        endcase
    end
end

// 6. Clock Domain Crossing (CDC) & PWM LED Video Mapping (Active Low)
// Synchronize vsync_r to sys_clk domain to safely sample the frame-static values
reg vsync_sys_0;
reg vsync_sys_1;
reg vsync_sys_2;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        vsync_sys_0 <= 1'b0;
        vsync_sys_1 <= 1'b0;
        vsync_sys_2 <= 1'b0;
    end else begin
        vsync_sys_0 <= vsync_r;
        vsync_sys_1 <= vsync_sys_0;
        vsync_sys_2 <= vsync_sys_1;
    end
end

wire vsync_sys_rose = vsync_sys_1 && !vsync_sys_2;

// Latch values in sys_clk domain at the end of each frame (VSYNC rising edge)
reg [7:0] led_val0;
reg [7:0] led_val1;
reg [7:0] led_val2;
reg [7:0] led_val3;
reg [7:0] led_val4;
reg [7:0] led_val5;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        led_val0 <= 8'd0;
        led_val1 <= 8'd0;
        led_val2 <= 8'd0;
        led_val3 <= 8'd0;
        led_val4 <= 8'd0;
        led_val5 <= 8'd0;
    end else if (vsync_sys_rose) begin
        led_val0 <= sample_p0;
        led_val1 <= sample_p1;
        led_val2 <= sample_p2;
        led_val3 <= sample_p3;
        led_val4 <= sample_p4;
        led_val5 <= sample_p5;
    end
end

// 8-bit PWM generator running at sys_clk (27MHz)
reg [7:0] pwm_cnt;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pwm_cnt <= 8'd0;
    end else begin
        pwm_cnt <= pwm_cnt + 8'd1;
    end
end

// Diagnostic registers to measure camera output parameters in hardware
reg [9:0] max_line_cnt;
reg [11:0] max_pixel_cnt;

// Use temporary register to track maximum pixel_cnt during a frame
reg [11:0] pixel_cnt_max_temp;
always @(negedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pixel_cnt_max_temp <= 12'd0;
    end else if (vsync_r) begin
        pixel_cnt_max_temp <= 12'd0;
    end else if (href_r && (pixel_cnt > pixel_cnt_max_temp)) begin
        pixel_cnt_max_temp <= pixel_cnt;
    end
end

// Latch frame-static values at the end of each frame (VSYNC rising edge)
always @(posedge vsync_r or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        max_line_cnt  <= 10'd0;
        max_pixel_cnt <= 12'd0;
    end else begin
        max_line_cnt  <= line_cnt;
        max_pixel_cnt <= pixel_cnt_max_temp;
    end
end

// Button S2 debouncer and display mode toggler
reg button_prev;
reg display_mode; // 0: show max_line_cnt, 1: show max_pixel_cnt
reg [19:0] btn_debounce_cnt;
reg btn_state;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        btn_debounce_cnt <= 20'd0;
        btn_state <= 1'b1;
    end else begin
        btn_debounce_cnt <= btn_debounce_cnt + 20'd1;
        if (btn_debounce_cnt == 20'd0) begin
            btn_state <= button;
        end
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        button_prev  <= 1'b1;
        display_mode <= 1'b0;
    end else begin
        button_prev <= btn_state;
        if (button_prev && !btn_state) begin // S2 falling edge (pressed)
            display_mode <= ~display_mode;
        end
    end
end

// Compare PWM counter with registered pixel intensity to set LED brightness (active-low)
wire led_out0 = (pwm_cnt < led_val0) ? 1'b0 : 1'b1;
wire led_out1 = (pwm_cnt < led_val1) ? 1'b0 : 1'b1;
wire led_out2 = (pwm_cnt < led_val2) ? 1'b0 : 1'b1;
wire led_out3 = (pwm_cnt < led_val3) ? 1'b0 : 1'b1;
wire led_out4 = (pwm_cnt < led_val4) ? 1'b0 : 1'b1;
wire led_out5 = (pwm_cnt < led_val5) ? 1'b0 : 1'b1;

// LEDs directly show camera brightness.
assign led[0] = led_out0;
assign led[1] = led_out1;
assign led[2] = led_out2;
assign led[3] = led_out3;
assign led[4] = led_out4;
assign led[5] = led_out5;

// ============================================================================
// HDMI Display & Camera PSRAM frame buffer
// ============================================================================

wire test_mode = display_mode;

// 1. PSRAM Clock Generation (166.5 MHz memory clock from 27 MHz sys_clk)
wire clk_psram_166;
wire pll_lock_psram;

rPLL rpll_psram (
    .CLKOUT(clk_psram_166),
    .LOCK(pll_lock_psram),
    .CLKOUTP(),
    .CLKOUTD(),
    .CLKOUTD3(),
    .RESET(1'b0),
    .RESET_P(1'b0),
    .CLKIN(sys_clk),
    .CLKFB(1'b0),
    .FBDSEL(6'b0),
    .IDSEL(6'b0),
    .ODSEL(6'b0),
    .PSDA(4'b0),
    .DUTYDA(4'b0),
    .FDLY(4'b0)
);
defparam rpll_psram.FCLKIN = "27";
defparam rpll_psram.DYN_IDIV_SEL = "false";
defparam rpll_psram.IDIV_SEL = 5;
defparam rpll_psram.DYN_FBDIV_SEL = "false";
defparam rpll_psram.FBDIV_SEL = 36;
defparam rpll_psram.DYN_ODIV_SEL = "false";
defparam rpll_psram.ODIV_SEL = 4;
defparam rpll_psram.DEVICE = "GW1NR-9C";

// 2. PSRAM Memory Interface HS IP Instantiation
wire [31:0] psram_wr_data;
wire [31:0] psram_rd_data;
wire psram_rd_data_valid;
wire [20:0] psram_addr;
wire psram_cmd;
wire psram_cmd_en;
wire psram_init_calib;
wire clk_out; // 83.25 MHz output clock from IP

PSRAM_Memory_Interface_HS_Top psram_inst (
    .clk(sys_clk), // reference clock (27MHz)
    .memory_clk(clk_psram_166),
    .pll_lock(pll_lock_psram),
    .rst_n(sys_rst_n),
    
    // Physical PSRAM pins connected to top level ports
    .O_psram_ck(O_psram_ck),
    .O_psram_ck_n(O_psram_ck_n),
    .IO_psram_dq(IO_psram_dq),
    .IO_psram_rwds(IO_psram_rwds),
    .O_psram_cs_n(O_psram_cs_n),
    .O_psram_reset_n(O_psram_reset_n),
    
    // User Interface
    .wr_data(psram_wr_data),
    .rd_data(psram_rd_data),
    .rd_data_valid(psram_rd_data_valid),
    .addr(psram_addr),
    .cmd(psram_cmd),
    .cmd_en(psram_cmd_en),
    .init_calib(psram_init_calib),
    .clk_out(clk_out),
    .data_mask(4'b0000) // no byte masking (write all 4 bytes)
);

// 3. Triple-Buffering Frame Control Logic
reg [1:0] wr_frame = 2'd0;
reg [1:0] rd_frame = 2'd0;
reg [1:0] last_wr_frame = 2'd0;

// Synchronize rd_frame to cam_pclk domain
reg [1:0] rd_frame_sync_0, rd_frame_sync_1;
always @(negedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        rd_frame_sync_0 <= 2'd0;
        rd_frame_sync_1 <= 2'd0;
    end else begin
        rd_frame_sync_0 <= rd_frame;
        rd_frame_sync_1 <= rd_frame_sync_0;
    end
end

// Advance wr_frame on camera VSYNC rising edge
always @(posedge vsync_r or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        wr_frame <= 2'd0;
        last_wr_frame <= 2'd0;
    end else begin
        last_wr_frame <= wr_frame;
        if (rd_frame_sync_1 == 2'd0)
            wr_frame <= (wr_frame == 2'd1) ? 2'd2 : 2'd1;
        else if (rd_frame_sync_1 == 2'd1)
            wr_frame <= (wr_frame == 2'd0) ? 2'd2 : 2'd0;
        else
            wr_frame <= (wr_frame == 2'd0) ? 2'd1 : 2'd0;
    end
end

// Synchronize last_wr_frame to clk_pixel domain
reg [1:0] last_wr_frame_sync_0, last_wr_frame_sync_1;
always @(posedge clk_pixel or negedge sys_resetn) begin
    if (!sys_resetn) begin
        last_wr_frame_sync_0 <= 2'd0;
        last_wr_frame_sync_1 <= 2'd0;
    end else begin
        last_wr_frame_sync_0 <= last_wr_frame;
        last_wr_frame_sync_1 <= last_wr_frame_sync_0;
    end
end

// Update rd_frame at the start of an HDMI frame
always @(posedge clk_pixel or negedge sys_resetn) begin
    if (!sys_resetn) begin
        rd_frame <= 2'd0;
    end else begin
        if (vcursor_out == 10'd0 && hcursor_out == 10'd0) begin
            rd_frame <= last_wr_frame_sync_1;
        end
    end
end

// Synchronize wr_frame and rd_frame to clk_out domain for PSRAM addressing
reg [1:0] wr_frame_clkout_0, wr_frame_clkout_1;
reg [1:0] rd_frame_clkout_0, rd_frame_clkout_1;
always @(posedge clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        wr_frame_clkout_0 <= 2'd0;
        wr_frame_clkout_1 <= 2'd0;
        rd_frame_clkout_0 <= 2'd0;
        rd_frame_clkout_1 <= 2'd0;
    end else begin
        wr_frame_clkout_0 <= wr_frame;
        wr_frame_clkout_1 <= wr_frame_clkout_0;
        rd_frame_clkout_0 <= rd_frame;
        rd_frame_clkout_1 <= rd_frame_clkout_0;
    end
end

localparam FRAME_OFFSET = 21'd256000; // 640x400 bytes per frame

wire [20:0] wr_base_addr = (wr_frame_clkout_1 == 2'd0) ? 21'd0 :
                           (wr_frame_clkout_1 == 2'd1) ? FRAME_OFFSET :
                                                         (FRAME_OFFSET << 1);

wire [20:0] rd_base_addr = (rd_frame_clkout_1 == 2'd0) ? 21'd0 :
                           (rd_frame_clkout_1 == 2'd1) ? FRAME_OFFSET :
                                                         (FRAME_OFFSET << 1);

// 4. Mixed-width Line Buffers (using 4 parallel banks to map cleanly to Block RAM)
// 640 bytes per line / 4 banks = 160 entries per bank.
// Write buffers are ping-pong (A/B) so consecutive camera lines don't race the PSRAM flush.
reg [7:0] bank_wr0_a [0:159];
reg [7:0] bank_wr1_a [0:159];
reg [7:0] bank_wr2_a [0:159];
reg [7:0] bank_wr3_a [0:159];

reg [7:0] bank_wr0_b [0:159];
reg [7:0] bank_wr1_b [0:159];
reg [7:0] bank_wr2_b [0:159];
reg [7:0] bank_wr3_b [0:159];

reg [7:0] bank_rd0_a [0:159];
reg [7:0] bank_rd1_a [0:159];
reg [7:0] bank_rd2_a [0:159];
reg [7:0] bank_rd3_a [0:159];

reg [7:0] bank_rd0_b [0:159];
reg [7:0] bank_rd1_b [0:159];
reg [7:0] bank_rd2_b [0:159];
reg [7:0] bank_rd3_b [0:159];

// Write Line Logic (camera clock domain: cam_pclk)
// Capture full 640x400 (no decimation). OV9281 is monochrome RAW8: 1 byte per pixel.
wire wr_en = synced && href_r && (line_cnt < 400) && (pixel_cnt < 640);

wire [9:0] cam_pixel_idx = pixel_cnt[9:0];    // 0..639 pixel index
wire [7:0] cam_bank_addr = cam_pixel_idx[9:2];
wire [1:0] cam_bank_sel  = cam_pixel_idx[1:0];

// Ping-pong select: even camera lines -> A, odd -> B
wire cam_wr_sel = line_cnt[0];

always @(posedge cam_pclk) begin
    if (wr_en) begin
        if (cam_wr_sel) begin
            case (cam_bank_sel)
                2'd0: bank_wr0_b[cam_bank_addr] <= data_r;
                2'd1: bank_wr1_b[cam_bank_addr] <= data_r;
                2'd2: bank_wr2_b[cam_bank_addr] <= data_r;
                2'd3: bank_wr3_b[cam_bank_addr] <= data_r;
            endcase
        end else begin
            case (cam_bank_sel)
                2'd0: bank_wr0_a[cam_bank_addr] <= data_r;
                2'd1: bank_wr1_a[cam_bank_addr] <= data_r;
                2'd2: bank_wr2_a[cam_bank_addr] <= data_r;
                2'd3: bank_wr3_a[cam_bank_addr] <= data_r;
            endcase
        end
    end
end

// Write CDC Handshake
reg line_wr_req = 1'b0;
reg [8:0] wr_line_num = 9'd0;
reg wr_buf_sel = 1'b0;
reg line_wr_ack_sync_0, line_wr_ack_sync_1;

always @(negedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        line_wr_req <= 1'b0;
        wr_line_num <= 9'd0;
        wr_buf_sel <= 1'b0;
        line_wr_ack_sync_0 <= 1'b0;
        line_wr_ack_sync_1 <= 1'b0;
    end else begin
        line_wr_ack_sync_0 <= line_wr_ack;
        line_wr_ack_sync_1 <= line_wr_ack_sync_0;

        if (!line_wr_req) begin
            // Trigger write at the end of every active camera row
            if (href_prev && !href_r && (line_cnt < 400)) begin
                line_wr_req <= 1'b1;
                wr_line_num <= line_cnt[8:0];
                wr_buf_sel  <= line_cnt[0]; // which ping-pong buffer holds this line
            end
        end else begin
            if (line_wr_ack_sync_1) begin
                line_wr_req <= 1'b0;
            end
        end
    end
end

// 5. HDMI Display Timing and CDC Read Handshake (clk_pixel domain)
wire [9:0] hcursor_out;
wire [9:0] vcursor_out;

reg [9:0] hcursor_r;
always @(posedge clk_pixel or negedge sys_resetn) begin
    if (!sys_resetn)
        hcursor_r <= 10'd0;
    else
        hcursor_r <= hcursor_out;
end

wire line_done = (hcursor_r == 10'd639 && hcursor_out == 10'd0);

reg line_rd_req = 1'b0;
reg [8:0] rd_line_num = 9'd0;
reg line_rd_ack_sync_0, line_rd_ack_sync_1;

always @(posedge clk_pixel or negedge sys_resetn) begin
    if (!sys_resetn) begin
        line_rd_req <= 1'b0;
        rd_line_num <= 9'd0;
        line_rd_ack_sync_0 <= 1'b0;
        line_rd_ack_sync_1 <= 1'b0;
    end else begin
        line_rd_ack_sync_0 <= line_rd_ack;
        line_rd_ack_sync_1 <= line_rd_ack_sync_0;

        if (!line_rd_req) begin
            // Prefetch each camera line one HDMI line ahead (1:1 mapping).
            // While scanning line at vcursor, fetch camera line (vcursor+1-40)
            // displayed next. Active camera lines 0..399 map to HDMI lines 40..439.
            if (line_done && (vcursor_out >= 10'd39) && (vcursor_out <= 10'd438)) begin
                line_rd_req <= 1'b1;
                rd_line_num <= (vcursor_out + 10'd1 - 10'd40);
            end
        end else begin
            if (line_rd_ack_sync_1) begin
                line_rd_req <= 1'b0;
            end
        end
    end
end

// Line buffer read index calculation for HDMI display (1:1, no upscale)
wire [9:0] next_hcursor = (hcursor_out == 10'd639) ? 10'd0 : (hcursor_out + 10'd1);
wire [9:0] next_vcursor_for_rd = (hcursor_out == 10'd639) ? ((vcursor_out == 10'd479) ? 10'd0 : (vcursor_out + 10'd1)) : vcursor_out;
wire [9:0] rd_buf_addr = next_hcursor;                       // full 640-pixel byte index
wire [9:0] rd_line_idx_for_buf = next_vcursor_for_rd - 10'd40; // camera line currently displayed
wire rd_buf_sel = rd_line_idx_for_buf[0];
reg [7:0] rd_data; // 8-bit read data fed to svo_hdmi_inst

always @(posedge clk_pixel) begin
    if (rd_buf_sel) begin
        case (rd_buf_addr[1:0])
            2'd0: rd_data <= bank_rd0_b[rd_buf_addr[9:2]];
            2'd1: rd_data <= bank_rd1_b[rd_buf_addr[9:2]];
            2'd2: rd_data <= bank_rd2_b[rd_buf_addr[9:2]];
            2'd3: rd_data <= bank_rd3_b[rd_buf_addr[9:2]];
        endcase
    end else begin
        case (rd_buf_addr[1:0])
            2'd0: rd_data <= bank_rd0_a[rd_buf_addr[9:2]];
            2'd1: rd_data <= bank_rd1_a[rd_buf_addr[9:2]];
            2'd2: rd_data <= bank_rd2_a[rd_buf_addr[9:2]];
            2'd3: rd_data <= bank_rd3_a[rd_buf_addr[9:2]];
        endcase
    end
end

// 6. PSRAM Arbiter State Machine (clk_out domain: 83.25 MHz)
reg line_wr_req_sync_0, line_wr_req_sync_1;
reg line_rd_req_sync_0, line_rd_req_sync_1;
reg [8:0] wr_line_num_clkout;
reg [8:0] rd_line_num_clkout;
reg [8:0] rd_line_num_latch;
always @(posedge clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        line_wr_req_sync_0 <= 1'b0;
        line_wr_req_sync_1 <= 1'b0;
        line_rd_req_sync_0 <= 1'b0;
        line_rd_req_sync_1 <= 1'b0;
        wr_line_num_clkout <= 9'd0;
        rd_line_num_clkout <= 9'd0;
    end else begin
        line_wr_req_sync_0 <= line_wr_req;
        line_wr_req_sync_1 <= line_wr_req_sync_0;
        line_rd_req_sync_0 <= line_rd_req;
        line_rd_req_sync_1 <= line_rd_req_sync_0;
        wr_line_num_clkout <= wr_line_num;
        rd_line_num_clkout <= rd_line_num;
    end
end

localparam S_IDLE       = 3'd0;
localparam S_READ_CMD   = 3'd1;
localparam S_READ_WAIT  = 3'd2;
localparam S_READ_DONE  = 3'd3;
localparam S_WRITE_CMD  = 3'd4;
localparam S_WRITE_DATA = 3'd5;
localparam S_WRITE_WAIT = 3'd6;
localparam S_WRITE_DONE = 3'd7;

reg [2:0] pstate = S_IDLE;
reg [4:0] burst_num = 5'd0;     // 0..19 (20 bursts of 8 words = 160 words = 640 bytes)
reg [4:0] cmd_timer = 5'd0;
reg [2:0] write_beat_cnt = 3'd0;
reg [7:0] wr_ptr = 8'd0;        // 0..159 read-data word pointer

reg line_wr_ack = 1'b0;
reg line_rd_ack = 1'b0;

reg psram_cmd_en_r = 1'b0;
reg psram_cmd_r = 1'b0;
reg [20:0] psram_addr_r = 21'd0;

assign psram_cmd_en = psram_cmd_en_r;
assign psram_cmd    = psram_cmd_r;
assign psram_addr   = psram_addr_r;

// Combinational pipeline address for reading from bank_wr (word index 0..159)
reg [7:0] wr_bank_rd_addr;
always @(*) begin
    if (pstate == S_IDLE) begin
        wr_bank_rd_addr = 8'd0; // pre-fetch first word of burst 0
    end else if (pstate == S_WRITE_CMD) begin
        wr_bank_rd_addr = {burst_num, 3'd0}; // first captured beat must be word0 (addr leads data by 1 clk_out)
    end else if (pstate == S_WRITE_DATA) begin
        if (write_beat_cnt < 3'd7)
            wr_bank_rd_addr = {burst_num, 3'd0} + {5'd0, write_beat_cnt} + 8'd1;
        else // write_beat_cnt == 7
            wr_bank_rd_addr = {(burst_num + 5'd1), 3'd0}; // pre-fetch first word of next burst
    end else begin
        wr_bank_rd_addr = 8'd0;
    end
end

// Register the bank_wr read data in clk_out domain.
// wr_line_num_clkout[0] selects which ping-pong write buffer holds the line being flushed.
// Using wr_line_num_clkout[0] avoids a separate CDC path for wr_buf_sel that could skew.
reg [31:0] psram_wr_data_r = 32'd0;
assign psram_wr_data = psram_wr_data_r;

always @(posedge clk_out) begin
    if (wr_line_num_clkout[0]) begin
        psram_wr_data_r <= {
            bank_wr3_b[wr_bank_rd_addr],
            bank_wr2_b[wr_bank_rd_addr],
            bank_wr1_b[wr_bank_rd_addr],
            bank_wr0_b[wr_bank_rd_addr]
        };
    end else begin
        psram_wr_data_r <= {
            bank_wr3_a[wr_bank_rd_addr],
            bank_wr2_a[wr_bank_rd_addr],
            bank_wr1_a[wr_bank_rd_addr],
            bank_wr0_a[wr_bank_rd_addr]
        };
    end
end

always @(posedge clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pstate <= S_IDLE;
        burst_num <= 5'd0;
        cmd_timer <= 5'd0;
        write_beat_cnt <= 3'd0;
        rd_line_num_latch <= 9'd0;
        line_wr_ack <= 1'b0;
        line_rd_ack <= 1'b0;
        psram_cmd_en_r <= 1'b0;
        psram_cmd_r <= 1'b0;
        psram_addr_r <= 21'd0;
    end else begin
        case (pstate)
            S_IDLE: begin
                burst_num <= 5'd0;
                cmd_timer <= 5'd0;
                write_beat_cnt <= 3'd0;
                psram_cmd_en_r <= 1'b0;
                
                if (psram_init_calib) begin
                    // Prioritize read requests to avoid screen flickering
                    if (line_rd_req_sync_1 && !line_rd_ack) begin
                        pstate <= S_READ_CMD;
                    end else if (line_wr_req_sync_1 && !line_wr_ack) begin
                        pstate <= S_WRITE_CMD;
                    end
                end
                
                if (!line_wr_req_sync_1) begin
                    line_wr_ack <= 1'b0;
                end
                if (!line_rd_req_sync_1) begin
                    line_rd_ack <= 1'b0;
                end
            end
            
            S_READ_CMD: begin
                psram_cmd_en_r <= 1'b1;
                psram_cmd_r <= 1'b0; // Read command
                if (burst_num == 5'd0)
                    rd_line_num_latch <= rd_line_num_clkout;
                psram_addr_r <= rd_base_addr + {rd_line_num_clkout, 9'b0} + {rd_line_num_clkout, 7'b0} + {burst_num, 5'b0};
                cmd_timer <= 5'd0;
                pstate <= S_READ_WAIT;
            end

            S_READ_WAIT: begin
                psram_cmd_en_r <= 1'b0;
                cmd_timer <= cmd_timer + 5'd1;

                if (cmd_timer >= 5'd31) begin
                    if (burst_num < 5'd19) begin
                        burst_num <= burst_num + 5'd1;
                        pstate <= S_READ_CMD;
                    end else begin
                        pstate <= S_READ_DONE;
                    end
                end
            end

            S_READ_DONE: begin
                if (wr_ptr == 8'd160) begin
                    line_rd_ack <= 1'b1;
                    if (!line_rd_req_sync_1) begin
                        line_rd_ack <= 1'b0;
                        pstate <= S_IDLE;
                    end
                end
            end
            
            S_WRITE_CMD: begin
                psram_cmd_en_r <= 1'b1;
                psram_cmd_r <= 1'b1; // Write command
                psram_addr_r <= wr_base_addr + {wr_line_num_clkout, 9'b0} + {wr_line_num_clkout, 7'b0} + {burst_num, 5'b0};
                cmd_timer <= 5'd0;
                write_beat_cnt <= 3'd0;
                pstate <= S_WRITE_DATA;
            end
            
            S_WRITE_DATA: begin
                psram_cmd_en_r <= 1'b0;
                cmd_timer <= cmd_timer + 5'd1;
                write_beat_cnt <= write_beat_cnt + 3'd1;
                
                if (write_beat_cnt == 3'd7) begin
                    pstate <= S_WRITE_WAIT;
                end
            end
            
            S_WRITE_WAIT: begin
                cmd_timer <= cmd_timer + 5'd1;

                if (cmd_timer >= 5'd31) begin
                    if (burst_num < 5'd19) begin
                        burst_num <= burst_num + 5'd1;
                        pstate <= S_WRITE_CMD;
                    end else begin
                        pstate <= S_WRITE_DONE;
                    end
                end
            end
            
            S_WRITE_DONE: begin
                line_wr_ack <= 1'b1;
                if (!line_wr_req_sync_1) begin
                    line_wr_ack <= 1'b0;
                    pstate <= S_IDLE;
                end
            end
            
            default: pstate <= S_IDLE;
        endcase
    end
end

// Write to bank_rd in clk_out domain when read data is valid
always @(posedge clk_out) begin
    if (psram_rd_data_valid) begin
        if (rd_line_num_latch[0]) begin
            bank_rd0_b[wr_ptr] <= psram_rd_data[7:0];
            bank_rd1_b[wr_ptr] <= psram_rd_data[15:8];
            bank_rd2_b[wr_ptr] <= psram_rd_data[23:16];
            bank_rd3_b[wr_ptr] <= psram_rd_data[31:24];
        end else begin
            bank_rd0_a[wr_ptr] <= psram_rd_data[7:0];
            bank_rd1_a[wr_ptr] <= psram_rd_data[15:8];
            bank_rd2_a[wr_ptr] <= psram_rd_data[23:16];
            bank_rd3_a[wr_ptr] <= psram_rd_data[31:24];
        end
    end
end

always @(posedge clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        wr_ptr <= 8'd0;
    end else begin
        if (psram_rd_data_valid) begin
            wr_ptr <= wr_ptr + 8'd1;
        end
        if (pstate == S_IDLE) begin
            wr_ptr <= 8'd0;
        end
    end
end

// 7. HDMI Clock Generation
wire clk_p5;    // 5x pixel clock
wire clk_pixel; // 1x pixel clock
wire pll_lock;
wire sys_resetn;

Gowin_rPLL u_pll (
    .clkin(sys_clk),
    .clkout(clk_p5),
    .lock(pll_lock)
);

Gowin_CLKDIV u_div_5 (
    .clkout(clk_pixel),
    .hclkin(clk_p5),
    .resetn(pll_lock)
);

Reset_Sync u_Reset_Sync (
    .resetn(sys_resetn),
    .ext_reset(sys_rst_n & pll_lock),
    .clk(clk_pixel)
);

// Instantiate HDMI Controller
wire [15:0] rd_addr; // unused dummy wire to satisfy svo_hdmi interface
svo_hdmi svo_hdmi_inst (
    .clk(clk_pixel),
    .resetn(sys_resetn),
    .test_mode(test_mode),
    .max_line_cnt(max_line_cnt),
    .max_pixel_cnt(max_pixel_cnt),

    // video clocks
    .clk_pixel(clk_pixel),
    .clk_5x_pixel(clk_p5),
    .locked(pll_lock),

    // RAM read interface
    .rd_addr(rd_addr),
    .rd_data(rd_data),

    // output signals
    .tmds_clk_n(tmds_clk_n),
    .tmds_clk_p(tmds_clk_p),
    .tmds_d_n(tmds_d_n),
    .tmds_d_p(tmds_d_p),

    // Cursor outputs
    .hcursor_out(hcursor_out),
    .vcursor_out(vcursor_out)
);

endmodule

// Helper Module for Reset Synchronization
module Reset_Sync (
    input clk,
    input ext_reset,
    output resetn
);
    reg [3:0] reset_cnt = 0;
    always @(posedge clk or negedge ext_reset) begin
        if (~ext_reset)
            reset_cnt <= 4'b0;
        else
            reset_cnt <= reset_cnt + !resetn;
    end
    assign resetn = &reset_cnt;
endmodule