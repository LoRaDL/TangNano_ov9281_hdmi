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
    output [2:0] tmds_d_p
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
        5'd19: begin // Write 0x31 to 0x3814 (TIMING_X_INC = subsampling)
            next_write_msb  = 8'h38;
            next_write_lsb  = 8'h14;
            next_write_data = 8'h31;
        end
        5'd20: begin // Write 0x22 to 0x3815 (TIMING_Y_INC = subsampling)
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
        5'd23: begin // Write 0x00 to 0x5E00 (Disable Test Pattern Bar)
            next_write_msb  = 8'h5E;
            next_write_lsb  = 8'h00;
            next_write_data = 8'h00;
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
// HDMI Display & Camera downsampler
// ============================================================================

// Dual-port Block RAM (infers SDP BRAM in Gowin)
// 320x200 = 64,000 pixels, 4 bits per pixel
reg [3:0] frame_ram [0:63999];

wire test_mode = display_mode;



// Write side (camera clock domain: cam_pclk)
// Input resolution from camera is 640x400.
// Downsample by 2x in both directions to get a 320x200 image stored in BRAM.
// Row address = line_cnt[9:1] * 320 = (line_cnt[9:1]<<8) + (line_cnt[9:1]<<6)  [shift-add, no multiplier]
wire wr_en = synced && href_r && (pixel_cnt[0] == 1'b0) && (line_cnt[0] == 1'b0)
             && (line_cnt < 400) && (pixel_cnt < 640);
wire [15:0] wr_addr = {line_cnt[9:1], 8'b0} + {line_cnt[9:1], 6'b0} + pixel_cnt[11:1];
wire [3:0]  wr_data = data_r[7:4];

always @(posedge cam_pclk) begin
    if (wr_en)
        frame_ram[wr_addr] <= wr_data;
end

// Read side (HDMI clock domain: clk_pixel)
wire [15:0] rd_addr;
reg [3:0] rd_data;

always @(posedge clk_pixel) begin
    rd_data <= frame_ram[rd_addr];
end

// HDMI Clock Generation
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
    .tmds_d_p(tmds_d_p)
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