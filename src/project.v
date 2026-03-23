`default_nettype none

module tt_um_SummerTT_HDL(
  input  wire [7:0] ui_in,    
  output wire [7:0] uo_out,   
  input  wire [7:0] uio_in,   
  output wire [7:0] uio_out,  
  output wire [7:0] uio_oe,   
  input  wire       ena,      
  input  wire       clk,      
  input  wire       rst_n     
);

// VGA signals
wire hsync;
wire vsync;
wire [1:0] R, G, B;
wire video_active;
wire [9:0] pix_x, pix_y;

wire running = ~ui_in[0];
wire randomize = ui_in[1];

// TinyVGA PMOD
assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};
assign uio_out = 0;
assign uio_oe  = 0;
wire _unused_ok = &{ena, ui_in, uio_in};

hvsync_generator hvsync_gen(
  .clk(clk),
  .reset(~rst_n),
  .hsync(hsync),
  .vsync(vsync),
  .display_on(video_active),
  .hpos(pix_x),
  .vpos(pix_y)
);

wire frame_active = (pix_x >= 64 && pix_x < 640-64 && pix_y >= 112 && pix_y < 480-112) ? 1 : 0;
wire icon_pixel = icon[pix_y[2:0]][pix_x[2:0]];

localparam CLOCK_FREQ = 24000000;
wire boot_reset = ~rst_n;

// ----------------- SIMULATION PARAMS -------------------------
localparam logWIDTH = 6, logHEIGHT = 5;         // 64x32 board
localparam UPDATE_INTERVAL = CLOCK_FREQ / 10;   // 5 Hz simulation update
localparam WIDTH = 2 ** logWIDTH;
localparam HEIGHT = 2 ** logHEIGHT;

// THE FIX: We use a 2D Array and ELIMINATE board_state_next entirely!
reg [WIDTH-1:0] board_state [0:HEIGHT-1];   

// Calculate Screen indexing
wire [logHEIGHT-1:0] draw_y = pix_y[7:3];
wire [logWIDTH-1:0] draw_x = pix_x[8:3];
wire is_alive = board_state[draw_y][draw_x];

assign R = (video_active & frame_active) ? {is_alive & icon_pixel, 1'b1} : 2'b00;
assign G = (video_active & frame_active) ? {is_alive & icon_pixel, 1'b1} : 2'b00;
assign B = 2'b01;

// ----------------- SIMULATION CONTROL LOGIC --------------------
localparam ACTION_IDLE = 0, ACTION_UPDATE = 1, ACTION_INIT = 2;
reg [1:0] action;
reg action_init_complete, action_update_complete;
reg [31:0] timer;

always @(posedge clk) begin
  if (boot_reset) begin
    action <= ACTION_INIT;
    timer <= 0;
  end else begin
    case (action)
      ACTION_IDLE: begin
        if (running) begin 
          if (timer < UPDATE_INTERVAL) begin
            timer <= timer + 1;
          end else if (~vsync) begin
            timer <= 0;
            action <= (~randomize) ? ACTION_UPDATE : ACTION_INIT;
          end
        end
      end

      ACTION_UPDATE: begin
        if (action_update_complete)
          action <= ACTION_IDLE;
      end

      ACTION_INIT: begin
        if (action_init_complete)
          action <= ACTION_IDLE;
      end

      default: action <= ACTION_IDLE;
    endcase
  end
end

// ----------------- MEMORY WRITE CONTROLLER --------------------
// THE FIX: Ensure we only write full WIDTH-sized words to memory!
reg [WIDTH-1:0] random_row; 

always @(posedge clk) begin
  // Write random data during INIT
  if (action == ACTION_INIT && !action_init_complete) begin
      // Shift random bit into our buffer
      random_row <= {random_row[WIDTH-2:0], rng};
      
      // When the buffer is full, commit the whole row to memory at once
      if (init_x == WIDTH - 1) begin
          board_state[init_y] <= {random_row[WIDTH-2:0], rng};
      end
  end
  // Write line buffers during UPDATE
  else if (action == ACTION_UPDATE && !action_update_complete) begin
      if (upd_state == ST_COMMIT && row_idx > 0) begin
          board_state[row_idx - 1] <= pending_row_write;
      end else if (upd_state == ST_FINISH) begin
          board_state[HEIGHT - 1] <= pending_row_write;
      end
  end
end

// ----------------- ACTION: RANDOMIZE SIMULATION STATE --------------------
reg [logWIDTH-1:0] init_x;
reg [logHEIGHT-1:0] init_y;

always @(posedge clk) begin
  if (boot_reset) begin
    action_init_complete <= 0;
    init_x <= 0;
    init_y <= 0;
  end else if (action == ACTION_INIT && !action_init_complete) begin
    if (init_x < WIDTH - 1) begin
      init_x <= init_x + 1;
    end else begin
      init_x <= 0;
      if (init_y < HEIGHT - 1) begin
        init_y <= init_y + 1;
      end else begin
        action_init_complete <= 1;
      end
    end
  end else if (action != ACTION_INIT) begin
    // Reset counters so the randomizer button works more than once
    action_init_complete <= 0;
    init_x <= 0;
    init_y <= 0;
  end
end

// ----------------- ACTION: COMPUTE (IN-PLACE LINE BUFFER) --------------------
localparam ST_CALC = 0, ST_COMMIT = 1, ST_FINISH = 2;
reg [1:0] upd_state;

reg [logHEIGHT:0] row_idx;
reg [logWIDTH:0] col_idx;

// The Line Buffers
reg [WIDTH-1:0] original_row_0;      
reg [WIDTH-1:0] current_row_next;    
reg [WIDTH-1:0] pending_row_write;   

// Combinatorial Neighbor Math 
wire [logHEIGHT-1:0] y_up = (row_idx == 0) ? HEIGHT - 1 : row_idx - 1;
wire [logHEIGHT-1:0] y_dn = (row_idx == HEIGHT - 1) ? 0 : row_idx + 1;
wire [logWIDTH-1:0] x_left = (col_idx == 0) ? WIDTH - 1 : col_idx - 1;
wire [logWIDTH-1:0] x_right = (col_idx == WIDTH - 1) ? 0 : col_idx + 1;

wire [WIDTH-1:0] row_top = board_state[y_up];
wire [WIDTH-1:0] row_mid = board_state[row_idx];
wire [WIDTH-1:0] row_bot = (y_dn == 0 && row_idx != 0) ? original_row_0 : board_state[y_dn];

wire top_left  = row_top[x_left];
wire top_mid   = row_top[col_idx];
wire top_right = row_top[x_right];
wire mid_left  = row_mid[x_left];
wire mid_right = row_mid[x_right];
wire bot_left  = row_bot[x_left];
wire bot_mid   = row_bot[col_idx];
wire bot_right = row_bot[x_right];

wire [3:0] num_neighbors = top_left + top_mid + top_right + mid_left + mid_right + bot_left + bot_mid + bot_right;
wire current_is_alive = row_mid[col_idx];
wire next_state = (current_is_alive && num_neighbors == 2) || (num_neighbors == 3);

always @(posedge clk) begin
  if (boot_reset) begin
    action_update_complete <= 0;
    row_idx <= 0;
    col_idx <= 0;
    upd_state <= ST_CALC;
  end else if (action == ACTION_UPDATE && !action_update_complete) begin
    
    case (upd_state)
      ST_CALC: begin
        if (row_idx == 0 && col_idx == 0) begin
            original_row_0 <= board_state[0];
        end

        current_row_next[col_idx] <= next_state;

        if (col_idx < WIDTH - 1) begin
            col_idx <= col_idx + 1;
        end else begin
            col_idx <= 0;
            upd_state <= ST_COMMIT;
        end
      end

      ST_COMMIT: begin
        pending_row_write <= current_row_next;
        
        if (row_idx < HEIGHT - 1) begin
            row_idx <= row_idx + 1;
            upd_state <= ST_CALC;
        end else begin
            upd_state <= ST_FINISH;
        end
      end

      ST_FINISH: begin
        action_update_complete <= 1;
      end
    endcase

  end else begin
    action_update_complete <= 0;
    row_idx <= 0;
    col_idx <= 0;
    upd_state <= ST_CALC;
  end 
end

// --------------- RNG --------------------
reg [15:0] lfsr_reg; 
wire feedback = lfsr_reg[15] ^ lfsr_reg[13] ^ lfsr_reg[12] ^ lfsr_reg[10];
wire rng = lfsr_reg[0]; 

always @(posedge clk) begin
  if (boot_reset) begin
    lfsr_reg <= 16'b0001; 
  end else begin
    lfsr_reg <= {lfsr_reg[14:0], feedback};
  end
end

// --------------- ICON FOR LIVE CELL --------------------
reg [7:0] icon[0:7];
initial begin
  icon[0] = 8'b00000000;
  icon[1] = 8'b00111100;
  icon[2] = 8'b01111110;
  icon[3] = 8'b01111110;
  icon[4] = 8'b01111110;
  icon[5] = 8'b01111110;
  icon[6] = 8'b00111100;
  icon[7] = 8'b00000000;
end

endmodule
