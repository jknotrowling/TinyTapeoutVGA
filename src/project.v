/*
 * Copyright (c) 2024 Uri Shaked
 * SPDX-License-Identifier: Apache-2.0
 */

// Three-body gravitational simulation rendered on a 640x480 VGA display.
// Three planets (A, B, C) attract each other using a simplified gravity model
// based on Manhattan distance. Physics are computed during VGA blanking intervals
// using a time-multiplexed micro-scheduler to minimize gate count.

`default_nettype none

// =============================================================================
// Top-level: TinyTapeout interface wiring and module instantiation
// =============================================================================
module tt_um_Jan_three_body_solution(
  input  wire [7:0] ui_in,
  output wire [7:0] uo_out,
  input  wire [7:0] uio_in,
  output wire [7:0] uio_out,
  output wire [7:0] uio_oe,
  input  wire       ena,
  input  wire       clk,
  input  wire       rst_n
);

  // VGA timing signals
  wire hsync, vsync, video_active;
  wire [9:0] pix_x, pix_y;

  // Planet positions (driven by gravity module)
  wire signed [9:0] AX, AY, BX, BY, CX, CY;

  // Rendered pixel color (raw, before blanking gate)
  wire [1:0] R_pix, G_pix, B_pix;

  // Blanking gate: force RGB to black during blanking, keep sync signals
  wire [1:0] R = video_active ? R_pix : 2'b00;
  wire [1:0] G = video_active ? G_pix : 2'b00;
  wire [1:0] B = video_active ? B_pix : 2'b00;

  // Map VGA signals to TinyVGA PMOD output pin order
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // Unused outputs
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // Suppress unused signals warning
  wire _unused_ok = &{ena, ui_in, uio_in};

  // Physics triggers
  wire frame_tick = (pix_x == 10'd0) && (pix_y == 10'd0);
  wire blanking   = ~video_active;

  // --- VGA timing generator ---
  hvsync_generator hvsync_gen(
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(video_active),
    .hpos(pix_x),
    .vpos(pix_y)
  );

  // --- Gravity physics engine ---
  gravity gravity_inst(
    .clk(clk),
    .rst_n(rst_n),
    .frame_tick(frame_tick),
    .blanking(blanking),
    .AX(AX), .AY(AY),
    .BX(BX), .BY(BY),
    .CX(CX), .CY(CY)
  );

  // --- Pixel renderer ---
  renderer renderer_inst(
    .pix_x(pix_x),
    .pix_y(pix_y),
    .AX(AX), .AY(AY),
    .BX(BX), .BY(BY),
    .CX(CX), .CY(CY),
    .R(R_pix),
    .G(G_pix),
    .B(B_pix)
  );

endmodule


// =============================================================================
// Shared utility: branchless absolute value via XOR+add
// =============================================================================
module abs11 (
  input  wire signed [10:0] val,
  output wire        [10:0] result
);
  wire sign = val[10];
  assign result = (val ^ {11{sign}}) + {10'd0, sign};
endmodule


// =============================================================================
// Renderer: pixel hit-testing and color assignment (pure combinational)
// =============================================================================
module renderer (
  input  wire [9:0]        pix_x,
  input  wire [9:0]        pix_y,
  input  wire signed [9:0] AX, AY,
  input  wire signed [9:0] BX, BY,
  input  wire signed [9:0] CX, CY,
  output wire [1:0]        R,
  output wire [1:0]        G,
  output wire [1:0]        B
);

  // Planet rendering parameters
  localparam [10:0] PLANET_HALF  = 11'd8;   // per-axis bounding box half-size
  localparam [11:0] PLANET_DIAM  = 12'd10;  // Manhattan-distance diamond threshold

  // Planet colors:               RR_GG_BB
  localparam [5:0] COLOR_A    = 6'b11_10_01;  // yellow-ish
  localparam [5:0] COLOR_B    = 6'b10_11_10;  // cyan-ish
  localparam [5:0] COLOR_C    = 6'b01_01_11;  // blue-ish
  localparam [5:0] COLOR_BG   = 6'b00_00_00;  // black background

  // Signed pixel coordinate (11-bit to avoid overflow with signed planet coords)
  wire signed [10:0] spx = $signed({1'b0, pix_x});
  wire signed [10:0] spy = $signed({1'b0, pix_y});

  // Signed distance from current pixel to each planet center
  wire signed [10:0] dxA = spx - $signed({AX[9], AX});
  wire signed [10:0] dyA = spy - $signed({AY[9], AY});
  wire signed [10:0] dxB = spx - $signed({BX[9], BX});
  wire signed [10:0] dyB = spy - $signed({BY[9], BY});
  wire signed [10:0] dxC = spx - $signed({CX[9], CX});
  wire signed [10:0] dyC = spy - $signed({CY[9], CY});

  // Absolute distances via shared abs module
  wire [10:0] ax, ay, bx, by, cx, cy;
  abs11 abs_dxA(.val(dxA), .result(ax));
  abs11 abs_dyA(.val(dyA), .result(ay));
  abs11 abs_dxB(.val(dxB), .result(bx));
  abs11 abs_dyB(.val(dyB), .result(by));
  abs11 abs_dxC(.val(dxC), .result(cx));
  abs11 abs_dyC(.val(dyC), .result(cy));

  // Hit test: pixel is inside a planet if within a diamond shape (Manhattan dist)
  wire hitA = (ax < PLANET_HALF) && (ay < PLANET_HALF) && ((ax + ay) < PLANET_DIAM);
  wire hitB = (bx < PLANET_HALF) && (by < PLANET_HALF) && ((bx + by) < PLANET_DIAM);
  wire hitC = (cx < PLANET_HALF) && (cy < PLANET_HALF) && ((cx + cy) < PLANET_DIAM);

  // Color mux: priority A > B > C > background
  wire [5:0] color = hitA ? COLOR_A :
                     hitB ? COLOR_B :
                     hitC ? COLOR_C : COLOR_BG;

  assign {R, G, B} = color;

endmodule


// =============================================================================
// Gravity: planet state, physics computation and sweep state machine
// =============================================================================
module gravity (
  input  wire       clk,
  input  wire       rst_n,
  input  wire       frame_tick,
  input  wire       blanking,
  output wire signed [9:0] AX, AY,
  output wire signed [9:0] BX, BY,
  output wire signed [9:0] CX, CY
);

  // --- Initial positions: equilateral triangle centered on screen (640x480) ---
  // Manhattan distances: A-B ≈ 230, A-C ≈ 230, B-C ≈ 180 → all in amag=1 zone
  localparam signed [12:0] INIT_AX = {10'sd320, 3'b0};  // top center
  localparam signed [12:0] INIT_AY = {10'sd160, 3'b0};
  localparam signed [12:0] INIT_BX = {10'sd230, 3'b0};  // bottom left
  localparam signed [12:0] INIT_BY = {10'sd300, 3'b0};
  localparam signed [12:0] INIT_CX = {10'sd410, 3'b0};  // bottom right
  localparam signed [12:0] INIT_CY = {10'sd300, 3'b0};

  // Number of planet pairs × 2 axes = 12 micro-steps (pairs: AB AC BA BC CA CB)
  localparam [2:0] NUM_PAIRS    = 3'd6;
  localparam [2:0] LAST_PAIR    = 3'd5;

  // --- Internal positions: 13-bit signed fixed point (10.3) ---
  reg signed [12:0] pAX, pAY, pBX, pBY, pCX, pCY;

  // --- Output integer positions (upper 10 bits) ---
  assign AX = pAX[12:3];
  assign AY = pAY[12:3];
  assign BX = pBX[12:3];
  assign BY = pBY[12:3];
  assign CX = pCX[12:3];
  assign CY = pCY[12:3];

  // --- Planet velocities (signed 8-bit: 5 integer + 3 fractional) ---
  reg signed [7:0] vAX, vAY, vBX, vBY, vCX, vCY;

  // --- Sweep state machine ---
  reg [2:0] rel;           // current pair index (0..5)
  reg       axis;          // current axis: 0=X, 1=Y
  reg       sweep_active;  // high during the 12-step gravity sweep each frame

  wire do_step = blanking && sweep_active;

  // ---------------------------------------------------------------------------
  // Planet index lookup tables: derive P and Q indices from pair number
  //   Pairs: 0=AB, 1=AC, 2=BA, 3=BC, 4=CA, 5=CB
  // ---------------------------------------------------------------------------
  wire [1:0] p_idx = (rel < 3'd2) ? 2'd0 :
                     (rel < 3'd4) ? 2'd1 : 2'd2;

  // Q index: the attractor planet for each pair
  wire [1:0] q_idx = (rel == 3'd0) ? 2'd1 :   // AB -> Q=B
                     (rel == 3'd1) ? 2'd2 :   // AC -> Q=C
                     (rel == 3'd2) ? 2'd0 :   // BA -> Q=A
                     (rel == 3'd3) ? 2'd2 :   // BC -> Q=C
                     (rel == 3'd4) ? 2'd0 :   // CA -> Q=A
                                    2'd1;     // CB -> Q=B

  // ---------------------------------------------------------------------------
  // Position muxes using planet indices (use integer part for gravity calc)
  // ---------------------------------------------------------------------------
  wire signed [9:0] PX = (p_idx == 2'd0) ? AX : (p_idx == 2'd1) ? BX : CX;
  wire signed [9:0] PY = (p_idx == 2'd0) ? AY : (p_idx == 2'd1) ? BY : CY;
  wire signed [9:0] QX = (q_idx == 2'd0) ? AX : (q_idx == 2'd1) ? BX : CX;
  wire signed [9:0] QY = (q_idx == 2'd0) ? AY : (q_idx == 2'd1) ? BY : CY;

  // ---------------------------------------------------------------------------
  // Gravity vector computation
  // ---------------------------------------------------------------------------

  // Vector from P to Q (direction of gravitational pull)
  wire signed [10:0] dx = $signed({QX[9], QX}) - $signed({PX[9], PX});
  wire signed [10:0] dy = $signed({QY[9], QY}) - $signed({PY[9], PY});

  // Absolute distances via shared abs module
  wire [10:0] abs_dx, abs_dy;
  abs11 abs_gx(.val(dx), .result(abs_dx));
  abs11 abs_gy(.val(dy), .result(abs_dy));

  // Manhattan distance |dx|+|dy| between P and Q
  wire [10:0] manhattan = abs_dx + abs_dy;

  // Gravity magnitude lookup: closer planets pull harder (1/r approximation).
  // 3-bit amag → 4 force levels; capped to prevent Euler-integrator blow-up.
  // Values are in 1/8 pixel units (3 fractional bits).
  wire [2:0] amag = |manhattan[10:8] ? 3'd0 :  // > 255 px: no force
                      manhattan[7]   ? 3'd2 :  // 128-255 px: weak  (0.25 px/f²)
                      manhattan[6]   ? 3'd4 :  // 64-127 px:  medium (0.50 px/f²)
                                      3'd6;   // < 64 px:    strong (0.75 px/f²)

  // Convert magnitude to signed velocity delta pointing from P toward Q
  wire sx = dx[10];
  wire sy = dy[10];
  wire signed [7:0] dvx = $signed(({5'b0, amag} ^ {8{sx}})) + $signed({7'd0, sx});
  wire signed [7:0] dvy = $signed(({5'b0, amag} ^ {8{sy}})) + $signed({7'd0, sy});

  // Select current velocity and delta for active axis
  wire signed [7:0] v_in  = axis ? ((p_idx == 2'd0) ? vAY : (p_idx == 2'd1) ? vBY : vCY)
                                 : ((p_idx == 2'd0) ? vAX : (p_idx == 2'd1) ? vBX : vCX);
  wire signed [7:0] dv_in = axis ? dvy : dvx;

  // New velocity = old velocity + gravitational acceleration (with overflow detection)
  wire signed [8:0] v_sum = {v_in[7], v_in} + {dv_in[7], dv_in};  // 9-bit to catch overflow

  // Clamp to 8-bit signed range: -128 to +127
  wire overflow_pos = ~v_sum[8] && v_sum[7];
  wire overflow_neg =  v_sum[8] && ~v_sum[7];
  wire signed [7:0] v_out = overflow_pos ? 8'sd127 :
                            overflow_neg ? -8'sd128 : v_sum[7:0];

  // ---------------------------------------------------------------------------
  // Sign-extended velocity for position update (8-bit -> 13-bit)
  // ---------------------------------------------------------------------------
  wire signed [12:0] vAX_ext = {{5{vAX[7]}}, vAX};
  wire signed [12:0] vAY_ext = {{5{vAY[7]}}, vAY};
  wire signed [12:0] vBX_ext = {{5{vBX[7]}}, vBX};
  wire signed [12:0] vBY_ext = {{5{vBY[7]}}, vBY};
  wire signed [12:0] vCX_ext = {{5{vCX[7]}}, vCX};
  wire signed [12:0] vCY_ext = {{5{vCY[7]}}, vCY};

  // ---------------------------------------------------------------------------
  // Sequential logic
  // ---------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pAX <= INIT_AX;  pAY <= INIT_AY;
      pBX <= INIT_BX;  pBY <= INIT_BY;
      pCX <= INIT_CX;  pCY <= INIT_CY;

      // CCW tangential velocities ≈ orbital speed for amag=2 at r≈100px
      // v_orb = sqrt(r·a) = sqrt(100 * 2/8)·8 = 40  (in 1/8 px/frame units)
      // Directions from COM=(320,253): A→right, B→up-right, C→down-right
      // Sum vx = 48-24-24 = 0, sum vy = 0-42+42 = 0 (momentum conserved)
      vAX <=  8'sd48;  vAY <=  8'sd0;
      vBX <= -8'sd24;  vBY <= -8'sd42;
      vCX <= -8'sd24;  vCY <=  8'sd42;

      rel          <= 3'd0;
      axis         <= 1'b0;
      sweep_active <= 1'b0;

    end else begin
      // At the start of each frame: apply velocities to positions,
      // then arm the 12-step gravity sweep for this frame
      if (frame_tick) begin
        pAX <= pAX + vAX_ext;
        pAY <= pAY + vAY_ext;
        pBX <= pBX + vBX_ext;
        pBY <= pBY + vBY_ext;
        pCX <= pCX + vCX_ext;
        pCY <= pCY + vCY_ext;

        rel          <= 3'd0;
        axis         <= 1'b0;
        sweep_active <= 1'b1;
      end

      // Execute one gravity micro-step per blanking clock cycle
      if (do_step) begin
        // Write updated velocity back to planet P's register
        case (p_idx)
          2'd0: if (!axis) vAX <= v_out; else vAY <= v_out;
          2'd1: if (!axis) vBX <= v_out; else vBY <= v_out;
          2'd2: if (!axis) vCX <= v_out; else vCY <= v_out;
          default: ;
        endcase

        // Advance sweep: X->Y for same pair, then next pair
        if (rel == LAST_PAIR && axis) begin
          // All 12 micro-steps done
          sweep_active <= 1'b0;
          axis         <= 1'b0;
          rel          <= 3'd0;
        end else if (!axis) begin
          axis <= 1'b1;
        end else begin
          axis <= 1'b0;
          rel  <= rel + 3'd1;
        end
      end
    end
  end

endmodule
