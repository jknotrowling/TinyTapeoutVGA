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
// Cannot use `function` in purely combinational context for all synth tools,
// so we use a small helper module instead for maximum portability.
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
  output reg signed [9:0] AX, AY,
  output reg signed [9:0] BX, BY,
  output reg signed [9:0] CX, CY
);

  // --- Initial positions (screen-center triangle) ---
  localparam signed [9:0] INIT_AX = 10'sd270, INIT_AY = 10'sd200;
  localparam signed [9:0] INIT_BX = 10'sd370, INIT_BY = 10'sd280;
  localparam signed [9:0] INIT_CX = 10'sd320, INIT_CY = 10'sd160;

  // Number of planet pairs × 2 axes = 12 micro-steps (pairs: AB AC BA BC CA CB)
  localparam [2:0] NUM_PAIRS    = 3'd6;
  localparam [2:0] LAST_PAIR    = 3'd5;

  // --- Planet velocities (signed 5-bit, wraps on overflow) ---
  reg signed [4:0] vel [0:2][0:1];  // vel[planet][axis]: 0=X, 1=Y

  // --- Sweep state machine ---
  reg [2:0] rel;           // current pair index (0..5)
  reg       axis;          // current axis: 0=X, 1=Y
  reg       sweep_active; // high during the 12-step gravity sweep each frame

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
  // Position muxes using planet indices
  // ---------------------------------------------------------------------------
  wire signed [9:0] pos [0:2][0:1];
  assign pos[0][0] = AX;  assign pos[0][1] = AY;
  assign pos[1][0] = BX;  assign pos[1][1] = BY;
  assign pos[2][0] = CX;  assign pos[2][1] = CY;

  wire signed [9:0] PX = pos[p_idx][0];
  wire signed [9:0] PY = pos[p_idx][1];
  wire signed [9:0] QX = pos[q_idx][0];
  wire signed [9:0] QY = pos[q_idx][1];

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

  // Gravity magnitude lookup: closer planets pull harder.
  // Uses MSB checks as power-of-two threshold approximation.
  // Priority encoding from farthest to nearest:
  //   bit 10 or 9 set -> very far  (amag=5)
  //   bit 8 set       -> far       (amag=4)
  //   bit 7 set       -> medium    (amag=3)
  //   bit 6 set       -> close     (amag=2)
  //   bit 5 set       -> very close(amag=1)
  //   otherwise        -> collision (amag=0, no force)
  wire [2:0] amag = |manhattan[10:9] ? 3'd5 :
                      manhattan[8]   ? 3'd4 :
                      manhattan[7]   ? 3'd3 :
                      manhattan[6]   ? 3'd2 :
                      manhattan[5]   ? 3'd1 : 3'd0;

  // Convert magnitude to signed velocity delta pointing from P toward Q
  wire sx = dx[10];
  wire sy = dy[10];
  wire signed [4:0] dvx = $signed(({2'b0, amag} ^ {5{sx}})) + $signed({4'd0, sx});
  wire signed [4:0] dvy = $signed(({2'b0, amag} ^ {5{sy}})) + $signed({4'd0, sy});

  // Select current velocity and delta for active axis
  wire signed [4:0] v_in  = axis ? vel[p_idx][1] : vel[p_idx][0];
  wire signed [4:0] dv_in = axis ? dvy : dvx;

  // New velocity = old velocity + gravitational acceleration
  wire signed [4:0] v_out = v_in + dv_in;

  // ---------------------------------------------------------------------------
  // Sign-extended velocity for position update
  // ---------------------------------------------------------------------------
  wire signed [9:0] vAX_ext = {{5{vel[0][0][4]}}, vel[0][0]};
  wire signed [9:0] vAY_ext = {{5{vel[0][1][4]}}, vel[0][1]};
  wire signed [9:0] vBX_ext = {{5{vel[1][0][4]}}, vel[1][0]};
  wire signed [9:0] vBY_ext = {{5{vel[1][1][4]}}, vel[1][1]};
  wire signed [9:0] vCX_ext = {{5{vel[2][0][4]}}, vel[2][0]};
  wire signed [9:0] vCY_ext = {{5{vel[2][1][4]}}, vel[2][1]};

  // ---------------------------------------------------------------------------
  // Sequential logic
  // ---------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      AX <= INIT_AX;  AY <= INIT_AY;
      BX <= INIT_BX;  BY <= INIT_BY;
      CX <= INIT_CX;  CY <= INIT_CY;

      vel[0][0] <= 5'sd0;  vel[0][1] <= 5'sd0;
      vel[1][0] <= 5'sd0;  vel[1][1] <= 5'sd0;
      vel[2][0] <= 5'sd0;  vel[2][1] <= 5'sd0;

      rel          <= 3'd0;
      axis         <= 1'b0;
      sweep_active <= 1'b0;

    end else begin
      // At the start of each frame: apply velocities to positions,
      // then arm the 12-step gravity sweep for this frame
      if (frame_tick) begin
        AX <= AX + vAX_ext;
        AY <= AY + vAY_ext;
        BX <= BX + vBX_ext;
        BY <= BY + vBY_ext;
        CX <= CX + vCX_ext;
        CY <= CY + vCY_ext;

        rel          <= 3'd0;
        axis         <= 1'b0;
        sweep_active <= 1'b1;
      end

      // Execute one gravity micro-step per blanking clock cycle
      if (do_step) begin
        // Write updated velocity back to planet P's register
        vel[p_idx][axis] <= v_out;

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