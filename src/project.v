/*
 * Copyright (c) 2024 Uri Shaked
 * SPDX-License-Identifier: Apache-2.0
 */


`default_nettype none

//tt_um_vga_example
module tt_um_Jan_three_body_solution(
  input  wire [7:0] ui_in,    // Dedicated inputs
  output wire [7:0] uo_out,   // Dedicated outputs
  input  wire [7:0] uio_in,   // IOs: Input path
  output wire [7:0] uio_out,  // IOs: Output path
  output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
  input  wire       ena,      // always 1 when the design is powered, so you can ignore it
  input  wire       clk,      // clock
  input  wire       rst_n     // reset_n - low to reset
);


  // VGA signals
  wire hsync;
  wire vsync;
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;
  wire video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;

  // TinyVGA PMOD
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // Unused outputs assigned to 0.
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // Suppress unused signals warning
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

  // positions (screen coordinates fit in signed 10-bit for your use)
  reg signed [9:0] AX, AY;
  reg signed [9:0] BX, BY;
  reg signed [9:0] CX, CY;

  // velocities: signed 5-bit (wraps on overflow), per your constraint
  reg signed [4:0] vAX, vAY;
  reg signed [4:0] vBX, vBY;
  reg signed [4:0] vCX, vCY;

  // ---------------------------------------------------------------------------
  // Pixel-to-planet deltas (for drawing)
  // ---------------------------------------------------------------------------
  wire signed [10:0] dxA = $signed({1'b0,pix_x}) - $signed({AX[9],AX});
  wire signed [10:0] dyA = $signed({1'b0,pix_y}) - $signed({AY[9],AY});
  wire signed [10:0] dxB = $signed({1'b0,pix_x}) - $signed({BX[9],BX});
  wire signed [10:0] dyB = $signed({1'b0,pix_y}) - $signed({BY[9],BY});
  wire signed [10:0] dxC = $signed({1'b0,pix_x}) - $signed({CX[9],CX});
  wire signed [10:0] dyC = $signed({1'b0,pix_y}) - $signed({CY[9],CY});

  // abs via XOR+add1 (explicit sizing to keep Verilator happy)
  wire sdxA = dxA[10], sdyA = dyA[10];
  wire sdxB = dxB[10], sdyB = dyB[10];
  wire sdxC = dxC[10], sdyC = dyC[10];

  wire [10:0] ax = (dxA ^ {11{sdxA}}) + {10'd0, sdxA};
  wire [10:0] ay = (dyA ^ {11{sdyA}}) + {10'd0, sdyA};
  wire [10:0] bx = (dxB ^ {11{sdxB}}) + {10'd0, sdxB};
  wire [10:0] by = (dyB ^ {11{sdyB}}) + {10'd0, sdyB};
  wire [10:0] cx = (dxC ^ {11{sdxC}}) + {10'd0, sdxC};
  wire [10:0] cy = (dyC ^ {11{sdyC}}) + {10'd0, sdyC};

  // small bodies
  wire hitA = (ax < 11'd8) && (ay < 11'd8) && ((ax + ay) < 12'd10);
  wire hitB = (bx < 11'd8) && (by < 11'd8) && ((bx + by) < 12'd10);
  wire hitC = (cx < 11'd8) && (cy < 11'd8) && ((cx + cy) < 12'd10);

  wire [1:0] R_pix =
    hitA ? 2'b11 :
    hitB ? 2'b10 :
    hitC ? 2'b01 : 2'b00;

  wire [1:0] G_pix =
    hitA ? 2'b10 :
    hitB ? 2'b11 :
    hitC ? 2'b01 : 2'b00;

  wire [1:0] B_pix =
    hitA ? 2'b01 :
    hitB ? 2'b10 :
    hitC ? 2'b11 : 2'b00;

  // ---------------------------------------------------------------------------
  // Blanking-time micro-scheduler: do AB, AC, BA, BC, CA, CB (X then Y)
  // Gate-min: no stored force, only update the planet being calculated for,
  // and only ONE shared 5-bit velocity adder (time-multiplexed).
  // ---------------------------------------------------------------------------
  wire frame_tick = (pix_x == 10'd0) && (pix_y == 10'd0);

  reg [2:0] rel;          // 0..5 : AB, AC, BA, BC, CA, CB
  reg       axis;         // 0=X, 1=Y
  reg       sweep_active; // 1 while we still need to execute the 12 micro-steps

  wire blanking = ~video_active;
  wire do_step  = blanking && sweep_active;

  // P = planet being updated, Q = other planet
  wire signed [9:0] PX = (rel==3'd0 || rel==3'd1) ? AX :
                         (rel==3'd2 || rel==3'd3) ? BX : CX;
  wire signed [9:0] PY = (rel==3'd0 || rel==3'd1) ? AY :
                         (rel==3'd2 || rel==3'd3) ? BY : CY;

  wire signed [9:0] QX = (rel==3'd0) ? BX :
                         (rel==3'd1) ? CX :
                         (rel==3'd2) ? AX :
                         (rel==3'd3) ? CX :
                         (rel==3'd4) ? AX : BX;   // rel==5: CB
  wire signed [9:0] QY = (rel==3'd0) ? BY :
                         (rel==3'd1) ? CY :
                         (rel==3'd2) ? AY :
                         (rel==3'd3) ? CY :
                         (rel==3'd4) ? AY : BY;

  // dx,dy = Q - P (11-bit signed)
  wire signed [10:0] dx = $signed({QX[9],QX}) - $signed({PX[9],PX});
  wire signed [10:0] dy = $signed({QY[9],QY}) - $signed({PY[9],PY});
  wire sx = dx[10];
  wire sy = dy[10];

  // abs(dx), abs(dy) (explicit sizing)
  wire [10:0] abs_dx = (dx ^ {11{sx}}) + {10'd0, sx};
  wire [10:0] abs_dy = (dy ^ {11{sy}}) + {10'd0, sy};

  // Manhattan distance
  wire [10:0] ABd = abs_dx + abs_dy;

  // Power-of-two cutoffs: 32/64/128/256/512 => just bit taps + one OR
  wire ge512 = |ABd[10:9];
  wire ge256 =  ABd[8];
  wire ge128 =  ABd[7];
  wire ge64  =  ABd[6];
  wire ge32  =  ABd[5];

  // magnitude (0..5), 3-bit
  wire [2:0] amag =
    ge512 ? 3'd5 :
    ge256 ? 3'd4 :
    ge128 ? 3'd3 :
    ge64  ? 3'd2 :
    ge32  ? 3'd1 : 3'd0;

  // signed dv (5-bit) toward Q for P, per axis (XOR+add1 with explicit sizing)
  wire signed [4:0] dvx =
    $signed(({2'b0, amag} ^ {5{sx}})) + $signed({4'd0, sx});
  wire signed [4:0] dvy =
    $signed(({2'b0, amag} ^ {5{sy}})) + $signed({4'd0, sy});

  // ONE shared 5-bit adder for velocity update
  wire signed [4:0] vP_X = (rel==3'd0 || rel==3'd1) ? vAX :
                           (rel==3'd2 || rel==3'd3) ? vBX : vCX;
  wire signed [4:0] vP_Y = (rel==3'd0 || rel==3'd1) ? vAY :
                           (rel==3'd2 || rel==3'd3) ? vBY : vCY;

  wire signed [4:0] v_in  = axis ? vP_Y : vP_X;
  wire signed [4:0] dv_in = axis ? dvy  : dvx;

  wire signed [4:0] v_out = v_in + dv_in;

  // ---------------------------------------------------------------------------
  // Sequential
  // ---------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      AX <= 10'sd270; AY <= 10'sd200;
      BX <= 10'sd370; BY <= 10'sd280;
      CX <= 10'sd320; CY <= 10'sd160;

      vAX <= 5'sd0; vAY <= 5'sd0;
      vBX <= 5'sd0; vBY <= 5'sd0;
      vCX <= 5'sd0; vCY <= 5'sd0;

      rel <= 3'd0;
      axis <= 1'b0;
      sweep_active <= 1'b0;
    end else begin
      // Once per frame: update positions and arm a 12-step blanking sweep
      if (frame_tick) begin
        AX <= AX + $signed({{5{vAX[4]}}, vAX});
        AY <= AY + $signed({{5{vAY[4]}}, vAY});
        BX <= BX + $signed({{5{vBX[4]}}, vBX});
        BY <= BY + $signed({{5{vBY[4]}}, vBY});
        CX <= CX + $signed({{5{vCX[4]}}, vCX});
        CY <= CY + $signed({{5{vCY[4]}}, vCY});

        rel <= 3'd0;
        axis <= 1'b0;
        sweep_active <= 1'b1;
      end

      // During blanking: AB,AC,BA,BC,CA,CB with X then Y (12 micro-steps)
      if (do_step) begin
        // Write back selected component only (planet P only)
        if (rel==3'd0 || rel==3'd1) begin
          if (!axis) vAX <= v_out;
          else       vAY <= v_out;
        end else if (rel==3'd2 || rel==3'd3) begin
          if (!axis) vBX <= v_out;
          else       vBY <= v_out;
        end else begin
          if (!axis) vCX <= v_out;
          else       vCY <= v_out;
        end

        // Stop after last micro-step: rel=5 (CB) and axis=1 (Y)
        if ((rel==3'd5) && axis) begin
          sweep_active <= 1'b0;
          axis <= 1'b0;
          rel  <= 3'd0;
        end else begin
          // advance axis/rel
          if (!axis) begin
            axis <= 1'b1;
          end else begin
            axis <= 1'b0;
            rel  <= (rel==3'd5) ? 3'd0 : (rel + 3'd1);
          end
        end
      end
    end
  end

  // Output pixels
  assign R = video_active ? R_pix : 2'b00;
  assign G = video_active ? G_pix : 2'b00;
  assign B = video_active ? B_pix : 2'b00;

endmodule
