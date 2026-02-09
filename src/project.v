/*
 * Copyright (c) 2024 Uri Shaked
 * SPDX-License-Identifier: Apache-2.0
 */


`default_nettype none


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


  // --------------------------
  // VGA signals (640x480 timing from generator)
  // --------------------------
  wire       hsync;
  wire       vsync;
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;
  wire       video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;

  // TinyVGA PMOD mapping
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // Unused outputs assigned to 0
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // Suppress unused signals warning
  wire _unused_ok = &{ena, ui_in, uio_in};

  // --------------------------
  // Sync generator
  // --------------------------
  hvsync_generator hvsync_gen(
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(video_active),
    .hpos(pix_x),
    .vpos(pix_y)
  );

  // --------------------------
  // 320x240 "logical" pixel coordinates (saves area)
  // --------------------------
  wire [8:0] pix_x320 = pix_x[9:1];   // 0..319
  wire [7:0] pix_y240 = pix_y[8:1];   // 0..239  (IMPORTANT: [8:1], not [9:1])

  // once per frame (still based on real timing)
  wire frame_tick = (pix_x == 10'd0) && (pix_y == 10'd0);

  // --------------------------
  // Small helper: absolute value (sized, lint-clean)
  // --------------------------
  function [9:0] abs10;
    input signed [9:0] x;
    reg s;
    begin
      s     = x[9];
      abs10 = (x ^ {10{s}}) + {{9{1'b0}}, s};
    end
  endfunction

  function [8:0] abs9;
    input signed [8:0] x;
    reg s;
    begin
      s    = x[8];
      abs9 = (x ^ {9{s}}) + {{8{1'b0}}, s};
    end
  endfunction

  // Apply sign to a tiny magnitude (mag is 0..2). Output is signed [-2..+2].
  function signed [3:0] signed_force4;
    input [1:0] mag;
    input       neg;   // 1 => negative
    reg [3:0] m4;
    begin
      m4 = {2'b00, mag};
      signed_force4 = (m4 ^ {4{neg}}) + {{3{1'b0}}, neg};
    end
  endfunction

  // --------------------------
  // State (stored at 320x240 resolution)
  // --------------------------
  reg [8:0] AX, BX, CX;     // X: 0..319
  reg [7:0] AY, BY, CY;     // Y: 0..239

  // Velocities: small signed (area saver)
  reg signed [7:0] vAX, vBX, vCX;
  reg signed [7:0] vAY, vBY, vCY;

  // optional debug counter (can remove to save a few flops)
  reg [9:0] counter;

  // --------------------------
  // PIXEL PATH: hit tests + color (320x240 math)
  // --------------------------
  wire signed [9:0] dxA = $signed({1'b0, pix_x320}) - $signed({1'b0, AX});
  wire signed [8:0] dyA = $signed({1'b0, pix_y240}) - $signed({1'b0, AY});

  wire signed [9:0] dxB = $signed({1'b0, pix_x320}) - $signed({1'b0, BX});
  wire signed [8:0] dyB = $signed({1'b0, pix_y240}) - $signed({1'b0, BY});

  wire signed [9:0] dxC = $signed({1'b0, pix_x320}) - $signed({1'b0, CX});
  wire signed [8:0] dyC = $signed({1'b0, pix_y240}) - $signed({1'b0, CY});

  wire [9:0] ax = abs10(dxA);
  wire [8:0] ay = abs9(dyA);
  wire [9:0] bx = abs10(dxB);
  wire [8:0] by = abs9(dyB);
  wire [9:0] cx = abs10(dxC);
  wire [8:0] cy = abs9(dyC);

  // CHEAP hitboxes (square): drop (ax+ay) diamond adders to save cells
  wire hitA = (ax < 10'd10) && (ay < 9'd10);
  wire hitB = (bx < 10'd10) && (by < 9'd10);
  wire hitC = (cx < 10'd10) && (cy < 9'd10);

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

  assign R = video_active ? R_pix : 2'b00;
  assign G = video_active ? G_pix : 2'b00;
  assign B = video_active ? B_pix : 2'b00;

  // --------------------------
  // FRAME PATH: forces + accel (computed combinationally, updates on frame_tick)
  // --------------------------
  wire signed [9:0] ABx = $signed({1'b0, BX}) - $signed({1'b0, AX});
  wire signed [8:0] ABy = $signed({1'b0, BY}) - $signed({1'b0, AY});
  wire signed [9:0] ACx = $signed({1'b0, CX}) - $signed({1'b0, AX});
  wire signed [8:0] ACy = $signed({1'b0, CY}) - $signed({1'b0, AY});
  wire signed [9:0] BCx = $signed({1'b0, CX}) - $signed({1'b0, BX});
  wire signed [8:0] BCy = $signed({1'b0, CY}) - $signed({1'b0, BY});

  // Manhattan distance approx (small widths)
  wire [9:0] ABd = abs10(ABx) + {1'b0, abs9(ABy)};
  wire [9:0] ACd = abs10(ACx) + {1'b0, abs9(ACy)};
  wire [9:0] BCd = abs10(BCx) + {1'b0, abs9(BCy)};

  // Force magnitudes (0..2) from comparisons
  wire AB_lt2 = (ABd < 10'd20);
  wire AB_lt1 = (ABd < 10'd150);
  wire [1:0] fABm = {AB_lt2, (AB_lt1 & ~AB_lt2)}; // 2, 1, or 0

  wire AC_lt2 = (ACd < 10'd20);
  wire AC_lt1 = (ACd < 10'd100);
  wire [1:0] fACm = {AC_lt2, (AC_lt1 & ~AC_lt2)};

  wire BC_lt2 = (BCd < 10'd20);
  wire BC_lt1 = (BCd < 10'd100);
  wire [1:0] fBCm = {BC_lt2, (BC_lt1 & ~BC_lt2)};

  // Signed forces per-axis ([-2..+2])
  wire signed [3:0] fABx_s = signed_force4(fABm, ABx[9]);
  wire signed [3:0] fABy_s = signed_force4(fABm, ABy[8]);
  wire signed [3:0] fACx_s = signed_force4(fACm, ACx[9]);
  wire signed [3:0] fACy_s = signed_force4(fACm, ACy[8]);
  wire signed [3:0] fBCx_s = signed_force4(fBCm, BCx[9]);
  wire signed [3:0] fBCy_s = signed_force4(fBCm, BCy[8]);

  // Accelerations (roughly [-4..+4])
  wire signed [4:0] aAX =  $signed(fABx_s) + $signed(fACx_s);
  wire signed [4:0] aAY =  $signed(fABy_s) + $signed(fACy_s);

  wire signed [4:0] aBX = -$signed(fABx_s) + $signed(fBCx_s);
  wire signed [4:0] aBY = -$signed(fABy_s) + $signed(fBCy_s);

  wire signed [4:0] aCX = -$signed(fACx_s) - $signed(fBCx_s);
  wire signed [4:0] aCY = -$signed(fACy_s) - $signed(fBCy_s);

  // --------------------------
  // Sequential update (once per frame)
  // --------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Start positions in 320x240 space
      AX <= 9'd150; AY <= 8'd75;
      BX <= 9'd210; BY <= 8'd150;
      CX <= 9'd100; CY <= 8'd10;

      vAX <= 8'sd0;  vAY <= 8'sd0;
      vBX <= 8'sd0;  vBY <= 8'sd0;
      vCX <= 8'sd0;  vCY <= 8'sd0;

      counter <= 10'd0;
    end else if (frame_tick) begin
      // velocity update (sign-extend accel into 8-bit)
      vAX <= vAX + {{3{aAX[4]}}, aAX};
      vAY <= vAY + {{3{aAY[4]}}, aAY};
      vBX <= vBX + {{3{aBX[4]}}, aBX};
      vBY <= vBY + {{3{aBY[4]}}, aBY};
      vCX <= vCX + {{3{aCX[4]}}, aCX};
      vCY <= vCY + {{3{aCY[4]}}, aCY};

      // position update (wrap naturally by truncation)
      AX <= AX + vAX;
      AY <= AY + vAY;
      BX <= BX + vBX;
      BY <= BY + vBY;
      CX <= CX + vCX;
      CY <= CY + vCY;

      counter <= counter + 10'd1;
    end
  end

endmodule
