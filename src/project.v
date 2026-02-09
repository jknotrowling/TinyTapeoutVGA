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


  // VGA signals
  wire hsync;
  wire vsync;
  wire [1:0] R, G, B;
  wire video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;

  // TinyVGA PMOD (ONLY ONCE)
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

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

  // --- 320x240 logical coordinates (render/game resolution) ---
  wire [8:0] pix_x320 = pix_x[9:1];   // 0..319
  wire [7:0] pix_y240 = pix_y[9:1];   // 0..239

  // once per frame
  wire frame_tick = (pix_x == 10'd0) && (pix_y == 10'd0);

  // --- positions stored at 320x240 ---
  reg [8:0] AX, BX, CX;   // 0..319
  reg [7:0] AY, BY, CY;   // 0..239

  // --- velocities (smaller signed) ---
  reg  signed [7:0] vAX, vBX, vCX;
  reg  signed [7:0] vAY, vBY, vCY;

  // dx/dy at 320x240 (keep a sign bit)
  wire signed [9:0] dxA = $signed({1'b0,pix_x320}) - $signed({1'b0,AX});
  wire signed [8:0] dyA = $signed({1'b0,pix_y240}) - $signed({1'b0,AY});
  wire signed [9:0] dxB = $signed({1'b0,pix_x320}) - $signed({1'b0,BX});
  wire signed [8:0] dyB = $signed({1'b0,pix_y240}) - $signed({1'b0,BY});
  wire signed [9:0] dxC = $signed({1'b0,pix_x320}) - $signed({1'b0,CX});
  wire signed [8:0] dyC = $signed({1'b0,pix_y240}) - $signed({1'b0,CY});

  // abs (smaller)
  wire [9:0] ax = dxA[9] ? -dxA : dxA;
  wire [8:0] ay = dyA[8] ? -dyA : dyA;
  wire [9:0] bx = dxB[9] ? -dxB : dxB;
  wire [8:0] by = dyB[8] ? -dyB : dyB;
  wire [9:0] cx = dxC[9] ? -dxC : dxC;
  wire [8:0] cy = dyC[8] ? -dyC : dyC;

  // planet-to-planet deltas in 320x240 space
  wire signed [9:0] ABx = $signed({1'b0,BX}) - $signed({1'b0,AX});
  wire signed [8:0] ABy = $signed({1'b0,BY}) - $signed({1'b0,AY});
  wire signed [9:0] ACx = $signed({1'b0,CX}) - $signed({1'b0,AX});
  wire signed [8:0] ACy = $signed({1'b0,CY}) - $signed({1'b0,AY});
  wire signed [9:0] BCx = $signed({1'b0,CX}) - $signed({1'b0,BX});
  wire signed [8:0] BCy = $signed({1'b0,CY}) - $signed({1'b0,BY});

  // Manhattan distance approx (smaller)
  wire [9:0] ABd = (ABx[9] ? -ABx : ABx) + (ABy[8] ? -ABy : ABy);
  wire [9:0] ACd = (ACx[9] ? -ACx : ACx) + (ACy[8] ? -ACy : ACy);
  wire [9:0] BCd = (BCx[9] ? -BCx : BCx) + (BCy[8] ? -BCy : BCy);

  // force magnitude (same idea, tweak thresholds if you want)
  wire signed [7:0] fAB =
    (ABd < 10'd20)  ? 8'sd2 :
    (ABd < 10'd150) ? 8'sd1 :
                      8'sd0;

  wire signed [7:0] fAC =
    (ACd < 10'd20)  ? 8'sd2 :
    (ACd < 10'd100) ? 8'sd1 :
                      8'sd0;

  wire signed [7:0] fBC =
    (BCd < 10'd20)  ? 8'sd2 :
    (BCd < 10'd100) ? 8'sd1 :
                      8'sd0;

  // accelerations (small signed)
  wire signed [7:0] aAX = (ABx[9] ? -fAB : fAB) + (ACx[9] ? -fAC : fAC);
  wire signed [7:0] aAY = (ABy[8] ? -fAB : fAB) + (ACy[8] ? -fAC : fAC);
  wire signed [7:0] aBX = (ABx[9] ?  fAB : -fAB) + (BCx[9] ? -fBC :  fBC);
  wire signed [7:0] aBY = (ABy[8] ?  fAB : -fAB) + (BCy[8] ? -fBC :  fBC);
  wire signed [7:0] aCX = (ACx[9] ?  fAC : -fAC) + (BCx[9] ?  fBC : -fBC);
  wire signed [7:0] aCY = (ACy[8] ?  fAC : -fAC) + (BCy[8] ?  fBC : -fBC);

  // hit test in 320x240 space (you can shrink radius too if you want)
  wire hitA = (ax < 10'd10) && (ay < 9'd10) && ((ax + ay) < 11'd13);
  wire hitB = (bx < 10'd10) && (by < 9'd10) && ((bx + by) < 11'd13);
  wire hitC = (cx < 10'd10) && (cy < 9'd10) && ((cx + cy) < 11'd13);

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

  // position update (with simple wrap to stay in range)
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      AX <= 9'd150; AY <= 8'd75;
      BX <= 9'd210; BY <= 8'd150;
      CX <= 9'd100; CY <= 8'd10;
      vAX <= 0; vAY <= 0;
      vBX <= 0; vBY <= 0;
      vCX <= 0; vCY <= 0;
    end else if (frame_tick) begin
      vAX <= vAX + aAX;  vAY <= vAY + aAY;
      vBX <= vBX + aBX;  vBY <= vBY + aBY;
      vCX <= vCX + aCX;  vCY <= vCY + aCY;

      // unsigned wrap-around (cheap). If you prefer bounce/clamp, say so.
      AX <= AX + vAX;
      AY <= AY + vAY;
      BX <= BX + vBX;
      BY <= BY + vBY;
      CX <= CX + vCX;
      CY <= CY + vCY;
    end
  end

  assign R = video_active ? R_pix : 2'b00;
  assign G = video_active ? G_pix : 2'b00;
  assign B = video_active ? B_pix : 2'b00;

endmodule
