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
  // VGA signals
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

  // Unused outputs assigned to 0.
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

  // once per frame
  wire frame_tick = (pix_x == 10'd0) && (pix_y == 10'd0);

  // --------------------------
  // Helper functions (gate-friendly abs / signed magnitude)
  // --------------------------
  function [10:0] abs11;
    input signed [10:0] x;
    reg s;
    begin
      s    = x[10];
      abs11 = (x ^ {11{s}}) + s;   // two's complement abs
    end
  endfunction

  function signed [3:0] signed_force4;
    input [1:0] mag;   // 0..2
    input       sign;  // 1 => negative
    reg [3:0] m4;
    begin
      m4 = {2'b00, mag};                 // 0..2 in 4 bits
      signed_force4 = (m4 ^ {4{sign}}) + sign;  // apply sign via xor+add
    end
  endfunction

  // --------------------------
  // State: planet positions and velocities (updated once per frame)
  // --------------------------
  reg  signed [9:0] AX, BX, CX;
  reg  signed [9:0] AY, BY, CY;

  reg  signed [9:0] vAX, vBX, vCX;
  reg  signed [9:0] vAY, vBY, vCY;

  reg  [9:0] counter;

  // --------------------------
  // PIXEL PATH (runs every pixel): distance-to-planet + hit tests + color
  // --------------------------
  wire signed [10:0] dxA = $signed({1'b0,pix_x}) - AX;
  wire signed [10:0] dyA = $signed({1'b0,pix_y}) - AY;
  wire signed [10:0] dxB = $signed({1'b0,pix_x}) - BX;
  wire signed [10:0] dyB = $signed({1'b0,pix_y}) - BY;
  wire signed [10:0] dxC = $signed({1'b0,pix_x}) - CX;
  wire signed [10:0] dyC = $signed({1'b0,pix_y}) - CY;

  wire [10:0] ax = abs11(dxA);
  wire [10:0] ay = abs11(dyA);
  wire [10:0] bx = abs11(dxB);
  wire [10:0] by = abs11(dyB);
  wire [10:0] cx = abs11(dxC);
  wire [10:0] cy = abs11(dyC);

  wire hitA = (ax < 11'd15) && (ay < 11'd15) && ((ax + ay) < 12'd20);
  wire hitB = (bx < 11'd15) && (by < 11'd15) && ((bx + by) < 12'd20);
  wire hitC = (cx < 11'd15) && (cy < 11'd15) && ((cx + cy) < 12'd20);

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
  // FRAME PATH (changes only when AX..CY change, i.e., once per frame)
  // Planet-to-planet deltas
  // --------------------------
  wire signed [10:0] ABx = BX - AX;
  wire signed [10:0] ABy = BY - AY;
  wire signed [10:0] ACx = CX - AX;
  wire signed [10:0] ACy = CY - AY;
  wire signed [10:0] BCx = CX - BX;
  wire signed [10:0] BCy = CY - BY;

  // Manhattan distance approximation
  wire [10:0] ABd = abs11(ABx) + abs11(ABy);
  wire [10:0] ACd = abs11(ACx) + abs11(ACy);
  wire [10:0] BCd = abs11(BCx) + abs11(BCy);

  // Force magnitudes are only 0..2 (2 bits), built from two comparisons (less muxy than nested ?:)
  wire cAB2 = (ABd < 11'd40);
  wire cAB1 = (ABd < 11'd300);
  wire [1:0] fABm = {cAB2, (cAB1 & ~cAB2)}; // 2 when <40, 1 when <300, else 0

  wire cAC2 = (ACd < 11'd40);
  wire cAC1 = (ACd < 11'd200);
  wire [1:0] fACm = {cAC2, (cAC1 & ~cAC2)}; // 2 when <40, 1 when <200, else 0

  wire cBC2 = (BCd < 11'd40);
  wire cBC1 = (BCd < 11'd200);
  wire [1:0] fBCm = {cBC2, (cBC1 & ~cBC2)}; // 2 when <40, 1 when <200, else 0

  // Signed forces (-2..+2), 4-bit signed
  wire signed [3:0] fABx_s = signed_force4(fABm, ABx[10]);
  wire signed [3:0] fABy_s = signed_force4(fABm, ABy[10]);
  wire signed [3:0] fACx_s = signed_force4(fACm, ACx[10]);
  wire signed [3:0] fACy_s = signed_force4(fACm, ACy[10]);
  wire signed [3:0] fBCx_s = signed_force4(fBCm, BCx[10]);
  wire signed [3:0] fBCy_s = signed_force4(fBCm, BCy[10]);

  // Accelerations (-4..+4), 4-bit signed
  // These match your original sign conventions but use pre-signed forces.
  wire signed [3:0] aAX =  fABx_s + fACx_s;
  wire signed [3:0] aAY =  fABy_s + fACy_s;

  wire signed [3:0] aBX = -fABx_s + fBCx_s;
  wire signed [3:0] aBY = -fABy_s + fBCy_s;

  wire signed [3:0] aCX = -fACx_s - fBCx_s;
  wire signed [3:0] aCY = -fACy_s - fBCy_s;

  // --------------------------
  // Sequential: update once per frame (keeps original nonblocking behavior)
  // --------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      AX <= 10'sd300; AY <= 10'sd150;
      BX <= 10'sd420; BY <= 10'sd300;
      CX <= 10'sd200; CY <= 10'sd20;

      vAX <= 10'sd0;  vAY <= 10'sd0;
      vBX <= 10'sd0;  vBY <= 10'sd0;
      vCX <= 10'sd0;  vCY <= 10'sd0;

      counter <= 10'd0;
    end else if (frame_tick) begin
      // velocity update
      vAX <= vAX + aAX;
      vAY <= vAY + aAY;
      vBX <= vBX + aBX;
      vBY <= vBY + aBY;
      vCX <= vCX + aCX;
      vCY <= vCY + aCY;

      // position update (uses OLD velocities, same as your original code)
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
