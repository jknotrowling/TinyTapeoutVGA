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
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;
  wire video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;

  // TinyVGA PMOD
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};


  // Unused outputs assigned to 0.
  assign uio_out = 0;
  assign uio_oe  = 0;


  // Suppress unused signals warning
  wire _unused_ok = &{ena, ui_in, uio_in};


  reg [9:0] counter;


  hvsync_generator hvsync_gen(
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(video_active),
    .hpos(pix_x),
    .vpos(pix_y)
  );

  // constants (frame-static points)
  reg signed [9:0] AX, BX, CX;
  reg signed [9:0] AY, BY, CY;


  // per-frame movement (pixels per frame)
  reg signed [9:0] vAX, vBX, vCX;
  reg signed [9:0] vAY, vBY, vCY;


  // calculate pixel to planet distance
  wire signed [10:0] dxA = $signed({1'b0,pix_x}) - AX;
  wire signed [10:0] dyA = $signed({1'b0,pix_y}) - AY;
  wire signed [10:0] dxB = $signed({1'b0,pix_x}) - BX;
  wire signed [10:0] dyB = $signed({1'b0,pix_y}) - BY;
  wire signed [10:0] dxC = $signed({1'b0,pix_x}) - CX;
  wire signed [10:0] dyC = $signed({1'b0,pix_y}) - CY;

  //get absolute pixel planet distance
  wire [10:0] ax = dxA[10] ? -dxA : dxA;
  wire [10:0] ay = dyA[10] ? -dyA : dyA;
  wire [10:0] bx = dxB[10] ? -dxB : dxB;
  wire [10:0] by = dyB[10] ? -dyB : dyB;
  wire [10:0] cx = dxC[10] ? -dxC : dxC;
  wire [10:0] cy = dyC[10] ? -dyC : dyC;

  // planet to planet distance
  wire signed [10:0] ABx = BX - AX;
  wire signed [10:0] ABy = BY - AY;
  wire signed [10:0] ACx = CX - AX;
  wire signed [10:0] ACy = CY - AY;
  wire signed [10:0] BCx = CX - BX;
  wire signed [10:0] BCy = CY - BY;

  //distance approximation
  wire [10:0] ABd = (ABx[10] ? -ABx : ABx) + (ABy[10] ? -ABy : ABy);
  wire [10:0] ACd = (ACx[10] ? -ACx : ACx) + (ACy[10] ? -ACy : ACy);
  wire [10:0] BCd = (BCx[10] ? -BCx : BCx) + (BCy[10] ? -BCy : BCy);

  //force magnitude
  wire signed [9:0] fAB =
    (ABd < 11'd40) ? 10'sd2 :
    (ABd < 11'd300) ? 10'sd1 :
                    10'sd0;

  wire signed [9:0] fAC =
    (ACd < 11'd40) ? 10'sd2 :
    (ACd < 11'd200) ? 10'sd1 :
                    10'sd0;

  wire signed [9:0] fBC =
    (BCd < 11'd40) ? 10'sd2 :
    (BCd < 11'd200) ? 10'sd1 :
                    10'sd0;

  //acceleration
  wire signed [9:0] aAX =
    (ABx[10] ? -fAB : fAB) +
    (ACx[10] ? -fAC : fAC);

  wire signed [9:0] aAY =
    (ABy[10] ? -fAB : fAB) +
    (ACy[10] ? -fAC : fAC);

  wire signed [9:0] aBX =
    (ABx[10] ?  fAB : -fAB) +
    (BCx[10] ? -fBC :  fBC);

  wire signed [9:0] aBY =
    (ABy[10] ?  fAB : -fAB) +
    (BCy[10] ? -fBC :  fBC);

  wire signed [9:0] aCX =
    (ACx[10] ?  fAC : -fAC) +
    (BCx[10] ?  fBC : -fBC);

  wire signed [9:0] aCY =
    (ACy[10] ?  fAC : -fAC) +
    (BCy[10] ?  fBC : -fBC);



  //check if pixel is supposed to be planet
  wire hitA = (ax < 11'd20) && (ay < 11'd20) && ((ax + ay) < 12'd25);
  wire hitB = (bx < 11'd20) && (by < 11'd20) && ((bx + by) < 12'd25);
  wire hitC = (cx < 11'd20) && (cy < 11'd20) && ((cx + cy) < 12'd25);

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


  //update planet positions each frame
  always @(posedge vsync or negedge rst_n) begin
    if (!rst_n) begin
      AX <= 10'sd300; AY <= 10'sd150;
      BX <= 10'sd420; BY <= 10'sd300;
      CX <= 10'sd200; CY <= 10'sd20;
      counter <= 0;
      vAX <= 0; vAY <= 0;
      vBX <= 0; vBY <= 0;
      vCX <= 0; vCY <= 0;
    end else begin
      // velocity update
      vAX <= vAX + aAX;
      vAY <= vAY + aAY;
      vBX <= vBX + aBX;
      vBY <= vBY + aBY;
      vCX <= vCX + aCX;
      vCY <= vCY + aCY;

      //position update
      AX <= AX + vAX;
      AY <= AY + vAY;
      BX <= BX + vBX;
      BY <= BY + vBY;
      CX <= CX + vCX;
      CY <= CY + vCY;
      counter <= counter + 1;
    end
  end



  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};


  assign R = video_active ? R_pix : 2'b00;
  assign G = video_active ? G_pix : 2'b00;
  assign B = video_active ? B_pix : 2'b00;


 
endmodule
