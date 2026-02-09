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

  // positions
  reg signed [9:0] AX, AY;
  reg signed [9:0] BX, BY;
  reg signed [9:0] CX, CY;

  // velocities (10-bit, per your constraint)
  reg signed [9:0] vAX, vAY;
  reg signed [9:0] vBX, vBY;
  reg signed [9:0] vCX, vCY;

  // ---------------------------------------------------------------------------
  // Pixel-to-planet deltas (for drawing)
  // ---------------------------------------------------------------------------
  wire signed [10:0] dxA = $signed({1'b0,pix_x}) - $signed({1'b0,AX});
  wire signed [10:0] dyA = $signed({1'b0,pix_y}) - $signed({1'b0,AY});
  wire signed [10:0] dxB = $signed({1'b0,pix_x}) - $signed({1'b0,BX});
  wire signed [10:0] dyB = $signed({1'b0,pix_y}) - $signed({1'b0,BY});
  wire signed [10:0] dxC = $signed({1'b0,pix_x}) - $signed({1'b0,CX});
  wire signed [10:0] dyC = $signed({1'b0,pix_y}) - $signed({1'b0,CY});

  // abs via XOR+add1 (often cheaper than ?: mux abs)
  wire sdxA = dxA[10], sdyA = dyA[10];
  wire sdxB = dxB[10], sdyB = dyB[10];
  wire sdxC = dxC[10], sdyC = dyC[10];

  wire [10:0] ax = (dxA ^ {11{sdxA}}) + sdxA;
  wire [10:0] ay = (dyA ^ {11{sdyA}}) + sdyA;
  wire [10:0] bx = (dxB ^ {11{sdxB}}) + sdxB;
  wire [10:0] by = (dyB ^ {11{sdyB}}) + sdyB;
  wire [10:0] cx = (dxC ^ {11{sdxC}}) + sdxC;
  wire [10:0] cy = (dyC ^ {11{sdyC}}) + sdyC;

  // small “diamond-ish” bodies
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
  // Frame tick + blanking micro-scheduler for minimum-gate force updates
  // ---------------------------------------------------------------------------
  wire frame_tick = (pix_x == 10'd0) && (pix_y == 10'd0);

  // Run exactly 12 micro-steps per frame during blanking:
  // rel 0..5 = AB, AC, BA, BC, CA, CB
  // axis 0/1 = X then Y
  reg [2:0] rel;
  reg       axis;
  reg [3:0] steps_left;   // needs to count 12 -> 4 bits

  wire blanking = ~video_active;
  wire do_step  = blanking && (steps_left != 4'd0);

  // Select P (planet being updated) and Q (other) from rel (cheap muxing)
  wire [9:0] PX = (rel==3'd0 || rel==3'd1) ? AX :
                  (rel==3'd2 || rel==3'd3) ? BX : CX;
  wire [9:0] PY = (rel==3'd0 || rel==3'd1) ? AY :
                  (rel==3'd2 || rel==3'd3) ? BY : CY;

  wire [9:0] QX = (rel==3'd0) ? BX :
                  (rel==3'd1) ? CX :
                  (rel==3'd2) ? AX :
                  (rel==3'd3) ? CX :
                  (rel==3'd4) ? AX : BX;   // rel==5: CB
  wire [9:0] QY = (rel==3'd0) ? BY :
                  (rel==3'd1) ? CY :
                  (rel==3'd2) ? AY :
                  (rel==3'd3) ? CY :
                  (rel==3'd4) ? AY : BY;

  // dx,dy = Q - P
  wire signed [10:0] dx = $signed({1'b0,QX}) - $signed({1'b0,PX});
  wire signed [10:0] dy = $signed({1'b0,QY}) - $signed({1'b0,PY});
  wire sx = dx[10];
  wire sy = dy[10];

  // abs(dx), abs(dy) using XOR+add1
  wire [10:0] abs_dx = (dx ^ {11{sx}}) + sx;
  wire [10:0] abs_dy = (dy ^ {11{sy}}) + sy;

  // Manhattan distance (required for your spec)
  wire [10:0] ABd = abs_dx + abs_dy;

  // Power-of-two cutoffs: 32/64/128/256/512
  // Encode magnitude as a small integer 0..5 (no LUT mapping => cheaper)
  wire ge512 = |ABd[10:9];
  wire ge256 =  ABd[8];
  wire ge128 =  ABd[7];
  wire ge64  =  ABd[6];
  wire ge32  =  ABd[5];

  // amag in [0..5] (3 bits)
  wire [2:0] amag =
    ge512 ? 3'd5 :
    ge256 ? 3'd4 :
    ge128 ? 3'd3 :
    ge64  ? 3'd2 :
    ge32  ? 3'd1 : 3'd0;

  // Signed per-axis delta for planet P toward Q:
  // dv = (amag ^ mask) + sign, mask=all1 if negative else 0
  // Extend amag to 10-bit with leading zeros (still minimal)
  wire signed [9:0] dvx = $signed( ({7'b0, amag} ^ {10{sx}}) ) + $signed(sx);
  wire signed [9:0] dvy = $signed( ({7'b0, amag} ^ {10{sy}}) ) + $signed(sy);

  // ONE shared 10-bit adder for velocity update
  wire signed [9:0] vP_X = (rel==3'd0 || rel==3'd1) ? vAX :
                          (rel==3'd2 || rel==3'd3) ? vBX : vCX;
  wire signed [9:0] vP_Y = (rel==3'd0 || rel==3'd1) ? vAY :
                          (rel==3'd2 || rel==3'd3) ? vBY : vCY;

  wire signed [9:0] v_in  = axis ? vP_Y : vP_X;
  wire signed [9:0] dv_in = axis ? dvy  : dvx;

  wire signed [9:0] v_out = v_in + dv_in;   // <-- the ONLY velocity adder

  // ---------------------------------------------------------------------------
  // Sequential
  // ---------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      AX <= 10'sd270; AY <= 10'sd200;
      BX <= 10'sd370; BY <= 10'sd280;
      CX <= 10'sd320; CY <= 10'sd160;

      vAX <= 10'sd0; vAY <= 10'sd0;
      vBX <= 10'sd0; vBY <= 10'sd0;
      vCX <= 10'sd0; vCY <= 10'sd0;

      rel        <= 3'd0;
      axis       <= 1'b0;
      steps_left <= 4'd0;
    end else begin
      // Start a new 12-step sweep each frame
      if (frame_tick) begin
        rel        <= 3'd0;
        axis       <= 1'b0;
        steps_left <= 4'd12;

        // Position update once per frame (you need these adders regardless)
        AX <= AX + vAX;  AY <= AY + vAY;
        BX <= BX + vBX;  BY <= BY + vBY;
        CX <= CX + vCX;  CY <= CY + vCY;
      end

      // During blanking, consume steps. Only update the planet P (your rule).
      if (do_step) begin
        // write back the selected component only
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

        // advance microcode: X then Y, then next relationship
        if (!axis) begin
          axis <= 1'b1;
        end else begin
          axis <= 1'b0;
          rel  <= (rel==3'd5) ? 3'd0 : (rel + 3'd1);
        end

        steps_left <= steps_left - 4'd1;
      end
    end
  end

  // Output pixels
  assign R = video_active ? R_pix : 2'b00;
  assign G = video_active ? G_pix : 2'b00;
  assign B = video_active ? B_pix : 2'b00;

endmodule
