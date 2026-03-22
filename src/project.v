/*
 * Copyright (c) 2024 Uri Shaked
 * SPDX-License-Identifier: Apache-2.0
 */

// Three-body gravitational simulation rendered on a 640x480 VGA display.
// Three planets (A, B, C) attract each other using a simplified gravity model
// based on Manhattan distance. Physics are computed during VGA blanking intervals
// using a time-multiplexed micro-scheduler to minimize gate count.

`default_nettype none

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

  wire hsync, vsync, video_active;
  wire [9:0] pix_x, pix_y;
  wire signed [10:0] AX, AY, BX, BY, CX, CY;
  wire flash;
  wire [1:0] R_pix, G_pix, B_pix;

  wire [1:0] R = ~video_active ? 2'b00 : flash ? 2'b11 : R_pix;
  wire [1:0] G = ~video_active ? 2'b00 : flash ? 2'b11 : G_pix;
  wire [1:0] B = ~video_active ? 2'b00 : flash ? 2'b11 : B_pix;

  assign uo_out  = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;
  wire _unused_ok = &{ena, ui_in, uio_in};

  wire frame_tick = (pix_x == 10'd0) & (pix_y == 10'd0);
  wire blanking   = ~video_active;

  hvsync_generator hvsync_gen(
    .clk(clk), .reset(~rst_n),
    .hsync(hsync), .vsync(vsync), .display_on(video_active),
    .hpos(pix_x), .vpos(pix_y)
  );

  gravity gravity_inst(
    .clk(clk), .rst_n(rst_n),
    .frame_tick(frame_tick), .blanking(blanking),
    .AX(AX), .AY(AY), .BX(BX), .BY(BY), .CX(CX), .CY(CY),
    .flash(flash)
  );

  renderer renderer_inst(
    .pix_x(pix_x), .pix_y(pix_y),
    .AX(AX), .AY(AY), .BX(BX), .BY(BY), .CX(CX), .CY(CY),
    .R(R_pix), .G(G_pix), .B(B_pix)
  );

endmodule


// =============================================================================
// Renderer: grid + planets + depth
// =============================================================================
module renderer (
  input  wire [9:0]        pix_x,
  input  wire [9:0]        pix_y,
  input  wire signed [10:0] AX, AY,
  input  wire signed [10:0] BX, BY,
  input  wire signed [10:0] CX, CY,
  output wire [1:0]        R,
  output wire [1:0]        G,
  output wire [1:0]        B
);

  // ---- Perspective grid ----
  wire below_hz = pix_y > 10'd80;
  wire [9:0] dy = pix_y - 10'd80;
  wire [9:0] dxg = (pix_x >= 10'd320) ? (pix_x - 10'd320) : (10'd320 - pix_x);

  // Horizontal lines
  wire hline = (pix_y == 10'd80) | (pix_y == 10'd130) | (pix_y == 10'd196)
             | (pix_y == 10'd294) | (pix_y == 10'd450) | (pix_y == 10'd479);

  // Vertical lines (v1 dropped to save area)
  wire [17:0] d18 = {8'b0, dy};

  wire [17:0] pk3 = (d18<<7)+(d18<<5)+(d18<<4)+(d18<<2);           // d18*180
  wire [17:0] pk5 = (d18<<8)+(d18<<6)+(d18<<2)+(d18<<1)+d18;       // d18*327
  wire [17:0] pk7 = (d18<<9)-(d18<<5);                             // d18*480
  wire [17:0] pk8 = pk7 + (d18<<7);                                // d18*608

  wire [9:0] o3 = {1'b0, pk3[17:9]};
  wire [9:0] o5 = {1'b0, pk5[17:9]} + 10'd8;
  wire [9:0] o7 = pk7[17:9] + 10'd28;
  wire [9:0] o8 = pk8[17:9] + 10'd80;
  wire [9:0] o9 = pk8[17:9] + {2'b0, dy[9:2]} + 10'd140;

  wire v0 = dxg < 10'd2;
  wire v3 = ((dxg>=o3 ? dxg-o3 : o3-dxg) < 10'd2) & (dy>10'd5);
  wire v5 = ((dxg>=o5 ? dxg-o5 : o5-dxg) < 10'd2) & (dy>10'd4);
  wire v7 = ((dxg>=o7 ? dxg-o7 : o7-dxg) < 10'd2) & (dy>10'd3);
  wire v8 = ((dxg>=o8 ? dxg-o8 : o8-dxg) < 10'd2) & (dy>10'd3);
  wire v9 = ((dxg>=o9 ? dxg-o9 : o9-dxg) < 10'd2) & (dy>10'd3);

  wire grid_on = below_hz & (hline | v0 | v3 | v5 | v7 | v8 | v9);

  // ---- Planet depth sizing (4 zones) ----
  wire [9:0] cyA = AY[10] ? 10'd0 : AY[9:0];
  wire [9:0] cyB = BY[10] ? 10'd0 : BY[9:0];
  wire [9:0] cyC = CY[10] ? 10'd0 : CY[9:0];

  wire [1:0] zA = cyA[8:7];
  wire [1:0] zB = cyB[8:7];
  wire [1:0] zC = cyC[8:7];

  wire [3:0] hfA = (zA==0)?4'd5:(zA==1)?4'd8:(zA==2)?4'd11:4'd14;
  wire [4:0] dmA = (zA==0)?5'd7:(zA==1)?5'd10:(zA==2)?5'd14:5'd18;
  wire [3:0] hfB = (zB==0)?4'd5:(zB==1)?4'd8:(zB==2)?4'd11:4'd14;
  wire [4:0] dmB = (zB==0)?5'd7:(zB==1)?5'd10:(zB==2)?5'd14:5'd18;
  wire [3:0] hfC = (zC==0)?4'd5:(zC==1)?4'd8:(zC==2)?4'd11:4'd14;
  wire [4:0] dmC = (zC==0)?5'd7:(zC==1)?5'd10:(zC==2)?5'd14:5'd18;

  // ---- Planet hit tests ----
  wire signed [10:0] spx = {1'b0, pix_x};
  wire signed [10:0] spy = {1'b0, pix_y};

  wire signed [10:0] dxA = spx - AX, dyA = spy - AY;
  wire signed [10:0] dxB = spx - BX, dyB = spy - BY;
  wire signed [10:0] dxC = spx - CX, dyC = spy - CY;

  wire [9:0] ax = dxA[10] ? -dxA[9:0] : dxA[9:0];
  wire [9:0] ay = dyA[10] ? -dyA[9:0] : dyA[9:0];
  wire [9:0] bx = dxB[10] ? -dxB[9:0] : dxB[9:0];
  wire [9:0] by = dyB[10] ? -dyB[9:0] : dyB[9:0];
  wire [9:0] cx = dxC[10] ? -dxC[9:0] : dxC[9:0];
  wire [9:0] cy = dyC[10] ? -dyC[9:0] : dyC[9:0];

  wire eA = ~|{ax[9:5], ay[9:5]};
  wire eB = ~|{bx[9:5], by[9:5]};
  wire eC = ~|{cx[9:5], cy[9:5]};

  wire [5:0] mA = ax[4:0] + ay[4:0];
  wire [5:0] mB = bx[4:0] + by[4:0];
  wire [5:0] mC = cx[4:0] + cy[4:0];

  wire hitA = eA & (ax[4:0]<{1'b0,hfA}) & (ay[4:0]<{1'b0,hfA}) & (mA<{1'b0,dmA});
  wire hitB = eB & (bx[4:0]<{1'b0,hfB}) & (by[4:0]<{1'b0,hfB}) & (mB<{1'b0,dmB});
  wire hitC = eC & (cx[4:0]<{1'b0,hfC}) & (cy[4:0]<{1'b0,hfC}) & (mC<{1'b0,dmC});

  // Depth-sorted color mux
  wire afb = AY >= BY, afc = AY >= CY, bfc = BY >= CY;
  wire at = hitA & (~hitB|afb) & (~hitC|afc);
  wire bt = hitB & ~at & (~hitC|bfc);
  wire ct = hitC & ~at & ~bt;

  wire [5:0] color = at      ? 6'b11_10_01 :
                     bt      ? 6'b10_11_10 :
                     ct      ? 6'b01_01_11 :
                     grid_on ? 6'b01_01_01 : 6'b00_00_00;

  assign {R, G, B} = color;

endmodule


// =============================================================================
// Gravity: physics, collision, flash, LFSR, wrapping
// =============================================================================
module gravity (
  input  wire       clk,
  input  wire       rst_n,
  input  wire       frame_tick,
  input  wire       blanking,
  output wire signed [10:0] AX, AY,
  output wire signed [10:0] BX, BY,
  output wire signed [10:0] CX, CY,
  output wire       flash
);

  reg signed [13:0] pAX, pAY, pBX, pBY, pCX, pCY;
  assign AX = pAX[13:3]; assign AY = pAY[13:3];
  assign BX = pBX[13:3]; assign BY = pBY[13:3];
  assign CX = pCX[13:3]; assign CY = pCY[13:3];

  reg signed [7:0] vAX, vAY, vBX, vBY, vCX, vCY;
  reg [7:0] lfsr;
  reg [3:0]  flash_ctr;
  reg        collide_flag;
  wire flashing = |flash_ctr;
  assign flash = flashing;

  // OPT: Replace separate rel[2:0]+axis with single step[3:0] counter
  // step[0] = axis, step[3:1] = rel, step[3:2] = p_idx
  reg [3:0]  step;
  reg        sweep_active;
  wire do_step = blanking & sweep_active;

  // OPT: p_idx = rel[2:1] = step[3:2] -- direct bit-slice, no comparators
  wire [1:0] p_idx = step[3:2];

  // OPT: q_idx via direct gate logic on step bits (replaces 5-way mux chain)
  // Derived from the pair enumeration: AB,AC,BA,BC,CA,CB
  wire [1:0] q_idx;
  assign q_idx[1] = step[1] & (~step[3] | step[2]);
  assign q_idx[0] = ~step[2] & ~(step[1] ^ step[3]);

  wire signed [10:0] PX=(p_idx==0)?AX:(p_idx==1)?BX:CX;
  wire signed [10:0] PY=(p_idx==0)?AY:(p_idx==1)?BY:CY;
  wire signed [10:0] QX=(q_idx==0)?AX:(q_idx==1)?BX:CX;
  wire signed [10:0] QY=(q_idx==0)?AY:(q_idx==1)?BY:CY;

  // Gravity + collision (shared distance)
  wire signed [10:0] dx = QX-PX;
  wire signed [10:0] dy = QY-PY;
  wire [9:0] adx = dx[10]?-dx[9:0]:dx[9:0];
  wire [9:0] ady = dy[10]?-dy[9:0]:dy[9:0];
  wire [10:0] manh = {1'b0,adx}+{1'b0,ady};

  // OPT: Replace 11-bit comparator with narrow bit checks
  // manh < 12 iff upper bits all zero AND lower nibble not 11xx
  wire pair_close = ~|manh[10:4] & ~(manh[3] & manh[2]) & ~step[0];

  wire [2:0] amag = |manh[10:8]?3'd0:manh[7]?3'd2:manh[6]?3'd4:3'd6;

  wire sx=dx[10], sy=dy[10];
  wire signed [7:0] dvx = $signed(({5'b0,amag}^{8{sx}}))+$signed({7'd0,sx});
  wire signed [7:0] dvy = $signed(({5'b0,amag}^{8{sy}}))+$signed({7'd0,sy});

  wire signed [7:0] v_in = step[0]?((p_idx==0)?vAY:(p_idx==1)?vBY:vCY)
                                   :((p_idx==0)?vAX:(p_idx==1)?vBX:vCX);
  wire signed [7:0] dv_in = step[0]?dvy:dvx;
  wire signed [8:0] v_sum = {v_in[7],v_in}+{dv_in[7],dv_in};

  // OPT: Simpler saturation using XOR overflow detect
  wire v_ov = v_sum[8] ^ v_sum[7];
  wire signed [7:0] v_out = v_ov ? {v_sum[8], {7{~v_sum[8]}}} : v_sum[7:0];

  wire signed [13:0] vAXe={{6{vAX[7]}},vAX}, vAYe={{6{vAY[7]}},vAY};
  wire signed [13:0] vBXe={{6{vBX[7]}},vBX}, vBYe={{6{vBY[7]}},vBY};
  wire signed [13:0] vCXe={{6{vCX[7]}},vCX}, vCYe={{6{vCY[7]}},vCY};

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pAX<={11'sd320,3'b0}; pAY<={11'sd160,3'b0};
      pBX<={11'sd230,3'b0}; pBY<={11'sd300,3'b0};
      pCX<={11'sd410,3'b0}; pCY<={11'sd300,3'b0};
      vAX<=8'sd48;  vAY<=8'sd0;
      vBX<=-8'sd24; vBY<=-8'sd42;
      vCX<=-8'sd24; vCY<=8'sd42;
      step<=0; sweep_active<=0;
      flash_ctr<=0; lfsr<=8'hA5;
      collide_flag<=0;
    end else begin
      lfsr <= {lfsr[6:0], lfsr[7]^lfsr[5]^lfsr[4]^lfsr[3]};

      if (flashing & frame_tick) begin
        flash_ctr <= flash_ctr - 4'd1;
        if (flash_ctr == 4'd1) begin
          pAX<={3'b0,lfsr[7:0],3'b0}+{11'd80,3'b0};
          pAY<={3'b0,lfsr[4:0],lfsr[7:5],3'b0}+{11'd100,3'b0};
          pBX<={3'b0,lfsr[3:0],lfsr[7:4],3'b0}+{11'd80,3'b0};
          pBY<={3'b0,lfsr[6:0],lfsr[7],3'b0}+{11'd100,3'b0};
          pCX<={3'b0,lfsr[2:0],lfsr[7:3],3'b0}+{11'd80,3'b0};
          pCY<={3'b0,lfsr[1:0],lfsr[7:2],3'b0}+{11'd100,3'b0};
          vAX<={lfsr[4],lfsr[4],lfsr[4],lfsr[4:0]};
          vAY<={lfsr[7],lfsr[7],lfsr[7],lfsr[7:3]};
          vBX<={~lfsr[2],~lfsr[2],~lfsr[2],~lfsr[2:0],lfsr[7],lfsr[6]};
          vBY<={lfsr[0],lfsr[0],lfsr[0],lfsr[0],lfsr[7:4]};
          vCX<=-$signed({lfsr[4],lfsr[4],lfsr[4],lfsr[4:0]})
               -$signed({~lfsr[2],~lfsr[2],~lfsr[2],~lfsr[2:0],lfsr[7],lfsr[6]});
          vCY<=-$signed({lfsr[7],lfsr[7],lfsr[7],lfsr[7:3]})
               -$signed({lfsr[0],lfsr[0],lfsr[0],lfsr[0],lfsr[7:4]});
          step<=0; sweep_active<=0;
          collide_flag<=0;
        end
      end else if (~flashing) begin
        // OPT: Merged frame_tick branches (collide vs normal)
        if (frame_tick) begin
          if (collide_flag) begin
            flash_ctr <= 4'd8;
            collide_flag <= 0;
          end else begin
            pAX<=pAX+vAXe; pAY<=pAY+vAYe;
            pBX<=pBX+vBXe; pBY<=pBY+vBYe;
            pCX<=pCX+vCXe; pCY<=pCY+vCYe;
            sweep_active<=1;
          end
          step<=0;
          collide_flag<=0;
        end
        // Wrapping
        if (~frame_tick) begin
          if (AX[10])          pAX<=pAX+14'sd5760;
          else if (AX[9])      pAX<=pAX-14'sd5760;
          if (BX[10])          pBX<=pBX+14'sd5760;
          else if (BX[9])      pBX<=pBX-14'sd5760;
          if (CX[10])          pCX<=pCX+14'sd5760;
          else if (CX[9])      pCX<=pCX-14'sd5760;
          if (AY[10])          pAY<=pAY+14'sd4480;
          else if (AY[9])      pAY<=pAY-14'sd4480;
          if (BY[10])          pBY<=pBY+14'sd4480;
          else if (BY[9])      pBY<=pBY-14'sd4480;
          if (CY[10])          pCY<=pCY+14'sd4480;
          else if (CY[9])      pCY<=pCY-14'sd4480;
        end
        // Gravity micro-steps + collision
        if (do_step) begin
          if (pair_close) collide_flag <= 1;
          case(p_idx)
            2'd0: if(~step[0]) vAX<=v_out; else vAY<=v_out;
            2'd1: if(~step[0]) vBX<=v_out; else vBY<=v_out;
            2'd2: if(~step[0]) vCX<=v_out; else vCY<=v_out;
            default:;
          endcase
          // OPT: Simple increment + terminal check replaces dual-counter FSM
          if (step == 4'd11) begin
            sweep_active<=0; step<=0;
          end else step <= step + 4'd1;
        end
      end
    end
  end
endmodule
