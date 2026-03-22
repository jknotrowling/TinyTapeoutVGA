/*
 * Copyright (c) 2024 Uri Shaked
 * SPDX-License-Identifier: Apache-2.0
 */

// Three-body gravitational simulation rendered on a 640x480 VGA display.
// Three planets (A, B, C) attract each other using a simplified gravity model
// based on Manhattan distance. Physics are computed during VGA blanking intervals
// using a time-multiplexed micro-scheduler to minimize gate count.
/*
 * Copyright (c) 2024 Uri Shaked
 * SPDX-License-Identifier: Apache-2.0
 */

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
// Renderer: grid + planets + depth (all inlined, no sub-modules)
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

  localparam [5:0] C_A  = 6'b11_10_01;
  localparam [5:0] C_B  = 6'b10_11_10;
  localparam [5:0] C_C  = 6'b01_01_11;
  localparam [5:0] C_BG = 6'b00_00_00;
  localparam [5:0] C_GR = 6'b01_01_01;

  // ---- Perspective grid (inlined) ----
  wire below_hz = pix_y > 10'd80;
  wire [9:0] dy = pix_y - 10'd80;
  wire [9:0] dxg = (pix_x >= 10'd320) ? (pix_x - 10'd320) : (10'd320 - pix_x);

  wire hline = (pix_y == 10'd80) | (pix_y == 10'd130) | (pix_y == 10'd196)
             | (pix_y == 10'd294) | (pix_y == 10'd450) | (pix_y == 10'd479);

  // Vertical line offsets: K*dy>>9 + spread
  // Need 18-bit intermediates: max K=736, max dy=399, max product=293664 < 2^18
  wire [17:0] d18 = {8'b0, dy};
  wire [17:0] pk1 = (d18<<6)+(d18<<4)+(d18<<2);                    // K=84
  wire [17:0] pk3 = (d18<<7)+(d18<<5)+(d18<<4)+(d18<<2);           // K=180
  wire [17:0] pk5 = (d18<<8)+(d18<<6)+(d18<<2)+(d18<<1)+d18;       // K=327
  wire [17:0] pk7 = (d18<<8)+(d18<<7)+(d18<<6)+(d18<<5);           // K=480
  wire [17:0] pk8 = (d18<<9)+(d18<<6)+(d18<<5);                    // K=608

  wire [9:0] o1 = pk1[17:9];
  wire [9:0] o3 = pk3[17:9];
  wire [9:0] o5 = pk5[17:9] + 10'd8;
  wire [9:0] o7 = pk7[17:9] + 10'd28;
  wire [9:0] o8 = pk8[17:9] + 10'd80;
  wire [9:0] o9 = pk8[17:9] + {2'b0, dy[9:2]} + 10'd140; // extra (dy<<7)>>9 = dy>>2

  // Hit: |dxg - offset| < 2
  wire v0 = dxg < 10'd2;
  wire v1 = ((dxg>=o1 ? dxg-o1 : o1-dxg) < 10'd2) & (dy>10'd3);
  wire v3 = ((dxg>=o3 ? dxg-o3 : o3-dxg) < 10'd2) & (dy>10'd5);
  wire v5 = ((dxg>=o5 ? dxg-o5 : o5-dxg) < 10'd2) & (dy>10'd4);
  wire v7 = ((dxg>=o7 ? dxg-o7 : o7-dxg) < 10'd2) & (dy>10'd3);
  wire v8 = ((dxg>=o8 ? dxg-o8 : o8-dxg) < 10'd2) & (dy>10'd3);
  wire v9 = ((dxg>=o9 ? dxg-o9 : o9-dxg) < 10'd2) & (dy>10'd3);

  wire grid_on = below_hz & (hline | v0 | v1 | v3 | v5 | v7 | v8 | v9);

  // ---- Planet depth sizing (4 zones, inline) ----
  wire [1:0] zA = AY[10] ? 2'd0 : AY[8:7];
  wire [1:0] zB = BY[10] ? 2'd0 : BY[8:7];
  wire [1:0] zC = CY[10] ? 2'd0 : CY[8:7];

  wire [3:0] hfA = (zA==0)?4'd5:(zA==1)?4'd8:(zA==2)?4'd11:4'd14;
  wire [4:0] dmA = (zA==0)?5'd7:(zA==1)?5'd10:(zA==2)?5'd14:5'd18;
  wire [3:0] hfB = (zB==0)?4'd5:(zB==1)?4'd8:(zB==2)?4'd11:4'd14;
  wire [4:0] dmB = (zB==0)?5'd7:(zB==1)?5'd10:(zB==2)?5'd14:5'd18;
  wire [3:0] hfC = (zC==0)?4'd5:(zC==1)?4'd8:(zC==2)?4'd11:4'd14;
  wire [4:0] dmC = (zC==0)?5'd7:(zC==1)?5'd10:(zC==2)?5'd14:5'd18;

  // ---- Planet hit tests (narrowed to 5-bit after early reject) ----
  wire signed [10:0] spx = {1'b0, pix_x};
  wire signed [10:0] spy = {1'b0, pix_y};

  wire signed [10:0] dxA = spx - AX, dyA = spy - AY;
  wire signed [10:0] dxB = spx - BX, dyB = spy - BY;
  wire signed [10:0] dxC = spx - CX, dyC = spy - CY;

  // Abs via sign flip (10-bit output, but we only care about low 5 bits if high bits=0)
  wire [9:0] ax = dxA[10] ? -dxA[9:0] : dxA[9:0];
  wire [9:0] ay = dyA[10] ? -dyA[9:0] : dyA[9:0];
  wire [9:0] bx = dxB[10] ? -dxB[9:0] : dxB[9:0];
  wire [9:0] by = dyB[10] ? -dyB[9:0] : dyB[9:0];
  wire [9:0] cx = dxC[10] ? -dxC[9:0] : dxC[9:0];
  wire [9:0] cy = dyC[10] ? -dyC[9:0] : dyC[9:0];

  // Early reject: if any high bit set, distance > 31 > max planet radius (18)
  wire bbA = ~|{ax[9:5], ay[9:5]};
  wire bbB = ~|{bx[9:5], by[9:5]};
  wire bbC = ~|{cx[9:5], cy[9:5]};

  wire [5:0] mA = ax[4:0] + ay[4:0];
  wire [5:0] mB = bx[4:0] + by[4:0];
  wire [5:0] mC = cx[4:0] + cy[4:0];

  wire hitA = bbA & (ax[4:0]<{1'b0,hfA}) & (ay[4:0]<{1'b0,hfA}) & (mA<{1'b0,dmA});
  wire hitB = bbB & (bx[4:0]<{1'b0,hfB}) & (by[4:0]<{1'b0,hfB}) & (mB<{1'b0,dmB});
  wire hitC = bbC & (cx[4:0]<{1'b0,hfC}) & (cy[4:0]<{1'b0,hfC}) & (mC<{1'b0,dmC});

  // Depth-sorted color mux
  wire afb = AY >= BY, afc = AY >= CY, bfc = BY >= CY;
  wire at = hitA & (~hitB|afb) & (~hitC|afc);
  wire bt = hitB & ~at & (~hitC|bfc);
  wire ct = hitC & ~at & ~bt;

  wire [5:0] color = at ? C_A : bt ? C_B : ct ? C_C : grid_on ? C_GR : C_BG;
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
  reg [15:0] lfsr;
  reg [3:0]  flash_ctr;
  wire flashing = |flash_ctr;
  assign flash = flashing;

  // Collision detection (inline abs, no modules)
  wire signed [10:0] cdABx = AX-BX, cdABy = AY-BY;
  wire signed [10:0] cdACx = AX-CX, cdACy = AY-CY;
  wire signed [10:0] cdBCx = BX-CX, cdBCy = BY-CY;
  wire [10:0] aABx = cdABx[10]?-cdABx:cdABx, aABy = cdABy[10]?-cdABy:cdABy;
  wire [10:0] aACx = cdACx[10]?-cdACx:cdACx, aACy = cdACy[10]?-cdACy:cdACy;
  wire [10:0] aBCx = cdBCx[10]?-cdBCx:cdBCx, aBCy = cdBCy[10]?-cdBCy:cdBCy;
  wire collide = ((aABx+aABy)<11'd12) | ((aACx+aACy)<11'd12) | ((aBCx+aBCy)<11'd12);

  // Random positions from LFSR
  wire [10:0] rAX = 11'd100+{2'b0,lfsr[8:0]>9'd440 ? 9'd440 : lfsr[8:0]};
  wire [10:0] rAY = 11'd100+{3'b0,lfsr[15:8]};
  wire [10:0] rBX = 11'd100+{2'b0,{lfsr[5:0],lfsr[15:13]}>9'd440 ? 9'd440 : {lfsr[5:0],lfsr[15:13]}};
  wire [10:0] rBY = 11'd100+{3'b0,{lfsr[3:0],lfsr[11:8]}};
  wire [10:0] rCX = 11'd100+{2'b0,{lfsr[12:7],lfsr[2:0]}>9'd440 ? 9'd440 : {lfsr[12:7],lfsr[2:0]}};
  wire [10:0] rCY = 11'd100+{3'b0,{lfsr[1:0],lfsr[13:8]}};

  // Sweep state
  reg [2:0] rel;
  reg       axis, sweep_active;
  wire do_step = blanking & sweep_active;

  wire [1:0] p_idx = (rel<3'd2)?2'd0:(rel<3'd4)?2'd1:2'd2;
  wire [1:0] q_idx = (rel==3'd0)?2'd1:(rel==3'd1)?2'd2:
                     (rel==3'd2)?2'd0:(rel==3'd3)?2'd2:
                     (rel==3'd4)?2'd0:2'd1;

  wire signed [10:0] PX=(p_idx==0)?AX:(p_idx==1)?BX:CX;
  wire signed [10:0] PY=(p_idx==0)?AY:(p_idx==1)?BY:CY;
  wire signed [10:0] QX=(q_idx==0)?AX:(q_idx==1)?BX:CX;
  wire signed [10:0] QY=(q_idx==0)?AY:(q_idx==1)?BY:CY;

  wire signed [10:0] dx = QX-PX;
  wire signed [10:0] dy = QY-PY;
  wire [9:0] adx = dx[10]?-dx[9:0]:dx[9:0];
  wire [9:0] ady = dy[10]?-dy[9:0]:dy[9:0];
  wire [10:0] manh = {1'b0,adx}+{1'b0,ady};

  wire [2:0] amag = |manh[10:8]?3'd0:manh[7]?3'd2:manh[6]?3'd4:3'd6;

  wire sx=dx[10], sy=dy[10];
  wire signed [7:0] dvx = $signed(({5'b0,amag}^{8{sx}}))+$signed({7'd0,sx});
  wire signed [7:0] dvy = $signed(({5'b0,amag}^{8{sy}}))+$signed({7'd0,sy});

  wire signed [7:0] v_in = axis?((p_idx==0)?vAY:(p_idx==1)?vBY:vCY)
                                :((p_idx==0)?vAX:(p_idx==1)?vBX:vCX);
  wire signed [7:0] dv_in = axis?dvy:dvx;
  wire signed [8:0] v_sum = {v_in[7],v_in}+{dv_in[7],dv_in};
  wire signed [7:0] v_out = (~v_sum[8]&v_sum[7])?8'sd127:
                            (v_sum[8]&~v_sum[7])?-8'sd128:v_sum[7:0];

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
      rel<=0; axis<=0; sweep_active<=0;
      flash_ctr<=0; lfsr<=16'hACE1;
    end else begin
      lfsr <= {lfsr[14:0], lfsr[15]^lfsr[14]^lfsr[12]^lfsr[3]};

      if (flashing & frame_tick) begin
        flash_ctr <= flash_ctr - 4'd1;
        if (flash_ctr == 4'd1) begin
          pAX<={rAX,3'b0}; pAY<={rAY,3'b0};
          pBX<={rBX,3'b0}; pBY<={rBY,3'b0};
          pCX<={rCX,3'b0}; pCY<={rCY,3'b0};
          vAX<={lfsr[4],lfsr[4],lfsr[4],lfsr[4:0]};
          vAY<={lfsr[12],lfsr[12],lfsr[12],lfsr[12:8]};
          vBX<={~lfsr[7],~lfsr[7],~lfsr[7],~lfsr[7:3]};
          vBY<={lfsr[2],lfsr[2],lfsr[2],lfsr[2:0],lfsr[15],lfsr[14]};
          vCX<=-$signed({lfsr[4],lfsr[4],lfsr[4],lfsr[4:0]})
               -$signed({~lfsr[7],~lfsr[7],~lfsr[7],~lfsr[7:3]});
          vCY<=-$signed({lfsr[12],lfsr[12],lfsr[12],lfsr[12:8]})
               -$signed({lfsr[2],lfsr[2],lfsr[2],lfsr[2:0],lfsr[15],lfsr[14]});
          rel<=0; axis<=0; sweep_active<=0;
        end
      end else if (~flashing) begin
        if (frame_tick & collide) flash_ctr <= 4'd8;
        if (frame_tick & ~collide) begin
          pAX<=pAX+vAXe; pAY<=pAY+vAYe;
          pBX<=pBX+vBXe; pBY<=pBY+vBYe;
          pCX<=pCX+vCXe; pCY<=pCY+vCYe;
          rel<=0; axis<=0; sweep_active<=1;
        end
        // Wrapping
        if (~frame_tick) begin
          if (AX<-11'sd40)      pAX<=pAX+14'sd5760;
          else if (AX>11'sd680) pAX<=pAX-14'sd5760;
          if (BX<-11'sd40)      pBX<=pBX+14'sd5760;
          else if (BX>11'sd680) pBX<=pBX-14'sd5760;
          if (CX<-11'sd40)      pCX<=pCX+14'sd5760;
          else if (CX>11'sd680) pCX<=pCX-14'sd5760;
          if (AY<-11'sd40)      pAY<=pAY+14'sd4480;
          else if (AY>11'sd520) pAY<=pAY-14'sd4480;
          if (BY<-11'sd40)      pBY<=pBY+14'sd4480;
          else if (BY>11'sd520) pBY<=pBY-14'sd4480;
          if (CY<-11'sd40)      pCY<=pCY+14'sd4480;
          else if (CY>11'sd520) pCY<=pCY-14'sd4480;
        end
        if (do_step) begin
          case(p_idx)
            2'd0: if(~axis) vAX<=v_out; else vAY<=v_out;
            2'd1: if(~axis) vBX<=v_out; else vBY<=v_out;
            2'd2: if(~axis) vCX<=v_out; else vCY<=v_out;
            default:;
          endcase
          if (rel==3'd5 & axis) begin
            sweep_active<=0; axis<=0; rel<=0;
          end else if (~axis) axis<=1;
          else begin axis<=0; rel<=rel+3'd1; end
        end
      end
    end
  end
endmodule
