// Project Goal: given x, compute Tanh(x) and Sigmoid(x)
// Input Format: 32-bit, with fractional bits = 14
// Output Format: 32-bit, with fractional bits = 14



`timescale 1ns / 1ps

module cordic_activation #(
    parameter integer IO_WIDTH = 32, // width of both input and output 
    parameter integer FRAC_E = 14,  // external fractional bits 
    parameter integer FRAC_I = 24,  // internal fractional bits 
    parameter integer DW = 28,  // internal datapath width 
    parameter integer N_ROT = 22,   // number of hyperbolic CORDIC iterations
    parameter integer N_DIV = 20    // number of division CORDIC iterations
)(
    input wire clk,
    input wire rst_n,   // active-low async
    input  wire valid_in,
    input  wire signed [IO_WIDTH-1:0] x_in,
    input  wire func_select,  // 1 = tanh, 0 = sigmoid

    output wire                       valid_out,
    output wire signed [IO_WIDTH-1:0] result,   // Q(FRAC_E)
    output wire signed [DW-1:0] result_wide   // Q(FRAC_I), full internal precision
);
    localparam signed [DW-1:0] ONE_I = {{(DW-FRAC_I-1){1'b0}}, 1'b1, {FRAC_I{1'b0}}};
    
    // constant tables 
    // shift amount function for rotation stage k
    function [5:0] shift_of; input integer s; begin
        case (s)
             0: shift_of=6'd1;   1: shift_of=6'd2;   2: shift_of=6'd3;   3: shift_of=6'd4;
             4: shift_of=6'd4;   5: shift_of=6'd5;   6: shift_of=6'd6;   7: shift_of=6'd7;
             8: shift_of=6'd8;   9: shift_of=6'd9;  10: shift_of=6'd10;  11: shift_of=6'd11;
            12: shift_of=6'd12;  13: shift_of=6'd13;  14: shift_of=6'd13;  15: shift_of=6'd14;
            16: shift_of=6'd15;  17: shift_of=6'd16;  18: shift_of=6'd17;  19: shift_of=6'd18;
            20: shift_of=6'd19;  21: shift_of=6'd20;
            default: shift_of = 6'd1;
        endcase
    end endfunction
    
    // atanh 
    function signed [DW-1:0] atanh_of; input integer s; begin
        case (s)
             0: atanh_of=28'sd9215828;   1: atanh_of=28'sd4285116;   2: atanh_of=28'sd2108178;   3: atanh_of=28'sd1049945;
             4: atanh_of=28'sd1049945;   5: atanh_of=28'sd524459;   6: atanh_of=28'sd262165;   7: atanh_of=28'sd131075;
             8: atanh_of=28'sd65536;   9: atanh_of=28'sd32768;  10: atanh_of=28'sd16384;  11: atanh_of=28'sd8192;
            12: atanh_of=28'sd4096;  13: atanh_of=28'sd2048;  14: atanh_of=28'sd2048;  15: atanh_of=28'sd1024;
            16: atanh_of=28'sd512;  17: atanh_of=28'sd256;  18: atanh_of=28'sd128;  19: atanh_of=28'sd64;
            20: atanh_of=28'sd32;  21: atanh_of=28'sd16;
            default: atanh_of = 28'sd0;
        endcase
    end endfunction
    
    // stage-0: convert x_in to input into CORDIC pipeline
    // CORDIC datapath uses Q(FRAC_I), while x_in is in Q(FRAC_E)
    // sign-extend x_in and then shift left by (FRAC_I - FRAC_E) bits
    // sigmoid = 0.5*tanh(x/2) + 0.5
    //  tanh : rotate by x, sigmoid : rotate by x/2
    
    wire signed [DW-1:0] x_extend  = {{(DW-FRAC_E-2){x_in[FRAC_E+1]}}, x_in[FRAC_E+1:0]};
    wire signed [DW-1:0] x_internal = x_extend <<< (FRAC_I - FRAC_E);
    wire signed [DW-1:0] z_seed = func_select ? x_internal : (x_internal >>> 1);
    
    // hyperbolic CORDIC pipeline registers
    reg signed [DW-1:0] xr [0:N_ROT];   // CORDIC x and y values 
    reg signed [DW-1:0] yr [0:N_ROT];   
    reg signed [DW-1:0] zr [0:N_ROT];   // angle to rotate
    reg vr [0:N_ROT];   // valid signal
    reg fr [0:N_ROT];   // func-select

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xr[0] <= {DW{1'b0}};
            yr[0] <= {DW{1'b0}};
            zr[0] <= {DW{1'b0}};
            vr[0] <= 1'b0;
            fr[0] <= 1'b0;
        end else begin
            xr[0] <= ONE_I;
            yr[0] <= {DW{1'b0}};
            zr[0] <= z_seed;
            vr[0] <= valid_in;
            fr[0] <= func_select;
        end
    end
    
    // hyperbolic rotation CORDIC
    // d = (z >= 0 ? +1 : -1)
    // x' = x + d*(y >> s), y' = y + d*(x >> s)
    // z' = z - d*atanh(2^-s)
    
    genvar k;
    generate
    for (k = 0; k < N_ROT; k = k + 1) begin : g_rot
        localparam [5:0]           SH = shift_of(k);
        localparam signed [DW-1:0] AT = atanh_of(k);

        wire d_pos = ~zr[k][DW-1];
        wire signed [DW-1:0] x_sh = xr[k] >>> SH;
        wire signed [DW-1:0] y_sh = yr[k] >>> SH;

        (* use_dsp = "no" *)
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                xr[k+1] <= {DW{1'b0}};
                yr[k+1] <= {DW{1'b0}};
                zr[k+1] <= {DW{1'b0}};
                vr[k+1] <= 1'b0;
                fr[k+1] <= 1'b0;
            end else begin
                xr[k+1] <= d_pos ? (xr[k] + y_sh) : (xr[k] - y_sh);
                yr[k+1] <= d_pos ? (yr[k] + x_sh) : (yr[k] - x_sh);
                zr[k+1] <= d_pos ? (zr[k] - AT)   : (zr[k] + AT);
                vr[k+1] <= vr[k];
                fr[k+1] <= fr[k];
            end
        end
    end
    endgenerate
    
    
    // divide y by x using linear CORDIC
    
    reg signed [DW-1:0] xd [0:N_DIV];
    reg signed [DW-1:0] yd [0:N_DIV];
    reg signed [DW-1:0] qd [0:N_DIV];
    
    reg vd [0:N_DIV];
    reg fd [0:N_DIV];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xd[0] <= {DW{1'b0}};
            yd[0] <= {DW{1'b0}};
            qd[0] <= {DW{1'b0}};
            vd[0] <= 1'b0;
            fd[0] <= 1'b0;
        end else begin
            xd[0] <= xr[N_ROT];
            yd[0] <= yr[N_ROT];
            qd[0] <= {DW{1'b0}};
            
            vd[0] <= vr[N_ROT];
            fd[0] <= fr[N_ROT];
        end
    end

    genvar j;
    
    generate
    for (j = 0; j < N_DIV; j = j + 1) begin : g_div
        localparam signed [DW-1:0] QSTEP = (1 <<< (FRAC_I - (j+1)));   // 2^-(j+1) in Q(FRAC_I)

        wire d_pos = ~yd[j][DW-1];
        wire signed [DW-1:0] x_sh = xd[j] >>> (j+1);

        (* use_dsp = "no" *)
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                xd[j+1] <= {DW{1'b0}};
                yd[j+1] <= {DW{1'b0}};
                qd[j+1] <= {DW{1'b0}};
                
                vd[j+1] <= 1'b0;
                fd[j+1] <= 1'b0;
                
            end else begin
                xd[j+1] <= xd[j];
                
                yd[j+1] <= d_pos ? (yd[j] - x_sh) : (yd[j] + x_sh);
                qd[j+1] <= d_pos ? (qd[j] + QSTEP)  : (qd[j] - QSTEP);
                
                vd[j+1] <= vd[j];
                fd[j+1] <= fd[j];
            end
        end
    end
    endgenerate

    // select between tanh and sigmoid
    // sigmoid = (q + 1) / 2
    
    reg signed [DW-1:0] sel_q;
    reg                 v_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sel_q <= {DW{1'b0}};
            v_sel <= 1'b0;
        end else begin
            if (fd[N_DIV])
                sel_q <= qd[N_DIV];
            else
                sel_q <= (qd[N_DIV] + ONE_I) >>> 1;

            v_sel <= vd[N_DIV];
        end
    end


    // convert from Q(FRAC_I) to Q(FRAC_E)

    reg signed [DW-1:0] out_q;
    reg signed [DW-1:0] out_w;
    reg                 v_out;

    wire signed [DW-1:0] rnd;

    assign rnd = sel_q + (1 <<< (FRAC_I - FRAC_E - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_q <= {DW{1'b0}};
            out_w <= {DW{1'b0}};
            v_out <= 1'b0;
        end else begin
            out_q <= rnd >>> (FRAC_I - FRAC_E);
            out_w <= sel_q;
            v_out <= v_sel;
        end
    end


    // outputs

    assign result = {{(IO_WIDTH-DW){out_q[DW-1]}}, out_q};
    assign result_wide = out_w;
    assign valid_out = v_out;

endmodule     

