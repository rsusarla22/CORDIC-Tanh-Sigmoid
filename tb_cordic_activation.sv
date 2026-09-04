`timescale 1ns/1ps

module tb_cordic_activation;

    localparam integer IO_WIDTH = 32;
    localparam integer FRAC_E   = 14;
    localparam integer FRAC_I   = 24;
    localparam integer DW       = 28;

    localparam integer N_ROT    = 22;
    localparam integer N_DIV    = 20;
    localparam integer LATENCY  = N_ROT + N_DIV + 4;

    localparam integer SB       = LATENCY;
    localparam integer NPTS     = 4001;

    localparam real CLK_PERIOD  = 5.0;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    reg valid_in = 1'b0;
    reg signed [IO_WIDTH-1:0] x_in = 0;
    reg func_select = 1'b1;

    wire valid_out;
    wire signed [IO_WIDTH-1:0] result;
    wire signed [DW-1:0] result_wide;


    cordic_activation #(
        .IO_WIDTH (IO_WIDTH),
        .FRAC_E   (FRAC_E),
        .FRAC_I   (FRAC_I),
        .DW       (DW),
        .N_ROT    (N_ROT),
        .N_DIV    (N_DIV)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .valid_in    (valid_in),
        .x_in        (x_in),
        .func_select (func_select),
        .valid_out   (valid_out),
        .result      (result),
        .result_wide (result_wide)
    );


    always #(CLK_PERIOD/2.0) clk = ~clk;


    // Reference functions

    function real ref_tanh(input real x);
        real e2;
        begin
            e2 = $exp(2.0*x);
            ref_tanh = (e2 - 1.0) / (e2 + 1.0);
        end
    endfunction


    function real ref_sigmoid(input real x);
        begin
            ref_sigmoid = 1.0 / (1.0 + $exp(-x));
        end
    endfunction


    // Expected values delayed by the DUT latency

    integer exp_x [0:SB-1];
    integer exp_f [0:SB-1];
    reg     exp_v [0:SB-1];

    integer s;

    always @(posedge clk) begin
        for (s = SB-1; s > 0; s = s - 1) begin
            exp_x[s] <= exp_x[s-1];
            exp_f[s] <= exp_f[s-1];
            exp_v[s] <= exp_v[s-1];
        end

        exp_x[0] <= x_in;
        exp_f[0] <= func_select;
        exp_v[0] <= rst_n && valid_in;
    end


    real sum_err   [0:1];
    real max_err   [0:1];

    real sum_err_w [0:1];
    real max_err_w [0:1];

    integer cnt    [0:1];
    integer errors;

    real xr_v;
    real got;
    real got_w;
    real refv;
    real e;
    real e_w;

    integer fi;


    always @(posedge clk) begin
        if (rst_n) begin

            if (valid_out !== exp_v[SB-1]) begin
                $display("[FAIL] valid_out mismatch at t=%0t : got %b expected %b",
                         $time, valid_out, exp_v[SB-1]);
                errors = errors + 1;
            end


            if (valid_out && exp_v[SB-1]) begin

                fi = exp_f[SB-1];

                xr_v  = $itor(exp_x[SB-1]) / (1.0 * (1 << FRAC_E));
                got   = $itor(result)      / (1.0 * (1 << FRAC_E));
                got_w = $itor(result_wide) / (1.0 * (1 << FRAC_I));

                refv = fi ?
                       ref_tanh(xr_v) :
                       ref_sigmoid(xr_v);

                e   = (got   > refv) ? (got   - refv) : (refv - got);
                e_w = (got_w > refv) ? (got_w - refv) : (refv - got_w);

                sum_err[fi]   = sum_err[fi] + e;
                sum_err_w[fi] = sum_err_w[fi] + e_w;

                if (e > max_err[fi])
                    max_err[fi] = e;

                if (e_w > max_err_w[fi])
                    max_err_w[fi] = e_w;

                cnt[fi] = cnt[fi] + 1;


                if (e > 1.0e-3) begin
                    $display("[FAIL] x=%f func=%0d got=%f ref=%f err=%e",
                             xr_v, fi, got, refv, e);
                    errors = errors + 1;
                end

            end
        end
    end


    integer k;
    integer f;
    integer lat_measured;
    real xr_drive;


    task automatic sweep(input integer func);
        integer kk;

        begin
            for (kk = 0; kk < NPTS; kk = kk + 1) begin

                @(negedge clk);

                xr_drive = -1.0 + 2.0 * kk / (NPTS - 1);

                x_in = $rtoi(
                    xr_drive * (1 << FRAC_E) +
                    ((xr_drive >= 0.0) ? 0.5 : -0.5)
                );

                func_select = func[0];
                valid_in = 1'b1;
            end

            @(negedge clk);
            valid_in = 1'b0;
        end
    endtask


    initial begin

        for (k = 0; k < 2; k = k + 1) begin
            sum_err[k]   = 0.0;
            max_err[k]   = 0.0;
            sum_err_w[k] = 0.0;
            max_err_w[k] = 0.0;
            cnt[k]       = 0;
        end

        for (k = 0; k < SB; k = k + 1) begin
            exp_x[k] = 0;
            exp_f[k] = 0;
            exp_v[k] = 1'b0;
        end

        errors = 0;


        // Reset

        rst_n = 1'b0;
        repeat (5) @(posedge clk);

        @(negedge clk);
        rst_n = 1'b1;

        repeat (2) @(posedge clk);


        // Check latency

        @(negedge clk);

        x_in        = 32'sd8192;
        func_select = 1'b1;
        valid_in    = 1'b1;

        @(negedge clk);
        valid_in = 1'b0;


        lat_measured = 1;

        while (!valid_out) begin
            @(negedge clk);
            lat_measured = lat_measured + 1;
        end


        $display(
            "Measured latency (valid_in -> valid_out) = %0d clocks (expected %0d)",
            lat_measured,
            LATENCY
        );


        if (lat_measured != LATENCY) begin
            $display("[FAIL] latency mismatch");
            errors = errors + 1;
        end


        repeat (LATENCY + 5) @(posedge clk);


        // Reset statistics after the latency test

        for (k = 0; k < 2; k = k + 1) begin
            sum_err[k]   = 0.0;
            max_err[k]   = 0.0;
            sum_err_w[k] = 0.0;
            max_err_w[k] = 0.0;
            cnt[k]       = 0;
        end


        // Back-to-back inputs

        sweep(1);

        repeat (LATENCY + 5) @(posedge clk);

        sweep(0);

        repeat (LATENCY + 5) @(posedge clk);


        // Gapped inputs

        for (k = 0; k < 64; k = k + 1) begin

            @(negedge clk);

            xr_drive = -1.0 + 2.0 * k / 63.0;

            x_in = $rtoi(
                xr_drive * (1 << FRAC_E) +
                ((xr_drive >= 0.0) ? 0.5 : -0.5)
            );

            func_select = k[0];
            valid_in = 1'b1;

            @(negedge clk);
            valid_in = 1'b0;

            @(negedge clk);
        end


        repeat (LATENCY + 10) @(posedge clk);


        $display("");
        $display("=========================================================================");
        $display(" cordic_activation accuracy over x in [-1, +1]");
        $display("=========================================================================");

        for (f = 1; f >= 0; f = f - 1) begin

            $display(" %s   (%0d samples checked)",
                     f ? "tanh   " : "sigmoid",
                     cnt[f]);

            $display(
                "   Q%0d output : MAE = %e   MAX = %e   (MAE = %0.3f LSB)",
                FRAC_E,
                sum_err[f] / cnt[f],
                max_err[f],
                (sum_err[f] / cnt[f]) * (1 << FRAC_E)
            );

            $display(
                "   Q%0d output : MAE = %e   MAX = %e   (MAE = %0.3f LSB)",
                FRAC_I,
                sum_err_w[f] / cnt[f],
                max_err_w[f],
                (sum_err_w[f] / cnt[f]) * (1 << FRAC_I)
            );

        end


        $display("=========================================================================");

        if (errors == 0)
            $display(" RESULT: PASS  (0 errors)");
        else
            $display(" RESULT: FAIL  (%0d errors)", errors);

        $display("=========================================================================");

        $finish;

    end

endmodule
