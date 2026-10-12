// ============================================================================
// Módulo: ecc_keygen_top.v
// Parámetro global WINDOW_SIZE = 6
// ============================================================================

module ecc_keygen_top #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6,   // Configurado a 6 bits
    parameter WORD_SIZE   = 16   
)(
    input  wire             clk,
    input  wire             rst_n,

    input  wire             start,
    output reg              done,

    input  wire [WIDTH-1:0] k_scalar,

    output reg  [WIDTH-1:0] public_key_x,
    output reg  [WIDTH-1:0] public_key_y
);

    // Constantes de la curva secp256k1
    localparam [WIDTH-1:0] P_PARAM  = 256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F;
    localparam [WIDTH-1:0] A_PARAM  = 256'h0000000000000000000000000000000000000000000000000000000000000000;
    localparam [WIDTH-1:0] B_PARAM  = 256'h0000000000000000000000000000000000000000000000000000000000000007;
    localparam [WIDTH-1:0] GX_PARAM = 256'h79BE667EF9DCBBAC55A06295CE870B07029BFCDB2DCE28D959F2815B16F81798;
    localparam [WIDTH-1:0] GY_PARAM = 256'h483ADA7726A3C4655DA4FBFC0E1108A8FD17B448A68554199C47D08FFB10D4B8;
    localparam [WIDTH-1:0] R2_PARAM = 256'h00000001000003D1000000000000000000000000000000000000000000000000;
    localparam [15:0]      N_PRIME0 = 16'h2ECD;

    localparam ST_IDLE       = 3'b000;
    localparam ST_CONVERT    = 3'b001;
    localparam ST_PRECOMPUTE = 3'b010;
    localparam ST_SCALAR_MUL = 3'b011;
    localparam ST_CONV_AFFINE= 3'b100;
    localparam ST_MAP_FINAL  = 3'b101;
    localparam ST_DONE       = 3'b110;

    reg [2:0] state;

    // Bus de interconexión
    reg              mult_start;
    wire             mult_done, mult_busy;
    reg  [WIDTH-1:0] mult_a, mult_b;
    wire [WIDTH-1:0] mult_result;

    reg              conv_start;
    wire             conv_done, conv_busy;
    wire [WIDTH-1:0] gx_mont, gy_mont;
    wire             conv_mult_start;
    wire [WIDTH-1:0] conv_mult_a, conv_mult_b;

    reg              precomp_start;
    wire             precomp_done, precomp_busy, precomp_ram_we;
    wire [WINDOW_SIZE-2:0] precomp_ram_waddr;
    wire [WIDTH-1:0]       precomp_ram_din;

    wire [WINDOW_SIZE-2:0] ram_raddr;
    wire [WIDTH-1:0]       ram_dout;

    reg              sw_start;
    wire             sw_done, sw_busy;
    wire [WIDTH-1:0] jacob_x, jacob_y, jacob_z;

    reg              inv_start;
    wire             inv_done, inv_busy;
    wire [WIDTH-1:0] z_inv_mont;
    wire             inv_mult_start;
    wire [WIDTH-1:0] inv_mult_a, inv_mult_b;

    reg              add_start, dbl_start;
    wire             add_done, dbl_done, add_busy, dbl_busy;
    wire [WIDTH-1:0] add_x1, add_y1, add_z1, add_x2, add_y2, add_z2;
    wire [WIDTH-1:0] dbl_x1, dbl_y1, dbl_z1;
    wire [WIDTH-1:0] add_x3, add_y3, add_z3;
    wire [WIDTH-1:0] dbl_x3, dbl_y3, dbl_z3;
    wire             add_mult_start, dbl_mult_start;
    wire [WIDTH-1:0] add_mult_a, add_mult_b, dbl_mult_a, dbl_mult_b;

    // 1. Memoria RAM M9K (32 palabras de 256 bits para w=6)
    ecc_point_ram #(
        .DATA_WIDTH(WIDTH),
        .ADDR_WIDTH(WINDOW_SIZE - 1)
    ) point_ram_inst (
        .clk       (clk),
        .we        (precomp_ram_we),
        .write_addr(precomp_ram_waddr),
        .data_in   (precomp_ram_din),
        .read_addr (ram_raddr),
        .data_out  (ram_dout)
    );

    // 2. Multiplicador de Montgomery
    montgomery_multiplier #(
        .WIDTH    (WIDTH),
        .WORD_SIZE(WORD_SIZE)
    ) mont_mult_inst (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (mult_start),
        .done    (mult_done),
        .busy    (mult_busy),
        .a       (mult_a),
        .b       (mult_b),
        .p       (P_PARAM),
        .n_prime0(N_PRIME0),
        .result  (mult_result)
    );

    // 3. Conversor de Dominio
    domain_coordinate_converter #(
        .W(WIDTH)
    ) domain_conv_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (conv_start),
        .done       (conv_done),
        .busy       (conv_busy),
        .gx_affine  (GX_PARAM),
        .gy_affine  (GY_PARAM),
        .p          (P_PARAM),
        .r2_mod_p   (R2_PARAM),
        .gx_mont    (gx_mont),
        .gy_mont    (gy_mont),
        .mult_start (conv_mult_start),
        .mult_a     (conv_mult_a),
        .mult_b     (conv_mult_b),
        .mult_done  (mult_done),
        .mult_result(mult_result)
    );

    // 4. Precomputación (w = 6)
    ecc_precompute_fsm #(
        .WIDTH      (WIDTH),
        .WINDOW_SIZE(WINDOW_SIZE)
    ) precomp_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (precomp_start),
        .done       (precomp_done),
        .busy       (precomp_busy),
        .gx_mont    (gx_mont),
        .gy_mont    (gy_mont),
        .p          (P_PARAM),
        .ram_we     (precomp_ram_we),
        .ram_waddr  (precomp_ram_waddr),
        .ram_din    (precomp_ram_din)
    );

    // 5. Motor de Ventana Deslizante (w = 6)
    ecc_sliding_window_engine #(
        .WIDTH      (WIDTH),
        .WINDOW_SIZE(WINDOW_SIZE)
    ) sw_engine_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (sw_start),
        .done       (sw_done),
        .busy       (sw_busy),
        .k_scalar   (k_scalar),
        .p          (P_PARAM),
        .ram_raddr  (ram_raddr),
        .ram_dout   (ram_dout),
        .x_out      (jacob_x),
        .y_out      (jacob_y),
        .z_out      (jacob_z)
    );

    // 6. Sumador Modular
    jacobian_point_add #(
        .WIDTH(WIDTH)
    ) point_add_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (add_start),
        .done       (add_done),
        .busy       (add_busy),
        .is_double  (),
        .x1(add_x1), .y1(add_y1), .z1(add_z1),
        .x2(add_x2), .y2(add_y2), .z2(add_z2),
        .p          (P_PARAM),
        .x3(add_x3), .y3(add_y3), .z3(add_z3),
        .mult_start (add_mult_start),
        .mult_a     (add_mult_a),
        .mult_b     (add_mult_b),
        .mult_done  (mult_done),
        .mult_result(mult_result)
    );

    // 7. Duplicador Modular
    jacobian_point_double #(
        .WIDTH(WIDTH)
    ) point_dbl_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (dbl_start),
        .done       (dbl_done),
        .busy       (dbl_busy),
        .x1(dbl_x1), .y1(dbl_y1), .z1(dbl_z1),
        .a_param    (A_PARAM),
        .p          (P_PARAM),
        .x3(dbl_x3), .y3(dbl_y3), .z3(dbl_z3),
        .mult_start (dbl_mult_start),
        .mult_a     (dbl_mult_a),
        .mult_b     (dbl_mult_b),
        .mult_done  (mult_done),
        .mult_result(mult_result)
    );

    // 8. Inversor Fermat
    fermat_inverter #(
        .WIDTH(WIDTH)
    ) fermat_inv_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (inv_start),
        .done       (inv_done),
        .busy       (inv_busy),
        .a          (jacob_z),
        .p          (P_PARAM),
        .result     (z_inv_mont),
        .mult_start (inv_mult_start),
        .mult_a     (inv_mult_a),
        .mult_b     (inv_mult_b),
        .mult_done  (mult_done),
        .mult_result(mult_result)
    );

    // Árbitro de la ALU compartida
    always @(*) begin
        if (conv_busy) begin
            mult_start = conv_mult_start;
            mult_a     = conv_mult_a;
            mult_b     = conv_mult_b;
        end else if (add_busy) begin
            mult_start = add_mult_start;
            mult_a     = add_mult_a;
            mult_b     = add_mult_b;
        end else if (dbl_busy) begin
            mult_start = dbl_mult_start;
            mult_a     = dbl_mult_a;
            mult_b     = dbl_mult_b;
        end else if (inv_busy) begin
            mult_start = inv_mult_start;
            mult_a     = inv_mult_a;
            mult_b     = inv_mult_b;
        end else begin
            mult_start = 1'b0;
            mult_a     = {WIDTH{1'b0}};
            mult_b     = {WIDTH{1'b0}};
        end
    end

    // FSM Principal
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            done         <= 1'b0;
            conv_start   <= 1'b0;
            precomp_start<= 1'b0;
            sw_start     <= 1'b0;
            inv_start    <= 1'b0;
            public_key_x <= {WIDTH{1'b0}};
            public_key_y <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        conv_start <= 1'b1;
                        state      <= ST_CONVERT;
                    end
                end

                ST_CONVERT: begin
                    conv_start <= 1'b0;
                    if (conv_done) begin
                        precomp_start <= 1'b1;
                        state         <= ST_PRECOMPUTE;
                    end
                end

                ST_PRECOMPUTE: begin
                    precomp_start <= 1'b0;
                    if (precomp_done) begin
                        sw_start <= 1'b1;
                        state    <= ST_SCALAR_MUL;
                    end
                end

                ST_SCALAR_MUL: begin
                    sw_start <= 1'b0;
                    if (sw_done) begin
                        inv_start <= 1'b1;
                        state     <= ST_CONV_AFFINE;
                    end
                end

                ST_CONV_AFFINE: begin
                    inv_start <= 1'b0;
                    if (inv_done) begin
                        state <= ST_MAP_FINAL;
                    end
                end

                ST_MAP_FINAL: begin
                    public_key_x <= jacob_x;
                    public_key_y <= jacob_y;
                    state        <= ST_DONE;
                end

                ST_DONE: begin
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule