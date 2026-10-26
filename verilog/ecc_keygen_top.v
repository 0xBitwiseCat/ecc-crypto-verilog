// ============================================================================
// Módulo: ecc_keygen_top.v
// Descripción: Generador de llave pública secp256k1: Q = k*G.
//                - ecc_sliding_window_engine : recorre k (ventana w = 6)
//                - jacobian_point_op         : procesador de puntos con banco
//                                              de registros en M9K
//                - montgomery_multiplier     : multiplicador CIOS de 16 bits
//              La tabla 1G..63G está precargada en el banco de registros
//              (generada por golden_model/gen_regfile.py).
// Escalar: k_scalar se captura en el flanco de start.
//          k = 0 o k = n producen el punto en el infinito -> Q = (0, 0).
//          k > n produce (k mod n)*G.
// Optimizado para: Intel Cyclone IV (EP4CE6E22C8)
// ============================================================================

module ecc_keygen_top #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control Principal
    input  wire                 start,
    input  wire [WIDTH-1:0]     k_scalar,
    output wire                 done,

    // Salida Final: Coordenadas Afines Normales Q = (x, y)
    output wire [WIDTH-1:0]     public_key_x,
    output wire [WIDTH-1:0]     public_key_y
);

    // ------------------------------------------------------------------------
    // Parámetros de la Curva secp256k1
    // ------------------------------------------------------------------------
    localparam [WIDTH-1:0] SECP256K1_P = 256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F;
    localparam [15:0]      N_PRIME0    = 16'h3531; // (-P^-1) mod 2^16 para secp256k1

    // ------------------------------------------------------------------------
    // Interconexión
    // ------------------------------------------------------------------------
    wire                   op_start;
    wire [1:0]             op_cmd;
    wire [WINDOW_SIZE-2:0] op_tbl_idx;
    wire                   op_done;
    wire                   op_busy;
    wire                   sw_busy;

    wire                   mult_start;
    wire                   mult_done;
    wire                   mult_busy;
    wire [WIDTH-1:0]       mult_a, mult_b;
    wire [WIDTH-1:0]       mult_result;

    // 1. Control de la ventana deslizante
    ecc_sliding_window_engine #(
        .WIDTH(WIDTH), .WINDOW_SIZE(WINDOW_SIZE)
    ) sw_engine_inst (
        .clk(clk), .rst_n(rst_n),
        .start(start), .done(done), .busy(sw_busy),
        .k_scalar(k_scalar),
        .op_start(op_start), .op_cmd(op_cmd), .op_tbl_idx(op_tbl_idx), .op_done(op_done)
    );

    // 2. Procesador de puntos (banco de registros en M9K)
    jacobian_point_op #(
        .WIDTH(WIDTH)
    ) point_op_inst (
        .clk(clk), .rst_n(rst_n),
        .start(op_start), .cmd(op_cmd), .tbl_idx(op_tbl_idx),
        .done(op_done), .busy(op_busy),
        .p(SECP256K1_P),
        .pub_x(public_key_x), .pub_y(public_key_y),
        .mult_start(mult_start), .mult_a(mult_a), .mult_b(mult_b),
        .mult_done(mult_done), .mult_result(mult_result)
    );

    // 3. Multiplicador de Montgomery
    montgomery_multiplier #(
        .WIDTH(WIDTH)
    ) mont_mult_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(mult_start),
        .done(mult_done),
        .busy(mult_busy),
        .a(mult_a),
        .b(mult_b),
        .p(SECP256K1_P),
        .n_prime0(N_PRIME0),
        .result(mult_result)
    );

endmodule
