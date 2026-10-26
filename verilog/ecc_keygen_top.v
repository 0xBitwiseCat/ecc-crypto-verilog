// ============================================================================
// Módulo: ecc_keygen_top.v (VERIFICADO Y OPTIMIZADO)
// Descripción: Orquestador Top-Level y Banco Central de Registros para secp256k1.
//              Gerencia el tiempo compartido del Operador Jacobiano Unificado
//              (jacobian_point_op) y del Multiplicador de Montgomery.
//              La tabla de puntos impares 1G..63G es constante (ecc_point_rom),
//              generada por golden_model/gen_point_rom.py.
// Escalar: k = 0 o k = n producen el punto en el infinito -> Q = (0, 0).
//          k > n produce (k mod n)*G.
// Optimizado para: Intel Cyclone IV (EP4CE6E22C8)
// ============================================================================

module ecc_keygen_top #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control Principal (k_scalar se captura en el mismo flanco que start)
    input  wire                 start,
    input  wire [WIDTH-1:0]     k_scalar,
    output reg                  done,

    // Salida Final: Coordenadas Afines Normales Q = (x, y)
    output wire [WIDTH-1:0]     public_key_x,
    output wire [WIDTH-1:0]     public_key_y
);

    // ------------------------------------------------------------------------
    // Parámetros de la Curva secp256k1 (Dominio Montgomery, R = 2^256)
    // ------------------------------------------------------------------------
    localparam [WIDTH-1:0] SECP256K1_P = 256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F;
    localparam [15:0]      N_PRIME0    = 16'h3531; // (-P^-1) mod 2^16 para secp256k1
    localparam [WIDTH-1:0] ONE_MONT    = 256'h1000003D1; // 1 en Montgomery = R mod p

    // ------------------------------------------------------------------------
    // Codificación de Estados de la FSM Top-Level
    // ------------------------------------------------------------------------
    localparam ST_IDLE       = 2'd0;
    localparam ST_SCALAR_MULT= 2'd1;
    localparam ST_CONVERT    = 2'd2;
    localparam ST_DONE       = 2'd3;

    reg [1:0] state;

    // ------------------------------------------------------------------------
    // Banco Central de Registros e Interconexión
    // ------------------------------------------------------------------------
    // Control FSM Principal: el motor arranca en el mismo flanco que start
    // para capturar k_scalar sin depender de que la entrada siga estable.
    wire sw_start = (state == ST_IDLE) && start;
    wire sw_done;
    wire sw_busy;
    wire [WIDTH-1:0] sw_x, sw_y, sw_z;

    reg  conv_start;
    wire conv_done;
    wire conv_busy;

    // Interfaz ROM (Puntos Precomputados)
    wire [WINDOW_SIZE-2:0]  rom_addr;
    wire [WIDTH-1:0]        rom_dout_x, rom_dout_y;

    // Señales del Motor de Ventana
    wire        sw_add_start, sw_dbl_start;
    wire [WIDTH-1:0] sw_add_x1, sw_add_y1, sw_add_z1;
    wire [WIDTH-1:0] sw_add_x2, sw_add_y2, sw_add_z2;
    wire [WIDTH-1:0] sw_dbl_x1, sw_dbl_y1, sw_dbl_z1;

    // Señales del Operador Jacobiano Unificado
    wire        op_start;
    wire        op_type;
    wire        op_done;
    wire        op_busy;
    wire [WIDTH-1:0] op_x1, op_y1, op_z1;
    wire [WIDTH-1:0] op_x2, op_y2, op_z2;
    wire [WIDTH-1:0] op_x3, op_y3, op_z3;
    wire        op_mult_start;
    wire [WIDTH-1:0] op_mult_a, op_mult_b;

    // Señales del Convertidor a Coordenadas Afines
    wire        conv_mult_start;
    wire [WIDTH-1:0] conv_mult_a, conv_mult_b;

    // Señales del Multiplicador de Montgomery Compartido
    wire        mult_start;
    wire        mult_done;
    wire        mult_busy;
    wire [WIDTH-1:0] mult_a, mult_b;
    wire [WIDTH-1:0] mult_result;

    // ------------------------------------------------------------------------
    // Multiplexado del Banco Central
    // ------------------------------------------------------------------------
    assign op_start = op_type ? sw_dbl_start : sw_add_start;
    assign op_x1    = op_type ? sw_dbl_x1    : sw_add_x1;
    assign op_y1    = op_type ? sw_dbl_y1    : sw_add_y1;
    assign op_z1    = op_type ? sw_dbl_z1    : sw_add_z1;
    assign op_x2    = sw_add_x2;
    assign op_y2    = sw_add_y2;
    assign op_z2    = sw_add_z2;

    assign mult_start = op_busy ? op_mult_start : conv_mult_start;
    assign mult_a     = op_busy ? op_mult_a     : conv_mult_a;
    assign mult_b     = op_busy ? op_mult_b     : conv_mult_b;

    // ------------------------------------------------------------------------
    // Instanciación de Submódulos
    // ------------------------------------------------------------------------

    // 1. ROM M9K de Puntos Precomputados (afines, Montgomery)
    ecc_point_rom #(
        .WIDTH(WIDTH),
        .WINDOW_SIZE(WINDOW_SIZE)
    ) point_rom_inst (
        .clk(clk),
        .addr(rom_addr),
        .dout_x(rom_dout_x),
        .dout_y(rom_dout_y)
    );

    // 2. Motor de Ventana
    ecc_sliding_window_engine #(
        .WIDTH(WIDTH), .WINDOW_SIZE(WINDOW_SIZE)
    ) sw_engine_inst (
        .clk(clk), .rst_n(rst_n),
        .start(sw_start), .done(sw_done), .busy(sw_busy),
        .op_type(op_type),
        .k_scalar(k_scalar), .p(SECP256K1_P), .one_mont(ONE_MONT),
        .ram_raddr(rom_addr), .ram_dout_x(rom_dout_x), .ram_dout_y(rom_dout_y),
        .add_start(sw_add_start), .add_done(op_done && sw_busy),
        .add_x1(sw_add_x1), .add_y1(sw_add_y1), .add_z1(sw_add_z1),
        .add_x2(sw_add_x2), .add_y2(sw_add_y2), .add_z2(sw_add_z2),
        .add_x3(op_x3), .add_y3(op_y3), .add_z3(op_z3),
        .dbl_start(sw_dbl_start), .dbl_done(op_done && sw_busy),
        .dbl_x1(sw_dbl_x1), .dbl_y1(sw_dbl_y1), .dbl_z1(sw_dbl_z1),
        .dbl_x3(op_x3), .dbl_y3(op_y3), .dbl_z3(op_z3),
        .x_out(sw_x), .y_out(sw_y), .z_out(sw_z)
    );

    // 3. Operador Jacobiano Unificado
    jacobian_point_op #(
        .WIDTH(WIDTH)
    ) point_op_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(op_start),
        .op_type(op_type),
        .done(op_done),
        .busy(op_busy),
        .x1(op_x1),
        .y1(op_y1),
        .z1(op_z1),
        .x2(op_x2),
        .y2(op_y2),
        .z2(op_z2),
        .p(SECP256K1_P),
        .x3(op_x3),
        .y3(op_y3),
        .z3(op_z3),
        .mult_start(op_mult_start),
        .mult_a(op_mult_a),
        .mult_b(op_mult_b),
        .mult_done(mult_done && op_busy),
        .mult_result(mult_result)
    );

    // 4. Multiplicador de Montgomery Compartido
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

    // 5. Conversor a Coordenadas Afines Normales
    domain_coordinate_converter #(
        .WIDTH(WIDTH)
    ) domain_conv_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(conv_start),
        .done(conv_done),
        .busy(conv_busy),
        .x_in(sw_x),
        .y_in(sw_y),
        .z_in(sw_z),
        .p(SECP256K1_P),
        .x_out(public_key_x),
        .y_out(public_key_y),
        .mult_start(conv_mult_start),
        .mult_a(conv_mult_a),
        .mult_b(conv_mult_b),
        .mult_done(mult_done && conv_busy),
        .mult_result(mult_result)
    );

    // ------------------------------------------------------------------------
    // FSM Principal
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            done          <= 1'b0;
            conv_start    <= 1'b0;
        end else begin
            case (state)

                ST_IDLE: begin
                    done          <= 1'b0;
                    conv_start    <= 1'b0;
                    if (start) begin
                        state     <= ST_SCALAR_MULT; // sw_start = start en este ciclo
                    end
                end

                ST_SCALAR_MULT: begin
                    if (sw_done) begin
                        conv_start <= 1'b1;
                        state      <= ST_CONVERT;
                    end
                end

                ST_CONVERT: begin
                    conv_start <= 1'b0;
                    if (conv_done) begin
                        state <= ST_DONE;
                    end
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
