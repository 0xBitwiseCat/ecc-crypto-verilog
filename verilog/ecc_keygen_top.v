// ============================================================================
// Módulo: ecc_keygen_top.v
// Descripción: Orquestador Top-Level del Núcleo Criptográfico secp256k1.
//              Integra FSM de Precomputación, Memoria RAM de Puntos,
//              Motor de Ventana Deslizante y Convertidor Afín.
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
    output reg                  done,

    // Salida Final: Coordenadas Afines Normales Q = (x, y)
    output wire [WIDTH-1:0]     public_key_x,
    output wire [WIDTH-1:0]     public_key_y
);

    // ------------------------------------------------------------------------
    // Parámetros de la Curva secp256k1 (Dominio Montgomery)
    // ------------------------------------------------------------------------
    localparam [WIDTH-1:0] SECP256K1_P = 256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F;
    localparam [WIDTH-1:0] SECP256K1_A = 256'd0;

    // Generador G(x, y) codificado en Dominio Montgomery (G * R mod p)
    localparam [WIDTH-1:0] GX_MONT = 256'h322362135D38C87A6564B59960256BCE66164223A4D0070A647EEB23C938221D;
    localparam [WIDTH-1:0] GY_MONT = 256'hB0118D6027A1A31C2C944D76DE5C0A02701F909A670F68FE9A04B6D8D701A79B;

    // ------------------------------------------------------------------------
    // Codificación de Estados de la FSM Principal
    // ------------------------------------------------------------------------
    localparam ST_IDLE       = 3'd0;
    localparam ST_PRECOMPUTE = 3'd1;
    localparam ST_SCALAR_MULT= 3'd2;
    localparam ST_CONVERT    = 3'd3;
    localparam ST_DONE       = 3'd4;

    reg [2:0] state;

    // ------------------------------------------------------------------------
    // Señales Intermedias y Muxes
    // ------------------------------------------------------------------------
    // Control de Precomputación
    reg         precomp_start;
    wire        precomp_done;
    wire        precomp_busy;

    // RAM de Puntos Precomputados
    wire                        ram_we;
    wire [WINDOW_SIZE-2:0]      ram_waddr;
    wire [WIDTH-1:0]            ram_din_x, ram_din_y;
    wire [WINDOW_SIZE-2:0]      ram_raddr;
    wire [WIDTH-1:0]            ram_dout_x, ram_dout_y;

    // Control de Motor de Ventana Deslizante
    reg         sw_start;
    wire        sw_done;
    wire        sw_busy;
    wire [WIDTH-1:0] sw_x, sw_y, sw_z;

    // Control del Convertidor de Coordenadas
    reg         conv_start;
    wire        conv_done;
    wire        conv_busy;

    // Submódulo Multiplicador de Montgomery Compartido
    wire        mult_start;
    wire [WIDTH-1:0] mult_a, mult_b;
    wire        mult_done;
    wire [WIDTH-1:0] mult_result;

    // Submódulo Point Addition Compartido
    wire        add_start;
    wire        add_done;
    wire        add_busy;
    wire [WIDTH-1:0] add_x1, add_y1, add_z1;
    wire [WIDTH-1:0] add_x2, add_y2, add_z2;
    wire [WIDTH-1:0] add_x3, add_y3, add_z3;
    wire        add_mult_start;
    wire [WIDTH-1:0] add_mult_a, add_mult_b;

    // Submódulo Point Doubling Compartido
    wire        dbl_start;
    wire        dbl_done;
    wire        dbl_busy;
    wire [WIDTH-1:0] dbl_x1, dbl_y1, dbl_z1;
    wire [WIDTH-1:0] dbl_x3, dbl_y3, dbl_z3;
    wire        dbl_mult_start;
    wire [WIDTH-1:0] dbl_mult_a, dbl_mult_b;

    // Conversor de Coordenadas -> Multiplicador
    wire        conv_mult_start;
    wire [WIDTH-1:0] conv_mult_a, conv_mult_b;

    // Señales de Mux entre Precomputador y Motor de Ventana
    wire precomp_add_start, precomp_dbl_start;
    wire [WIDTH-1:0] precomp_add_x1, precomp_add_y1, precomp_add_z1;
    wire [WIDTH-1:0] precomp_add_x2, precomp_add_y2, precomp_add_z2;
    wire [WIDTH-1:0] precomp_dbl_x1, precomp_dbl_y1, precomp_dbl_z1;

    wire sw_add_start, sw_dbl_start;
    wire [WIDTH-1:0] sw_add_x1, sw_add_y1, sw_add_z1;
    wire [WIDTH-1:0] sw_add_x2, sw_add_y2, sw_add_z2;
    wire [WIDTH-1:0] sw_dbl_x1, sw_dbl_y1, sw_dbl_z1;

    // ------------------------------------------------------------------------
    // Asignación de Muxes para Recursos Compartidos
    // ------------------------------------------------------------------------
    // Mux para Point Addition
    assign add_start = precomp_busy ? precomp_add_start : sw_add_start;
    assign add_x1    = precomp_busy ? precomp_add_x1    : sw_add_x1;
    assign add_y1    = precomp_busy ? precomp_add_y1    : sw_add_y1;
    assign add_z1    = precomp_busy ? precomp_add_z1    : sw_add_z1;
    assign add_x2    = precomp_busy ? precomp_add_x2    : sw_add_x2;
    assign add_y2    = precomp_busy ? precomp_add_y2    : sw_add_y2;
    assign add_z2    = precomp_busy ? precomp_add_z2    : sw_add_z2;

    // Mux para Point Doubling
    assign dbl_start = precomp_busy ? precomp_dbl_start : sw_dbl_start;
    assign dbl_x1    = precomp_busy ? precomp_dbl_x1    : sw_dbl_x1;
    assign dbl_y1    = precomp_busy ? precomp_dbl_y1    : sw_dbl_y1;
    assign dbl_z1    = precomp_busy ? precomp_dbl_z1    : sw_dbl_z1;

    // Mux para Multiplicador de Montgomery
    assign mult_start = add_busy  ? add_mult_start  :
                        (dbl_busy ? dbl_mult_start  : conv_mult_start);

    assign mult_a     = add_busy  ? add_mult_a      :
                        (dbl_busy ? dbl_mult_a      : conv_mult_a);

    assign mult_b     = add_busy  ? add_mult_b      :
                        (dbl_busy ? dbl_mult_b      : conv_mult_b);

    // ------------------------------------------------------------------------
    // Instanciación de Submódulos
    // ------------------------------------------------------------------------
    // 1. Memoria RAM Dual-Port para Puntos Precomputados
    ecc_point_ram #(
        .WIDTH(WIDTH), .WINDOW_SIZE(WINDOW_SIZE)
    ) point_ram_inst (
        .clk(clk),
        .we(ram_we),
        .waddr(ram_waddr),
        .din_x(ram_din_x),
        .din_y(ram_din_y),
        .raddr(ram_raddr),
        .dout_x(ram_dout_x),
        .dout_y(ram_dout_y)
    );

    // 2. FSM de Precomputación (1G a 63G)
    ecc_precompute_fsm #(
        .WIDTH(WIDTH), .WINDOW_SIZE(WINDOW_SIZE)
    ) precomp_inst (
        .clk(clk), .rst_n(rst_n),
        .start(precomp_start), .done(precomp_done), .busy(precomp_busy),
        .gx_mont(GX_MONT), .gy_mont(GY_MONT), .p(SECP256K1_P),
        .ram_we(ram_we), .ram_waddr(ram_waddr),
        .ram_din_x(ram_din_x), .ram_din_y(ram_din_y),
        .dbl_start(precomp_dbl_start), .dbl_done(dbl_done && precomp_busy),
        .dbl_x1(precomp_dbl_x1), .dbl_y1(precomp_dbl_y1), .dbl_z1(precomp_dbl_z1),
        .dbl_x3(dbl_x3), .dbl_y3(dbl_y3), .dbl_z3(dbl_z3),
        .add_start(precomp_add_start), .add_done(add_done && precomp_busy),
        .add_x1(precomp_add_x1), .add_y1(precomp_add_y1), .add_z1(precomp_add_z1),
        .add_x2(precomp_add_x2), .add_y2(precomp_add_y2), .add_z2(precomp_add_z2),
        .add_x3(add_x3), .add_y3(add_y3), .add_z3(add_z3)
    );

    // 3. Motor de Ventana Deslizante (Multiplicación Escalar k*G)
    ecc_sliding_window_engine #(
        .WIDTH(WIDTH), .WINDOW_SIZE(WINDOW_SIZE)
    ) sw_engine_inst (
        .clk(clk), .rst_n(rst_n),
        .start(sw_start), .done(sw_done), .busy(sw_busy),
        .k_scalar(k_scalar), .p(SECP256K1_P),
        .ram_raddr(ram_raddr), .ram_dout_x(ram_dout_x), .ram_dout_y(ram_dout_y),
        .add_start(sw_add_start), .add_done(add_done && sw_busy),
        .add_x1(sw_add_x1), .add_y1(sw_add_y1), .add_z1(sw_add_z1),
        .add_x2(sw_add_x2), .add_y2(sw_add_y2), .add_z2(sw_add_z2),
        .add_x3(add_x3), .add_y3(add_y3), .add_z3(add_z3),
        .dbl_start(sw_dbl_start), .dbl_done(dbl_done && sw_busy),
        .dbl_x1(sw_dbl_x1), .dbl_y1(sw_dbl_y1), .dbl_z1(sw_dbl_z1),
        .dbl_x3(dbl_x3), .dbl_y3(dbl_y3), .dbl_z3(dbl_z3),
        .x_out(sw_x), .y_out(sw_y), .z_out(sw_z)
    );

    // 4. Operaciones Geométricas Base
    jacobian_point_add #(.WIDTH(WIDTH)) point_add_inst (
        .clk(clk), .rst_n(rst_n),
        .start(add_start), .done(add_done), .busy(add_busy), .is_double(),
        .x1(add_x1), .y1(add_y1), .z1(add_z1),
        .x2(add_x2), .y2(add_y2), .z2(add_z2),
        .p(SECP256K1_P),
        .x3(add_x3), .y3(add_y3), .z3(add_z3),
        .mult_start(add_mult_start), .mult_a(add_mult_a), .mult_b(add_mult_b),
        .mult_done(mult_done && add_busy), .mult_result(mult_result)
    );

    jacobian_point_double #(.WIDTH(WIDTH)) point_dbl_inst (
        .clk(clk), .rst_n(rst_n),
        .start(dbl_start), .done(dbl_done), .busy(dbl_busy),
        .x1(dbl_x1), .y1(dbl_y1), .z1(dbl_z1),
        .a_param(SECP256K1_A), .p(SECP256K1_P),
        .x3(dbl_x3), .y3(dbl_y3), .z3(dbl_z3),
        .mult_start(dbl_mult_start), .mult_a(dbl_mult_a), .mult_b(dbl_mult_b),
        .mult_done(mult_done && dbl_busy), .mult_result(mult_result)
    );

    // 5. Multiplicador de Montgomery Físico Único
    montgomery_multiplier #(.WIDTH(WIDTH)) mont_mult_inst (
        .clk(clk), .rst_n(rst_n),
        .start(mult_start), .done(mult_done),
        .a(mult_a), .b(mult_b), .p(SECP256K1_P),
        .result(mult_result)
    );

    // 6. Conversor Jacobianas-Montgomery a Afines Normales
    domain_coordinate_converter #(.WIDTH(WIDTH)) domain_conv_inst (
        .clk(clk), .rst_n(rst_n),
        .start(conv_start), .done(conv_done), .busy(conv_busy),
        .x_in(sw_x), .y_in(sw_y), .z_in(sw_z),
        .p(SECP256K1_P),
        .x_out(public_key_x), .y_out(public_key_y),
        .mult_start(conv_mult_start), .mult_a(conv_mult_a), .mult_b(conv_mult_b),
        .mult_done(mult_done && conv_busy), .mult_result(mult_result)
    );

    // ------------------------------------------------------------------------
    // FSM Principal de Secuenciación Criptográfica
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            done          <= 1'b0;
            precomp_start <= 1'b0;
            sw_start      <= 1'b0;
            conv_start    <= 1'b0;
        end else begin
            case (state)

                ST_IDLE: begin
                    done          <= 1'b0;
                    precomp_start <= 1'b0;
                    sw_start      <= 1'b0;
                    conv_start    <= 1'b0;
                    if (start) begin
                        precomp_start <= 1'b1;
                        state         <= ST_PRECOMPUTE;
                    end
                end

                // 1. Generar la tabla de puntos impares en RAM
                ST_PRECOMPUTE: begin
                    precomp_start <= 1'b0;
                    if (precomp_done) begin
                        sw_start <= 1'b1;
                        state    <= ST_SCALAR_MULT;
                    end
                end

                // 2. Ejecutar k*G en coordenadas Jacobianas Montgomery
                ST_SCALAR_MULT: begin
                    sw_start <= 1'b0;
                    if (sw_done) begin
                        conv_start <= 1'b1;
                        state      <= ST_CONVERT;
                    end
                end

                // 3. Convertir de Jacobianas a Coordenadas Afines Normales
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