// ============================================================================
// Módulo: domain_coordinate_converter.v
// Descripción: Conversión de Coordenadas Jacobianas Montgomery (X, Y, Z) 
//              a Coordenadas Afines Normales (Qx, Qy) para secp256k1.
// Optimizado para: Intel Cyclone IV (Reutilización de registros de 256 bits)
// ============================================================================

module domain_coordinate_converter #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    output reg                  done,
    output reg                  busy,

    // Entradas en coordenadas Jacobianas Montgomery
    input  wire [WIDTH-1:0]     x_in,
    input  wire [WIDTH-1:0]     y_in,
    input  wire [WIDTH-1:0]     z_in,
    input  wire [WIDTH-1:0]     p,

    // Salidas en coordenadas Afines Normales
    output reg  [WIDTH-1:0]     x_out,
    output reg  [WIDTH-1:0]     y_out,

    // Interfaz con el multiplicador de Montgomery compartido
    output wire                 mult_start,
    output wire [WIDTH-1:0]     mult_a,
    output wire [WIDTH-1:0]     mult_b,
    input  wire                 mult_done,
    input  wire [WIDTH-1:0]     mult_result
);

    // Constante R^3 mod p para secp256k1 (usada para convertir z_inv a dominio Montgomery)
    localparam [WIDTH-1:0] R3_MOD_P = 256'h100000B7300000B7300000001;

    // ------------------------------------------------------------------------
    // Codificación de Estados de la FSM
    // ------------------------------------------------------------------------
    localparam ST_IDLE        = 4'd0;
    localparam ST_CHECK_Z     = 4'd1;
    localparam ST_WAIT_INV    = 4'd2;
    localparam ST_CALC_ZINV_M = 4'd3;
    localparam ST_CALC_ZINV2  = 4'd4;
    localparam ST_CALC_ZINV3  = 4'd5;
    localparam ST_CALC_X_MONT = 4'd6;
    localparam ST_CALC_Y_MONT = 4'd7;
    localparam ST_CALC_X_NORM = 4'd8;
    localparam ST_CALC_Y_NORM = 4'd9;
    localparam ST_MULT_WAIT   = 4'd10;
    localparam ST_DONE        = 4'd11;

    reg [3:0] state, next_state;

    // Solo 2 registros temporales de 256 bits (reutilizados dinámicamente)
    reg [WIDTH-1:0] temp_a;
    reg [WIDTH-1:0] temp_b;

    // Control de FSM para Multiplicación
    reg             fsm_mult_start;
    reg [WIDTH-1:0] fsm_mult_a;
    reg [WIDTH-1:0] fsm_mult_b;

    // Instancia del Inversor Modular de Fermat
    reg             inv_start;
    wire            inv_done;
    wire            inv_busy;
    wire [WIDTH-1:0] z_inv_raw;
    wire            inv_mult_start;
    wire [WIDTH-1:0] inv_mult_a;
    wire [WIDTH-1:0] inv_mult_b;

    fermat_inverter #(.WIDTH(WIDTH)) fermat_inv_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(inv_start),
        .done(inv_done),
        .busy(inv_busy),
        .a(z_in),
        .p(p),
        .result(z_inv_raw),
        .mult_start(inv_mult_start),
        .mult_a(inv_mult_a),
        .mult_b(inv_mult_b),
        .mult_done(mult_done && inv_busy),
        .mult_result(mult_result)
    );

    // Mux para compartir el multiplicador de Montgomery
    assign mult_start = inv_busy ? inv_mult_start : fsm_mult_start;
    assign mult_a     = inv_busy ? inv_mult_a      : fsm_mult_a;
    assign mult_b     = inv_busy ? inv_mult_b      : fsm_mult_b;

    // ------------------------------------------------------------------------
    // Máquina de Estados Finita (FSM)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            next_state     <= ST_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            inv_start      <= 1'b0;
            fsm_mult_start <= 1'b0;
            fsm_mult_a     <= {WIDTH{1'b0}};
            fsm_mult_b     <= {WIDTH{1'b0}};
            x_out          <= {WIDTH{1'b0}};
            y_out          <= {WIDTH{1'b0}};
            temp_a         <= {WIDTH{1'b0}};
            temp_b         <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done           <= 1'b0;
                    inv_start      <= 1'b0;
                    fsm_mult_start <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_CHECK_Z;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                // Evaluar casos especiales Z = 0 o Z = 1
                ST_CHECK_Z: begin
                    if (z_in == {WIDTH{1'b0}}) begin
                        x_out <= {WIDTH{1'b0}};
                        y_out <= {WIDTH{1'b0}};
                        state <= ST_DONE;
                    end else if (z_in == 256'd1) begin
                        // Z = 1: Convertir X_in y Y_in directamente fuera de Montgomery
                        fsm_mult_a     <= x_in;
                        fsm_mult_b     <= 256'd1; // MontMult(X_mont, 1) = X_norm
                        fsm_mult_start <= 1'b1;
                        temp_b         <= y_in;   // Almacenar Y_in temporalmente
                        next_state     <= ST_CALC_X_NORM;
                        state          <= ST_MULT_WAIT;
                    end else begin
                        // Iniciar inversión modular de Z_in
                        inv_start <= 1'b1;
                        state     <= ST_WAIT_INV;
                    end
                end

                // Esperar a que el inversor de Fermat complete Z_inv
                ST_WAIT_INV: begin
                    inv_start <= 1'b0;
                    if (inv_done) begin
                        // Convertir z_inv_raw a Dominio Montgomery
                        fsm_mult_a     <= z_inv_raw;
                        fsm_mult_b     <= R3_MOD_P;
                        fsm_mult_start <= 1'b1;
                        next_state     <= ST_CALC_ZINV_M;
                        state          <= ST_MULT_WAIT;
                    end
                end

                // mult_result = Z_inv_mont
                ST_CALC_ZINV_M: begin
                    temp_a         <= mult_result; // temp_a = Z_inv_mont
                    // Z_inv2_mont = Z_inv_mont^2 mod p
                    fsm_mult_a     <= mult_result;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_CALC_ZINV2;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Z_inv2_mont
                ST_CALC_ZINV2: begin
                    temp_b         <= mult_result; // temp_b = Z_inv2_mont
                    // Z_inv3_mont = Z_inv2_mont * Z_inv_mont mod p
                    fsm_mult_a     <= mult_result;
                    fsm_mult_b     <= temp_a;      // temp_a = Z_inv_mont
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_CALC_ZINV3;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Z_inv3_mont
                ST_CALC_ZINV3: begin
                    temp_a         <= mult_result; // temp_a = Z_inv3_mont
                    // X_mont = X_in * Z_inv2_mont mod p
                    fsm_mult_a     <= x_in;
                    fsm_mult_b     <= temp_b;      // temp_b = Z_inv2_mont
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_CALC_X_MONT;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = X_mont
                ST_CALC_X_MONT: begin
                    x_out          <= mult_result; // Reutilizar x_out para guardar X_mont
                    // Y_mont = Y_in * Z_inv3_mont mod p
                    fsm_mult_a     <= y_in;
                    fsm_mult_b     <= temp_a;      // temp_a = Z_inv3_mont
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_CALC_Y_MONT;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Y_mont
                ST_CALC_Y_MONT: begin
                    temp_b         <= mult_result; // temp_b = Y_mont
                    // Convertir X_mont a Afín Normal: X_out = MontMult(X_mont, 1)
                    fsm_mult_a     <= x_out;       // x_out contiene X_mont
                    fsm_mult_b     <= 256'd1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_CALC_X_NORM;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = X_norm (Final)
                ST_CALC_X_NORM: begin
                    x_out          <= mult_result; // X_norm final guardado en x_out
                    // Convertir Y_mont a Afín Normal: Y_out = MontMult(Y_mont, 1)
                    fsm_mult_a     <= temp_b;      // temp_b contiene Y_mont
                    fsm_mult_b     <= 256'd1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_CALC_Y_NORM;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Y_norm (Final)
                ST_CALC_Y_NORM: begin
                    y_out <= mult_result;
                    state <= ST_DONE;
                end

                // Subrutina de espera para el multiplicador de Montgomery
                ST_MULT_WAIT: begin
                    fsm_mult_start <= 1'b0;
                    if (mult_done) begin
                        state <= next_state;
                    end
                end

                ST_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule