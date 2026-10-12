// ============================================================================
// Módulo: jacobian_point_double.v
// Descripción: Doblamiento Geométrico-Jacobiano 2P = R en Dominio Montgomery
//              para secp256k1 (a=0) con recurso compartido y reutilización de registros.
// Optimizado para: Intel Cyclone IV
// ============================================================================

module jacobian_point_double #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    output reg                  done,
    output reg                  busy,

    // Coordenadas Jacobianas de Entrada (Montgomery)
    input  wire [WIDTH-1:0]     x1, y1, z1,
    input  wire [WIDTH-1:0]     a_param, // Parámetro a=0 de secp256k1
    input  wire [WIDTH-1:0]     p,

    // Coordenadas Jacobianas de Salida
    output reg  [WIDTH-1:0]     x3, y3, z3,

    // Interfaz con el multiplicador de Montgomery compartido
    output wire                 mult_start,
    output wire [WIDTH-1:0]     mult_a,
    output wire [WIDTH-1:0]     mult_b,
    input  wire                 mult_done,
    input  wire [WIDTH-1:0]     mult_result
);

    // ------------------------------------------------------------------------
    // Estados de la FSM
    // ------------------------------------------------------------------------
    localparam ST_IDLE          = 5'd0;
    localparam ST_CHECK_INF     = 5'd1;
    localparam ST_XX            = 5'd2;
    localparam ST_YY            = 5'd3;
    localparam ST_YYYY          = 5'd4;
    localparam ST_M_PREP        = 5'd5;
    localparam ST_M_FINAL       = 5'd6;
    localparam ST_X1_YY         = 5'd7;
    localparam ST_S_PREP        = 5'd8;
    localparam ST_M2            = 5'd9;
    localparam ST_X3            = 5'd10;
    localparam ST_Y1_Z1         = 5'd11;
    localparam ST_Z3            = 5'd12;
    localparam ST_M_S_X3        = 5'd13;
    localparam ST_8YYYY_1       = 5'd14;
    localparam ST_8YYYY_2       = 5'd15;
    localparam ST_8YYYY_3       = 5'd16;
    localparam ST_Y3            = 5'd17;
    localparam ST_MULT_WAIT     = 5'd18;
    localparam ST_DONE          = 5'd19;

    reg [4:0] state, next_state;

    // Solo 5 Registros Temporales de 256 bits
    reg [WIDTH-1:0] t1, t2, t3;
    reg [WIDTH-1:0] m_reg, s_reg;

    // Control FSM del Multiplicador
    reg             fsm_mult_start;
    reg [WIDTH-1:0] fsm_mult_a;
    reg [WIDTH-1:0] fsm_mult_b;

    assign mult_start = fsm_mult_start;
    assign mult_a     = fsm_mult_a;
    assign mult_b     = fsm_mult_b;

    // ------------------------------------------------------------------------
    // Unidad de Suma / Resta Modular Única Compartida (Evita Adders Duplicados)
    // ------------------------------------------------------------------------
    reg  [WIDTH-1:0] add_in_a, add_in_b;
    wire [WIDTH-1:0] add_out;
    reg  [WIDTH-1:0] sub_in_a, sub_in_b;
    wire [WIDTH-1:0] sub_out;

    wire [WIDTH:0] add_sum  = add_in_a + add_in_b;
    assign add_out = (add_sum >= p) ? (add_sum - p) : add_sum[WIDTH-1:0];

    wire [WIDTH:0] sub_diff = (sub_in_a >= sub_in_b) ? (sub_in_a - sub_in_b) : ((sub_in_a + p) - sub_in_b);
    assign sub_out = sub_diff[WIDTH-1:0];

    // ------------------------------------------------------------------------
    // FSM de Secuenciación Geométrica
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            next_state     <= ST_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            fsm_mult_start <= 1'b0;
            fsm_mult_a     <= {WIDTH{1'b0}};
            fsm_mult_b     <= {WIDTH{1'b0}};
            x3             <= {WIDTH{1'b0}};
            y3             <= {WIDTH{1'b0}};
            z3             <= {WIDTH{1'b0}};
            t1             <= {WIDTH{1'b0}};
            t2             <= {WIDTH{1'b0}};
            t3             <= {WIDTH{1'b0}};
            m_reg          <= {WIDTH{1'b0}};
            s_reg          <= {WIDTH{1'b0}};
            add_in_a       <= {WIDTH{1'b0}};
            add_in_b       <= {WIDTH{1'b0}};
            sub_in_a       <= {WIDTH{1'b0}};
            sub_in_b       <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done           <= 1'b0;
                    fsm_mult_start <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_CHECK_INF;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                // Evaluar caso especial Z1 = 0
                ST_CHECK_INF: begin
                    if (z1 == {WIDTH{1'b0}}) begin
                        x3    <= {WIDTH{1'b0}};
                        y3    <= {WIDTH{1'b0}};
                        z3    <= {WIDTH{1'b0}};
                        state <= ST_DONE;
                    end else begin
                        // XX = X1^2
                        fsm_mult_a     <= x1;
                        fsm_mult_b     <= x1;
                        fsm_mult_start <= 1'b1;
                        next_state     <= ST_XX;
                        state          <= ST_MULT_WAIT;
                    end
                end

                // t1 = XX = X1^2
                ST_XX: begin
                    t1             <= mult_result;
                    // YY = Y1^2
                    fsm_mult_a     <= y1;
                    fsm_mult_b     <= y1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_YY;
                    state          <= ST_MULT_WAIT;
                end

                // t2 = YY = Y1^2
                ST_YY: begin
                    t2             <= mult_result;
                    // YYYY = YY^2
                    fsm_mult_a     <= mult_result;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_YYYY;
                    state          <= ST_MULT_WAIT;
                end

                // t3 = YYYY = YY^2
                ST_YYYY: begin
                    t3       <= mult_result;
                    // M = 3 * XX = (XX + XX) + XX
                    add_in_a <= t1;
                    add_in_b <= t1;
                    state    <= ST_M_PREP;
                end

                // Calcular 2 * XX y preparar M
                ST_M_PREP: begin
                    add_in_a <= add_out; // 2 * XX
                    add_in_b <= t1;      // + XX
                    state    <= ST_M_FINAL;
                end

                // m_reg = M = 3 * XX
                ST_M_FINAL: begin
                    m_reg          <= add_out;
                    // X1 * YY
                    fsm_mult_a     <= x1;
                    fsm_mult_b     <= t2; // t2 = YY
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_X1_YY;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = X1 * YY
                ST_X1_YY: begin
                    // S = 4 * X1 * YY = 2 * (2 * X1 * YY)
                    add_in_a <= mult_result;
                    add_in_b <= mult_result;
                    state    <= ST_S_PREP;
                end

                // s_reg = S = 4 * X1 * YY
                ST_S_PREP: begin
                    add_in_a       <= add_out; // 2 * X1 * YY
                    add_in_b       <= add_out; // -> 4 * X1 * YY
                    s_reg          <= add_out; // Guardar 2 * X1 * YY
                    // M^2
                    fsm_mult_a     <= m_reg;
                    fsm_mult_b     <= m_reg;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_M2;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = M^2
                ST_M2: begin
                    s_reg    <= add_out; // S final = 4 * X1 * YY
                    // 2 * S
                    add_in_a <= add_out;
                    add_in_b <= add_out;
                    sub_in_a <= mult_result; // M^2
                    sub_in_b <= add_out;     // 2 * S
                    state    <= ST_X3;
                end

                // x3 = X3 = M^2 - 2S
                ST_X3: begin
                    x3             <= sub_out;
                    // Y1 * Z1
                    fsm_mult_a     <= y1;
                    fsm_mult_b     <= z1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_Y1_Z1;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Y1 * Z1
                ST_Y1_Z1: begin
                    // Z3 = 2 * Y1 * Z1
                    add_in_a <= mult_result;
                    add_in_b <= mult_result;
                    // Prepara S - X3
                    sub_in_a <= s_reg;
                    sub_in_b <= x3;
                    state    <= ST_Z3;
                end

                // z3 = Z3
                ST_Z3: begin
                    z3             <= add_out;
                    t2             <= sub_out; // t2 = S - X3
                    // M * (S - X3)
                    fsm_mult_a     <= m_reg;
                    fsm_mult_b     <= sub_out;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_M_S_X3;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = M * (S - X3)
                ST_M_S_X3: begin
                    t1       <= mult_result; // t1 = M * (S - X3)
                    // Paso 1: 2 * YYYY (t3 = YYYY)
                    add_in_a <= t3;
                    add_in_b <= t3;
                    state    <= ST_8YYYY_1;
                end

                // Paso 2: 4 * YYYY
                ST_8YYYY_1: begin
                    add_in_a <= add_out;
                    add_in_b <= add_out;
                    state    <= ST_8YYYY_2;
                end

                // Paso 3: 8 * YYYY
                ST_8YYYY_2: begin
                    add_in_a <= add_out;
                    add_in_b <= add_out;
                    state    <= ST_8YYYY_3;
                end

                // Preparar Y3 = M * (S - X3) - 8 * YYYY
                ST_8YYYY_3: begin
                    sub_in_a <= t1;      // M * (S - X3)
                    sub_in_b <= add_out; // 8 * YYYY
                    state    <= ST_Y3;
                end

                // y3 = Y3 Final
                ST_Y3: begin
                    y3    <= sub_out;
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