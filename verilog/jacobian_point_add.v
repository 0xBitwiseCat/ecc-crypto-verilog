// ============================================================================
// Módulo: jacobian_point_add.v
// Descripción: Adición Geométrico-Jacobiana P + Q = R en Dominio Montgomery
//              para secp256k1 con recurso compartido y reutilización de registros.
// Optimizado para: Intel Cyclone IV
// ============================================================================

module jacobian_point_add #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    output reg                  done,
    output reg                  busy,
    output reg                  is_double, // Detecta si P == Q

    // Coordenadas Jacobianas de Entrada (Montgomery)
    input  wire [WIDTH-1:0]     x1, y1, z1,
    input  wire [WIDTH-1:0]     x2, y2, z2,
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
    localparam ST_Z1_SQ         = 5'd2;
    localparam ST_Z2_SQ         = 5'd3;
    localparam ST_U1            = 5'd4;
    localparam ST_U2            = 5'd5;
    localparam ST_CALC_H        = 5'd6;
    localparam ST_Z1_CU         = 5'd7;
    localparam ST_S2            = 5'd8;
    localparam ST_Z2_CU         = 5'd9;
    localparam ST_S1            = 5'd10;
    localparam ST_CALC_R        = 5'd11;
    localparam ST_H2            = 5'd12;
    localparam ST_H3            = 5'd13;
    localparam ST_V             = 5'd14;
    localparam ST_R2            = 5'd15;
    localparam ST_CALC_X3_PREP  = 5'd16;
    localparam ST_CALC_X3_SUB2V = 5'd17;
    localparam ST_CALC_V_X3     = 5'd18;
    localparam ST_R_VX3         = 5'd19;
    localparam ST_S1_H3         = 5'd20;
    localparam ST_CALC_Y3       = 5'd21;
    localparam ST_Z1_Z2         = 5'd22;
    localparam ST_Z3            = 5'd23;
    localparam ST_MULT_WAIT     = 5'd24;
    localparam ST_DONE          = 5'd25;

    reg [4:0] state, next_state;

    // Solo 6 Registros Temporales de 256 bits
    reg [WIDTH-1:0] t1, t2, t3;
    reg [WIDTH-1:0] h_reg, r_reg, s1_reg;

    // Control FSM del Multiplicador
    reg             fsm_mult_start;
    reg [WIDTH-1:0] fsm_mult_a;
    reg [WIDTH-1:0] fsm_mult_b;

    assign mult_start = fsm_mult_start;
    assign mult_a     = fsm_mult_a;
    assign mult_b     = fsm_mult_b;

    // ------------------------------------------------------------------------
    // Unidad de Suma / Resta Modular Única Compartida (Evita Múltiples Adders)
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
            is_double      <= 1'b0;
            fsm_mult_start <= 1'b0;
            fsm_mult_a     <= {WIDTH{1'b0}};
            fsm_mult_b     <= {WIDTH{1'b0}};
            x3             <= {WIDTH{1'b0}};
            y3             <= {WIDTH{1'b0}};
            z3             <= {WIDTH{1'b0}};
            t1             <= {WIDTH{1'b0}};
            t2             <= {WIDTH{1'b0}};
            t3             <= {WIDTH{1'b0}};
            h_reg          <= {WIDTH{1'b0}};
            r_reg          <= {WIDTH{1'b0}};
            s1_reg         <= {WIDTH{1'b0}};
            add_in_a       <= {WIDTH{1'b0}};
            add_in_b       <= {WIDTH{1'b0}};
            sub_in_a       <= {WIDTH{1'b0}};
            sub_in_b       <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done           <= 1'b0;
                    is_double      <= 1'b0;
                    fsm_mult_start <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_CHECK_INF;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                // Evaluar casos especiales de Punto en el Infinito (Z=0)
                ST_CHECK_INF: begin
                    if (z1 == {WIDTH{1'b0}}) begin
                        x3    <= x2; y3 <= y2; z3 <= z2;
                        state <= ST_DONE;
                    end else if (z2 == {WIDTH{1'b0}}) begin
                        x3    <= x1; y3 <= y1; z3 <= z1;
                        state <= ST_DONE;
                    end else begin
                        // Calcular Z1^2
                        fsm_mult_a     <= z1;
                        fsm_mult_b     <= z1;
                        fsm_mult_start <= 1'b1;
                        next_state     <= ST_Z1_SQ;
                        state          <= ST_MULT_WAIT;
                    end
                end

                // t1 = Z1^2
                ST_Z1_SQ: begin
                    t1             <= mult_result;
                    // Calcular Z2^2
                    fsm_mult_a     <= z2;
                    fsm_mult_b     <= z2;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_Z2_SQ;
                    state          <= ST_MULT_WAIT;
                end

                // t2 = Z2^2
                ST_Z2_SQ: begin
                    t2             <= mult_result;
                    // U1 = X1 * Z2^2
                    fsm_mult_a     <= x1;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_U1;
                    state          <= ST_MULT_WAIT;
                end

                // t3 = U1
                ST_U1: begin
                    t3             <= mult_result;
                    // U2 = X2 * Z1^2
                    fsm_mult_a     <= x2;
                    fsm_mult_b     <= t1; // t1 contiene Z1^2
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_U2;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = U2
                ST_U2: begin
                    h_reg    <= mult_result; // h_reg = U2
                    sub_in_a <= mult_result;
                    sub_in_b <= t3;          // t3 = U1
                    state    <= ST_CALC_H;
                end

                // H = U2 - U1
                ST_CALC_H: begin
                    h_reg          <= sub_out; // h_reg = H
                    // Z1^3 = Z1 * Z1^2
                    fsm_mult_a     <= z1;
                    fsm_mult_b     <= t1;      // t1 contiene Z1^2
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_Z1_CU;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Z1^3
                ST_Z1_CU: begin
                    t1             <= mult_result;
                    // S2 = Y2 * Z1^3
                    fsm_mult_a     <= y2;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_S2;
                    state          <= ST_MULT_WAIT;
                end

                // r_reg = S2
                ST_S2: begin
                    r_reg          <= mult_result;
                    // Z2^3 = Z2 * Z2^2
                    fsm_mult_a     <= z2;
                    fsm_mult_b     <= t2;      // t2 contiene Z2^2
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_Z2_CU;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Z2^3
                ST_Z2_CU: begin
                    t2             <= mult_result;
                    // S1 = Y1 * Z2^3
                    fsm_mult_a     <= y1;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_S1;
                    state          <= ST_MULT_WAIT;
                end

                // s1_reg = S1
                ST_S1: begin
                    s1_reg   <= mult_result;
                    sub_in_a <= r_reg;    // r_reg = S2
                    sub_in_b <= mult_result;
                    state    <= ST_CALC_R;
                end

                // R = S2 - S1
                ST_CALC_R: begin
                    r_reg <= sub_out; // r_reg = R
                    if (h_reg == {WIDTH{1'b0}}) begin
                        if (sub_out == {WIDTH{1'b0}}) begin
                            is_double <= 1'b1; // P == Q: Se requiere Point Doubling
                        end
                        x3    <= {WIDTH{1'b0}};
                        y3    <= {WIDTH{1'b0}};
                        z3    <= {WIDTH{1'b0}};
                        state <= ST_DONE;
                    end else begin
                        // H^2 = H * H
                        fsm_mult_a     <= h_reg;
                        fsm_mult_b     <= h_reg;
                        fsm_mult_start <= 1'b1;
                        next_state     <= ST_H2;
                        state          <= ST_MULT_WAIT;
                    end
                end

                // t1 = H^2
                ST_H2: begin
                    t1             <= mult_result;
                    // H^3 = H * H^2
                    fsm_mult_a     <= h_reg;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_H3;
                    state          <= ST_MULT_WAIT;
                end

                // t2 = H^3
                ST_H3: begin
                    t2             <= mult_result;
                    // V = U1 * H^2
                    fsm_mult_a     <= t3;      // t3 = U1
                    fsm_mult_b     <= t1;      // t1 = H^2
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_V;
                    state          <= ST_MULT_WAIT;
                end

                // t1 = V
                ST_V: begin
                    t1             <= mult_result;
                    // R^2 = R * R
                    fsm_mult_a     <= r_reg;
                    fsm_mult_b     <= r_reg;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_R2;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = R^2
                ST_R2: begin
                    t3       <= mult_result;   // t3 = R^2
                    sub_in_a <= mult_result;
                    sub_in_b <= t2;            // t2 = H^3
                    add_in_a <= t1;            // t1 = V
                    add_in_b <= t1;
                    state    <= ST_CALC_X3_PREP;
                end

                // Preparar X3 = (R^2 - H^3) - 2V
                ST_CALC_X3_PREP: begin
                    x3       <= sub_out;       // x3 temp = R^2 - H^3
                    sub_in_a <= sub_out;
                    sub_in_b <= add_out;       // add_out = 2V
                    state    <= ST_CALC_X3_SUB2V;
                end

                // x3 = X3 Final
                ST_CALC_X3_SUB2V: begin
                    x3       <= sub_out;
                    sub_in_a <= t1;            // t1 = V
                    sub_in_b <= sub_out;       // sub_out = X3
                    state    <= ST_CALC_V_X3;
                end

                // t1 = V - X3
                ST_CALC_V_X3: begin
                    t1             <= sub_out;
                    // R * (V - X3)
                    fsm_mult_a     <= r_reg;
                    fsm_mult_b     <= sub_out;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_R_VX3;
                    state          <= ST_MULT_WAIT;
                end

                // t3 = R * (V - X3)
                ST_R_VX3: begin
                    t3             <= mult_result;
                    // S1 * H^3
                    fsm_mult_a     <= s1_reg;
                    fsm_mult_b     <= t2;      // t2 = H^3
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_S1_H3;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = S1 * H^3
                ST_S1_H3: begin
                    sub_in_a <= t3;            // t3 = R * (V - X3)
                    sub_in_b <= mult_result;
                    state    <= ST_CALC_Y3;
                end

                // y3 = Y3 Final
                ST_CALC_Y3: begin
                    y3             <= sub_out;
                    // Z1 * Z2
                    fsm_mult_a     <= z1;
                    fsm_mult_b     <= z2;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_Z1_Z2;
                    state          <= ST_MULT_WAIT;
                end

                // mult_result = Z1 * Z2
                ST_Z1_Z2: begin
                    // Z3 = (Z1 * Z2) * H
                    fsm_mult_a     <= mult_result;
                    fsm_mult_b     <= h_reg;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_Z3;
                    state          <= ST_MULT_WAIT;
                end

                // z3 = Z3 Final
                ST_Z3: begin
                    z3    <= mult_result;
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