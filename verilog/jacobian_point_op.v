// ============================================================================
// Módulo: jacobian_point_op.v
// Descripción: Operador Geométrico-Jacobiano Unificado (Adición y Doblamiento)
//              para secp256k1 (a=0) en Dominio Montgomery.
// Operación: op_type = 0 -> R = P + Q (Point Addition)
//            op_type = 1 -> R = 2P    (Point Doubling)
// Optimizado para: Intel Cyclone IV
// ============================================================================

module jacobian_point_op #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    input  wire                 op_type,   // 0: Addition (P + Q), 1: Doubling (2P)
    output reg                  done,
    output reg                  busy,
    output reg                  is_double, // Detecta P == Q durante Adición

    // Coordenadas Jacobianas de Entrada (Montgomery)
    input  wire [WIDTH-1:0]     x1, y1, z1,
    input  wire [WIDTH-1:0]     x2, y2, z2, // Requerido solo para op_type = 0
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
    // Codificación de Estados de la FSM
    // ------------------------------------------------------------------------
    localparam ST_IDLE          = 6'd0;
    localparam ST_CHECK_INF     = 6'd1;

    // Estados de Doblamiento (op_type == 1)
    localparam ST_DBL_XX        = 6'd2;
    localparam ST_DBL_YY        = 6'd3;
    localparam ST_DBL_YYYY      = 6'd4;
    localparam ST_DBL_M_PREP    = 6'd5;
    localparam ST_DBL_M_FINAL   = 6'd6;
    localparam ST_DBL_X1_YY     = 6'd7;
    localparam ST_DBL_S_PREP    = 6'd8;
    localparam ST_DBL_M2        = 6'd9;
    localparam ST_DBL_X3        = 6'd10;
    localparam ST_DBL_Y1_Z1     = 6'd11;
    localparam ST_DBL_Z3        = 6'd12;
    localparam ST_DBL_M_S_X3    = 6'd13;
    localparam ST_DBL_8YYYY_1   = 6'd14;
    localparam ST_DBL_8YYYY_2   = 6'd15;
    localparam ST_DBL_8YYYY_3   = 6'd16;
    localparam ST_DBL_Y3        = 6'd17;

    // Estados de Adición (op_type == 0)
    localparam ST_ADD_Z1_SQ     = 6'd18;
    localparam ST_ADD_Z2_SQ     = 6'd19;
    localparam ST_ADD_U1        = 6'd20;
    localparam ST_ADD_U2        = 6'd21;
    localparam ST_ADD_CALC_H    = 6'd22;
    localparam ST_ADD_Z1_CU     = 6'd23;
    localparam ST_ADD_S2        = 6'd24;
    localparam ST_ADD_Z2_CU     = 6'd25;
    localparam ST_ADD_S1        = 6'd26;
    localparam ST_ADD_CALC_R    = 6'd27;
    localparam ST_ADD_H2        = 6'd28;
    localparam ST_ADD_H3        = 6'd29;
    localparam ST_ADD_V         = 6'd30;
    localparam ST_ADD_R2        = 6'd31;
    localparam ST_ADD_X3_PREP   = 6'd32;
    localparam ST_ADD_X3_SUB2V  = 6'd33;
    localparam ST_ADD_V_X3      = 6'd34;
    localparam ST_ADD_R_VX3     = 6'd35;
    localparam ST_ADD_S1_H3     = 6'd36;
    localparam ST_ADD_Y3        = 6'd37;
    localparam ST_ADD_Z1_Z2     = 6'd38;
    localparam ST_ADD_Z3        = 6'd39;

    localparam ST_MULT_WAIT     = 6'd40;
    localparam ST_DONE          = 6'd41;

    reg [5:0] state, next_state;

    // Banco de Registros Temporales Compartidos (Solo 6 registros de 256 bits)
    reg [WIDTH-1:0] t1, t2, t3;
    reg [WIDTH-1:0] reg_a, reg_b, reg_c;

    // Control FSM del Multiplicador
    reg             fsm_mult_start;
    reg [WIDTH-1:0] fsm_mult_a;
    reg [WIDTH-1:0] fsm_mult_b;

    assign mult_start = fsm_mult_start;
    assign mult_a     = fsm_mult_a;
    assign mult_b     = fsm_mult_b;

    // ------------------------------------------------------------------------
    // Unidad Aritmética Modular Única Compartida (Suma / Resta)
    // Se emplean señales intermedias para evitar truncamientos e incompatibilidad
    // de slicing directo sobre operaciones en Quartus.
    // ------------------------------------------------------------------------
    reg  [WIDTH-1:0] add_in_a, add_in_b;
    wire [WIDTH-1:0] add_out;
    reg  [WIDTH-1:0] sub_in_a, sub_in_b;
    wire [WIDTH-1:0] sub_out;

    wire [WIDTH:0] add_sum = add_in_a + add_in_b;
    wire [WIDTH:0] add_sub = add_sum - p;
    assign add_out = (add_sum >= p) ? add_sub[WIDTH-1:0] : add_sum[WIDTH-1:0];

    wire [WIDTH:0] sub_diff1  = sub_in_a - sub_in_b;
    wire [WIDTH:0] sub_plus_p = sub_in_a + p;
    wire [WIDTH:0] sub_diff2  = sub_plus_p - sub_in_b;
    wire [WIDTH:0] sub_diff   = (sub_in_a >= sub_in_b) ? sub_diff1 : sub_diff2;
    assign sub_out = sub_diff[WIDTH-1:0];

    // ------------------------------------------------------------------------
    // FSM de Control
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
            reg_a          <= {WIDTH{1'b0}};
            reg_b          <= {WIDTH{1'b0}};
            reg_c          <= {WIDTH{1'b0}};
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

                ST_CHECK_INF: begin
                    if (op_type == 1'b1) begin // Doubling
                        if (z1 == {WIDTH{1'b0}}) begin
                            x3 <= {WIDTH{1'b0}}; y3 <= {WIDTH{1'b0}}; z3 <= {WIDTH{1'b0}};
                            state <= ST_DONE;
                        end else begin
                            fsm_mult_a     <= x1;
                            fsm_mult_b     <= x1;
                            fsm_mult_start <= 1'b1;
                            next_state     <= ST_DBL_XX;
                            state          <= ST_MULT_WAIT;
                        end
                    end else begin // Addition
                        if (z1 == {WIDTH{1'b0}}) begin
                            x3 <= x2; y3 <= y2; z3 <= z2;
                            state <= ST_DONE;
                        end else if (z2 == {WIDTH{1'b0}}) begin
                            x3 <= x1; y3 <= y1; z3 <= z1;
                            state <= ST_DONE;
                        end else begin
                            fsm_mult_a     <= z1;
                            fsm_mult_b     <= z1;
                            fsm_mult_start <= 1'b1;
                            next_state     <= ST_ADD_Z1_SQ;
                            state          <= ST_MULT_WAIT;
                        end
                    end
                end

                // ============================================================
                // RUTA DE DOBLAMIENTO (op_type == 1)
                // ============================================================
                ST_DBL_XX: begin
                    t1             <= mult_result; // XX
                    fsm_mult_a     <= y1;
                    fsm_mult_b     <= y1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_DBL_YY;
                    state          <= ST_MULT_WAIT;
                end

                ST_DBL_YY: begin
                    t2             <= mult_result; // YY
                    fsm_mult_a     <= mult_result;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_DBL_YYYY;
                    state          <= ST_MULT_WAIT;
                end

                ST_DBL_YYYY: begin
                    t3       <= mult_result; // YYYY
                    add_in_a <= t1;          // XX + XX
                    add_in_b <= t1;
                    state    <= ST_DBL_M_PREP;
                end

                ST_DBL_M_PREP: begin
                    add_in_a <= add_out;     // 2XX + XX
                    add_in_b <= t1;
                    state    <= ST_DBL_M_FINAL;
                end

                ST_DBL_M_FINAL: begin
                    reg_a          <= add_out; // M = 3XX
                    fsm_mult_a     <= x1;
                    fsm_mult_b     <= t2;      // YY
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_DBL_X1_YY;
                    state          <= ST_MULT_WAIT;
                end

                ST_DBL_X1_YY: begin
                    add_in_a <= mult_result;
                    add_in_b <= mult_result;
                    state    <= ST_DBL_S_PREP;
                end

                ST_DBL_S_PREP: begin
                    add_in_a       <= add_out;
                    add_in_b       <= add_out;
                    reg_b          <= add_out;
                    fsm_mult_a     <= reg_a; // M
                    fsm_mult_b     <= reg_a;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_DBL_M2;
                    state          <= ST_MULT_WAIT;
                end

                ST_DBL_M2: begin
                    reg_b    <= add_out; // S
                    add_in_a <= add_out;
                    add_in_b <= add_out;
                    sub_in_a <= mult_result;
                    sub_in_b <= add_out;
                    state    <= ST_DBL_X3;
                end

                ST_DBL_X3: begin
                    x3             <= sub_out;
                    fsm_mult_a     <= y1;
                    fsm_mult_b     <= z1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_DBL_Y1_Z1;
                    state          <= ST_MULT_WAIT;
                end

                ST_DBL_Y1_Z1: begin
                    add_in_a <= mult_result;
                    add_in_b <= mult_result;
                    sub_in_a <= reg_b;
                    sub_in_b <= x3;
                    state    <= ST_DBL_Z3;
                end

                ST_DBL_Z3: begin
                    z3             <= add_out;
                    t2             <= sub_out;
                    fsm_mult_a     <= reg_a; // M
                    fsm_mult_b     <= sub_out;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_DBL_M_S_X3;
                    state          <= ST_MULT_WAIT;
                end

                ST_DBL_M_S_X3: begin
                    t1       <= mult_result;
                    add_in_a <= t3; // YYYY
                    add_in_b <= t3;
                    state    <= ST_DBL_8YYYY_1;
                end

                ST_DBL_8YYYY_1: begin
                    add_in_a <= add_out;
                    add_in_b <= add_out;
                    state    <= ST_DBL_8YYYY_2;
                end

                ST_DBL_8YYYY_2: begin
                    add_in_a <= add_out;
                    add_in_b <= add_out;
                    state    <= ST_DBL_8YYYY_3;
                end

                ST_DBL_8YYYY_3: begin
                    sub_in_a <= t1;
                    sub_in_b <= add_out;
                    state    <= ST_DBL_Y3;
                end

                ST_DBL_Y3: begin
                    y3    <= sub_out;
                    state <= ST_DONE;
                end

                // ============================================================
                // RUTA DE ADICIÓN (op_type == 0)
                // ============================================================
                ST_ADD_Z1_SQ: begin
                    t1             <= mult_result; // Z1^2
                    fsm_mult_a     <= z2;
                    fsm_mult_b     <= z2;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_Z2_SQ;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_Z2_SQ: begin
                    t2             <= mult_result; // Z2^2
                    fsm_mult_a     <= x1;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_U1;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_U1: begin
                    t3             <= mult_result; // U1
                    fsm_mult_a     <= x2;
                    fsm_mult_b     <= t1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_U2;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_U2: begin
                    reg_a    <= mult_result; // U2
                    sub_in_a <= mult_result;
                    sub_in_b <= t3;
                    state    <= ST_ADD_CALC_H;
                end

                ST_ADD_CALC_H: begin
                    reg_a          <= sub_out; // H
                    fsm_mult_a     <= z1;
                    fsm_mult_b     <= t1;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_Z1_CU;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_Z1_CU: begin
                    t1             <= mult_result;
                    fsm_mult_a     <= y2;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_S2;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_S2: begin
                    reg_b          <= mult_result; // S2
                    fsm_mult_a     <= z2;
                    fsm_mult_b     <= t2;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_Z2_CU;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_Z2_CU: begin
                    t2             <= mult_result;
                    fsm_mult_a     <= y1;
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_S1;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_S1: begin
                    reg_c    <= mult_result; // S1
                    sub_in_a <= reg_b;
                    sub_in_b <= mult_result;
                    state    <= ST_ADD_CALC_R;
                end

                ST_ADD_CALC_R: begin
                    reg_b <= sub_out; // R
                    if (reg_a == {WIDTH{1'b0}}) begin
                        if (sub_out == {WIDTH{1'b0}}) begin
                            is_double <= 1'b1;
                        end
                        x3 <= {WIDTH{1'b0}}; y3 <= {WIDTH{1'b0}}; z3 <= {WIDTH{1'b0}};
                        state <= ST_DONE;
                    end else begin
                        fsm_mult_a     <= reg_a; // H
                        fsm_mult_b     <= reg_a;
                        fsm_mult_start <= 1'b1;
                        next_state     <= ST_ADD_H2;
                        state          <= ST_MULT_WAIT;
                    end
                end

                ST_ADD_H2: begin
                    t1             <= mult_result; // H^2
                    fsm_mult_a     <= reg_a;       // H
                    fsm_mult_b     <= mult_result;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_H3;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_H3: begin
                    t2             <= mult_result; // H^3
                    fsm_mult_a     <= t3;          // U1
                    fsm_mult_b     <= t1;          // H^2
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_V;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_V: begin
                    t1             <= mult_result; // V
                    fsm_mult_a     <= reg_b;       // R
                    fsm_mult_b     <= reg_b;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_R2;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_R2: begin
                    t3       <= mult_result; // R^2
                    sub_in_a <= mult_result;
                    sub_in_b <= t2;          // H^3
                    add_in_a <= t1;          // V
                    add_in_b <= t1;
                    state    <= ST_ADD_X3_PREP;
                end

                ST_ADD_X3_PREP: begin
                    x3       <= sub_out;
                    sub_in_a <= sub_out;
                    sub_in_b <= add_out;     // 2V
                    state    <= ST_ADD_X3_SUB2V;
                end

                ST_ADD_X3_SUB2V: begin
                    x3       <= sub_out;
                    sub_in_a <= t1;          // V
                    sub_in_b <= sub_out;     // X3
                    state    <= ST_ADD_V_X3;
                end

                ST_ADD_V_X3: begin
                    t1             <= sub_out;
                    fsm_mult_a     <= reg_b; // R
                    fsm_mult_b     <= sub_out;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_R_VX3;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_R_VX3: begin
                    t3             <= mult_result;
                    fsm_mult_a     <= reg_c; // S1
                    fsm_mult_b     <= t2;    // H^3
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_S1_H3;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_S1_H3: begin
                    sub_in_a <= t3;
                    sub_in_b <= mult_result;
                    state    <= ST_ADD_Y3;
                end

                ST_ADD_Y3: begin
                    y3             <= sub_out;
                    fsm_mult_a     <= z1;
                    fsm_mult_b     <= z2;
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_Z1_Z2;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_Z1_Z2: begin
                    fsm_mult_a     <= mult_result;
                    fsm_mult_b     <= reg_a; // H
                    fsm_mult_start <= 1'b1;
                    next_state     <= ST_ADD_Z3;
                    state          <= ST_MULT_WAIT;
                end

                ST_ADD_Z3: begin
                    z3    <= mult_result;
                    state <= ST_DONE;
                end

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