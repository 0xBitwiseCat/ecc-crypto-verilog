// ============================================================================
// Módulo: jacobian_point_add.v (Genérico / Curve-Agnostic)
// Operación: P3 (X3, Y3, Z3) = P1 (X1, Y1, Z1) + P2 (X2, Y2, Z2)
// Coordenadas: Jacobianas (x = X/Z^2, y = Y/Z^3)
// Optimizado para: Intel Cyclone IV (Reutiliza el Multiplicador de Montgomery)
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
    output reg                  is_double, // Indica si P1 == P2 (requiere Point Double)

    // Entradas P1, P2 y Módulo P (en dominio Montgomery)
    input  wire [WIDTH-1:0]     x1, y1, z1,
    input  wire [WIDTH-1:0]     x2, y2, z2,
    input  wire [WIDTH-1:0]     p,

    // Salidas P3
    output reg  [WIDTH-1:0]     x3, y3, z3,

    // Interfaz con el Multiplicador de Montgomery compartido
    output reg                  mult_start,
    output reg  [WIDTH-1:0]     mult_a,
    output reg  [WIDTH-1:0]     mult_b,
    input  wire                 mult_done,
    input  wire [WIDTH-1:0]     mult_result
);

    // ------------------------------------------------------------------------
    // Funciones Modulares Axiliares (Aritmética combinacional)
    // ------------------------------------------------------------------------
    function [WIDTH-1:0] mod_add;
        input [WIDTH-1:0] a, b, mod;
        reg [WIDTH:0] sum;
        begin
            sum = a + b;
            mod_add = (sum >= mod) ? (sum - mod) : sum[WIDTH-1:0];
        end
    endfunction

    function [WIDTH-1:0] mod_sub;
        input [WIDTH-1:0] a, b, mod;
        begin
            mod_sub = (a >= b) ? (a - b) : (a + mod - b);
        end
    endfunction

    // ------------------------------------------------------------------------
    // Codificación de Estados
    // ------------------------------------------------------------------------
    localparam ST_IDLE       = 5'd0;
    localparam ST_CHECK_ZERO = 5'd1;
    
    // Secuencia de multiplicaciones modulares
    localparam ST_CALC_Z1SQ  = 5'd2;  // t1 = Z1^2
    localparam ST_CALC_Z2SQ  = 5'd3;  // t2 = Z2^2
    localparam ST_CALC_U1    = 5'd4;  // U1 = X1 * Z2^2
    localparam ST_CALC_U2    = 5'd5;  // U2 = X2 * Z1^2
    localparam ST_CALC_Z2CU  = 5'd6;  // t3 = Z2^3
    localparam ST_CALC_Z1CU  = 5'd7;  // t4 = Z1^3
    localparam ST_CALC_S1    = 5'd8;  // S1 = Y1 * Z2^3
    localparam ST_CALC_S2    = 5'd9;  // S2 = Y2 * Z1^3
    localparam ST_CHECK_EQ   = 5'd10; // H = U2 - U1, R = S2 - S1
    localparam ST_CALC_HSQ   = 5'd11; // H2 = H^2
    localparam ST_CALC_HCU   = 5'd12; // H3 = H^3
    localparam ST_CALC_V     = 5'd13; // V  = U1 * H^2
    localparam ST_CALC_RSQ   = 5'd14; // R2 = R^2
    localparam ST_CALC_X3    = 5'd15; // X3 = R^2 - H^3 - 2V
    localparam ST_CALC_Y3_A  = 5'd16; // t5 = R * (V - X3)
    localparam ST_CALC_Y3_B  = 5'd17; // t6 = S1 * H^3
    localparam ST_CALC_Z3_A  = 5'd18; // t7 = Z1 * Z2
    localparam ST_CALC_Z3_B  = 5'd19; // Z3 = t7 * H

    localparam ST_MULT_WAIT  = 5'd20;
    localparam ST_DONE       = 5'd21;

    reg [4:0] state, next_state;

    // ------------------------------------------------------------------------
    // Registros temporales de cálculo
    // ------------------------------------------------------------------------
    reg [WIDTH-1:0] z1_sq, z2_sq;
    reg [WIDTH-1:0] u1, u2;
    reg [WIDTH-1:0] s1, s2;
    reg [WIDTH-1:0] h, r;
    reg [WIDTH-1:0] h_sq, h_cu;
    reg [WIDTH-1:0] v;
    reg [WIDTH-1:0] r_sq;
    reg [WIDTH-1:0] t_y3a, t_y3b, t_z3a;

    // ------------------------------------------------------------------------
    // Máquina de Estados Finita (FSM)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            next_state <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            is_double  <= 1'b0;
            mult_start <= 1'b0;
            mult_a     <= {WIDTH{1'b0}};
            mult_b     <= {WIDTH{1'b0}};
            x3         <= {WIDTH{1'b0}};
            y3         <= {WIDTH{1'b0}};
            z3         <= {WIDTH{1 meb0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done      <= 1'b0;
                    is_double <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_CHECK_ZERO;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                // ------------------------------------------------------------
                // Casos límite de puntos en el infinito (Z = 0)
                // ------------------------------------------------------------
                ST_CHECK_ZERO: begin
                    if (z1 == {WIDTH{1'b0}}) begin
                        x3    <= x2; y3 <= y2; z3 <= z2;
                        state <= ST_DONE;
                    end else if (z2 == {WIDTH{1'b0}}) begin
                        x3    <= x1; y3 <= y1; z3 <= z1;
                        state <= ST_DONE;
                    end else begin
                        // Iniciar cálculo: t1 = Z1^2
                        mult_a     <= z1;
                        mult_b     <= z1;
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_Z1SQ;
                        state      <= ST_MULT_WAIT;
                    end
                end

                // 1. Z1^2
                ST_CALC_Z1SQ: begin
                    z1_sq      <= mult_result;
                    mult_a     <= z2;
                    mult_b     <= z2;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z2SQ;
                    state      <= ST_MULT_WAIT;
                end

                // 2. Z2^2
                ST_CALC_Z2SQ: begin
                    z2_sq      <= mult_result;
                    mult_a     <= x1;
                    mult_b     <= mult_result; // Z2^2
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_U1;
                    state      <= ST_MULT_WAIT;
                end

                // 3. U1 = X1 * Z2^2
                ST_CALC_U1: begin
                    u1         <= mult_result;
                    mult_a     <= x2;
                    mult_b     <= z1_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_U2;
                    state      <= ST_MULT_WAIT;
                end

                // 4. U2 = X2 * Z1^2
                ST_CALC_U2: begin
                    u2         <= mult_result;
                    mult_a     <= z2;
                    mult_b     <= z2_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z2CU;
                    state      <= ST_MULT_WAIT;
                end

                // 5. Z2^3 = Z2 * Z2^2
                ST_CALC_Z2CU: begin
                    mult_a     <= z1;
                    mult_b     <= z1_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z1CU;
                    state      <= ST_MULT_WAIT;
                    z2_sq      <= mult_result; // Guardar Z2^3 en z2_sq temporalmente
                end

                // 6. Z1^3 = Z1 * Z1^2
                ST_CALC_Z1CU: begin
                    mult_a     <= y1;
                    mult_b     <= z2_sq; // Z2^3
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_S1;
                    state      <= ST_MULT_WAIT;
                    z1_sq      <= mult_result; // Guardar Z1^3 en z1_sq temporalmente
                end

                // 7. S1 = Y1 * Z2^3
                ST_CALC_S1: begin
                    s1         <= mult_result;
                    mult_a     <= y2;
                    mult_b     <= z1_sq; // Z1^3
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_S2;
                    state      <= ST_MULT_WAIT;
                end

                // 8. S2 = Y2 * Z1^3
                ST_CALC_S2: begin
                    s2    <= mult_result;
                    state <= ST_CHECK_EQ;
                end

                // ------------------------------------------------------------
                // Comprobar H = U2 - U1, R = S2 - S1
                // ------------------------------------------------------------
                ST_CHECK_EQ: begin
                    h <= mod_sub(u2, u1, p);
                    r <= mod_sub(s2, s1, p);

                    if (u1 == u2) begin
                        if (s1 == s2) begin
                            // P1 == P2: se debe llamar a Point Double
                            is_double <= 1'b1;
                            state     <= ST_DONE;
                        end else begin
                            // P1 == -P2: El resultado es el punto al infinito (0,0,0)
                            x3 <= {WIDTH{1'b0}};
                            y3 <= {WIDTH{1 meb0}};
                            z3 <= {WIDTH{1'b0}};
                            state <= ST_DONE;
                        end
                    end else begin
                        // H^2 = H * H
                        mult_a     <= mod_sub(u2, u1, p);
                        mult_b     <= mod_sub(u2, u1, p);
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_HSQ;
                        state      <= ST_MULT_WAIT;
                    end
                end

                // 9. H^2
                ST_CALC_HSQ: begin
                    h_sq       <= mult_result;
                    mult_a     <= h;
                    mult_b     <= mult_result;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_HCU;
                    state      <= ST_MULT_WAIT;
                end

                // 10. H^3 = H * H^2
                ST_CALC_HCU: begin
                    h_cu       <= mult_result;
                    mult_a     <= u1;
                    mult_b     <= h_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_V;
                    state      <= ST_MULT_WAIT;
                end

                // 11. V = U1 * H^2
                ST_CALC_V: begin
                    v          <= mult_result;
                    mult_a     <= r;
                    mult_b     <= r;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_RSQ;
                    state      <= ST_MULT_WAIT;
                end

                // 12. R^2
                ST_CALC_RSQ: begin
                    r_sq  <= mult_result;
                    state <= ST_CALC_X3;
                end

                // 13. X3 = R^2 - H^3 - 2V
                ST_CALC_X3: begin
                    begin
                        reg [WIDTH-1:0] temp_v2;
                        reg [WIDTH-1:0] temp_sub1;
                        temp_v2   = mod_add(v, v, p); // 2V
                        temp_sub1 = mod_sub(r_sq, h_cu, p); // R^2 - H^3
                        x3        <= mod_sub(temp_sub1, temp_v2, p);
                    end

                    // t5 = R * (V - X3)
                    mult_a     <= r;
                    mult_b     <= mod_sub(v, mod_sub(mod_sub(r_sq, h_cu, p), mod_add(v, v, p), p), p);
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Y3_A;
                    state      <= ST_MULT_WAIT;
                end

                // 14. t5 = R * (V - X3)
                ST_CALC_Y3_A: begin
                    t_y3a      <= mult_result;
                    mult_a     <= s1;
                    mult_b     <= h_cu;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Y3_B;
                    state      <= ST_MULT_WAIT;
                end

                // 15. t6 = S1 * H^3  ==>  Y3 = t5 - t6
                ST_CALC_Y3_B: begin
                    t_y3b <= mult_result;
                    y3    <= mod_sub(t_y3a, mult_result, p);

                    // t7 = Z1 * Z2
                    mult_a     <= z1;
                    mult_b     <= z2;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z3_A;
                    state      <= ST_MULT_WAIT;
                end

                // 16. t7 = Z1 * Z2
                ST_CALC_Z3_A: begin
                    t_z3a      <= mult_result;
                    mult_a     <= mult_result;
                    mult_b     <= h;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z3_B;
                    state      <= ST_MULT_WAIT;
                end

                // 17. Z3 = t7 * H
                ST_CALC_Z3_B: begin
                    z3    <= mult_result;
                    state <= ST_DONE;
                end

                // Sub-rutina de espera del multiplicador de Montgomery
                ST_MULT_WAIT: begin
                    mult_start <= 1'b0;
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