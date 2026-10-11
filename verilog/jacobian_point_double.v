// ============================================================================
// Módulo: jacobian_point_double.v (Genérico / Curve-Agnostic)
// Operación: P3 (X3, Y3, Z3) = 2 * P1 (X1, Y1, Z1)
// Coordenadas: Jacobianas (x = X/Z^2, y = Y/Z^3)
// Optimizado para: Intel Cyclone IV (Reutiliza el Multiplicador de Montgomery)
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

    // Entradas P1, Coeficiente 'a' y Módulo P (en dominio Montgomery)
    input  wire [WIDTH-1:0]     x1, y1, z1,
    input  wire [WIDTH-1:0]     a_param, // Coeficiente 'a' de la curva (0 para secp256k1)
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
    // Funciones Modulares Auxiliares (Aritmética combinacional)
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
    localparam ST_IDLE       = 4'd0;
    localparam ST_CHECK_ZERO = 4'd1;
    localparam ST_CALC_Y1SQ  = 4'd2;  // T1 = Y1^2
    localparam ST_CALC_X1SQ  = 4'd3;  // T2 = X1^2
    localparam ST_CALC_S_RAW = 4'd4;  // t_s = X1 * Y1^2
    localparam ST_CALC_Y1CU  = 4'd5;  // T3 = Y1^4
    localparam ST_CHECK_A    = 4'd6;  // Evaluar a_param
    localparam ST_CALC_Z1SQ  = 4'd7;  // Z1^2 (Si a != 0)
    localparam ST_CALC_Z1QU  = 4'd8;  // Z1^4 (Si a != 0)
    localparam ST_CALC_AZ1QU = 4'd9;  // a * Z1^4 (Si a != 0)
    localparam ST_CALC_M_SQ  = 4'd10; // M^2 = M * M
    localparam ST_CALC_Y3_RAW= 4'd11; // t_y = M * (S - X3)
    localparam ST_CALC_Z3_RAW= 4'd12; // t_z = Y1 * Z1
    localparam ST_MULT_WAIT  = 4'd13;
    localparam ST_DONE       = 4'd14;

    reg [3:0] state, next_state;

    // ------------------------------------------------------------------------
    // Registros temporales de cálculo
    // ------------------------------------------------------------------------
    reg [WIDTH-1:0] y1_sq;
    reg [WIDTH-1:0] x1_sq;
    reg [WIDTH-1:0] s;       // S = 4 * X1 * Y1^2
    reg [WIDTH-1:0] c;       // C = 8 * Y1^4
    reg [WIDTH-1:0] z1_sq;
    reg [WIDTH-1:0] z1_qu;
    reg [WIDTH-1:0] m;       // M = 3 * X1^2 + a * Z1^4
    reg [WIDTH-1:0] t_y3;

    // ------------------------------------------------------------------------
    // Máquina de Estados Finita (FSM)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            next_state <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            mult_start <= 1'b0;
            mult_a     <= {WIDTH{1'b0}};
            mult_b     <= {WIDTH{1'b0}};
            x3         <= {WIDTH{1'b0}};
            y3         <= {WIDTH{1'b0}};
            z3         <= {WIDTH{1'b0}};
            y1_sq      <= {WIDTH{1'b0}};
            x1_sq      <= {WIDTH{1'b0}};
            s          <= {WIDTH{1'b0}};
            c          <= {WIDTH{1'b0}};
            z1_sq      <= {WIDTH{1'b0}};
            z1_qu      <= {WIDTH{1'b0}};
            m          <= {WIDTH{1'b0}};
            t_y3       <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ST_CHECK_ZERO;
                    end else begin
                        busy  <= 1'b0;
                    end
                end

                // ------------------------------------------------------------
                // Casos límite: Punto al infinito (Z = 0 o Y = 0)
                // ------------------------------------------------------------
                ST_CHECK_ZERO: begin
                    if (z1 == {WIDTH{1'b0}} || y1 == {WIDTH{1'b0}}) begin
                        x3    <= {WIDTH{1'b0}};
                        y3    <= {WIDTH{1'b0}};
                        z3    <= {WIDTH{1'b0}};
                        state <= ST_DONE;
                    end else begin
                        // 1. T1 = Y1^2
                        mult_a     <= y1;
                        mult_b     <= y1;
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_Y1SQ;
                        state      <= ST_MULT_WAIT;
                    end
                end

                // 1. T1 = Y1^2
                ST_CALC_Y1SQ: begin
                    y1_sq      <= mult_result;
                    // 2. T2 = X1^2
                    mult_a     <= x1;
                    mult_b     <= x1;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_X1SQ;
                    state      <= ST_MULT_WAIT;
                end

                // 2. T2 = X1^2
                ST_CALC_X1SQ: begin
                    x1_sq      <= mult_result;
                    // 3. t_s = X1 * Y1^2
                    mult_a     <= x1;
                    mult_b     <= y1_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_S_RAW;
                    state      <= ST_MULT_WAIT;
                end

                // 3. S = 4 * X1 * Y1^2
                ST_CALC_S_RAW: begin
                    begin
                        reg [WIDTH-1:0] s_x2;
                        s_x2 = mod_add(mult_result, mult_result, p); // 2 * t_s
                        s    <= mod_add(s_x2, s_x2, p);              // 4 * t_s
                    end
                    // 4. T3 = Y1^4
                    mult_a     <= y1_sq;
                    mult_b     <= y1_sq;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Y1CU;
                    state      <= ST_MULT_WAIT;
                end

                // 4. C = 8 * Y1^4
                ST_CALC_Y1CU: begin
                    begin
                        reg [WIDTH-1:0] c_x2, c_x4;
                        c_x2 = mod_add(mult_result, mult_result, p); // 2 * Y1^4
                        c_x4 = mod_add(c_x2, c_x2, p);              // 4 * Y1^4
                        c    <= mod_add(c_x4, c_x4, p);              // 8 * Y1^4
                    end
                    state <= ST_CHECK_A;
                end

                // ------------------------------------------------------------
                // Cálculo de M = 3 * X1^2 + a * Z1^4
                // ------------------------------------------------------------
                ST_CHECK_A: begin
                    if (a_param == {WIDTH{1'b0}}) begin
                        // Caso a = 0 (secp256k1): M = 3 * X1^2
                        m <= mod_add(mod_add(x1_sq, x1_sq, p), x1_sq, p);
                        
                        // M^2
                        mult_a     <= mod_add(mod_add(x1_sq, x1_sq, p), x1_sq, p);
                        mult_b     <= mod_add(mod_add(x1_sq, x1_sq, p), x1_sq, p);
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_M_SQ;
                        state      <= ST_MULT_WAIT;
                    end else begin
                        // Caso a != 0: Z1^2
                        mult_a     <= z1;
                        mult_b     <= z1;
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_Z1SQ;
                        state      <= ST_MULT_WAIT;
                    end
                end

                // Si a != 0: Z1^2
                ST_CALC_Z1SQ: begin
                    z1_sq      <= mult_result;
                    mult_a     <= mult_result;
                    mult_b     <= mult_result;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z1QU;
                    state      <= ST_MULT_WAIT;
                end

                // Si a != 0: Z1^4
                ST_CALC_Z1QU: begin
                    z1_qu      <= mult_result;
                    mult_a     <= a_param;
                    mult_b     <= mult_result;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_AZ1QU;
                    state      <= ST_MULT_WAIT;
                end

                // Si a != 0: a * Z1^4  ==> M = 3 * X1^2 + a * Z1^4
                ST_CALC_AZ1QU: begin
                    begin
                        reg [WIDTH-1:0] x1_sq_3;
                        x1_sq_3 = mod_add(mod_add(x1_sq, x1_sq, p), x1_sq, p);
                        m       <= mod_add(x1_sq_3, mult_result, p);
                        
                        mult_a     <= mod_add(x1_sq_3, mult_result, p);
                        mult_b     <= mod_add(x1_sq_3, mult_result, p);
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_M_SQ;
                        state      <= ST_MULT_WAIT;
                    end
                end

                // 5. X3 = M^2 - 2*S
                ST_CALC_M_SQ: begin
                    begin
                        reg [WIDTH-1:0] s_x2;
                        s_x2 = mod_add(s, s, p);
                        x3   <= mod_sub(mult_result, s_x2, p);
                        
                        // t_y = M * (S - X3)
                        mult_a     <= m;
                        mult_b     <= mod_sub(s, mod_sub(mult_result, s_x2, p), p);
                        mult_start <= 1'b1;
                        next_state <= ST_CALC_Y3_RAW;
                        state      <= ST_MULT_WAIT;
                    end
                end

                // 6. Y3 = M * (S - X3) - C
                ST_CALC_Y3_RAW: begin
                    y3 <= mod_sub(mult_result, c, p);
                    
                    // 7. Z3 = 2 * Y1 * Z1
                    mult_a     <= y1;
                    mult_b     <= z1;
                    mult_start <= 1'b1;
                    next_state <= ST_CALC_Z3_RAW;
                    state      <= ST_MULT_WAIT;
                end

                // 7. Z3 = 2 * (Y1 * Z1)
                ST_CALC_Z3_RAW: begin
                    z3    <= mod_add(mult_result, mult_result, p);
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