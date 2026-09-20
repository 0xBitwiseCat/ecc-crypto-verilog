`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: Doblado de Punto Jacobiano (2P)
// OBJETIVO: Optimización de Área y evasión de multiplicaciones por constantes
// MATEMÁTICA (secp256k1, a=0): 
// A = Y1^2, B = 4 * X1 * A, C = 8 * A^2, D = 3 * X1^2
// X3 = D^2 - 2B, Y3 = D * (B - X3) - C, Z3 = 2 * Y1 * Z1
// ============================================================================

module jacobian_point_double #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    
    // Punto de entrada P
    input  wire [W-1:0] X1, Y1, Z1,
    // Módulo P constante
    input  wire [W-1:0] P_MOD,
    
    // Resultado 2P
    output reg  [W-1:0] X3, Y3, Z3,
    output reg          done
);

    // ========================================================================
    // 1. REGISTROS INTERNOS TEMPORALES
    // ========================================================================
    reg [W-1:0] A, B, C, D, T1;

    // ========================================================================
    // 2. INTERFAZ DEL MULTIPLICADOR ÚNICO (Resource Sharing)
    // ========================================================================
    reg          mult_start;
    reg  [W-1:0] mult_A, mult_B;
    wire [W-1:0] mult_out;
    wire         mult_done;

    montgomery_multiplier #(.W(W)) u_mult (
        .clk(clk),
        .rst(rst),
        .start(mult_start),
        .A(mult_A),
        .B(mult_B),
        .P(P_MOD),
        .S_out(mult_out),
        .done(mult_done)
    );

    // ========================================================================
    // 3. ALU COMBINACIONAL (Doblados y Restas Modulares)
    // Justificación: Las multiplicaciones por 2 o 3 se logran sumando el valor 
    // consigo mismo, ahorrando los 256 ciclos de la FSM de Montgomery.
    // ========================================================================
    reg  [W-1:0] alu_in_1, alu_in_2;
    wire [W:0]   sum_val = alu_in_1 + alu_in_2;
    wire [W-1:0] mod_add_res = (sum_val >= P_MOD) ? (sum_val - P_MOD) : sum_val;
    
    // Circuito dedicado para doblar (X + X mod P)
    wire [W:0]   double_val = alu_in_1 + alu_in_1;
    wire [W-1:0] mod_double_res = (double_val >= P_MOD) ? (double_val - P_MOD) : double_val;

    // ========================================================================
    // 4. MÁQUINA DE ESTADOS FINITOS (FSM)
    // ========================================================================
    reg [4:0] state;
    localparam S_IDLE       = 5'd0;
    localparam S_A_CALC     = 5'd1;  // A = Y1^2
    localparam S_WAIT_1     = 5'd2;
    localparam S_Z3_MUL     = 5'd3;  // T1 = Y1 * Z1
    localparam S_WAIT_2     = 5'd4;
    localparam S_Z3_DOUBLE  = 5'd5;  // Z3 = 2 * T1 (Combinacional)
    localparam S_D_SQR      = 5'd6;  // T1 = X1^2
    localparam S_WAIT_3     = 5'd7;
    localparam S_D_CALC_1   = 5'd8;  // T1_doble = 2 * X1^2
    localparam S_D_CALC_2   = 5'd9;  // D = T1_doble + X1^2 = 3 * X1^2
    // ... Estados subsiguientes omitidos por brevedad ...
    localparam S_DONE       = 5'd31;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            mult_start <= 0;
            done <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) state <= S_A_CALC;
                end

                // --- ETAPA 1: Calcular A = Y1^2 ---
                S_A_CALC: begin
                    mult_A <= Y1;
                    mult_B <= Y1;
                    mult_start <= 1;
                    state <= S_WAIT_1;
                end
                S_WAIT_1: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        A <= mult_out;
                        state <= S_Z3_MUL;
                    end
                end

                // --- ETAPA 2: Calcular Z3 = 2 * Y1 * Z1 ---
                S_Z3_MUL: begin
                    mult_A <= Y1;
                    mult_B <= Z1;
                    mult_start <= 1;
                    state <= S_WAIT_2;
                end
                S_WAIT_2: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; // T1 = Y1 * Z1
                        state <= S_Z3_DOUBLE;
                    end
                end
                S_Z3_DOUBLE: begin
                    // Justificación: Z3 requiere multiplicar por 2.
                    // En lugar de usar el multiplicador, usamos el circuito 
                    // de doblado combinacional en un (1) ciclo de reloj.
                    alu_in_1 <= T1;
                    Z3 <= mod_double_res; // Z3 final completado
                    state <= S_D_SQR;
                end

                // --- ETAPA 3: Calcular D = 3 * X1^2 ---
                S_D_SQR: begin
                    mult_A <= X1;
                    mult_B <= X1;
                    mult_start <= 1;
                    state <= S_WAIT_3;
                end
                S_WAIT_3: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; // T1 = X1^2
                        state <= S_D_CALC_1;
                    end
                end
                S_D_CALC_1: begin
                    // Doblamos T1 combinacionalmente
                    alu_in_1 <= T1;
                    D <= mod_double_res; // Ahora D temporalmente tiene 2*X1^2
                    state <= S_D_CALC_2;
                end
                S_D_CALC_2: begin
                    // Sumamos T1 original a D para obtener 3*X1^2
                    alu_in_1 <= D;       // 2*X1^2
                    alu_in_2 <= T1;      // 1*X1^2
                    D <= mod_add_res;    // D final completado en 2 ciclos combinacionales
                    // state <= S_B_CALC; // Saltaría a calcular B = 4 * X1 * A
                    state <= S_DONE; // Truncado para el ejemplo
                end

                S_DONE: begin
                    done <= 1;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule