`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: Suma de Puntos Jacobianos (P + Q)
// OBJETIVO: Optimización extrema de Área mediante "Resource Sharing"
// MATEMÁTICA: X3, Y3, Z3 a partir de (X1,Y1,Z1) + (X2,Y2,Z2)
// ============================================================================

module jacobian_point_add #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    
    // Punto P
    input  wire [W-1:0] X1, Y1, Z1,
    // Punto Q
    input  wire [W-1:0] X2, Y2, Z2,
    // Módulo de la curva
    input  wire [W-1:0] P_MOD,
    
    // Resultado Punto R = P+Q
    output reg  [W-1:0] X3, Y3, Z3,
    output reg          done
);

    // ========================================================================
    // 1. REGISTROS INTERNOS TEMPORALES (RAM Distribuida o Flip-Flops)
    // Justificación: Reutilizamos estos registros para guardar resultados 
    // intermedios, evitando crear un registro nuevo para cada variable.
    // ========================================================================
    reg [W-1:0] T1, T2, T3, T4; 
    reg [W-1:0] U1, U2, S1, S2, H, R;

    // ========================================================================
    // 2. INTERFAZ DEL MULTIPLICADOR ÚNICO (Resource Sharing)
    // ========================================================================
    reg          mult_start;
    reg  [W-1:0] mult_A;
    reg  [W-1:0] mult_B;
    wire [W-1:0] mult_out;
    wire         mult_done;

    // Instanciación del módulo diseñado anteriormente
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
    // 3. ALU COMBINACIONAL (Sumas y Restas Modulares Rápidas)
    // Justificación: Las sumas modulares (A+B mod P) y restas (A-B mod P) toman 
    // 1 solo ciclo de reloj. Se infieren como lógica combinacional (LUTs).
    // ========================================================================
    wire [W:0] sum_val = mult_A + mult_B; // Bit extra para acarreo
    wire [W-1:0] mod_add_res = (sum_val >= P_MOD) ? (sum_val - P_MOD) : sum_val;
    wire [W-1:0] mod_sub_res = (mult_A >= mult_B) ? (mult_A - mult_B) : (mult_A + P_MOD - mult_B);

    // ========================================================================
    // 4. MÁQUINA DE ESTADOS FINITOS (FSM - Controlador Matemático)
    // ========================================================================
    
    reg [4:0] state;
    localparam S_IDLE    = 5'd0;
    localparam S_Z2_SQR  = 5'd1;  // T1 = Z2^2
    localparam S_WAIT_1  = 5'd2;
    localparam S_Z1_SQR  = 5'd3;  // T2 = Z1^2
    localparam S_WAIT_2  = 5'd4;
    localparam S_U1      = 5'd5;  // U1 = X1 * T1
    localparam S_WAIT_3  = 5'd6;
    localparam S_U2      = 5'd7;  // U2 = X2 * T2
    localparam S_WAIT_4  = 5'd8;
    localparam S_CALC_H  = 5'd9;  // H = U2 - U1 (Resta combinacional 1 ciclo)
    // ... Estados subsiguientes omitidos por brevedad ...
    localparam S_DONE    = 5'd31;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            mult_start <= 0;
            done <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) state <= S_Z2_SQR;
                end

                // --- ETAPA 1: Calcular Z2^2 ---
                S_Z2_SQR: begin
                    mult_A <= Z2;       // Enrutador multiplexa Z2 hacia A
                    mult_B <= Z2;       // Enrutador multiplexa Z2 hacia B
                    mult_start <= 1;    // Dispara el multiplicador de 256 ciclos
                    state <= S_WAIT_1;
                end
                S_WAIT_1: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; // Guarda el resultado de Z2^2 en T1
                        state <= S_Z1_SQR;
                    end
                end

                // --- ETAPA 2: Calcular Z1^2 ---
                S_Z1_SQR: begin
                    mult_A <= Z1;
                    mult_B <= Z1;
                    mult_start <= 1;
                    state <= S_WAIT_2;
                end
                S_WAIT_2: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T2 <= mult_out; // Guarda Z1^2 en T2
                        state <= S_U1;
                    end
                end

                // --- ETAPA 3: Calcular U1 = X1 * Z2^2 ---
                S_U1: begin
                    mult_A <= X1;
                    mult_B <= T1;       // T1 contiene Z2^2
                    mult_start <= 1;
                    state <= S_WAIT_3;
                end
                S_WAIT_3: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        U1 <= mult_out; // U1 completado
                        state <= S_U2;
                    end
                end

                // --- ETAPA 4: Calcular U2 = X2 * Z1^2 ---
                S_U2: begin
                    mult_A <= X2;
                    mult_B <= T2;       // T2 contiene Z1^2
                    mult_start <= 1;
                    state <= S_WAIT_4;
                end
                S_WAIT_4: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        U2 <= mult_out; // U2 completado
                        state <= S_CALC_H;
                    end
                end

                // --- ETAPA 5: Calcular H = U2 - U1 ---
                S_CALC_H: begin
                    // Justificación: No usamos el multiplicador aquí. 
                    // Usamos la ALU combinacional (mod_sub_res). Toma solo 1 ciclo de reloj.
                    mult_A <= U2;
                    mult_B <= U1;
                    H <= mod_sub_res;   
                    // state <= S_S1; (Saltaría a calcular S1 = Y1 * Z2^3)
                    state <= S_DONE; // Termina aquí en este ejemplo truncado
                end

                S_DONE: begin
                    // Aquí se asignarían X3, Y3, Z3 finales.
                    done <= 1;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule