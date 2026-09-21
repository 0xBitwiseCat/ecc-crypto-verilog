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

    // 1. REGISTROS INTERNOS TEMPORALES
    reg [W-1:0] T1, T2, T3, T4; 
    reg [W-1:0] U1, U2, S1, S2, H, R;

    // 2. INTERFAZ DEL MULTIPLICADOR ÚNICO
    reg          mult_start;
    reg  [W-1:0] mult_A;
    reg  [W-1:0] mult_B;
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

    // 3. ALU COMBINACIONAL
    wire [W:0] sum_val = mult_A + mult_B; 
    wire [W-1:0] mod_add_res = (sum_val >= P_MOD) ? (sum_val - P_MOD) : sum_val;
    wire [W-1:0] mod_sub_res = (mult_A >= mult_B) ? (mult_A - mult_B) : (mult_A + P_MOD - mult_B);

    // 4. MÁQUINA DE ESTADOS FINITOS (Actualizada a 6 bits para soportar más de 31 estados)
    reg [5:0] state;
    
    // Estados Iniciales
    localparam S_IDLE    = 6'd0;
    localparam S_Z2_SQR  = 6'd1;  
    localparam S_WAIT_1  = 6'd2;
    localparam S_Z1_SQR  = 6'd3;  
    localparam S_WAIT_2  = 6'd4;
    localparam S_U1      = 6'd5;  
    localparam S_WAIT_3  = 6'd6;
    localparam S_U2      = 6'd7;  
    localparam S_WAIT_4  = 6'd8;
    localparam S_CALC_H  = 6'd9;  
    
    // Estados Nuevos
    localparam S_Z2_CUBE = 6'd10;
    localparam S_WAIT_5  = 6'd11;
    localparam S_Z1_CUBE = 6'd12;
    localparam S_WAIT_6  = 6'd13;
    localparam S_S1      = 6'd14;
    localparam S_WAIT_7  = 6'd15;
    localparam S_S2      = 6'd16;
    localparam S_WAIT_8  = 6'd17;
    localparam S_CALC_R  = 6'd18;
    localparam S_H_SQR   = 6'd19;
    localparam S_WAIT_9  = 6'd20;
    localparam S_H_CUBE  = 6'd21;
    localparam S_WAIT_10 = 6'd22;
    localparam S_U1_H2   = 6'd23;
    localparam S_WAIT_11 = 6'd24;
    localparam S_R_SQR   = 6'd25;
    localparam S_WAIT_12 = 6'd26;
    
    localparam S_X3_1    = 6'd27;
    localparam S_X3_2    = 6'd28;
    localparam S_X3_3    = 6'd29;
    
    localparam S_Y3_1    = 6'd30;
    localparam S_Y3_MUL1 = 6'd31;
    localparam S_WAIT_13 = 6'd32;
    localparam S_Y3_MUL2 = 6'd33;
    localparam S_WAIT_14 = 6'd34;
    localparam S_Y3_2    = 6'd35;
    
    localparam S_Z3_MUL1 = 6'd36;
    localparam S_WAIT_15 = 6'd37;
    localparam S_Z3_MUL2 = 6'd38;
    localparam S_WAIT_16 = 6'd39;

    localparam S_DONE    = 6'd63;

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
                    mult_A <= Z2;      
                    mult_B <= Z2;      
                    mult_start <= 1;   
                    state <= S_WAIT_1;
                end
                S_WAIT_1: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; 
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
                        T2 <= mult_out;
                        state <= S_U1;
                    end
                end

                // --- ETAPA 3: Calcular U1 = X1 * Z2^2 ---
                S_U1: begin
                    mult_A <= X1;
                    mult_B <= T1;       
                    mult_start <= 1;
                    state <= S_WAIT_3;
                end
                S_WAIT_3: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        U1 <= mult_out; 
                        state <= S_U2;
                    end
                end

                // --- ETAPA 4: Calcular U2 = X2 * Z1^2 ---
                S_U2: begin
                    mult_A <= X2;
                    mult_B <= T2;       
                    mult_start <= 1;
                    state <= S_WAIT_4;
                end
                S_WAIT_4: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        U2 <= mult_out; 
                        state <= S_CALC_H;
                    end
                end

                // --- ETAPA 5: Calcular H = U2 - U1 ---
                S_CALC_H: begin
                    mult_A <= U2;
                    mult_B <= U1;
                    H <= mod_sub_res;   
                    state <= S_Z2_CUBE;
                end

                // --- ETAPA 6: Calcular Z2^3 = Z2^2 * Z2 ---
                S_Z2_CUBE: begin
                    mult_A <= T1; // T1 = Z2^2
                    mult_B <= Z2;
                    mult_start <= 1;
                    state <= S_WAIT_5;
                end
                S_WAIT_5: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T3 <= mult_out; // T3 = Z2^3
                        state <= S_Z1_CUBE;
                    end
                end

                // --- ETAPA 7: Calcular Z1^3 = Z1^2 * Z1 ---
                S_Z1_CUBE: begin
                    mult_A <= T2; // T2 = Z1^2
                    mult_B <= Z1;
                    mult_start <= 1;
                    state <= S_WAIT_6;
                end
                S_WAIT_6: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T4 <= mult_out; // T4 = Z1^3
                        state <= S_S1;
                    end
                end

                // --- ETAPA 8: Calcular S1 = Y1 * Z2^3 ---
                S_S1: begin
                    mult_A <= Y1;
                    mult_B <= T3;
                    mult_start <= 1;
                    state <= S_WAIT_7;
                end
                S_WAIT_7: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        S1 <= mult_out;
                        state <= S_S2;
                    end
                end

                // --- ETAPA 9: Calcular S2 = Y2 * Z1^3 ---
                S_S2: begin
                    mult_A <= Y2;
                    mult_B <= T4;
                    mult_start <= 1;
                    state <= S_WAIT_8;
                end
                S_WAIT_8: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        S2 <= mult_out;
                        state <= S_CALC_R;
                    end
                end

                // --- ETAPA 10: Calcular R = S2 - S1 ---
                S_CALC_R: begin
                    mult_A <= S2;
                    mult_B <= S1;
                    R <= mod_sub_res;
                    state <= S_H_SQR;
                end

                // --- ETAPA 11: Calcular H^2 ---
                S_H_SQR: begin
                    mult_A <= H;
                    mult_B <= H;
                    mult_start <= 1;
                    state <= S_WAIT_9;
                end
                S_WAIT_9: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; // Sobrescribe T1 con H^2
                        state <= S_H_CUBE;
                    end
                end

                // --- ETAPA 12: Calcular H^3 = H^2 * H ---
                S_H_CUBE: begin
                    mult_A <= T1;
                    mult_B <= H;
                    mult_start <= 1;
                    state <= S_WAIT_10;
                end
                S_WAIT_10: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T2 <= mult_out; // Sobrescribe T2 con H^3
                        state <= S_U1_H2;
                    end
                end

                // --- ETAPA 13: Calcular U1 * H^2 ---
                S_U1_H2: begin
                    mult_A <= U1;
                    mult_B <= T1; // H^2
                    mult_start <= 1;
                    state <= S_WAIT_11;
                end
                S_WAIT_11: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T3 <= mult_out; // Sobrescribe T3 con U1*H^2
                        state <= S_R_SQR;
                    end
                end

                // --- ETAPA 14: Calcular R^2 ---
                S_R_SQR: begin
                    mult_A <= R;
                    mult_B <= R;
                    mult_start <= 1;
                    state <= S_WAIT_12;
                end
                S_WAIT_12: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T4 <= mult_out; // Sobrescribe T4 con R^2
                        state <= S_X3_1;
                    end
                end

                // --- ETAPA 15: Calcular X3 = R^2 - H^3 - 2*U1*H^2 ---
                S_X3_1: begin
                    mult_A <= T4; // R^2
                    mult_B <= T2; // H^3
                    U2 <= mod_sub_res; // Reutilizamos U2 para (R^2 - H^3)
                    state <= S_X3_2;
                end
                S_X3_2: begin
                    mult_A <= T3; // U1*H^2
                    mult_B <= T3; // U1*H^2
                    T4 <= mod_add_res; // Reutilizamos T4 para (2 * U1*H^2)
                    state <= S_X3_3;
                end
                S_X3_3: begin
                    mult_A <= U2; // R^2 - H^3
                    mult_B <= T4; // 2*U1*H^2
                    X3 <= mod_sub_res; // << X3 COMPLETADO >>
                    state <= S_Y3_1;
                end

                // --- ETAPA 16: Calcular Y3 = R*(U1*H^2 - X3) - S1*H^3 ---
                S_Y3_1: begin
                    mult_A <= T3; // U1*H^2
                    mult_B <= X3;
                    T3 <= mod_sub_res; // T3 pasa a ser (U1*H^2 - X3)
                    state <= S_Y3_MUL1;
                end
                S_Y3_MUL1: begin
                    mult_A <= R;
                    mult_B <= T3; // (U1*H^2 - X3)
                    mult_start <= 1;
                    state <= S_WAIT_13;
                end
                S_WAIT_13: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T4 <= mult_out; // T4 = R*(U1*H^2 - X3)
                        state <= S_Y3_MUL2;
                    end
                end
                S_Y3_MUL2: begin
                    mult_A <= S1;
                    mult_B <= T2; // H^3
                    mult_start <= 1;
                    state <= S_WAIT_14;
                end
                S_WAIT_14: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        U2 <= mult_out; // U2 = S1*H^3
                        state <= S_Y3_2;
                    end
                end
                S_Y3_2: begin
                    mult_A <= T4; // R*(U1*H^2 - X3)
                    mult_B <= U2; // S1*H^3
                    Y3 <= mod_sub_res; // << Y3 COMPLETADO >>
                    state <= S_Z3_MUL1;
                end

                // --- ETAPA 17: Calcular Z3 = H * Z1 * Z2 ---
                S_Z3_MUL1: begin
                    mult_A <= Z1;
                    mult_B <= Z2;
                    mult_start <= 1;
                    state <= S_WAIT_15;
                end
                S_WAIT_15: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; // T1 = Z1*Z2
                        state <= S_Z3_MUL2;
                    end
                end
                S_Z3_MUL2: begin
                    mult_A <= H;
                    mult_B <= T1; // Z1*Z2
                    mult_start <= 1;
                    state <= S_WAIT_16;
                end
                S_WAIT_16: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        Z3 <= mult_out; // << Z3 COMPLETADO >>
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    done <= 1;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule