`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: Doblado de Punto Jacobiano (2P)
// OBJETIVO: Optimización para secp256k1 (a=0) mediante "Resource Sharing"
// ============================================================================

module jacobian_point_double #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    
    // Punto P
    input  wire [W-1:0] X1, Y1, Z1,
    // Módulo de la curva
    input  wire [W-1:0] P_MOD,
    
    // Resultado Punto R = 2P
    output reg  [W-1:0] X3, Y3, Z3,
    output reg          done
);

    // 1. REGISTROS INTERNOS TEMPORALES
    reg [W-1:0] T1, T2, T3, T4;

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

    // 3. ALU COMBINACIONAL (Sumas, restas y multiplicaciones por 2)
    wire [W:0] sum_val = mult_A + mult_B; 
    wire [W-1:0] mod_add_res = (sum_val >= P_MOD) ? (sum_val - P_MOD) : sum_val;
    wire [W-1:0] mod_sub_res = (mult_A >= mult_B) ? (mult_A - mult_B) : (mult_A + P_MOD - mult_B);

    // 4. MÁQUINA DE ESTADOS FINITOS (6 bits)
    reg [5:0] state;
    
    localparam S_IDLE       = 6'd0;
    
    localparam S_T1_SQR     = 6'd1;
    localparam S_WAIT_1     = 6'd2;
    localparam S_T2_SQR     = 6'd3;
    localparam S_WAIT_2     = 6'd4;
    localparam S_T3_MUL     = 6'd5;
    localparam S_WAIT_3     = 6'd6;
    
    localparam S_M_ADD1     = 6'd7;
    localparam S_M_ADD2     = 6'd8;
    localparam S_S_ADD1     = 6'd9;
    localparam S_S_ADD2     = 6'd10;
    
    localparam S_M_SQR      = 6'd11;
    localparam S_WAIT_4     = 6'd12;
    localparam S_X3_CALC1   = 6'd13;
    localparam S_X3_CALC2   = 6'd14;
    
    localparam S_T4_SQR     = 6'd15;
    localparam S_WAIT_5     = 6'd16;
    localparam S_U_ADD1     = 6'd17;
    localparam S_U_ADD2     = 6'd18;
    localparam S_U_ADD3     = 6'd19;
    
    localparam S_Y3_SUB1    = 6'd20;
    localparam S_Y3_MUL     = 6'd21;
    localparam S_WAIT_6     = 6'd22;
    localparam S_Y3_CALC    = 6'd23;
    
    localparam S_Z3_MUL     = 6'd24;
    localparam S_WAIT_7     = 6'd25;
    localparam S_Z3_CALC    = 6'd26;
    
    localparam S_DONE       = 6'd63;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            mult_start <= 0;
            done <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) state <= S_T1_SQR;
                end

                // --- ETAPA 1: T1 = X1^2 ---
                S_T1_SQR: begin
                    mult_A <= X1;      
                    mult_B <= X1;      
                    mult_start <= 1;   
                    state <= S_WAIT_1;
                end
                S_WAIT_1: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; 
                        state <= S_T2_SQR;
                    end
                end

                // --- ETAPA 2: T2 = Y1^2 ---
                S_T2_SQR: begin
                    mult_A <= Y1;
                    mult_B <= Y1;
                    mult_start <= 1;
                    state <= S_WAIT_2;
                end
                S_WAIT_2: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T2 <= mult_out;
                        state <= S_T3_MUL;
                    end
                end

                // --- ETAPA 3: T3 = X1 * Y1^2 ---
                S_T3_MUL: begin
                    mult_A <= X1;
                    mult_B <= T2; // Y1^2      
                    mult_start <= 1;
                    state <= S_WAIT_3;
                end
                S_WAIT_3: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T3 <= mult_out; 
                        state <= S_M_ADD1;
                    end
                end

                // --- ETAPA 4: M = 3 * X1^2 (Combinacional) ---
                S_M_ADD1: begin
                    mult_A <= T1;
                    mult_B <= T1;
                    T4 <= mod_add_res; // T4 = 2 * X1^2
                    state <= S_M_ADD2;
                end
                S_M_ADD2: begin
                    mult_A <= T1; // X1^2
                    mult_B <= T4; // 2 * X1^2
                    T1 <= mod_add_res; // Sobrescribe T1 -> M = 3 * X1^2
                    state <= S_S_ADD1;
                end

                // --- ETAPA 5: S = 4 * X1 * Y1^2 (Combinacional) ---
                S_S_ADD1: begin
                    mult_A <= T3;
                    mult_B <= T3;
                    T4 <= mod_add_res; // T4 = 2 * (X1 * Y1^2)
                    state <= S_S_ADD2;
                end
                S_S_ADD2: begin
                    mult_A <= T4;
                    mult_B <= T4;
                    T3 <= mod_add_res; // Sobrescribe T3 -> S = 4 * (X1 * Y1^2)
                    state <= S_M_SQR;
                end

                // --- ETAPA 6: M^2 ---
                S_M_SQR: begin
                    mult_A <= T1; // M
                    mult_B <= T1; // M
                    mult_start <= 1;
                    state <= S_WAIT_4;
                end
                S_WAIT_4: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T4 <= mult_out; // T4 = M^2
                        state <= S_X3_CALC1;
                    end
                end

                // --- ETAPA 7: X3 = M^2 - 2S ---
                S_X3_CALC1: begin
                    mult_A <= T3; // S
                    mult_B <= T3; // S
                    X3 <= mod_add_res; // Temporalmente usamos X3 para 2S (ahorra área)
                    state <= S_X3_CALC2;
                end
                S_X3_CALC2: begin
                    mult_A <= T4; // M^2
                    mult_B <= X3; // 2S
                    X3 <= mod_sub_res; // << X3 COMPLETADO >>
                    state <= S_T4_SQR;
                end

                // --- ETAPA 8: Y1^4 = (Y1^2)^2 ---
                S_T4_SQR: begin
                    mult_A <= T2; // Y1^2
                    mult_B <= T2; // Y1^2
                    mult_start <= 1;
                    state <= S_WAIT_5;
                end
                S_WAIT_5: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T2 <= mult_out; // Sobrescribe T2 con Y1^4
                        state <= S_U_ADD1;
                    end
                end

                // --- ETAPA 9: U = 8 * Y1^4 (Combinacional) ---
                S_U_ADD1: begin
                    mult_A <= T2;
                    mult_B <= T2;
                    T4 <= mod_add_res; // T4 = 2 * Y1^4
                    state <= S_U_ADD2;
                end
                S_U_ADD2: begin
                    mult_A <= T4;
                    mult_B <= T4;
                    T2 <= mod_add_res; // T2 = 4 * Y1^4
                    state <= S_U_ADD3;
                end
                S_U_ADD3: begin
                    mult_A <= T2;
                    mult_B <= T2;
                    T2 <= mod_add_res; // Sobrescribe T2 -> U = 8 * Y1^4
                    state <= S_Y3_SUB1;
                end

                // --- ETAPA 10: S - X3 ---
                S_Y3_SUB1: begin
                    mult_A <= T3; // S
                    mult_B <= X3; 
                    T4 <= mod_sub_res; // T4 = (S - X3)
                    state <= S_Y3_MUL;
                end

                // --- ETAPA 11: M * (S - X3) ---
                S_Y3_MUL: begin
                    mult_A <= T1; // M
                    mult_B <= T4; // (S - X3)
                    mult_start <= 1;
                    state <= S_WAIT_6;
                end
                S_WAIT_6: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T4 <= mult_out; // T4 = M * (S - X3)
                        state <= S_Y3_CALC;
                    end
                end

                // --- ETAPA 12: Y3 = M * (S - X3) - U ---
                S_Y3_CALC: begin
                    mult_A <= T4; // M * (S - X3)
                    mult_B <= T2; // U (8 * Y1^4)
                    Y3 <= mod_sub_res; // << Y3 COMPLETADO >>
                    state <= S_Z3_MUL;
                end

                // --- ETAPA 13: Y1 * Z1 ---
                S_Z3_MUL: begin
                    mult_A <= Y1; 
                    mult_B <= Z1; 
                    mult_start <= 1;
                    state <= S_WAIT_7;
                end
                S_WAIT_7: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T4 <= mult_out; 
                        state <= S_Z3_CALC;
                    end
                end

                // --- ETAPA 14: Z3 = 2 * Y1 * Z1 ---
                S_Z3_CALC: begin
                    mult_A <= T4; // Y1 * Z1
                    mult_B <= T4; // Y1 * Z1
                    Z3 <= mod_add_res; // << Z3 COMPLETADO >>
                    state <= S_DONE;
                end

                S_DONE: begin
                    done <= 1;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule