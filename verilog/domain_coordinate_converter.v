`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: Conversor de Coordenadas y Dominio (Afín <-> Jacobiana Montgomery)
// OBJETIVO: Reutilizar el núcleo de Montgomery para entrar y salir del dominio.
// ============================================================================

module domain_coordinate_converter #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    input  wire         mode,       // 0: Entrada (Afín->Jac), 1: Salida (Jac->Afín)
    
    // Entradas
    input  wire [W-1:0] X_in, Y_in, Z_in, 
    input  wire [W-1:0] Z_inv_in,   // Z^-1 precalculado por Fermat en el Top-Level
    
    // Constantes de secp256k1
    input  wire [W-1:0] P_MOD,
    input  wire [W-1:0] R_MOD_P,    // R mod p (Equivale a 1 en Montgomery)
    input  wire [W-1:0] R2_MOD_P,   // R^2 mod p
    
    // Salidas
    output reg  [W-1:0] X_out, Y_out, Z_out,
    output reg          done,

    // Interfaz hacia el Multiplicador de Montgomery compartido
    output reg          mult_start,
    output reg  [W-1:0] mult_A, mult_B,
    input  wire [W-1:0] mult_out,
    input  wire         mult_done
);

    reg [W-1:0] T1; // Registro temporal

    // ========================================================================
    // MÁQUINA DE ESTADOS FINITOS (FSM)
    // ========================================================================
    reg [4:0] state;
    
    // Estados de Entrada
    localparam S_IDLE       = 5'd0;
    localparam S_IN_X       = 5'd1;
    localparam S_IN_X_WAIT  = 5'd2;
    localparam S_IN_Y       = 5'd3;
    localparam S_IN_Y_WAIT  = 5'd4;
    
    // Estados de Salida
    localparam S_OUT_Z2      = 5'd5;
    localparam S_OUT_Z2_WAIT = 5'd6;
    localparam S_OUT_X       = 5'd7;
    localparam S_OUT_X_WAIT  = 5'd8;
    localparam S_OUT_Z3      = 5'd9;
    localparam S_OUT_Z3_WAIT = 5'd10;
    localparam S_OUT_Y       = 5'd11;
    localparam S_OUT_Y_WAIT  = 5'd12;
    localparam S_EXIT_X      = 5'd13;
    localparam S_EXIT_X_WAIT = 5'd14;
    localparam S_EXIT_Y      = 5'd15;
    localparam S_EXIT_Y_WAIT = 5'd16;
    localparam S_DONE        = 5'd31;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            done <= 0;
            mult_start <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        if (mode == 1'b0) state <= S_IN_X;      // Ruta de Entrada
                        else              state <= S_OUT_Z2;    // Ruta de Salida
                    end
                end

                // ============================================================
                // RUTA 1: ENTRADA (Afín -> Jacobiana Montgomery)
                // Matemáticas: X_jac = MonPro(X, R^2), Y_jac = MonPro(Y, R^2), Z_jac = R
                // ============================================================
                S_IN_X: begin
                    mult_A <= X_in;
                    mult_B <= R2_MOD_P; 
                    mult_start <= 1;
                    state <= S_IN_X_WAIT;
                end
                S_IN_X_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        X_out <= mult_out;
                        state <= S_IN_Y;
                    end
                end
                S_IN_Y: begin
                    mult_A <= Y_in;
                    mult_B <= R2_MOD_P;
                    mult_start <= 1;
                    state <= S_IN_Y_WAIT;
                end
                S_IN_Y_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        Y_out <= mult_out;
                        Z_out <= R_MOD_P;   // Z = 1 transformado a Montgomery es R mod p
                        state <= S_DONE;
                    end
                end

                // ============================================================
                // RUTA 2: SALIDA (Jacobiana Montgomery -> Afín Normal)
                // Requiere Z_inv_in precalculado externamente por Fermat.
                // ============================================================
                
                // 1. Calcular Z^-2 = MonPro(Z^-1, Z^-1)
                S_OUT_Z2: begin
                    mult_A <= Z_inv_in;
                    mult_B <= Z_inv_in;
                    mult_start <= 1;
                    state <= S_OUT_Z2_WAIT;
                end
                S_OUT_Z2_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; // T1 = Z^-2 (Aún en dominio Montgomery)
                        state <= S_OUT_X;
                    end
                end
                
                // 2. Calcular X_aff_mont = MonPro(X_jac, Z^-2)
                S_OUT_X: begin
                    mult_A <= X_in;
                    mult_B <= T1;
                    mult_start <= 1;
                    state <= S_OUT_X_WAIT;
                end
                S_OUT_X_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        X_out <= mult_out; // X en dominio Montgomery
                        state <= S_OUT_Z3;
                    end
                end
                
                // 3. Calcular Z^-3 = MonPro(Z^-2, Z^-1)
                S_OUT_Z3: begin
                    mult_A <= T1;
                    mult_B <= Z_inv_in;
                    mult_start <= 1;
                    state <= S_OUT_Z3_WAIT;
                end
                S_OUT_Z3_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        T1 <= mult_out; // T1 = Z^-3
                        state <= S_OUT_Y;
                    end
                end
                
                // 4. Calcular Y_aff_mont = MonPro(Y_jac, Z^-3)
                S_OUT_Y: begin
                    mult_A <= Y_in;
                    mult_B <= T1;
                    mult_start <= 1;
                    state <= S_OUT_Y_WAIT;
                end
                S_OUT_Y_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        Y_out <= mult_out; // Y en dominio Montgomery
                        state <= S_EXIT_X;
                    end
                end
                
                // 5. Destransformar X: X_final = MonPro(X_aff_mont, 1)
                S_EXIT_X: begin
                    mult_A <= X_out;
                    mult_B <= 256'd1;      // Multiplicar por 1 remueve la constante R
                    mult_start <= 1;
                    state <= S_EXIT_X_WAIT;
                end
                S_EXIT_X_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        X_out <= mult_out; // Coordenada X Final (Afín Real)
                        state <= S_EXIT_Y;
                    end
                end
                
                // 6. Destransformar Y: Y_final = MonPro(Y_aff_mont, 1)
                S_EXIT_Y: begin
                    mult_A <= Y_out;
                    mult_B <= 256'd1;      // Multiplicar por 1 remueve la constante R
                    mult_start <= 1;
                    state <= S_EXIT_Y_WAIT;
                end
                S_EXIT_Y_WAIT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        Y_out <= mult_out; // Coordenada Y Final (Afín Real)
                        Z_out <= 256'd1;   // Z vuelve a ser estrictamente 1
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