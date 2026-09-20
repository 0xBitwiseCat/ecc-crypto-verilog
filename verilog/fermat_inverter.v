`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: Inversor Modular (Pequeño Teorema de Fermat)
// OBJETIVO: Calcular Z^-1 = Z^(p-2) mod p usando Square-and-Multiply.
// ============================================================================

module fermat_inverter #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    
    // Entrada en Dominio Montgomery
    input  wire [W-1:0] Z_in,
    
    // Constantes secp256k1
    input  wire [W-1:0] R_MOD_P, // El número "1" en dominio Montgomery
    
    // Salida
    output reg  [W-1:0] Z_inv,
    output reg          done,

    // Interfaz compartida hacia el Multiplicador de Montgomery
    output reg          mult_start,
    output reg  [W-1:0] mult_A, mult_B,
    input  wire [W-1:0] mult_out,
    input  wire         mult_done
);

    // ========================================================================
    // CONSTANTE: p - 2 para secp256k1
    // p = FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFE FFFFFC2F
    // p-2 = FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFE FFFFFC2D
    // ========================================================================
    localparam [255:0] P_MINUS_2 = 256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2D;

    // Registros internos
    reg [W-1:0] base_reg;
    reg [W-1:0] result_reg;
    reg [8:0]   bit_counter; // Cuenta de 0 a 255

    // ========================================================================
    // MÁQUINA DE ESTADOS FINITOS (FSM)
    // ========================================================================
    reg [2:0] state;
    
    localparam S_IDLE       = 3'd0;
    localparam S_CHECK_BIT  = 3'd1;
    localparam S_MULTIPLY   = 3'd2;
    localparam S_WAIT_MULT  = 3'd3;
    localparam S_SQUARE     = 3'd4;
    localparam S_WAIT_SQR   = 3'd5;
    localparam S_DONE       = 3'd6;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            mult_start <= 0;
            done <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        base_reg    <= Z_in;       // Z original
                        result_reg  <= R_MOD_P;    // Inicia en "1" (Montgomery)
                        bit_counter <= 0;          // Iniciamos desde el bit 0 (LSB)
                        state       <= S_CHECK_BIT;
                    end
                end

                // Algoritmo Square-and-Multiply de Derecha a Izquierda
                S_CHECK_BIT: begin
                    if (bit_counter == 256) begin
                        Z_inv <= result_reg;
                        state <= S_DONE;
                    end else if (P_MINUS_2[bit_counter] == 1'b1) begin
                        // Si el bit es 1: Result = Result * Base
                        mult_A <= result_reg;
                        mult_B <= base_reg;
                        mult_start <= 1;
                        state <= S_WAIT_MULT;
                    end else begin
                        // Si el bit es 0, saltamos directo a cuadrar la base
                        state <= S_SQUARE;
                    end
                end

                S_WAIT_MULT: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        result_reg <= mult_out;
                        state <= S_SQUARE;
                    end
                end

                S_SQUARE: begin
                    // Base = Base * Base
                    mult_A <= base_reg;
                    mult_B <= base_reg;
                    mult_start <= 1;
                    state <= S_WAIT_SQR;
                end

                S_WAIT_SQR: begin
                    mult_start <= 0;
                    if (mult_done) begin
                        base_reg <= mult_out;
                        bit_counter <= bit_counter + 1;
                        state <= S_CHECK_BIT;
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