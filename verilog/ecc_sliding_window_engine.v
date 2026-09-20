`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: ecc_sliding_window_engine
// OBJETIVO: Procesar 1 ventana (w=6). Ejecuta 6 doblados (2P) y luego
//           construye dinámicamente el múltiplo compuesto sumando primos y 2P.
// ============================================================================

module ecc_sliding_window_engine #(
    parameter W = 256
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    
    // Entrada: Valor actual de la ventana (0 a 63)
    input  wire [5:0]   window_val,
    
    // Estado del acumulador antes de procesar esta ventana
    input  wire [W-1:0] acc_in_X, acc_in_Y, acc_in_Z,
    
    // Estado del acumulador actualizado
    output reg  [W-1:0] acc_out_X, acc_out_Y, acc_out_Z,
    output reg          done,
    
    // Interfaz de Lectura hacia la RAM de Primos
    output reg  [4:0]   ram_addr,
    input  wire [W-1:0] ram_rX, ram_rY, ram_rZ,
    
    // Interfaz compartida hacia Point Double (2P)
    output reg          dbl_start,
    output reg  [W-1:0] dbl_in_X, dbl_in_Y, dbl_in_Z,
    input  wire         dbl_done,
    input  wire [W-1:0] dbl_out_X, dbl_out_Y, dbl_out_Z,
    
    // Interfaz compartida hacia Point Add (P+Q)
    output reg          add_start,
    output reg  [W-1:0] add_in1_X, add_in1_Y, add_in1_Z, // Acumulador
    output reg  [W-1:0] add_in2_X, add_in2_Y, add_in2_Z, // Punto de la RAM
    input  wire         add_done,
    input  wire [W-1:0] add_out_X, add_out_Y, add_out_Z
);

    // ========================================================================
    // 1. ROM COMBINACIONAL DE DESCOMPOSICIÓN
    // Formato de salida (7 bits):
    // [6:5] extra_adds : Cuántas veces sumar 2P (0, 1 o 2 veces)
    // [4:0] base_idx   : Índice en la RAM del primo base (0 a 18)
    // ========================================================================
    reg [6:0] decomp;
    
    always @(*) begin
        case(window_val)
            // PRISMOS PUROS (0 sumas extra de 2P)
            6'd1:  decomp = {2'd0, 5'd0};  // 1P (idx 0)
            6'd3:  decomp = {2'd0, 5'd2};  // 3P
            6'd5:  decomp = {2'd0, 5'd3};  // 5P
            6'd7:  decomp = {2'd0, 5'd4};  // 7P
            6'd11: decomp = {2'd0, 5'd5};  // 11P
            6'd13: decomp = {2'd0, 5'd6};  // 13P
            6'd17: decomp = {2'd0, 5'd7};  // 17P
            6'd19: decomp = {2'd0, 5'd8};  // 19P
            6'd23: decomp = {2'd0, 5'd9};  // 23P
            6'd29: decomp = {2'd0, 5'd10}; // 29P
            6'd31: decomp = {2'd0, 5'd11}; // 31P
            6'd37: decomp = {2'd0, 5'd12}; // 37P
            6'd41: decomp = {2'd0, 5'd13}; // 41P
            6'd43: decomp = {2'd0, 5'd14}; // 43P
            6'd47: decomp = {2'd0, 5'd15}; // 47P
            6'd53: decomp = {2'd0, 5'd16}; // 53P
            6'd59: decomp = {2'd0, 5'd17}; // 59P
            6'd61: decomp = {2'd0, 5'd18}; // 61P
            
            // COMPUESTOS DE 1 SUMA EXTRA (Primo + 2P)
            6'd9:  decomp = {2'd1, 5'd4};  // 9P  = 7P  + 2P
            6'd15: decomp = {2'd1, 5'd6};  // 15P = 13P + 2P
            6'd21: decomp = {2'd1, 5'd8};  // 21P = 19P + 2P
            6'd25: decomp = {2'd1, 5'd9};  // 25P = 23P + 2P
            6'd33: decomp = {2'd1, 5'd11}; // 33P = 31P + 2P
            6'd39: decomp = {2'd1, 5'd12}; // 39P = 37P + 2P
            6'd45: decomp = {2'd1, 5'd14}; // 45P = 43P + 2P
            6'd49: decomp = {2'd1, 5'd15}; // 49P = 47P + 2P
            6'd55: decomp = {2'd1, 5'd16}; // 55P = 53P + 2P
            6'd63: decomp = {2'd1, 5'd18}; // 63P = 61P + 2P
            
            // COMPUESTOS DE 2 SUMAS EXTRA (Primo + 2P + 2P)
            6'd27: decomp = {2'd2, 5'd9};  // 27P = 23P + 4P
            6'd35: decomp = {2'd2, 5'd11}; // 35P = 31P + 4P
            6'd51: decomp = {2'd2, 5'd15}; // 51P = 47P + 4P
            6'd57: decomp = {2'd2, 5'd16}; // 57P = 53P + 4P
            
            // Números pares (se ignoran en una ventana deslizante estándar)
            default: decomp = 7'd0;
        endcase
    end

    // ========================================================================
    // 2. MÁQUINA DE ESTADOS (FSM)
    // ========================================================================
    reg [3:0] state;
    reg [2:0] shift_count; // Contador de doblados (0 a 6)
    reg [1:0] extra_adds;  // Contador de sumas extras de 2P

    localparam S_IDLE          = 4'd0;
    localparam S_SHIFT_START   = 4'd1;
    localparam S_SHIFT_WAIT    = 4'd2;
    localparam S_DECODE        = 4'd3;
    localparam S_FETCH_BASE    = 4'd4;
    localparam S_ADD_BASE      = 4'd5;
    localparam S_ADD_BASE_WAIT = 4'd6;
    localparam S_CHECK_EXTRA   = 4'd7;
    localparam S_FETCH_2P      = 4'd8;
    localparam S_ADD_EXTRA     = 4'd9;
    localparam S_ADD_EXTRA_WAIT= 4'd10;
    localparam S_DONE          = 4'd11;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            done  <= 0;
            dbl_start <= 0; add_start <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        acc_out_X <= acc_in_X;
                        acc_out_Y <= acc_in_Y;
                        acc_out_Z <= acc_in_Z;
                        shift_count <= 6; // w = 6
                        state <= S_SHIFT_START;
                    end
                end

                // --- FASE 1: DESPLAZAMIENTO (DOBLADO) x6 ---
                S_SHIFT_START: begin
                    if (shift_count > 0) begin
                        dbl_in_X <= acc_out_X; dbl_in_Y <= acc_out_Y; dbl_in_Z <= acc_out_Z;
                        dbl_start <= 1;
                        state <= S_SHIFT_WAIT;
                    end else begin
                        state <= S_DECODE;
                    end
                end
                
                S_SHIFT_WAIT: begin
                    dbl_start <= 0;
                    if (dbl_done) begin
                        acc_out_X <= dbl_out_X; acc_out_Y <= dbl_out_Y; acc_out_Z <= dbl_out_Z;
                        shift_count <= shift_count - 1;
                        state <= S_SHIFT_START;
                    end
                end

                // --- FASE 2: RESOLVER EL COMPUESTO ---
                S_DECODE: begin
                    if (window_val == 0) begin
                        state <= S_DONE; // Ventana vacía, solo requeríamos doblar
                    end else begin
                        extra_adds <= decomp[6:5];
                        ram_addr   <= decomp[4:0]; // Pedir el primo base a la RAM
                        state      <= S_FETCH_BASE;
                    end
                end
                
                S_FETCH_BASE: begin
                    // Ciclo muerto para permitir que la RAM síncrona entregue el dato
                    state <= S_ADD_BASE;
                end

                S_ADD_BASE: begin
                    add_in1_X <= acc_out_X; add_in1_Y <= acc_out_Y; add_in1_Z <= acc_out_Z;
                    add_in2_X <= ram_rX;    add_in2_Y <= ram_rY;    add_in2_Z <= ram_rZ;
                    add_start <= 1;
                    state <= S_ADD_BASE_WAIT;
                end
                
                S_ADD_BASE_WAIT: begin
                    add_start <= 0;
                    if (add_done) begin
                        acc_out_X <= add_out_X; acc_out_Y <= add_out_Y; acc_out_Z <= add_out_Z;
                        state <= S_CHECK_EXTRA;
                    end
                end

                // --- FASE 3: SUMAS DE "RELLENO" (2P) ---
                S_CHECK_EXTRA: begin
                    if (extra_adds > 0) begin
                        ram_addr <= 5'd1; // El índice 1 de la RAM siempre debe contener "2P"
                        state <= S_FETCH_2P;
                    end else begin
                        state <= S_DONE;
                    end
                end
                
                S_FETCH_2P: begin
                    state <= S_ADD_EXTRA;
                end
                
                S_ADD_EXTRA: begin
                    add_in1_X <= acc_out_X; add_in1_Y <= acc_out_Y; add_in1_Z <= acc_out_Z;
                    add_in2_X <= ram_rX;    add_in2_Y <= ram_rY;    add_in2_Z <= ram_rZ; // Aquí entra 2P
                    add_start <= 1;
                    state <= S_ADD_EXTRA_WAIT;
                end
                
                S_ADD_EXTRA_WAIT: begin
                    add_start <= 0;
                    if (add_done) begin
                        acc_out_X <= add_out_X; acc_out_Y <= add_out_Y; acc_out_Z <= add_out_Z;
                        extra_adds <= extra_adds - 1;
                        state <= S_CHECK_EXTRA; // Repetir si era un salto de +4P (27, 35, 51, 57)
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