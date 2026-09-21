`timescale 1ns / 1ps

module ecc_precompute_fsm #(
    parameter NUM_POINTS = 4'd15 // Precálculo estándar de 15 puntos (ventana de 4 bits)
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    output reg          done,

    // --------------------------------------------------------
    // PUNTO BASE DE LA CURVA (Normalmente G_x y G_y)
    // --------------------------------------------------------
    input  wire [255:0] base_X,
    input  wire [255:0] base_Y,

    // --------------------------------------------------------
    // INTERFAZ CON EL SUMADOR DE PUNTOS COMPARTIDO (En el Top)
    // --------------------------------------------------------
    output reg          add_start,
    input  wire         add_done,
    output reg  [255:0] arg_X1_add,
    output reg  [255:0] arg_Y1_add,
    output reg  [255:0] arg_Z1_add,
    output reg  [255:0] arg_X2_add,
    output reg  [255:0] arg_Y2_add,
    output reg  [255:0] arg_Z2_add,
    input  wire [255:0] res_X_add,
    input  wire [255:0] res_Y_add,
    input  wire [255:0] res_Z_add,

    // --------------------------------------------------------
    // INTERFAZ CON EL DUPLICADOR DE PUNTOS COMPARTIDO (En el Top)
    // --------------------------------------------------------
    output reg          dbl_start,
    input  wire         dbl_done,
    output reg  [255:0] arg_X1_dbl,
    output reg  [255:0] arg_Y1_dbl,
    output reg  [255:0] arg_Z1_dbl,
    input  wire [255:0] res_X_dbl,
    input  wire [255:0] res_Y_dbl,
    input  wire [255:0] res_Z_dbl,

    // --------------------------------------------------------
    // INTERFAZ CON LA MEMORIA RAM DE PRE-CÓMPUTO
    // --------------------------------------------------------
    output reg          mem_we,
    output reg  [3:0]   mem_addr,
    output reg  [255:0] mem_data_X,
    output reg  [255:0] mem_data_Y,
    output reg  [255:0] mem_data_Z
);

    // ========================================================
    // CODIFICACIÓN DE ESTADOS (FSM)
    // ========================================================
    localparam [3:0] IDLE       = 4'd0,
                     SAVE_1P    = 4'd1,
                     START_DBL  = 4'd2,
                     WAIT_DBL   = 4'd3,
                     SAVE_2P    = 4'd4,
                     START_ADD  = 4'd5,
                     WAIT_ADD   = 4'd6,
                     SAVE_ADD   = 4'd7,
                     FINISH     = 4'd8;

    reg [3:0] state, next_state;

    // ========================================================
    // REGISTROS INTERNOS (Data Path)
    // ========================================================
    reg [3:0]   point_idx;
    reg [255:0] prev_X, prev_Y, prev_Z;
    
    // Constante 1 en 256 bits (Para la coordenada Z inicial)
    wire [255:0] ONE_256 = 256'd1;

    // ========================================================
    // LÓGICA SECUENCIAL: Actualización de Estado y Registros
    // ========================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= IDLE;
            point_idx  <= 4'd1;
            
            // Limpieza de interfaces de control
            add_start  <= 1'b0;
            dbl_start  <= 1'b0;
            mem_we     <= 1'b0;
            done       <= 1'b0;
            
            // Limpieza de operandos y memoria
            mem_addr   <= 4'd0;
            mem_data_X <= 256'd0; mem_data_Y <= 256'd0; mem_data_Z <= 256'd0;
            arg_X1_add <= 256'd0; arg_Y1_add <= 256'd0; arg_Z1_add <= 256'd0;
            arg_X2_add <= 256'd0; arg_Y2_add <= 256'd0; arg_Z2_add <= 256'd0;
            arg_X1_dbl <= 256'd0; arg_Y1_dbl <= 256'd0; arg_Z1_dbl <= 256'd0;
            prev_X     <= 256'd0; prev_Y     <= 256'd0; prev_Z     <= 256'd0;
        end else begin
            // Por defecto, apagar señales de control de un solo ciclo
            add_start <= 1'b0;
            dbl_start <= 1'b0;
            mem_we    <= 1'b0;
            
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        state     <= SAVE_1P;
                        point_idx <= 4'd1;
                    end
                end

                // Guardar 1P (El punto base)
                SAVE_1P: begin
                    mem_we     <= 1'b1;
                    mem_addr   <= point_idx;
                    mem_data_X <= base_X;
                    mem_data_Y <= base_Y;
                    mem_data_Z <= ONE_256;
                    
                    state      <= START_DBL;
                end

                // Preparar argumentos para 2P = DBL(1P)
                START_DBL: begin
                    dbl_start  <= 1'b1;
                    arg_X1_dbl <= base_X;
                    arg_Y1_dbl <= base_Y;
                    arg_Z1_dbl <= ONE_256;
                    
                    state      <= WAIT_DBL;
                end

                // Esperar a que el duplicador termine
                WAIT_DBL: begin
                    if (dbl_done) begin
                        // Guardar el resultado en los registros previos
                        prev_X <= res_X_dbl;
                        prev_Y <= res_Y_dbl;
                        prev_Z <= res_Z_dbl;
                        
                        point_idx <= point_idx + 1'b1; // idx = 2
                        state     <= SAVE_2P;
                    end
                end

                // Guardar 2P
                SAVE_2P: begin
                    mem_we     <= 1'b1;
                    mem_addr   <= point_idx;
                    mem_data_X <= prev_X;
                    mem_data_Y <= prev_Y;
                    mem_data_Z <= prev_Z;
                    
                    state      <= START_ADD;
                end

                // Preparar argumentos para sumar: iP = (i-1)P + 1P
                START_ADD: begin
                    add_start  <= 1'b1;
                    
                    // Operando 1: El punto previo (2P, 3P, etc.)
                    arg_X1_add <= prev_X;
                    arg_Y1_add <= prev_Y;
                    arg_Z1_add <= prev_Z;
                    
                    // Operando 2: El punto base (1P)
                    arg_X2_add <= base_X;
                    arg_Y2_add <= base_Y;
                    arg_Z2_add <= ONE_256;
                    
                    state      <= WAIT_ADD;
                end

                // Esperar a que el sumador termine
                WAIT_ADD: begin
                    if (add_done) begin
                        prev_X <= res_X_add;
                        prev_Y <= res_Y_add;
                        prev_Z <= res_Z_add;
                        
                        point_idx <= point_idx + 1'b1;
                        state     <= SAVE_ADD;
                    end
                end

                // Guardar iP
                SAVE_ADD: begin
                    mem_we     <= 1'b1;
                    mem_addr   <= point_idx;
                    mem_data_X <= prev_X;
                    mem_data_Y <= prev_Y;
                    mem_data_Z <= prev_Z;
                    
                    // Condición de salida del bucle
                    if (point_idx == NUM_POINTS) begin
                        state <= FINISH;
                    end else begin
                        state <= START_ADD;
                    end
                end

                FINISH: begin
                    done <= 1'b1;
                    if (!start) begin
                        state <= IDLE; // Esperar a que bajen la señal start para reiniciar
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule