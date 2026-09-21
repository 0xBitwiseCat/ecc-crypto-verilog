`timescale 1ns / 1ps

// ============================================================================
// MÓDULO: RAM de Puntos Jacobianos (256-bit x 3)
// OBJETIVO: Almacenar los múltiplos precalculados para la ventana deslizante
// ============================================================================

module ecc_point_ram #(
    parameter W = 256,
    parameter ADDR_BITS = 5 // 32 posiciones (suficiente para índices 0 a 18)
)(
    input  wire                 clk,
    
    // ==========================================
    // PUERTO 0: ESCRITURA (Usado por el Pre-calculador)
    // ==========================================
    input  wire                 we,          // Write Enable
    input  wire [ADDR_BITS-1:0] addr_w,      // Dirección de escritura
    input  wire [W-1:0]         X_w,         // Coordenada X a guardar
    input  wire [W-1:0]         Y_w,         // Coordenada Y a guardar
    input  wire [W-1:0]         Z_w,         // Coordenada Z a guardar
    
    // ==========================================
    // PUERTO 1: LECTURA (Usado por ecc_sliding_window_engine)
    // ==========================================
    input  wire [ADDR_BITS-1:0] addr_r,      // Dirección de lectura
    output reg  [W-1:0]         X_r,         // Coordenada X leída
    output reg  [W-1:0]         Y_r,         // Coordenada Y leída
    output reg  [W-1:0]         Z_r          // Coordenada Z leída
);

    // Arrays de memoria para cada coordenada (Inferencia de BRAM)
    reg [W-1:0] mem_X [0:(1<<ADDR_BITS)-1];
    reg [W-1:0] mem_Y [0:(1<<ADDR_BITS)-1];
    reg [W-1:0] mem_Z [0:(1<<ADDR_BITS)-1];

    // Operación Síncrona de Escritura y Lectura
    always @(posedge clk) begin
        if (we) begin
            mem_X[addr_w] <= X_w;
            mem_Y[addr_w] <= Y_w;
            mem_Z[addr_w] <= Z_w;
        end
        
        // La lectura es síncrona para asegurar que se utilicen Block RAMs de la FPGA
        // y no LUTs (Flip-Flops), ahorrando drásticamente el área lógica.
        X_r <= mem_X[addr_r];
        Y_r <= mem_Y[addr_r];
        Z_r <= mem_Z[addr_r];
    end

endmodule