// ============================================================================
// Módulo: ecc_point_ram.v
// Objetivo: Almacenamiento de puntos precomputados (coordenadas Jacobianas/Afines)
// Optimizado para: Intel Cyclone IV (Inferencia nativa de Bloques M9K)
// ============================================================================

module ecc_point_ram #(
    parameter DATA_WIDTH = 256,  // Ancho de palabra para coordenadas de 256 bits (secp256k1 / NIST P-256)
    parameter ADDR_WIDTH = 4     // 2^4 = 16 posiciones (suficiente para tabla de precomputo con w=4)
)(
    input  wire                  clk,
    
    // --- Puerto de Escritura ---
    input  wire                  we,
    input  wire [ADDR_WIDTH-1:0] write_addr,
    input  wire [DATA_WIDTH-1:0] data_in,
    
    // --- Puerto de Lectura ---
    input  wire [ADDR_WIDTH-1:0] read_addr,
    output reg  [DATA_WIDTH-1:0] data_out
);

    // Directiva explícita de síntesis para Quartus Prime / Quartus II
    (* ramstyle = "M9K, no_rw_check" *) reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

    // 1. Escritura Síncrona
    always @(posedge clk) begin
        if (we) begin
            ram[write_addr] <= data_in;
        end
    end

    // 2. Lectura Estrictamente Síncrona (Requisito fundamental para inferir M9K)
    always @(posedge clk) begin
        data_out <= ram[read_addr];
    end

endmodule