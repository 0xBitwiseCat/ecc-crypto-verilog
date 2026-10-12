// ============================================================================
// Módulo: ecc_point_ram.v
// Descripción: Memoria RAM de Bloque Dual-Port para almacenamiento de 
//              puntos precomputados (Qx, Qy) en secp256k1.
// ============================================================================

module ecc_point_ram #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6
)(
    input  wire                       clk,
    input  wire                       we,
    input  wire [WINDOW_SIZE-2:0]     waddr,
    input  wire [WIDTH-1:0]           din_x,
    input  wire [WIDTH-1:0]           din_y,
    input  wire [WINDOW_SIZE-2:0]     raddr,
    output reg  [WIDTH-1:0]           dout_x,
    output reg  [WIDTH-1:0]           dout_y
);

    // Tamaño de la memoria RAM: 2^(WINDOW_SIZE-1) = 32 palabras
    localparam RAM_DEPTH = 1 << (WINDOW_SIZE - 1);

    // Declaración de bloques RAM
    reg [WIDTH-1:0] ram_x [0:RAM_DEPTH-1];
    reg [WIDTH-1:0] ram_y [0:RAM_DEPTH-1];

    // Puerto de Escritura
    always @(posedge clk) begin
        if (we) begin
            ram_x[waddr] <= din_x;
            ram_y[waddr] <= din_y;
        end
    end

    // Puerto de Lectura
    always @(posedge clk) begin
        dout_x <= ram_x[raddr];
        dout_y <= ram_y[raddr];
    end

endmodule