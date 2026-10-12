// ============================================================================
// Módulo: ecc_point_ram.v
// Descripción: Inferencia directa en bloques BRAM M9K de Intel Cyclone IV
// Capacidad: 32 palabras x 512 bits (16 Kbits)
// ============================================================================

(* ramstyle = "M9K" *) module ecc_point_ram #(
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

    localparam RAM_DEPTH = 1 << (WINDOW_SIZE - 1); // 32 entradas

    // Directiva de compilador para forzar el uso de bloques M9K
    (* ramstyle = "M9K" *) reg [WIDTH-1:0] ram_x [0:RAM_DEPTH-1];
    (* ramstyle = "M9K" *) reg [WIDTH-1:0] ram_y [0:RAM_DEPTH-1];

    // Operación síncrona en un solo flanco de reloj (sin reset asíncrono)
    always @(posedge clk) begin
        if (we) begin
            ram_x[waddr] <= din_x;
            ram_y[waddr] <= din_y;
        end
        dout_x <= ram_x[raddr];
        dout_y <= ram_y[raddr];
    end

endmodule