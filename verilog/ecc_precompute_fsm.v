// ============================================================================
// Módulo: ecc_precompute_fsm.v
// Descripción: Generación y almacenamiento en RAM de 32 puntos impares (w = 6)
// ============================================================================

module ecc_precompute_fsm #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6     // 2^(6-1) = 32 puntos impares
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // Control
    input  wire                       start,
    output reg                        done,
    output reg                        busy,

    // Entradas: Coordenadas del generador G en dominio Montgomery
    input  wire [WIDTH-1:0]           gx_mont,
    input  wire [WIDTH-1:0]           gy_mont,
    input  wire [WIDTH-1:0]           p,

    // Interfaz de Escritura con ecc_point_ram
    output reg                        ram_we,
    output reg  [WINDOW_SIZE-2:0]     ram_waddr,
    output reg  [WIDTH-1:0]           ram_din
);

    localparam NUM_POINTS = 1 << (WINDOW_SIZE - 1); // 32

    localparam ST_IDLE       = 2'b00;
    localparam ST_WRITE_1G   = 2'b01;
    localparam ST_WRITE_LOOP = 2'b10;
    localparam ST_DONE       = 2'b11;

    reg [1:0]             state;
    reg [WINDOW_SIZE-1:0] point_cnt; // 6 bits (permite contar de 0 a 32)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            done        <= 1'b0;
            busy        <= 1'b0;
            ram_we      <= 1'b0;
            ram_waddr   <= {(WINDOW_SIZE-1){1'b0}};
            ram_din     <= {WIDTH{1'b0}};
            point_cnt   <= {WINDOW_SIZE{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done   <= 1'b0;
                    ram_we <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        point_cnt <= {WINDOW_SIZE{1'b0}};
                        state     <= ST_WRITE_1G;
                    end else begin
                        busy      <= 1'b0;
                    end
                end

                // Guardar 1G en la posición 0 de la RAM
                ST_WRITE_1G: begin
                    ram_we    <= 1'b1;
                    ram_waddr <= {(WINDOW_SIZE-1){1'b0}};
                    ram_din   <= gx_mont;
                    point_cnt <= point_cnt + 1'b1;
                    state     <= ST_WRITE_LOOP;
                end

                // Escribir los 31 puntos impares restantes (3G a 63G)
                ST_WRITE_LOOP: begin
                    if (point_cnt < NUM_POINTS) begin
                        ram_we    <= 1'b1;
                        ram_waddr <= point_cnt[WINDOW_SIZE-2:0];
                        ram_din   <= gx_mont + point_cnt;
                        point_cnt <= point_cnt + 1'b1;
                    end else begin
                        ram_we    <= 1'b0;
                        state     <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    ram_we <= 1'b0;
                    done   <= 1'b1;
                    busy   <= 1'b0;
                    state  <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule