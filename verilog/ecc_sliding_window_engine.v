// ============================================================================
// Módulo: ecc_sliding_window_engine.v
// Descripción: Orquestador FSM para multiplicación escalar k*G usando
//              Coordenadas Jacobianas en Dominio Montgomery.
// ============================================================================

module ecc_sliding_window_engine #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // Control
    input  wire                       start,
    output reg                        done,
    output reg                        busy,

    // Datos
    input  wire [WIDTH-1:0]           k_scalar,
    input  wire [WIDTH-1:0]           p,

    // Interfaz con RAM de Puntos Precomputados
    output reg  [WINDOW_SIZE-2:0]     ram_raddr,
    input  wire [WIDTH-1:0]           ram_dout_x,
    input  wire [WIDTH-1:0]           ram_dout_y,

    // Interfaz de Control con jacobian_point_add
    output reg                        add_start,
    input  wire                       add_done,
    output reg  [WIDTH-1:0]           add_x1, add_y1, add_z1,
    output reg  [WIDTH-1:0]           add_x2, add_y2, add_z2,
    input  wire [WIDTH-1:0]           add_x3, add_y3, add_z3,

    // Interfaz de Control con jacobian_point_double
    output reg                        dbl_start,
    input  wire                       dbl_done,
    output reg  [WIDTH-1:0]           dbl_x1, dbl_y1, dbl_z1,
    input  wire [WIDTH-1:0]           dbl_x3, dbl_y3, dbl_z3,

    // Salida final Q(X, Y, Z) en coordenadas Jacobianas Montgomery
    output reg  [WIDTH-1:0]           x_out,
    output reg  [WIDTH-1:0]           y_out,
    output reg  [WIDTH-1:0]           z_out
);

    // Codificación de Estados
    localparam ST_IDLE       = 3'd0;
    localparam ST_SCAN       = 3'd1;
    localparam ST_DBL        = 3'd2;
    localparam ST_WAIT_DBL   = 3'd3;
    localparam ST_READ_RAM   = 3'd4;
    localparam ST_ADD        = 3'd5;
    localparam ST_WAIT_ADD   = 3'd6;
    localparam ST_DONE       = 3'd7;

    reg [2:0]       state;
    integer         bit_idx;
    reg [WIDTH-1:0] k_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            add_start  <= 1'b0;
            dbl_start  <= 1'b0;
            ram_raddr  <= {(WINDOW_SIZE-1){1'b0}};
            x_out      <= {WIDTH{1'b0}};
            y_out      <= {WIDTH{1'b0}};
            z_out      <= {WIDTH{1'b0}};
            add_x1     <= {WIDTH{1'b0}}; add_y1 <= {WIDTH{1'b0}}; add_z1 <= {WIDTH{1'b0}};
            add_x2     <= {WIDTH{1'b0}}; add_y2 <= {WIDTH{1'b0}}; add_z2 <= {WIDTH{1'b0}};
            dbl_x1     <= {WIDTH{1'b0}}; dbl_y1 <= {WIDTH{1'b0}}; dbl_z1 <= {WIDTH{1'b0}};
            bit_idx    <= 255;
            k_reg      <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done      <= 1'b0;
                    add_start <= 1'b0;
                    dbl_start <= 1'b0;
                    if (start) begin
                        busy    <= 1'b1;
                        k_reg   <= k_scalar;
                        bit_idx <= 255;
                        state   <= ST_SCAN;
                    end else begin
                        busy    <= 1'b0;
                    end
                end

                // Explorar bits de k desde MSB a LSB
                ST_SCAN: begin
                    if (bit_idx < 0) begin
                        state <= ST_DONE;
                    end else if (k_reg[bit_idx]) begin
                        // Iniciar consulta de punto en RAM
                        ram_raddr <= k_reg[bit_idx -: (WINDOW_SIZE-1)];
                        state     <= ST_READ_RAM;
                    end else begin
                        // Ejecutar duplicación de punto
                        dbl_x1    <= x_out;
                        dbl_y1    <= y_out;
                        dbl_z1    <= z_out;
                        dbl_start <= 1'b1;
                        state     <= ST_WAIT_DBL;
                    end
                end

                ST_WAIT_DBL: begin
                    dbl_start <= 1'b0;
                    if (dbl_done) begin
                        x_out   <= dbl_x3;
                        y_out   <= dbl_y3;
                        z_out   <= dbl_z3;
                        bit_idx <= bit_idx - 1;
                        state   <= ST_SCAN;
                    end
                end

                ST_READ_RAM: begin
                    state <= ST_ADD;
                end

                ST_ADD: begin
                    add_x1    <= x_out;
                    add_y1    <= y_out;
                    add_z1    <= z_out;
                    add_x2    <= ram_dout_x;
                    add_y2    <= ram_dout_y;
                    add_z2    <= 256'd1; // Z=1 para puntos afines precomputados
                    add_start <= 1'b1;
                    state     <= ST_WAIT_ADD;
                end

                ST_WAIT_ADD: begin
                    add_start <= 1'b0;
                    if (add_done) begin
                        x_out   <= add_x3;
                        y_out   <= add_y3;
                        z_out   <= add_z3;
                        bit_idx <= bit_idx - WINDOW_SIZE;
                        state   <= ST_SCAN;
                    end
                end

                ST_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule