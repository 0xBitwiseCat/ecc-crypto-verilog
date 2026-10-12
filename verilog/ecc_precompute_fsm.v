// ============================================================================
// Módulo: ecc_precompute_fsm.v
// Descripción: FSM para precomputación de la tabla de puntos impares
//              usada en el algoritmo Sliding Window (w=6).
// Optimizado para: Intel Cyclone IV
// ============================================================================

module ecc_precompute_fsm #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // Control Principal
    input  wire                       start,
    output reg                        done,
    output reg                        busy,
    output wire                       op_type, // 0: Add, 1: Double

    // Datos de entrada
    input  wire [WIDTH-1:0]           gx_mont,
    input  wire [WIDTH-1:0]           gy_mont,
    input  wire [WIDTH-1:0]           p,

    // Interfaz con RAM de Puntos Precomputados
    output reg                        ram_we,
    output reg  [WINDOW_SIZE-2:0]     ram_waddr,
    output reg  [WIDTH-1:0]           ram_din_x,
    output reg  [WIDTH-1:0]           ram_din_y,

    // Interfaz con Operador Jacobiano (Doubling)
    output reg                        dbl_start,
    input  wire                       dbl_done,
    output reg  [WIDTH-1:0]           dbl_x1, dbl_y1, dbl_z1,
    input  wire [WIDTH-1:0]           dbl_x3, dbl_y3, dbl_z3,

    // Interfaz con Operador Jacobiano (Addition)
    output reg                        add_start,
    input  wire                       add_done,
    output reg  [WIDTH-1:0]           add_x1, add_y1, add_z1,
    output reg  [WIDTH-1:0]           add_x2, add_y2, add_z2,
    input  wire [WIDTH-1:0]           add_x3, add_y3, add_z3
);

    localparam NUM_POINTS = 1 << (WINDOW_SIZE - 2); // 16 puntos impares

    // Codificación de Estados
    localparam ST_IDLE      = 3'd0;
    localparam ST_STORE_G   = 3'd1;
    localparam ST_CALC_2G   = 3'd2;
    localparam ST_WAIT_2G   = 3'd3;
    localparam ST_LOOP_ADD  = 3'd4;
    localparam ST_WAIT_ADD  = 3'd5;
    localparam ST_DONE      = 3'd6;

    reg [2:0] state;
    reg [WINDOW_SIZE-2:0] point_idx;

    // Registros temporales para 2G
    reg [WIDTH-1:0] g2_x, g2_y, g2_z;

    // Generación de op_type usando las constantes válidas de este módulo
    assign op_type = (state == ST_CALC_2G || state == ST_WAIT_2G) ? 1'b1 : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            done      <= 1'b0;
            busy      <= 1'b0;
            ram_we    <= 1'b0;
            ram_waddr <= {(WINDOW_SIZE-1){1'b0}};
            ram_din_x <= {WIDTH{1'b0}};
            ram_din_y <= {WIDTH{1'b0}};
            dbl_start <= 1'b0;
            add_start <= 1'b0;
            point_idx <= {(WINDOW_SIZE-1){1'b0}};
            g2_x      <= {WIDTH{1'b0}};
            g2_y      <= {WIDTH{1'b0}};
            g2_z      <= {WIDTH{1'b0}};
            dbl_x1    <= {WIDTH{1'b0}}; dbl_y1 <= {WIDTH{1'b0}}; dbl_z1 <= {WIDTH{1'b0}};
            add_x1    <= {WIDTH{1'b0}}; add_y1 <= {WIDTH{1'b0}}; add_z1 <= {WIDTH{1'b0}};
            add_x2    <= {WIDTH{1'b0}}; add_y2 <= {WIDTH{1'b0}}; add_z2 <= {WIDTH{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done   <= 1'b0;
                    ram_we <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        point_idx <= {(WINDOW_SIZE-1){1'b0}};
                        state     <= ST_STORE_G;
                    end else begin
                        busy      <= 1'b0;
                    end
                end

                // 1. Guardar 1G en la primera posición de RAM
                ST_STORE_G: begin
                    ram_we    <= 1'b1;
                    ram_waddr <= {(WINDOW_SIZE-1){1'b0}};
                    ram_din_x <= gx_mont;
                    ram_din_y <= gy_mont;

                    // Iniciar cálculo de 2G = G + G
                    dbl_x1    <= gx_mont;
                    dbl_y1    <= gy_mont;
                    dbl_z1    <= 256'd1;
                    dbl_start <= 1'b1;
                    state     <= ST_CALC_2G;
                end

                ST_CALC_2G: begin
                    ram_we    <= 1'b0;
                    dbl_start <= 1'b0;
                    state     <= ST_WAIT_2G;
                end

                ST_WAIT_2G: begin
                    if (dbl_done) begin
                        g2_x      <= dbl_x3;
                        g2_y      <= dbl_y3;
                        g2_z      <= dbl_z3;
                        point_idx <= 5'd1;

                        // Iniciar cálculo de 3G = 1G + 2G
                        add_x1    <= gx_mont;
                        add_y1    <= gy_mont;
                        add_z1    <= 256'd1;
                        add_x2    <= dbl_x3;
                        add_y2    <= dbl_y3;
                        add_z2    <= dbl_z3;
                        add_start <= 1'b1;
                        state     <= ST_LOOP_ADD;
                    end
                end

                ST_LOOP_ADD: begin
                    add_start <= 1'b0;
                    state     <= ST_WAIT_ADD;
                end

                ST_WAIT_ADD: begin
                    if (add_done) begin
                        // Guardar el punto impar calculado
                        ram_we    <= 1'b1;
                        ram_waddr <= point_idx;
                        ram_din_x <= add_x3;
                        ram_din_y <= add_y3;

                        if (point_idx == (NUM_POINTS - 1)) begin
                            state <= ST_DONE;
                        end else begin
                            point_idx <= point_idx + 1'b1;
                            // Siguiente punto impar: (2i+1)G = (2i-1)G + 2G
                            add_x1    <= add_x3;
                            add_y1    <= add_y3;
                            add_z1    <= add_z3;
                            add_x2    <= g2_x;
                            add_y2    <= g2_y;
                            add_z2    <= g2_z;
                            add_start <= 1'b1;
                            state     <= ST_LOOP_ADD;
                        end
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