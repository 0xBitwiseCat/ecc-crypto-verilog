// ============================================================================
// Módulo: ecc_precompute_fsm.v
// Descripción: Generador FSM de 32 Puntos Impares (1G a 63G) para secp256k1
//              utilizando Point Doubling (2G) y Point Addition secuencial.
// ============================================================================

module ecc_precompute_fsm #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6     // 2^(6-1) = 32 puntos impares
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // Control Principal
    input  wire                       start,
    output reg                        done,
    output reg                        busy,
    output wire                       op_type,

    // Entradas: Coordenadas de G en Dominio Montgomery y Módulo P
    input  wire [WIDTH-1:0]           gx_mont,
    input  wire [WIDTH-1:0]           gy_mont,
    input  wire [WIDTH-1:0]           p,

    // Interfaz de Escritura con ecc_point_ram
    output reg                        ram_we,
    output reg  [WINDOW_SIZE-2:0]     ram_waddr,
    output reg  [WIDTH-1:0]           ram_din_x,
    output reg  [WIDTH-1:0]           ram_din_y,

    // Interfaz de Control con jacobian_point_double (para calcular 2G)
    output reg                        dbl_start,
    input  wire                       dbl_done,
    output reg  [WIDTH-1:0]           dbl_x1, dbl_y1, dbl_z1,
    input  wire [WIDTH-1:0]           dbl_x3, dbl_y3, dbl_z3,

    // Interfaz de Control con jacobian_point_add (para sumar 2G repetidamente)
    output reg                        add_start,
    input  wire                       add_done,
    output reg  [WIDTH-1:0]           add_x1, add_y1, add_z1,
    output reg  [WIDTH-1:0]           add_x2, add_y2, add_z2,
    input  wire [WIDTH-1:0]           add_x3, add_y3, add_z3
);

    localparam NUM_POINTS = 1 << (WINDOW_SIZE - 1); // 32

    localparam ST_IDLE       = 3'd0;
    localparam ST_WRITE_1G   = 3'd1;
    localparam ST_CALC_2G    = 3'd2;
    localparam ST_WAIT_2G    = 3'd3;
    localparam ST_ADD_LOOP   = 3'd4;
    localparam ST_WAIT_ADD   = 3'd5;
    localparam ST_WRITE_RAM  = 3'd6;
    localparam ST_DONE       = 3'd7;

    reg [2:0]             state;
    reg [WINDOW_SIZE-1:0] point_cnt; // 6 bits (cuenta de 0 a 32 sin overflow)

    // Registros temporales para 2G
    reg [WIDTH-1:0] g2_x, g2_y, g2_z;
    // Registros del último punto impar (P_prev)
    reg [WIDTH-1:0] prev_x, prev_y, prev_z;

    // Indicar op_type = 1 (Double) durante el estado de doblamiento
    assign op_type = (state == ST_WAIT_DBL) ? 1'b1 : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            ram_we     <= 1'b0;
            ram_waddr  <= {(WINDOW_SIZE-1){1'b0}};
            ram_din_x  <= {WIDTH{1'b0}};
            ram_din_y  <= {WIDTH{1'b0}};
            dbl_start  <= 1'b0;
            add_start  <= 1'b0;
            dbl_x1     <= {WIDTH{1'b0}}; dbl_y1 <= {WIDTH{1'b0}}; dbl_z1 <= {WIDTH{1'b0}};
            add_x1     <= {WIDTH{1'b0}}; add_y1 <= {WIDTH{1'b0}}; add_z1 <= {WIDTH{1'b0}};
            add_x2     <= {WIDTH{1'b0}}; add_y2 <= {WIDTH{1'b0}}; add_z2 <= {WIDTH{1'b0}};
            g2_x       <= {WIDTH{1'b0}}; g2_y   <= {WIDTH{1'b0}}; g2_z   <= {WIDTH{1'b0}};
            prev_x     <= {WIDTH{1'b0}}; prev_y <= {WIDTH{1'b0}}; prev_z <= {WIDTH{1'b0}};
            point_cnt  <= {WINDOW_SIZE{1'b0}};
        end else begin
            case (state)

                ST_IDLE: begin
                    done      <= 1'b0;
                    ram_we    <= 1'b0;
                    dbl_start <= 1'b0;
                    add_start <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        point_cnt <= {WINDOW_SIZE{1'b0}};
                        state     <= ST_WRITE_1G;
                    end else begin
                        busy      <= 1'b0;
                    end
                end

                // 1. Almacenar 1G en la dirección 0 de la RAM
                ST_WRITE_1G: begin
                    ram_we    <= 1'b1;
                    ram_waddr <= 5'd0;
                    ram_din_x <= gx_mont;
                    ram_din_y <= gy_mont;

                    // Inicializar P_prev = 1G
                    prev_x    <= gx_mont;
                    prev_y    <= gy_mont;
                    prev_z    <= 256'd1; // Z=1 para el punto generador

                    point_cnt <= 6'd1;
                    state     <= ST_CALC_2G;
                end

                // 2. Iniciar cálculo de 2G = 1G + 1G mediante Point Doubling
                ST_CALC_2G: begin
                    ram_we    <= 1'b0;
                    dbl_x1    <= gx_mont;
                    dbl_y1    <= gy_mont;
                    dbl_z1    <= 256'd1;
                    dbl_start <= 1'b1;
                    state     <= ST_WAIT_2G;
                end

                ST_WAIT_2G: begin
                    dbl_start <= 1'b0;
                    if (dbl_done) begin
                        g2_x  <= dbl_x3;
                        g2_y  <= dbl_y3;
                        g2_z  <= dbl_z3;
                        state <= ST_ADD_LOOP;
                    end
                end

                // 3. Bucle para calcular P_{2i+1} = P_{2i-1} + 2G
                ST_ADD_LOOP: begin
                    if (point_cnt < NUM_POINTS) begin
                        add_x1    <= prev_x;
                        add_y1    <= prev_y;
                        add_z1    <= prev_z;
                        add_x2    <= g2_x;
                        add_y2    <= g2_y;
                        add_z2    <= g2_z;
                        add_start <= 1'b1;
                        state     <= ST_WAIT_ADD;
                    end else begin
                        state     <= ST_DONE;
                    end
                end

                ST_WAIT_ADD: begin
                    add_start <= 1'b0;
                    if (add_done) begin
                        prev_x <= add_x3;
                        prev_y <= add_y3;
                        prev_z <= add_z3;
                        state  <= ST_WRITE_RAM;
                    end
                end

                // 4. Escribir P_{2i+1} en la posición i de la RAM
                ST_WRITE_RAM: begin
                    ram_we    <= 1'b1;
                    ram_waddr <= point_cnt[WINDOW_SIZE-2:0];
                    ram_din_x <= prev_x;
                    ram_din_y <= prev_y;
                    point_cnt <= point_cnt + 1'b1;
                    state     <= ST_ADD_LOOP;
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