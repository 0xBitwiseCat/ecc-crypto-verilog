// ============================================================================
// Módulo: ecc_sliding_window_engine.v
// Descripción: Control del algoritmo Sliding Window (w=6, Horner) para Q = k*G.
//              Recorre k de MSB a LSB y ordena al procesador de puntos
//              (jacobian_point_op) las rutinas INIT, DBL, ADD y CONV.
//              Q arranca en el infinito: los ceros iniciales son doblados del
//              infinito (casi gratuitos) y la primera ventana es inf + T.
//              k se desplaza 1 bit por cada doblado (sin desplazador de barril).
// Escalar: k_scalar se captura en el flanco de start.
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

    // Interfaz con el procesador de puntos
    output reg                        op_start,
    output reg  [1:0]                 op_cmd,
    output reg  [WINDOW_SIZE-2:0]     op_tbl_idx,
    input  wire                       op_done
);

    localparam [1:0] CMD_INIT = 2'd0;
    localparam [1:0] CMD_DBL  = 2'd1;
    localparam [1:0] CMD_ADD  = 2'd2;
    localparam [1:0] CMD_CONV = 2'd3;

    // Codificación de Estados
    localparam ST_IDLE = 3'd0;
    localparam ST_SCAN = 3'd1;  // Decide: bit 0 (un doblado) o ventana impar
    localparam ST_DBL  = 3'd2;  // Doblados pendientes y, al final, la suma
    localparam ST_WAIT = 3'd3;  // Espera a que el procesador termine
    localparam ST_DONE = 3'd4;

    reg [2:0]       state;
    reg [2:0]       ret_state;   // Estado al que se vuelve tras ST_WAIT
    reg [8:0]       bits_left;   // Bits de k por procesar (256 a 0)
    reg [WIDTH-1:0] k_reg;       // Registro de desplazamiento (MSB primero)
    reg [2:0]       dbl_cnt;     // Doblados pendientes de la ventana actual
    reg             in_window;   // La ventana termina con una suma

    // Decodificador de ventana Sliding Window (Ventana Impar Dinámica de 1 a 6 bits)
    // La tabla guarda (2i+1)G en el índice i: una ventana impar de valor v
    // (MSB en k_reg[255]) usa el índice v >> 1.
    reg [2:0]             dec_len;
    reg [WINDOW_SIZE-2:0] dec_idx;

    always @(*) begin
        if (k_reg[250]) begin
            dec_len = 3'd6;
            dec_idx = k_reg[255:251];
        end else if (k_reg[251]) begin
            dec_len = 3'd5;
            dec_idx = {1'b0, k_reg[255:252]};
        end else if (k_reg[252]) begin
            dec_len = 3'd4;
            dec_idx = {2'b00, k_reg[255:253]};
        end else if (k_reg[253]) begin
            dec_len = 3'd3;
            dec_idx = {3'b000, k_reg[255:254]};
        end else if (k_reg[254]) begin
            dec_len = 3'd2;
            dec_idx = 5'd1; // 3G
        end else begin
            dec_len = 3'd1;
            dec_idx = 5'd0; // 1G
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            ret_state  <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            op_start   <= 1'b0;
            op_cmd     <= CMD_INIT;
            op_tbl_idx <= {(WINDOW_SIZE-1){1'b0}};
            k_reg      <= {WIDTH{1'b0}};
            bits_left  <= 9'd0;
            dbl_cnt    <= 3'd0;
            in_window  <= 1'b0;
        end else begin
            done     <= 1'b0;
            op_start <= 1'b0;

            case (state)

                // Q = infinito
                ST_IDLE: begin
                    busy <= start;
                    if (start) begin
                        k_reg     <= k_scalar;
                        bits_left <= 9'd256;
                        op_cmd    <= CMD_INIT;
                        op_start  <= 1'b1;
                        ret_state <= ST_SCAN;
                        state     <= ST_WAIT;
                    end
                end

                ST_SCAN: begin
                    if (bits_left == 9'd0) begin
                        op_cmd    <= CMD_CONV;
                        op_start  <= 1'b1;
                        ret_state <= ST_DONE;
                        state     <= ST_WAIT;
                    end else if (!k_reg[255]) begin
                        dbl_cnt   <= 3'd1;      // Bit '0': un doblado
                        in_window <= 1'b0;
                        state     <= ST_DBL;
                    end else begin
                        dbl_cnt    <= dec_len;  // Ventana: dec_len doblados + suma
                        op_tbl_idx <= dec_idx;
                        in_window  <= 1'b1;
                        state      <= ST_DBL;
                    end
                end

                ST_DBL: begin
                    if (dbl_cnt != 3'd0) begin
                        op_cmd    <= CMD_DBL;
                        op_start  <= 1'b1;
                        ret_state <= ST_DBL;
                        state     <= ST_WAIT;
                    end else if (in_window) begin
                        in_window <= 1'b0;
                        op_cmd    <= CMD_ADD;
                        op_start  <= 1'b1;
                        ret_state <= ST_SCAN;
                        state     <= ST_WAIT;
                    end else begin
                        state     <= ST_SCAN;
                    end
                end

                // Cada doblado consume un bit de k
                ST_WAIT: begin
                    if (op_done) begin
                        if (op_cmd == CMD_DBL) begin
                            k_reg     <= {k_reg[WIDTH-2:0], 1'b0};
                            bits_left <= bits_left - 1'b1;
                            dbl_cnt   <= dbl_cnt - 1'b1;
                        end
                        state <= ret_state;
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
