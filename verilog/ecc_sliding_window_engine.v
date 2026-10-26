// ============================================================================
// Módulo: ecc_sliding_window_engine.v (CORREGIDO)
// Descripción: Motor de Ventana Deslizante (w=6) con Decodificador Dinámico
//              y Secuenciador Matemático Correcto (Horner's Rule).
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
    output wire                       op_type,

    // Datos
    input  wire [WIDTH-1:0]           k_scalar,
    input  wire [WIDTH-1:0]           p,
    input  wire [WIDTH-1:0]           one_mont, // 1 en dominio Montgomery (R mod p)

    // Interfaz con la ROM de Puntos Precomputados (afines, Z = 1)
    // Lectura síncrona: ram_dout es válido el ciclo siguiente a fijar ram_raddr
    output reg  [WINDOW_SIZE-2:0]     ram_raddr,
    input  wire [WIDTH-1:0]           ram_dout_x,
    input  wire [WIDTH-1:0]           ram_dout_y,

    // Interfaz de Control con Operador Jacobiano (Adición)
    output reg                        add_start,
    input  wire                       add_done,
    output reg  [WIDTH-1:0]           add_x1, add_y1, add_z1,
    output reg  [WIDTH-1:0]           add_x2, add_y2, add_z2,
    input  wire [WIDTH-1:0]           add_x3, add_y3, add_z3,

    // Interfaz de Control con Operador Jacobiano (Doblamiento)
    output reg                        dbl_start,
    input  wire                       dbl_done,
    output reg  [WIDTH-1:0]           dbl_x1, dbl_y1, dbl_z1,
    input  wire [WIDTH-1:0]           dbl_x3, dbl_y3, dbl_z3,

    // Salida final Q(X, Y, Z)
    output reg  [WIDTH-1:0]           x_out,
    output reg  [WIDTH-1:0]           y_out,
    output reg  [WIDTH-1:0]           z_out
);

    // Codificación de Estados
    localparam ST_IDLE       = 4'd0;
    localparam ST_SCAN       = 4'd1;
    localparam ST_READ_RAM   = 4'd2;
    localparam ST_DBL_LOOP   = 4'd3;
    localparam ST_WAIT_DBL   = 4'd4;
    localparam ST_ADD        = 4'd5;
    localparam ST_WAIT_ADD   = 4'd6;
    localparam ST_DONE       = 4'd7;
    localparam ST_ROM_WAIT   = 4'd8;  // Latencia de lectura de la ROM síncrona

    reg [3:0]       state;
    reg [8:0]       bits_left;   // Contador de 256 a 0
    reg [WIDTH-1:0] k_reg;       // Registro de desplazamiento
    reg             is_first;    // Carga inicial del primer punto
    reg [2:0]       win_len;     // Longitud de la ventana actual (1 a 6)
    reg [2:0]       dbl_cnt;     // Contador de doblamientos pendientes

    assign op_type = (state == ST_DBL_LOOP || state == ST_WAIT_DBL) ? 1'b1 : 1'b0;

    // Decodificador de ventana Sliding Window (Ventana Impar Dinámica de 1 a 6 bits)
    // La ROM guarda (2i+1)G en la dirección i: una ventana impar de valor v
    // (MSB en k_reg[255]) se lee en la dirección v >> 1.
    reg [2:0]             dec_len;
    reg [WINDOW_SIZE-2:0] dec_addr;

    always @(*) begin
        if (k_reg[250]) begin
            dec_len  = 3'd6;
            dec_addr = k_reg[255:251];
        end else if (k_reg[251]) begin
            dec_len  = 3'd5;
            dec_addr = {1'b0, k_reg[255:252]};
        end else if (k_reg[252]) begin
            dec_len  = 3'd4;
            dec_addr = {2'b00, k_reg[255:253]};
        end else if (k_reg[253]) begin
            dec_len  = 3'd3;
            dec_addr = {3'b000, k_reg[255:254]};
        end else if (k_reg[254]) begin
            dec_len  = 3'd2;
            dec_addr = 5'd1; // 3G
        end else begin
            dec_len  = 3'd1;
            dec_addr = 5'd0; // 1G
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            add_start  <= 1'b0;
            dbl_start  <= 1'b0;
            ram_raddr  <= 5'd0;
            x_out      <= {WIDTH{1'b0}};
            y_out      <= {WIDTH{1'b0}};
            z_out      <= {WIDTH{1'b0}};
            add_x1     <= {WIDTH{1'b0}}; add_y1 <= {WIDTH{1'b0}}; add_z1 <= {WIDTH{1'b0}};
            add_x2     <= {WIDTH{1'b0}}; add_y2 <= {WIDTH{1'b0}}; add_z2 <= {WIDTH{1'b0}};
            dbl_x1     <= {WIDTH{1'b0}}; dbl_y1 <= {WIDTH{1'b0}}; dbl_z1 <= {WIDTH{1'b0}};
            k_reg      <= {WIDTH{1'b0}};
            bits_left  <= 9'd0;
            is_first   <= 1'b1;
            win_len    <= 3'd0;
            dbl_cnt    <= 3'd0;
        end else begin
            case (state)

                ST_IDLE: begin
                    done      <= 1'b0;
                    add_start <= 1'b0;
                    dbl_start <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        k_reg     <= k_scalar;
                        bits_left <= 9'd256;
                        is_first  <= 1'b1;
                        // Q = infinito (Z = 0): con k = 0 no se reutiliza el resultado anterior
                        x_out     <= {WIDTH{1'b0}};
                        y_out     <= {WIDTH{1'b0}};
                        z_out     <= {WIDTH{1'b0}};
                        state     <= ST_SCAN;
                    end else begin
                        busy      <= 1'b0;
                    end
                end

                ST_SCAN: begin
                    if (bits_left == 9'd0 || bits_left > 9'd256) begin
                        state <= ST_DONE;
                    end else if (!k_reg[255]) begin
                        if (is_first) begin
                            k_reg     <= {k_reg[254:0], 1'b0};
                            bits_left <= bits_left - 1'b1;
                        end else begin
                            // Doblamiento simple de 1 bit para '0'
                            win_len   <= 3'd1;
                            dbl_cnt   <= 3'd1;
                            ram_raddr <= 5'd0;
                            state     <= ST_DBL_LOOP;
                        end
                    end else begin
                        // Bit MSB = 1: Extraer ventana impar
                        win_len   <= dec_len;
                        dbl_cnt   <= dec_len;
                        ram_raddr <= dec_addr;
                        state     <= ST_ROM_WAIT;
                    end
                end

                // La ROM captura ram_raddr en este flanco: ram_dout es válido en ST_READ_RAM
                ST_ROM_WAIT: state <= ST_READ_RAM;

                ST_READ_RAM: begin
                    if (is_first) begin
                        // Cargar primer punto directamente sin doblar ni sumar
                        x_out     <= ram_dout_x;
                        y_out     <= ram_dout_y;
                        z_out     <= one_mont;
                        is_first  <= 1'b0;
                        k_reg     <= k_reg << win_len;
                        bits_left <= bits_left - win_len;
                        state     <= ST_SCAN;
                    end else begin
                        // Iniciar ciclo de win_len doblamientos
                        state <= ST_DBL_LOOP;
                    end
                end

                // Ejecutar win_len doblamientos consecutivos
                ST_DBL_LOOP: begin
                    if (dbl_cnt > 3'd0) begin
                        dbl_x1    <= x_out;
                        dbl_y1    <= y_out;
                        dbl_z1    <= z_out;
                        dbl_start <= 1'b1;
                        state     <= ST_WAIT_DBL;
                    end else begin
                        if (!k_reg[255]) begin
                            // Era un '0' aislado: actualizar k_reg y escanear
                            k_reg     <= {k_reg[254:0], 1'b0};
                            bits_left <= bits_left - 1'b1;
                            state     <= ST_SCAN;
                        end else begin
                            // Tras los doblamientos, sumar el punto leído de la RAM
                            state <= ST_ADD;
                        end
                    end
                end

                ST_WAIT_DBL: begin
                    dbl_start <= 1'b0;
                    if (dbl_done) begin
                        x_out   <= dbl_x3;
                        y_out   <= dbl_y3;
                        z_out   <= dbl_z3;
                        dbl_cnt <= dbl_cnt - 1'b1;
                        state   <= ST_DBL_LOOP;
                    end
                end

                ST_ADD: begin
                    add_x1    <= x_out;
                    add_y1    <= y_out;
                    add_z1    <= z_out;
                    add_x2    <= ram_dout_x;
                    add_y2    <= ram_dout_y;
                    add_z2    <= one_mont;
                    add_start <= 1'b1;
                    state     <= ST_WAIT_ADD;
                end

                ST_WAIT_ADD: begin
                    add_start <= 1'b0;
                    if (add_done) begin
                        x_out     <= add_x3;
                        y_out     <= add_y3;
                        z_out     <= add_z3;
                        k_reg     <= k_reg << win_len;
                        bits_left <= bits_left - win_len;
                        state     <= ST_SCAN;
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