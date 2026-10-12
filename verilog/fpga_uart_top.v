// ============================================================================
// Módulo: fpga_uart_top.v
// Descripción: Wrapper UART optimizado y desacoplado para ecc_keygen_top
// ============================================================================

module fpga_uart_top (
    input  wire clk,        // Reloj principal de la FPGA (50 MHz)
    input  wire rst_n,      // Botón de Reset (activo en bajo)
    input  wire rx_pin,     // Pin físico de entrada RX (UART)
    output wire tx_pin      // Pin físico de salida TX (UART)
);

    // ------------------------------------------------------------------------
    // Señales de Interconexión con ecc_keygen_top
    // ------------------------------------------------------------------------
    reg  [255:0] k_scalar_reg;
    reg          start_keygen;
    wire         done_keygen;
    wire [255:0] pub_key_x;
    wire [255:0] pub_key_y;

    // Buffer de transmisión independiente de 512 bits (32B Qx + 32B Qy)
    reg  [511:0] tx_buffer;

    // ------------------------------------------------------------------------
    // Instancia del Núcleo Criptográfico
    // ------------------------------------------------------------------------
    ecc_keygen_top keygen_inst (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start_keygen),
        .k_scalar     (k_scalar_reg),
        .done         (done_keygen),
        .public_key_x (pub_key_x),
        .public_key_y (pub_key_y)
    );

    // ------------------------------------------------------------------------
    // Lógica de Recepción y Transmisión UART (Controlador de Bytes)
    // ------------------------------------------------------------------------
    localparam ST_RX_BYTES = 2'd0;
    localparam ST_PROCESS  = 2'd1;
    localparam ST_TX_START = 2'd2;
    localparam ST_TX_WAIT  = 2'd3;

    reg [1:0]  state;
    reg [5:0]  byte_cnt; // Contador de 0 a 63 bytes
    
    // Señales de los transceptores UART
    wire [7:0] rx_byte;
    wire       rx_valid;
    reg  [7:0] tx_byte;
    reg        tx_start;
    wire       tx_busy;

    // Instancia de submódulos UART RX y TX (estándar de 8 bits a 115200 baudios)
    uart_rx #(.CLK_FREQ(50000000), .BAUD_RATE(115200)) uart_rx_inst (
        .clk(clk), .rst_n(rst_n), .rx(rx_pin), .data_out(rx_byte), .valid(rx_valid)
    );

    uart_tx #(.CLK_FREQ(50000000), .BAUD_RATE(115200)) uart_tx_inst (
        .clk(clk), .rst_n(rst_n), .tx(tx_pin), .data_in(tx_byte), .start(tx_start), .busy(tx_busy)
    );

    // ------------------------------------------------------------------------
    // FSM de Orquestación Serial Desacoplada
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_RX_BYTES;
            byte_cnt     <= 6'd0;
            start_keygen <= 1'b0;
            k_scalar_reg <= 256'd0;
            tx_buffer    <= 512'd0;
            tx_start     <= 1'b0;
            tx_byte      <= 8'd0;
        end else begin
            case (state)

                // 1. Recibir 32 bytes de k desde Python (256 bits, MSB a LSB)
                ST_RX_BYTES: begin
                    start_keygen <= 1'b0;
                    tx_start     <= 1'b0;
                    if (rx_valid) begin
                        k_scalar_reg <= {k_scalar_reg[247:0], rx_byte};
                        if (byte_cnt == 6'd31) begin
                            byte_cnt     <= 6'd0;
                            start_keygen <= 1'b1; // Iniciar cálculo ECC
                            state        <= ST_PROCESS;
                        end else begin
                            byte_cnt <= byte_cnt + 1'b1;
                        end
                    end
                end

                // 2. Esperar a que ecc_keygen_top termine y capturar las coordenadas
                ST_PROCESS: begin
                    start_keygen <= 1'b0;
                    if (done_keygen) begin
                        // Capturar Qx (MSB) y Qy (LSB) en el buffer estático
                        tx_buffer <= {pub_key_x, pub_key_y};
                        byte_cnt  <= 6'd0;
                        state     <= ST_TX_START;
                    end
                end

                // 3. Cargar el byte correspondiente y generar el pulso tx_start
                ST_TX_START: begin
                    tx_byte  <= tx_buffer[(63 - byte_cnt)*8 +: 8];
                    tx_start <= 1'b1;
                    state    <= ST_TX_WAIT;
                end

                // 4. Esperar la transmisión completa del byte antes de avanzar
                ST_TX_WAIT: begin
                    tx_start <= 1'b0;
                    if (!tx_busy && !tx_start) begin
                        if (byte_cnt == 6'd63) begin
                            byte_cnt <= 6'd0;
                            state    <= ST_RX_BYTES; // Listo para la siguiente prueba
                        end else begin
                            byte_cnt <= byte_cnt + 1'b1;
                            state    <= ST_TX_START;
                        end
                    end
                end

                default: state <= ST_RX_BYTES;
            endcase
        end
    end

endmodule