// ============================================================================
// Módulo: fpga_uart_top.v
// Descripción: Wrapper UART que conecta el puerto serie con ecc_keygen_top
// ============================================================================

module fpga_uart_top (
    input  wire clk,        // Reloj principal de la FPGA (ej. 50 MHz)
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
    localparam ST_RX_BYTES = 2'b00;
    localparam ST_PROCESS  = 2'b01;
    localparam ST_TX_BYTES = 2'b10;

    reg [1:0]  state;
    reg [5:0]  byte_cnt; // Contador de 0 a 63 bytes
    
    // Señales de los transceptores UART
    wire [7:0] rx_byte;
    wire       rx_valid;
    reg  [7:0] tx_byte;
    reg        tx_start;
    wire       tx_busy;

    // Instancia de submódulos UART RX y TX (estándar de 8 bits)
    uart_rx #(.CLK_FREQ(50000000), .BAUD_RATE(115200)) uart_rx_inst (
        .clk(clk), .rst_n(rst_n), .rx(rx_pin), .data_out(rx_byte), .valid(rx_valid)
    );

    uart_tx #(.CLK_FREQ(50000000), .BAUD_RATE(115200)) uart_tx_inst (
        .clk(clk), .rst_n(rst_n), .tx(tx_pin), .data_in(tx_byte), .start(tx_start), .busy(tx_busy)
    );

    // ------------------------------------------------------------------------
    // FSM de Orquestación Serial
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_RX_BYTES;
            byte_cnt     <= 6'd0;
            start_keygen <= 1'b0;
            k_scalar_reg <= 256'd0;
            tx_start     <= 1'b0;
            tx_byte      <= 8'd0;
        end else begin
            case (state)

                // 1. Recibir 32 bytes de k desde Python (256 bits)
                ST_RX_BYTES: begin
                    start_keygen <= 1'b0;
                    if (rx_valid) begin
                        k_scalar_reg <= {k_scalar_reg[247:0], rx_byte};
                        if (byte_cnt == 6'd31) begin
                            byte_cnt     <= 6'd0;
                            start_keygen <= 1'b1; // Iniciar cálculo
                            state        <= ST_PROCESS;
                        end else begin
                            byte_cnt <= byte_cnt + 1'b1;
                        end
                    end
                end

                // 2. Esperar a que ecc_keygen_top termine
                ST_PROCESS: begin
                    start_keygen <= 1'b0;
                    if (done_keygen) begin
                        byte_cnt <= 6'd0;
                        state    <= ST_TX_BYTES;
                    end
                end

                // 3. Enviar 64 bytes hacia Python (32 B de Qx + 32 B de Qy)
                ST_TX_BYTES: begin
                    if (!tx_busy && !tx_start) begin
                        if (byte_cnt < 6'd32) begin
                            // Transmitir Qx byte por byte
                            tx_byte  <= pub_key_x[(31 - byte_cnt)*8 +: 8];
                        end else begin
                            // Transmitir Qy byte por byte
                            tx_byte  <= pub_key_y[(63 - byte_cnt)*8 +: 8];
                        end
                        
                        tx_start <= 1'b1;
                    end else begin
                        tx_start <= 1'b0;
                        if (tx_start && tx_busy) begin
                            if (byte_cnt == 6'd63) begin
                                byte_cnt <= 6'd0;
                                state    <= ST_RX_BYTES; // Listo para la siguiente llave
                            end else begin
                                byte_cnt <= byte_cnt + 1'b1;
                            end
                        end
                    end
                end

                default: state <= ST_RX_BYTES;
            endcase
        end
    end

endmodule