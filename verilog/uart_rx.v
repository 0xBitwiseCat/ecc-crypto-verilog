// ============================================================================
// Módulo: uart_rx.v
// Descripción: Receptor UART RS-232 genérico (8 bits, 1 bit stop, sin paridad)
// ============================================================================

module uart_rx #(
    parameter CLK_FREQ  = 50000000, // Frecuencia del reloj principal (50 MHz)
    parameter BAUD_RATE = 115200    // Velocidad de transmisión UART
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data_out,
    output reg        valid
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam ST_IDLE  = 2'b00;
    localparam ST_START = 2'b01;
    localparam ST_DATA  = 2'b10;
    localparam ST_STOP  = 2'b11;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  rx_shift;

    // Sincronización de doble registro para evitar metaestabilidad en el pin RX
    reg rx_sync1, rx_sync;
    always @(posedge clk) begin
        rx_sync1 <= rx;
        rx_sync  <= rx_sync1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            valid    <= 1'b0;
            clk_cnt  <= 16'd0;
            bit_idx  <= 3'd0;
            data_out <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    valid   <= 1'b0;
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (!rx_sync) state <= ST_START; // Flanco de bajada (bit START)
                end

                ST_START: begin
                    if (clk_cnt == (CLKS_PER_BIT - 1) / 2) begin // Muestreo en el centro del bit
                        if (!rx_sync) begin
                            clk_cnt <= 16'd0;
                            state   <= ST_DATA;
                        end else begin
                            state   <= ST_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        clk_cnt           <= 16'd0;
                        rx_shift[bit_idx] <= rx_sync;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end
                end

                ST_STOP: begin
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        data_out <= rx_shift;
                        valid    <= 1'b1;
                        clk_cnt  <= 16'd0;
                        state    <= ST_IDLE;
                    end
                end
            endcase
        end
    end

endmodule