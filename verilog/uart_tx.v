// ============================================================================
// Módulo: uart_tx.v (CORREGIDO)
// Descripción: Transmisor UART RS-232 con respuesta combinacional en busy
//              para eliminar condiciones de carrera con la FSM Top.
// ============================================================================

module uart_tx #(
    parameter CLK_FREQ  = 50000000,
    parameter BAUD_RATE = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    output reg        tx,
    input  wire [7:0] data_in,
    input  wire       start,
    output wire       busy     // Salida combinacional directa
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam ST_IDLE  = 2'b00;
    localparam ST_START = 2'b01;
    localparam ST_DATA  = 2'b10;
    localparam ST_STOP  = 2'b11;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  tx_shift;

    // Respuesta inmediata a 'start' para evitar lecturas en falso de 'busy'
    assign busy = (state != ST_IDLE) || start;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            tx       <= 1'b1;
            clk_cnt  <= 16'd0;
            bit_idx  <= 3'd0;
            tx_shift <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx      <= 1'b1;
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (start) begin
                        tx_shift <= data_in;
                        state    <= ST_START;
                    end
                end

                ST_START: begin
                    tx <= 1'b0; // Bit START
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        clk_cnt <= 16'd0;
                        state   <= ST_DATA;
                    end
                end

                ST_DATA: begin
                    tx <= tx_shift[bit_idx]; // LSB primero
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        clk_cnt <= 16'd0;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end
                end

                ST_STOP: begin
                    tx <= 1'b1; // Bit STOP
                    if (clk_cnt < CLKS_PER_BIT - 1) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end else begin
                        clk_cnt <= 16'd0;
                        state   <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule