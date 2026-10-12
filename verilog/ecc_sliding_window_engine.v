// ============================================================================
// Módulo: ecc_sliding_window_engine.v
// ============================================================================

module ecc_sliding_window_engine #(
    parameter WIDTH       = 256,
    parameter WINDOW_SIZE = 6
)(
    input  wire                       clk,
    input  wire                       rst_n,

    input  wire                       start,
    output reg                        done,
    output reg                        busy,

    input  wire [WIDTH-1:0]           k_scalar,
    input  wire [WIDTH-1:0]           p,

    output reg  [WINDOW_SIZE-2:0]     ram_raddr,
    input  wire [WIDTH-1:0]           ram_dout,

    output reg  [WIDTH-1:0]           x_out,
    output reg  [WIDTH-1:0]           y_out,
    output reg  [WIDTH-1:0]           z_out
);

    localparam ST_IDLE       = 3'b000;
    localparam ST_SCAN_BIT   = 3'b001;
    localparam ST_EXTRACT_W  = 3'b010;
    localparam ST_READ_RAM   = 3'b011;
    localparam ST_ACCUMULATE = 3'b100;
    localparam ST_DONE       = 3'b101;

    reg [2:0]       state;
    integer         bit_idx;
    reg [WIDTH-1:0] k_reg;
    reg [2:0]       shift_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            done      <= 1'b0;
            busy      <= 1'b0;
            ram_raddr <= {(WINDOW_SIZE-1){1'b0}};
            x_out     <= {WIDTH{1'b0}};
            y_out     <= {WIDTH{1'b0}};
            z_out     <= {WIDTH{1'b0}};
            bit_idx   <= 255;
            k_reg     <= {WIDTH{1'b0}};
            shift_cnt <= 3'd0;
        end else begin
            case (state)

                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy    <= 1'b1;
                        k_reg   <= k_scalar;
                        bit_idx <= 255;
                        state   <= ST_SCAN_BIT;
                    end else begin
                        busy    <= 1'b0;
                    end
                end

                ST_SCAN_BIT: begin
                    if (bit_idx < 0) begin
                        state <= ST_DONE;
                    end else if (k_reg[bit_idx]) begin
                        state <= ST_EXTRACT_W;
                    end else begin
                        bit_idx <= bit_idx - 1;
                    end
                end

                ST_EXTRACT_W: begin
                    begin: extract_block
                        reg [WIDTH-1:0]       shifted_k;
                        reg [WINDOW_SIZE-1:0] raw_win;
                        integer win_len;

                        // Desplazamiento seguro sin mezcla de tipos signed/unsigned
                        if (bit_idx >= WINDOW_SIZE - 1)
                            shifted_k = k_reg >> (bit_idx - (WINDOW_SIZE - 1));
                        else
                            shifted_k = k_reg << (WINDOW_SIZE - 1 - bit_idx);

                        // Slice explícito para evitar truncamiento implícito
                        raw_win = shifted_k[WINDOW_SIZE-1:0];

                        win_len = WINDOW_SIZE;
                        while (win_len > 1 && raw_win[0] == 1'b0) begin
                            raw_win = raw_win >> 1;
                            win_len = win_len - 1;
                        end

                        ram_raddr <= raw_win[WINDOW_SIZE-1:1];
                        shift_cnt <= win_len[2:0];
                        state     <= ST_READ_RAM;
                    end
                end

                ST_READ_RAM: begin
                    state <= ST_ACCUMULATE;
                end

                ST_ACCUMULATE: begin
                    x_out <= ram_dout;
                    y_out <= (ram_dout + 1'b1)[WIDTH-1:0]; // Slice explícito [255:0]
                    z_out <= 256'd1;

                    if (bit_idx < shift_cnt) begin
                        bit_idx <= -1;
                        state   <= ST_DONE;
                    end else begin
                        bit_idx <= bit_idx - shift_cnt;
                        state   <= ST_SCAN_BIT;
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