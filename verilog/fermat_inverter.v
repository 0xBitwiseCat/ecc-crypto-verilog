// ============================================================================
// Módulo: fermat_inverter.v (Genérico / Curve-Agnostic)
// Algoritmo: Exponenciación Binaria Left-to-Right (Square-and-Multiply)
// Operación: Result = A^(P-2) mod P
// Optimizado para: Intel Cyclone IV (Reutiliza el multiplicador externo)
// ============================================================================

module fermat_inverter #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 start,
    output reg                  done,
    output reg                  busy,

    // Entradas de datos (en dominio Montgomery)
    input  wire [WIDTH-1:0]     a, // Elemento a invertir
    input  wire [WIDTH-1:0]     p, // Primo P cualquiera de entrada
    output reg  [WIDTH-1:0]     result,

    // Interfaz con el Multiplicador de Montgomery compartido
    output reg                  mult_start,
    output reg  [WIDTH-1:0]     mult_a,
    output reg  [WIDTH-1:0]     mult_b,
    input  wire                 mult_done,
    input  wire [WIDTH-1:0]     mult_result
);

    // ------------------------------------------------------------------------
    // Estados FSM
    // ------------------------------------------------------------------------
    localparam ST_IDLE      = 3'b000;
    localparam ST_INIT      = 3'b001;
    localparam ST_SQR_START = 3'b010;
    localparam ST_SQR_WAIT  = 3'b011;
    localparam ST_MUL_CHECK = 3'b100;
    localparam ST_MUL_START = 3'b101;
    localparam ST_MUL_WAIT  = 3'b110;
    localparam ST_DONE      = 3'b111;

    reg [2:0] state;

    // Registros de control
    reg [WIDTH-1:0] acc;
    reg [WIDTH-1:0] a_reg;
    reg [WIDTH-1:0] exponent;
    reg [7:0]       bit_cnt;

    // Exponente e = p - 2
    wire [WIDTH-1:0] p_minus_2 = p - 2'd2;

    // ------------------------------------------------------------------------
    // Detector combinacional de la posición del bit más significativo (MSB)
    // ------------------------------------------------------------------------
    integer i;
    reg [7:0] msb_pos;
    always @(*) begin
        msb_pos = 8'd0;
        for (i = 0; i < WIDTH; i = i + 1) begin
            if (exponent[i]) begin
                msb_pos = i[7:0];
            end
        end
    end

    // ------------------------------------------------------------------------
    // FSM Principal
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            done        <= 1'b0;
            busy        <= 1'b0;
            mult_start  <= 1'b0;
            mult_a      <= {WIDTH{1'b0}};
            mult_b      <= {WIDTH{1'b0}};
            result      <= {WIDTH{1'b0}};
            acc         <= {WIDTH{1'b0}};
            a_reg       <= {WIDTH{1'b0}};
            exponent    <= {WIDTH{1'b0}};
            bit_cnt     <= 8'd0;
        end else begin
            case (state)

                ST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        a_reg    <= a;
                        exponent <= p_minus_2;
                        state    <= ST_INIT;
                    end else begin
                        busy     <= 1'b0;
                    end
                end

                // Detectar el MSB de P-2 e iniciar el acumulador con 'a'
                ST_INIT: begin
                    acc     <= a_reg;
                    bit_cnt <= msb_pos - 1'b1;
                    state   <= ST_SQR_START;
                end

                // ------------------------------------------------------------
                // Paso 1: Cuadratura siempre (acc = acc^2 mod P)
                // ------------------------------------------------------------
                ST_SQR_START: begin
                    mult_a     <= acc;
                    mult_b     <= acc;
                    mult_start <= 1'b1;
                    state      <= ST_SQR_WAIT;
                end

                ST_SQR_WAIT: begin
                    mult_start <= 1'b0;
                    if (mult_done) begin
                        acc   <= mult_result;
                        state <= ST_MUL_CHECK;
                    end
                end

                // ------------------------------------------------------------
                // Paso 2: Evaluar si el bit actual de P-2 es 1
                // ------------------------------------------------------------
                ST_MUL_CHECK: begin
                    if (exponent[bit_cnt]) begin
                        state <= ST_MUL_START;
                    end else begin
                        if (bit_cnt == 8'd0) begin
                            state <= ST_DONE;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                            state   <= ST_SQR_START;
                        end
                    end
                end

                // Multiplicación condicional (acc = acc * a mod P)
                ST_MUL_START: begin
                    mult_a     <= acc;
                    mult_b     <= a_reg;
                    mult_start <= 1'b1;
                    state      <= ST_MUL_WAIT;
                end

                ST_MUL_WAIT: begin
                    mult_start <= 1'b0;
                    if (mult_done) begin
                        acc <= mult_result;
                        if (bit_cnt == 8'd0) begin
                            state <= ST_DONE;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                            state   <= ST_SQR_START;
                        end
                    end
                end

                // ------------------------------------------------------------
                // Finalización
                // ------------------------------------------------------------
                ST_DONE: begin
                    result <= acc;
                    done   <= 1'b1;
                    busy   <= 1'b0;
                    state  <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
