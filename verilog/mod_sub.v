// ============================================================================
// Módulo: mod_sub.v (2-Stage Pipelined)
// Operación: Result = (A - B) mod P  (con A < P y B < P)
// Latencia: 2 ciclos de reloj | Throughput: 1 resta por ciclo
// Optimizado para: Intel Cyclone IV (Segmentación de cadenas de préstamo/acarreo)
// ============================================================================

module mod_sub #(
    parameter WIDTH = 256
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control
    input  wire                 in_valid,
    output reg                  out_valid,

    // Entradas de datos
    input  wire [WIDTH-1:0]     a,
    input  wire [WIDTH-1:0]     b,
    input  wire [WIDTH-1:0]     p,

    // Salida
    output reg  [WIDTH-1:0]     result
);

    // ------------------------------------------------------------------------
    // ETAPA 1: Resta de 257 bits (A - B) y registro de P
    // ------------------------------------------------------------------------
    reg [WIDTH:0]   stage1_diff;
    reg [WIDTH-1:0] stage1_p;
    reg             stage1_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage1_diff  <= {(WIDTH+1){1'b0}};
            stage1_p     <= {WIDTH{1'b0}};
            stage1_valid <= 1'b0;
        end else begin
            if (in_valid) begin
                // El bit [WIDTH] actuará como bandera de préstamo (underflow/negative)
                stage1_diff  <= {1'b0, a} - {1'b0, b};
                stage1_p     <= p;
                stage1_valid <= 1'b1;
            end else begin
                stage1_valid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // ETAPA 2: Adición condicional de P (+ P) y selección de resultado
    // ------------------------------------------------------------------------
    wire [WIDTH-1:0] stage2_add_p = stage1_diff[WIDTH-1:0] + stage1_p;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result    <= {WIDTH{1'b0}};
            out_valid <= 1'b0;
        end else begin
            if (stage1_valid) begin
                // Si el bit MSB es 1, hubo underflow (A < B), por lo que se suma P
                if (stage1_diff[WIDTH]) begin
                    result <= stage2_add_p;
                end else begin
                    result <= stage1_diff[WIDTH-1:0];
                end
                out_valid <= 1'b1;
            end else begin
                out_valid <= 1'b0;
            end
        end
    end

endmodule
