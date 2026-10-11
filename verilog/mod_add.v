// ============================================================================
// Módulo: mod_add.v (2-Stage Pipelined)
// Operación: Result = (A + B) mod P  (con A < P y B < P)
// Latencia: 2 ciclos de reloj | Throughput: 1 suma por ciclo
// Optimizado para: Intel Cyclone IV (Segmentación de cadenas de acarreo)
// ============================================================================

module mod_add #(
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
    // ETAPA 1: Suma de 257 bits (A + B) y registro de P
    // ------------------------------------------------------------------------
    reg [WIDTH:0]   stage1_sum;
    reg [WIDTH-1:0] stage1_p;
    reg             stage1_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage1_sum   <= {(WIDTH+1){1'b0}};
            stage1_p     <= {WIDTH{1'b0}};
            stage1_valid <= 1'b0;
        end else begin
            if (in_valid) begin
                stage1_sum   <= a + b; // Genera acarreo en el bit [WIDTH]
                stage1_p     <= p;
                stage1_valid <= 1'b1;
            end else begin
                stage1_valid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // ETAPA 2: Resta condicional (Suma - P) y selección de resultado
    // ------------------------------------------------------------------------
    wire [WIDTH:0] stage2_sub = stage1_sum - stage1_p;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result    <= {WIDTH{1'b0}};
            out_valid <= 1'b0;
        end else begin
            if (stage1_valid) begin
                // Si hubo acarreo en (A+B) O si (A+B) >= P (es decir, la resta no dio underflow)
                if (stage1_sum[WIDTH] || !stage2_sub[WIDTH]) begin
                    result <= stage2_sub[WIDTH-1:0];
                end else begin
                    result <= stage1_sum[WIDTH-1:0];
                end
                out_valid <= 1'b1;
            end else begin
                out_valid <= 1'b0;
            end
        end
    end

endmodule